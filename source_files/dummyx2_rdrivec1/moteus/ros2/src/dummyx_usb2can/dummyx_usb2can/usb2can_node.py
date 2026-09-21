#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. Muzixiaowen(xin.li at switchpi.com) All rights reserved.
# For more details, check out in https://gitee.com/switchpi/dummyx2

import sys
try:
    import moteus
except ImportError:
    # Add workspace moteus lib if system moteus is not found
    # sys.path.insert(0, '/home/liyq/moteus/lib/python')
    # import moteus
    print("Import moteus failed")
    sys.exit(1)

import rclpy
from rclpy.node import Node
import asyncio
import threading
import json
import math
import argparse
import time
import traceback
import csv
import os

from dummyx_interface.srv import InitUsb2Can, WriteUsb2Can, ReadUsb2Can
from sensor_msgs.msg import JointState
try:
    from control_msgs.msg import JointTrajectoryControllerState
except ImportError:
    JointTrajectoryControllerState = None

# Input Modes (Kept for compatibility with launch files)
INPUT_MODE_PASSTHROUGH = 1
INPUT_MODE_TRAP_TRAJ = 5
DEFAULT_VEL_LIMIT = 0.4    # turns/s  (≈ 2.51 rad/s, matches joint_limits.yaml)
DEFAULT_ACCEL_LIMIT = 0.1  # turns/s² (≈ 0.63 rad/s², slightly above TOTG planned max ~0.08 turns/s²)
                            # 降低此值使Moteus内部加速行为与TOTG规划加速度更匹配，减少路点间冲击
DEFAULT_MAX_TORQUE = 16.0   # Nm  缺省最大扭矩，足以抵抗重力，不会损坏关节

class MoteusWorker:
    def __init__(self, interface: str, channel: str, motor_ids: list, vel_limit: float, accel_limit: float,
                 kp_des: float = 1200.0, kd_des: float = 12.0,
                 max_torque: float = DEFAULT_MAX_TORQUE,
                 ff_torques: dict = None,
                 grav_k2: float = 0.0, grav_k3: float = 0.0,
                 logger=None):
        self.interface = interface
        self.channel = channel
        self.motor_ids = motor_ids
        self.vel_limit = vel_limit
        self.accel_limit = accel_limit
        self.kp_des = kp_des
        self.kd_des = kd_des
        self.max_torque = max_torque
        # 固定前馈力矩字典: {motor_id: Nm}，当 grav_k2==0 时对 J2/J3 生效，否则被动态重力补偿替代
        self.ff_torques = ff_torques if ff_torques is not None else {}
        # 动态重力补偿参数（模型：力矩正比于余弦，随关节角实时计算替代固定ff_torques）
        # tau_J2 = grav_k2 * cos(j2)        <- J2自身+前臂的重力矩补偿
        # tau_J3 = grav_k3 * cos(j2 + j3)   <- J3前臂的重力矩补偿
        # 初始建议值（力学推导）: grav_k2~1.35 Nm, grav_k3~0.30 Nm，需实验微调
        self.grav_k2 = grav_k2   # J2重力补偿系数(Nm); 0=禁用动态补偿，用固定ff_torques
        self.grav_k3 = grav_k3   # J3重力补偿系数(Nm); 0=禁用动态补偿
        self.logger = logger
        
        self.transport = None
        self.controllers = {}
        self.enabled = False
        self.last_sent_positions = {}
        
        # Threading for asyncio
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        
        self.commands_lock = threading.Lock()
        self.pending_commands = []
        self.last_cmds = []  # 缓存最近一次发送的指令，在无新指令时以200Hz重发、保持Moteus状态稳定
        self._last_command_debug_time = 0.0
        
        self._shutdown_event = threading.Event()
        self._last_cmd_time = 0.0  # 最近一次收到新指令的时间戳（monotonic）

    def start(self):
        self.thread.start()
        # Wait a brief moment to let transport initialize
        time.sleep(0.5)
        
    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._async_main())
        
    async def _async_main(self):
        try:
            self.transport = moteus.get_singleton_transport()
            if self.logger:
                self.logger.info("Initialized Moteus transport (auto-detect, same as webgui)")
        except Exception as e:
            if self.logger:
                self.logger.error(f"Transport initialization failed: {e}")
            return
            
        qr = moteus.QueryResolution()
        qr.q_current = moteus.INT16
        qr.motor_temperature = moteus.INT16
        
        for node_id in self.motor_ids:
            self.controllers[node_id] = moteus.Controller(
                id=node_id, transport=self.transport, query_resolution=qr)
            self.last_sent_positions[node_id] = None
            
        # Send initial stops to clear any existing faults
        stops = [c.make_stop() for c in self.controllers.values()]
        if stops:
            try:
                await self.transport.cycle(stops)
            except Exception as e:
                print(f"Error sending init stop: {e}")
                
        await asyncio.sleep(0.1)
        print("Moteus transport initialized")

        # ── Auto-enable: position=nan 保持关节当前位置，防止启动时机械臂因重力下落 ──
        # 在 MoveIt2 下发第一条 joint_state 前，持续保活各关节（看门狗 servo.timeout_s=0.1s）
        hold_cmds = [
            c.make_position(
                position=math.nan,
                velocity=0.0,
                maximum_torque=self.max_torque,
                query=True
            )
            for c in self.controllers.values()
        ]
        if hold_cmds:
            try:
                await asyncio.wait_for(self.transport.cycle(hold_cmds), timeout=0.5)
                print("[INIT] Auto-enable: all joints holding current position (prevent arm drop)")
                self.last_cmds = hold_cmds   # 缓存，主循环会重发直到 MoveIt2 接管
            except asyncio.TimeoutError:
                if self.logger:
                    self.logger.warn("[INIT] Auto-enable timeout (>0.5s); continuing to command loop")
            except Exception as e:
                print(f"[INIT] Auto-enable warning: {e}")

        if self.logger:
            self.logger.warn("[DIAG] Entering Moteus command loop")
        
        cycle_count = 0
        empty_count = 0
        _CYCLE_PERIOD = 0.010  # 100Hz 目标周期 (10ms)
        _target_t = time.monotonic() + _CYCLE_PERIOD  # 绝对 deadline，每轮递推

        # Moteus cycle 时序日志（CSV）
        # 记录每次 cycle 的时刻、间隔、发送指令数、Moteus 反馈坐标
        _log_path = '/home/liyq/moteus/tmp/moteus_cycle_log.csv'
        _log_enabled = False   # 可设为 False 关闭日志
        _log_file = None
        _log_writer = None
        _log_max_rows = 100000  # 最多记彖10万行（避免磁盘满）
        _prev_cycle_t = None
        if _log_enabled:
            try:
                os.makedirs(os.path.dirname(_log_path), exist_ok=True)
                _log_file = open(_log_path, 'w', newline='')
                _log_fieldnames = [
                    'cycle', 'timestamp', 'dt_ms', 'num_cmds',
                    'm2_pos', 'm2_vel', 'm3_pos', 'm3_vel', 'm1_pos', 'm1_vel'
                ]
                _log_writer = csv.DictWriter(_log_file, fieldnames=_log_fieldnames)
                _log_writer.writeheader()
                if self.logger:
                    self.logger.info(f"Moteus cycle CSV log: {_log_path}")
            except Exception as e:
                if self.logger:
                    self.logger.warn(f"Could not open cycle log: {e}")
                _log_file = None
                _log_writer = None
        while not self._shutdown_event.is_set():
            cmds = []
            with self.commands_lock:
                if self.pending_commands:
                    cmds = list(self.pending_commands)
                    self.pending_commands.clear()
                    self.last_cmds = cmds  # 更新缓存
                    self.last_cmd_time = time.monotonic()  # 记录收到新指令的时间
                elif self.last_cmds:
                    # 重发上次指令，维持 Moteus 位置保持模式。
                    # 若不重发：超过 servo.timeout_s(0.1s) 无 CAN 帧 → 电机进入 kStopped → 机械臂因重力下落。
                    # 重发时机：启动到 MoveIt2 下发第一条指令之前、以及两条轨迹指令之间的间隙。
                    cmds = list(self.last_cmds)
            
            if cmds:
                empty_count = 0
                try:
                    results = await asyncio.wait_for(self.transport.cycle(cmds), timeout=0.5)
                    cycle_count += 1
                    now_t = time.monotonic()

                    # 写入 Moteus cycle CSV
                    if _log_writer is not None and cycle_count <= _log_max_rows:
                        try:
                            _dt_ms = (now_t - _prev_cycle_t) * 1000.0 if _prev_cycle_t else 0.0
                            _prev_cycle_t = now_t
                            _row = {
                                'cycle':     cycle_count,
                                'timestamp': now_t,
                                'dt_ms':     round(_dt_ms, 3),
                                'num_cmds':  len(cmds),
                                'm2_pos': '', 'm2_vel': '',
                                'm3_pos': '', 'm3_vel': '',
                                'm1_pos': '', 'm1_vel': '',
                            }
                            if results:
                                for r in results:
                                    # 电机ID直接读 r.id 属性，不是寄存器
                                    _mid = getattr(r, 'id', None)
                                    _pos = r.values.get(moteus.Register.POSITION, '')
                                    _vel = r.values.get(moteus.Register.VELOCITY, '')
                                    if _mid in (1, 2, 3):
                                        _row[f'm{_mid}_pos'] = round(float(_pos) * 2 * math.pi, 5) if _pos != '' else ''
                                        _row[f'm{_mid}_vel'] = round(float(_vel) * 2 * math.pi, 5) if _vel != '' else ''
                            _log_writer.writerow(_row)
                            if cycle_count % 1000 == 0:
                                _log_file.flush()
                            if cycle_count == _log_max_rows:
                                if self.logger:
                                    self.logger.warn(f"Cycle log reached {_log_max_rows} rows, stopping log.")
                        except Exception as log_e:
                            # CSV 日志错误不影响电机控制
                            if cycle_count <= 3 and self.logger:
                                self.logger.warn(f"Cycle log error (suppressed): {log_e}")

                    # Log response every 50 cycles using WARN level
                    if cycle_count % 50 == 1:
                        if results:
                            for r in results:
                                mode = r.values.get(moteus.Register.MODE, '?')
                                pos = r.values.get(moteus.Register.POSITION, '?')
                                vel = r.values.get(moteus.Register.VELOCITY, '?')
                                fault = r.values.get(moteus.Register.FAULT, '?')
                                if self.logger:
                                    self.logger.warn(f"[DIAG] cycle#{cycle_count} sent {len(cmds)} cmds, response: mode={mode} pos={pos} vel={vel} fault={fault}")
                        else:
                            if self.logger:
                                self.logger.warn(f"[DIAG] cycle#{cycle_count} sent {len(cmds)} cmds, NO RESPONSE!")
                except asyncio.TimeoutError:
                    if self.logger:
                        self.logger.warn(f"[DIAG] cycle#{cycle_count} timeout waiting for Moteus response (>0.5s).")
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"Error in moteus transport cycle: {e}")
                    else:
                        print(f"Error in moteus transport cycle: {e}")
            else:
                empty_count += 1
                if empty_count == 200:
                    if self.logger:
                        self.logger.warn(f"[DIAG] No commands received for ~1s (cycle_count so far: {cycle_count})")
                    empty_count = 0

            # 绝对 deadline 定时：不依赖实际时刻，自动补偿 CAN 延迟抖动
            # 若本轮超时 > 10ms，下轮 sleep=0 立即补偿；若提前完成则补足剩余时间
            _now = time.monotonic()
            _sleep_t = max(0.0, _target_t - _now)
            await asyncio.sleep(_sleep_t)
            _target_t += _CYCLE_PERIOD  # 向前推进一个周期（不依赖实际发生时刻）

        # 关闭 CSV 文件
        if _log_file is not None:
            try:
                _log_file.flush()
                _log_file.close()
                if self.logger:
                    self.logger.info(f"Moteus cycle log closed: {_log_path} ({cycle_count} cycles)")
            except Exception:
                pass

    def enable_all(self):
        self.enabled = True
        
    def disable_all(self):
        self.enabled = False
        with self.commands_lock:
            self.pending_commands = [c.make_stop() for c in self.controllers.values()]
            
    def apply_joint_commands(self, commands):
        if not self.enabled:
            return

        moteus_cmds = []

        # MiT Mode Parameters
        CONFIG_KP = 900.0
        CONFIG_KD = 9.0
        kp_scale = self.kp_des / CONFIG_KP
        kd_scale = self.kd_des / CONFIG_KD

        # 动态重力补偿: 预扫描 J2/J3 指令角度(turns->rad), 计算 cos 模型重力矩
        # 物理模型: tau_J2 = k2*cos(j2), tau_J3 = k3*cos(j2+j3)
        # cos 模型合理因为: 水平时重力矩最大(cos=1), 竖直时为0(cos=0)
        grav_ff = {}
        if self.grav_k2 > 0.0:
            j2_rad = 0.0
            j3_rad = 0.0
            for mid, pos_turns, _, __, ___ in commands:
                if mid == 2:
                    j2_rad = pos_turns * 2 * math.pi
                elif mid == 3:
                    j3_rad = pos_turns * 2 * math.pi
            grav_ff[2] = self.grav_k2 * math.cos(j2_rad)
            if self.grav_k3 > 0.0:
                grav_ff[3] = self.grav_k3 * math.cos(j2_rad + j3_rad)

        for motor_id, pos_turns, vel_turns_s, torque_nm, accel in commands:
            if motor_id in self.controllers:
                c = self.controllers[motor_id]

                moveit_ff = torque_nm if (torque_nm is not None and not math.isnan(torque_nm)) else 0.0

                # 前馈力矩优先级: 动态重力补偿(已对J2/J3计算) > 固定 ff_torques
                if motor_id in grav_ff:
                    static_ff = grav_ff[motor_id]
                else:
                    static_ff = self.ff_torques.get(motor_id, 0.0)

                m_ff_torque = moveit_ff + static_ff

                raw_accel = accel if (accel is not None and not math.isnan(accel)) else self.accel_limit
                m_accel_limit = None if (raw_accel == 0.0) else max(0.0, raw_accel)

                moteus_cmds.append(c.make_position(
                    position=pos_turns,
                    velocity=vel_turns_s,
                    kp_scale=kp_scale,
                    kd_scale=kd_scale,
                    feedforward_torque=m_ff_torque,
                    maximum_torque=self.max_torque,
                    accel_limit=m_accel_limit,
                    query=True
                ))

        if moteus_cmds:
            with self.commands_lock:
                self.pending_commands = moteus_cmds

            now = time.monotonic()
            if self.logger and now - self._last_command_debug_time >= 1.0:
                command_parts = []
                for motor_id, pos_turns, vel_turns_s, torque_nm, accel in commands:
                    command_parts.append(
                        f"M{motor_id}: pos={pos_turns:.6f}turn/"
                        f"{pos_turns * 2 * math.pi:.4f}rad, "
                        f"vel={vel_turns_s:.6f}turn/s, "
                        f"ff={torque_nm if torque_nm is not None else 'None'}"
                    )
                self.logger.warn(
                    f"[DEBUG] queued {len(moteus_cmds)} moteus position commands -> "
                    + "; ".join(command_parts)
                )
                self._last_command_debug_time = now

    def shutdown(self):
        self._shutdown_event.set()
        # Thread will exit after finishing cycle naturally


class USB2CANNode(Node):
    def __init__(self, input_mode: int = INPUT_MODE_TRAP_TRAJ, vel_limit: float = DEFAULT_VEL_LIMIT,
                 accel_limit: float = DEFAULT_ACCEL_LIMIT,
                 kp_des: float = 900.0, kd_des: float = 15.0,
                 max_torque: float = DEFAULT_MAX_TORQUE,
                 ff_torques_json: str = '{}',
                 grav_k2: float = 0.0, grav_k3: float = 0.0):
        super().__init__('usb2can_node')

        # Declare parameters
        self.declare_parameter('interface', 'socketcan')
        self.declare_parameter('channel', 'can0')
        self.declare_parameter('motor_config_json', '{"1":30.0,"2":50.0,"3":50.0,"4":30.0,"5":30.0,"6":30.0}')
        self.declare_parameter('joint_names', ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6'])
        self.declare_parameter('vel_limit', vel_limit)
        self.declare_parameter('accel_limit', accel_limit)
        self.declare_parameter('kp_des', kp_des)
        self.declare_parameter('kd_des', kd_des)
        self.declare_parameter('max_torque', max_torque)
        self.declare_parameter('input_mode', input_mode)
        self.declare_parameter('ff_torques_json', ff_torques_json)
        # 动态重力补偿系数（随关节角变化，解决圆弧椭圆失真）
        self.declare_parameter('grav_k2', grav_k2)
        self.declare_parameter('grav_k3', grav_k3)
        self._last_joint_state_debug_time = 0.0
        self._joint_state_debug_count = 0
        self._last_controller_state_debug_time = 0.0
        self._controller_state_debug_count = 0

        self.srv_init = self.create_service(InitUsb2Can, 'init_usb2can', self.init_usb2can_callback)
        self.srv_write = self.create_service(WriteUsb2Can, 'write_usb2can', self.write_usb2can_callback)
        self.srv_read = self.create_service(ReadUsb2Can, 'read_usb2can', self.read_usb2can_callback)

        # Get parameter values
        interface = self.get_parameter('interface').value
        channel = self.get_parameter('channel').value
        motor_config_json = self.get_parameter('motor_config_json').value
        self.joint_names = self.get_parameter('joint_names').value
        self.vel_limit = self.get_parameter('vel_limit').value
        self.accel_limit = self.get_parameter('accel_limit').value
        self.kp_des = self.get_parameter('kp_des').value
        self.kd_des = self.get_parameter('kd_des').value
        self.max_torque = self.get_parameter('max_torque').value
        self.input_mode = self.get_parameter('input_mode').value
        grav_k2_val = self.get_parameter('grav_k2').value
        grav_k3_val = self.get_parameter('grav_k3').value

        # 解析前馈力矩参数
        ff_torques_json = self.get_parameter('ff_torques_json').value
        try:
            ff_torques_raw = json.loads(ff_torques_json)
            ff_torques = {int(k): float(v) for k, v in ff_torques_raw.items()}
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f"Invalid ff_torques_json: {e}, using zero feedforward")
            ff_torques = {}
        if ff_torques:
            self.get_logger().info(f"前馈力矩(固定): { {f'J{k}': v for k, v in ff_torques.items()} } Nm")
        if grav_k2_val > 0:
            self.get_logger().info(
                f"动态重力补偿已启用: k2={grav_k2_val:.3f} Nm, k3={grav_k3_val:.3f} Nm"
                f" [tau_J2=k2*cos(J2), tau_J3=k3*cos(J2+J3)]"
            )
        
        try:
            config_dict = json.loads(motor_config_json)
            self.motor_ids = [int(k) for k in config_dict.keys()]
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().error(f"Invalid motor_config_json format: {e}")
            self.motor_ids = [1, 2, 3, 4, 5, 6]
        
        input_mode_name = 'TRAP_TRAJ' if self.input_mode == INPUT_MODE_TRAP_TRAJ else 'PASSTHROUGH'
        self.get_logger().info(f"Initializing USB2CAN with interface: {interface}, channel: {channel}")
        self.get_logger().info(f"Motor IDs: {self.motor_ids}")
        self.get_logger().info(f"Input mode: {input_mode_name}")
        self.get_logger().info(f"Velocity limit: {self.vel_limit} turns/s, Accel limit: {self.accel_limit} turns/s^2")
        self.get_logger().info(f"Max torque: {self.max_torque} Nm")
        self.get_logger().info(f"MiT Config: Kp={self.kp_des}, Kd={self.kd_des}")
        
        try:
            # Initialize MoteusWorker using the asyncio thread
            self.moteus_worker = MoteusWorker(
                interface=interface,
                channel=channel,
                motor_ids=self.motor_ids,
                vel_limit=self.vel_limit,
                accel_limit=self.accel_limit,
                kp_des=self.kp_des,
                kd_des=self.kd_des,
                max_torque=self.max_torque,
                ff_torques=ff_torques,
                grav_k2=grav_k2_val,
                grav_k3=grav_k3_val,
                logger=self.get_logger()
            )
            self.moteus_worker.start()
            self.get_logger().info("Successfully initialized Moteus background worker")
            
            # Enable all motors by default
            self.moteus_worker.enable_all()
            self.get_logger().info(f"Enabled all motors with vel_limit: {self.vel_limit} turns/s")
            
            # ✅ 修改：队列深度改为 1（严格 KEEP_LAST=1）避免旧指令积压导致延迟抖动
            # 默认 depth=10 下，若 ros2_control 出现瞬时堆积，usc2can_node 会一次处理多个旧指令导致量化跳变
            self.joint_state_sub = self.create_subscription(
                JointState,
                'joint_states',
                self.joint_state_callback,
                1  # depth=1: 只保留最新的一帧指令
            )
            self.get_logger().warn(
                f"[DEBUG] Subscribed to joint_states, expected joints: {self.joint_names}"
            )
            if JointTrajectoryControllerState is not None:
                self.controller_state_sub = self.create_subscription(
                    JointTrajectoryControllerState,
                    '/dummyx2_arm_controller/controller_state',
                    self.controller_state_callback,
                    1
                )
                self.get_logger().warn(
                    "[DEBUG] Subscribed to /dummyx2_arm_controller/controller_state "
                    "for MoveIt/ros2_control target diagnostics"
                )
            else:
                self.get_logger().warn(
                    "[DEBUG] control_msgs is not available; controller_state debug disabled"
                )
            
        except Exception as e:
            self.get_logger().error(f"Failed to initialize USB2CAN MoteusWorker: {str(e)}")
            raise

    def controller_state_callback(self, msg):
        self._controller_state_debug_count += 1
        now = time.monotonic()
        if now - self._last_controller_state_debug_time < 1.0:
            return

        def fmt_point(label, point):
            parts = []
            for name in self.joint_names:
                if name in msg.joint_names:
                    idx = msg.joint_names.index(name)
                    pos = point.positions[idx] if len(point.positions) > idx else None
                    vel = point.velocities[idx] if len(point.velocities) > idx else None
                    parts.append(f"{name}: pos={pos}, vel={vel}")
                else:
                    parts.append(f"{name}: MISSING")
            return f"{label}: " + "; ".join(parts)

        self.get_logger().warn(
            f"[DEBUG] controller_state #{self._controller_state_debug_count}: "
            f"joint_names={list(msg.joint_names)} | "
            + fmt_point("reference", msg.reference)
            + " | "
            + fmt_point("output", msg.output)
            + " | "
            + fmt_point("feedback", msg.feedback)
        )
        self._last_controller_state_debug_time = now
    
    def joint_state_callback(self, msg: JointState):
        self._joint_state_debug_count += 1
        now = time.monotonic()
        should_debug = now - self._last_joint_state_debug_time >= 1.0
        if should_debug:
            received_parts = []
            for name in self.joint_names:
                if name in msg.name:
                    idx = msg.name.index(name)
                    pos = msg.position[idx] if len(msg.position) > idx else None
                    vel = msg.velocity[idx] if len(msg.velocity) > idx else None
                    received_parts.append(f"{name}: pos={pos}, vel={vel}")
                else:
                    received_parts.append(f"{name}: MISSING")
            self.get_logger().warn(
                f"[DEBUG] joint_states #{self._joint_state_debug_count}: "
                f"names={list(msg.name)}, positions_len={len(msg.position)}, "
                f"velocities_len={len(msg.velocity)}, efforts_len={len(msg.effort)} | "
                + "; ".join(received_parts)
            )
            self._last_joint_state_debug_time = now

        # Check if we have all expected joints
        if not all(name in msg.name for name in self.joint_names):
            self.get_logger().warn("Received joint states don't contain all expected joints")
            return
        
        commands = []
        for i, joint_name in enumerate(self.joint_names):
            try:
                joint_index = msg.name.index(joint_name)
                # Ensure we have data for this joint
                joint_position_rad = msg.position[joint_index] if len(msg.position) > joint_index else 0.0
                joint_velocity_rad = msg.velocity[joint_index] if len(msg.velocity) > joint_index else 0.0
                
                # Check if effort (torque) is provided in the message
                joint_effort_nm = msg.effort[joint_index] if len(msg.effort) > joint_index else None
                
                # Acceleration is generally not provided in standard sensor_msgs/JointState
                # If you use a custom message or trajectory message, you can map it here.
                joint_accel = None
                
                # Motor IDs are 1-6 corresponding to joints 1-6
                motor_id = i + 1
                if motor_id in self.motor_ids:
                    # Moteus firmware already handles reduction, just convert rad to turns
                    motor_turns = joint_position_rad / (2 * math.pi)
                    motor_vel_turns_s = joint_velocity_rad / (2 * math.pi)
                    
                    # You might also want to apply the (-1) direction flip to torque if relevant
                    if motor_id == 1:
                        cmd = (motor_id, motor_turns * -1, motor_vel_turns_s * -1, joint_effort_nm, joint_accel)
                    else:
                        # For MiT mode, effort is used as feedforward_torque which is a signed value, 
                        # so it should be flipped to match position/velocity direction
                        ff_effort_nm = -joint_effort_nm if joint_effort_nm is not None else None
                        cmd = (motor_id, motor_turns * 1, motor_vel_turns_s * 1, ff_effort_nm, joint_accel)
                        
                    commands.append(cmd)
                        
            except (ValueError, IndexError) as e:
                self.get_logger().error(f"Error processing joint state for {joint_name}: {str(e)}")

        if commands:
            if should_debug:
                command_parts = []
                for motor_id, pos_turns, vel_turns_s, torque_nm, accel in commands:
                    command_parts.append(
                        f"M{motor_id}: pos_cmd={pos_turns:.6f}turn/"
                        f"{pos_turns * 2 * math.pi:.4f}rad, "
                        f"vel_cmd={vel_turns_s:.6f}turn/s"
                    )
                self.get_logger().warn(
                    "[DEBUG] joint_states converted to moteus commands -> "
                    + "; ".join(command_parts)
                )
            self.moteus_worker.apply_joint_commands(commands)

    def init_usb2can_callback(self, request, response):
        self.get_logger().info(f"Init USB2CAN callback: action={request.action}")
        if hasattr(self, 'worker'):
            if request.action == 'stop':
                self.moteus_worker.disable_all()
            elif request.action == 'start':
                self.moteus_worker.enable_all()
        response.success = True
        return response

    def write_usb2can_callback(self, request, response):
        if not hasattr(self, 'worker'):
            response.success = False
            return response
        response.success = True  
        return response

    def read_usb2can_callback(self, request, response):
        response.pos_commands = [0.0] * 6
        response.vel_commands = [0.0] * 6
        response.success = True  
        return response

    def on_shutdown(self):
        self.get_logger().info("Shutting down USB2CAN controller (moteus)")
        if hasattr(self, 'worker'):
            self.moteus_worker.disable_all()
            self.moteus_worker.shutdown()

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='USB2CAN Node for Moteus motor control')
    parser.add_argument(
        '--input-mode', 
        type=str, 
        choices=['trap_traj', 'passthrough'],
        default='trap_traj',
        help='Input mode: trap_traj (default) or passthrough'
    )
    parser.add_argument(
        '--vel-limit',
        type=float,
        default=DEFAULT_VEL_LIMIT,
        help=f'Velocity limit in turns/s (default: {DEFAULT_VEL_LIMIT})'
    )
    parser.add_argument(
        '--accel-limit',
        type=float,
        default=DEFAULT_ACCEL_LIMIT,
        help=f'Acceleration limit in turns/s^2 (default: {DEFAULT_ACCEL_LIMIT})'
    )
    parser.add_argument(
        '--kp-des',
        type=float,
        default=900.0,
        help='Desired Kp in MiT mode (default 900 = rated KP, less vibration than 1200)'
    )
    parser.add_argument(
        '--kd-des',
        type=float,
        default=15.0,
        help='Desired Kd in MiT mode (default 15, higher = more damping)'
    )
    parser.add_argument(
        '--ff-torques',
        type=str,
        default='{}',
        help='按关节设置静态前馈力矩(Nm)，格式: JSON {"motor_id": Nm}。当 --grav-k2>0 时，J2/J3 被动态补偿替代'
    )
    parser.add_argument(
        '--grav-k2',
        type=float,
        default=0.0,
        help=(
            '动态重力补偿系数 k2 (Nm): tau_J2 = k2 * cos(J2)。'
            '理论初始值约 1.35。从 1.0 开始调，椭圆缩短方向增大，过高则反向。'
            '启用后替代 J2 的固定 ff_torques (default: 0=禁用)'
        )
    )
    parser.add_argument(
        '--grav-k3',
        type=float,
        default=0.0,
        help=(
            '动态重力补偿系数 k3 (Nm): tau_J3 = k3 * cos(J2+J3)。'
            '理论初始值约 0.30。仅当 --grav-k2 > 0 时生效 (default: 0=禁用)'
        )
    )
    parser.add_argument(
        '--max-torque',
        type=float,
        default=DEFAULT_MAX_TORQUE,
        help=f'缺省最大扭矩限制 (Nm)，同时用于启动保持和运行阶段 (default: {DEFAULT_MAX_TORQUE})'
    )
    
    # Parse known args to allow ROS2 args to pass through
    parsed_args, remaining_args = parser.parse_known_args()
    
    input_mode = INPUT_MODE_PASSTHROUGH if parsed_args.input_mode == 'passthrough' else INPUT_MODE_TRAP_TRAJ
    vel_limit = parsed_args.vel_limit
    accel_limit = parsed_args.accel_limit
    kp_des = parsed_args.kp_des
    kd_des = parsed_args.kd_des
    ff_torques_json = parsed_args.ff_torques
    max_torque = parsed_args.max_torque
    
    rclpy.init(args=remaining_args)
    node = None
    
    try:
        node = USB2CANNode(input_mode=input_mode, vel_limit=vel_limit, accel_limit=accel_limit,
                           kp_des=kp_des, kd_des=kd_des, max_torque=max_torque,
                           ff_torques_json=ff_torques_json)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if node is not None:
            node.get_logger().error(f"Error in USB2CAN moteus node: {str(e)}")
        else:
            print(f"Error creating USB2CAN moteus node: {str(e)}")
    finally:
        if node is not None:
            node.on_shutdown()
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
