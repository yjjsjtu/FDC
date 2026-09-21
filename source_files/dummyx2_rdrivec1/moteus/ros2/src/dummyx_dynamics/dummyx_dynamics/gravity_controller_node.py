#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# Copyright (c) 2025. DummyX2 Team. All rights reserved.

"""
dummyx_dynamics · gravity_controller_node
==========================================
Pinocchio 动力学驱动的重力补偿控制节点。

功能：
  - 独立运行（无需 MoveIt2 / usb2can_node / ros2_control 参与）
  - 启动后自动读取 Moteus 当前关节位置，就地悬浮
  - 使用 Pinocchio 实时计算重力补偿力矩
  - 若 /joint_states 发布（MoveIt2 轨迹跟踪），叠加到轨迹指令上

运行方式（代替 usb2can_node）：
  source install/setup.bash
  ros2 run dummyx_dynamics gravity_controller \\
      --kp-des 2600.0 --kd-des 28.0 --accel-limit 0.08 \\
      --mode gravity

注意：本节点与 usb2can_node 互斥，二者共享同一 CAN 总线。
"""

import sys
import os
import time
import math
import asyncio
import argparse
import threading
import traceback
import glob as _glob

import numpy as np

# ─────────────────────────────────────────────────────────────
# Pinocchio 动态查找（不依赖绝对路径，优先系统路径）
# ─────────────────────────────────────────────────────────────
def _find_and_import_pinocchio():
    """Try to import pinocchio; fall back to searching conda envs."""
    try:
        import pinocchio as pin
        return pin
    except ImportError:
        pass

    home = os.path.expanduser('~')
    patterns = [
        os.path.join(home, 'miniconda3', 'envs', '*',
                     'lib', 'python3.*', 'site-packages',
                     'cmeel.prefix', 'lib', 'python3.*', 'site-packages'),
        os.path.join(home, 'anaconda3', 'envs', '*',
                     'lib', 'python3.*', 'site-packages',
                     'cmeel.prefix', 'lib', 'python3.*', 'site-packages'),
    ]
    for pat in patterns:
        for path in sorted(_glob.glob(pat)):
            if path not in sys.path:
                sys.path.insert(0, path)
            try:
                import pinocchio as pin
                return pin
            except ImportError:
                sys.path.remove(path)

    raise ImportError(
        "Cannot find pinocchio. Install via:\n"
        "  conda install pinocchio -c conda-forge"
    )


pin = _find_and_import_pinocchio()

# ─────────────────────────────────────────────────────────────
# moteus & ROS2
# ─────────────────────────────────────────────────────────────
try:
    import moteus
except ImportError:
    print("[FATAL] Cannot import moteus. Install with: pip install moteus")
    sys.exit(1)

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

# ─────────────────────────────────────────────────────────────
# Constants
# ─────────────────────────────────────────────────────────────
CONFIG_KP_BASE = 900.0
CONFIG_KD_BASE = 9.0

# All joints: 30:1 harmonic reducer + Moteus dual encoder
# Dual encoder measures OUTPUT shaft directly → position & feedforward are both
# at joint output shaft level (Nm). No manual reduction arithmetic needed.
#
# Motor direction sign for Moteus position command (matches usb2can_node convention):
#   Motor1: pos_turns = -joint_rad / 2π   (negated to match physical wiring)
#   Motor2-6: pos_turns = +joint_rad / 2π
MOTOR_DIR     = {1: -1, 2: 1, 3: 1, 4: 1, 5: 1, 6: 1}
MOTOR_REDUCTN = {i: 30 for i in range(1, 7)}  # all 30:1, informational only

ROS_JOINT_NAMES = ['Joint1', 'Joint2', 'Joint3', 'Joint4', 'Joint5', 'Joint6']


# ─────────────────────────────────────────────────────────────
# Pinocchio dynamics wrapper
# ─────────────────────────────────────────────────────────────
class DummyX2Dynamics:
    """Thin wrapper around Pinocchio for DummyX2 dynamics."""

    def __init__(self, urdf_path: str):
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data  = self.model.createData()
        self.nq    = self.model.nq
        self.nv    = self.model.nv

        pin_names = [self.model.names[i] for i in range(1, self.model.njoints)]
        self._pin_to_ros = []
        for pname in pin_names:
            self._pin_to_ros.append(
                ROS_JOINT_NAMES.index(pname) if pname in ROS_JOINT_NAMES else None)

        print(f"[Dynamics] Model: {self.model.name}, nq={self.nq}")
        print(f"[Dynamics] Pinocchio joints: {pin_names}")

    def _make_q(self, ros_q: list) -> np.ndarray:
        q = np.zeros(self.nq)
        for pi, ri in enumerate(self._pin_to_ros):
            if ri is not None and ri < len(ros_q):
                q[pi] = ros_q[ri]
        return q

    def _make_v(self, ros_v: list) -> np.ndarray:
        v = np.zeros(self.nv)
        for pi, ri in enumerate(self._pin_to_ros):
            if ri is not None and ri < len(ros_v):
                v[pi] = ros_v[ri]
        return v

    def gravity(self, ros_q: list) -> list:
        """Return gravity torque [J1..J6] Nm at joint output shaft."""
        tau = pin.computeGeneralizedGravity(self.model, self.data, self._make_q(ros_q))
        out = [0.0] * 6
        for pi, ri in enumerate(self._pin_to_ros):
            if ri is not None and pi < len(tau):
                out[ri] = float(tau[pi])
        return out

    def full_dynamics(self, ros_q, ros_v, ros_a) -> list:
        """Return full inverse dynamics torque [J1..J6] Nm."""
        a = np.zeros(self.nv)
        for pi, ri in enumerate(self._pin_to_ros):
            if ri is not None and ri < len(ros_a):
                a[pi] = ros_a[ri]
        tau = pin.rnea(self.model, self.data,
                       self._make_q(ros_q), self._make_v(ros_v), a)
        out = [0.0] * 6
        for pi, ri in enumerate(self._pin_to_ros):
            if ri is not None and pi < len(tau):
                out[ri] = float(tau[pi])
        return out


# ─────────────────────────────────────────────────────────────
# Moteus async worker
# ─────────────────────────────────────────────────────────────
class DynamicsMoteus:
    """
    Moteus CAN worker.

    Key feature vs usb2can_node:
      - Reads initial motor positions at startup (for autonomous hold)
      - Exposes feedback_q_rad [6] from latest Moteus response
    """

    def __init__(self, motor_ids: list, vel_limit: float, accel_limit: float,
                 kp_des: float, kd_des: float, logger=None):
        self.motor_ids   = motor_ids
        self.vel_limit   = vel_limit
        self.accel_limit = accel_limit
        self.kp_des      = kp_des
        self.kd_des      = kd_des
        self.logger      = logger

        self.transport    = None
        self.controllers  = {}
        self.enabled      = False

        self.commands_lock    = threading.Lock()
        self.pending_commands = []

        self._shutdown_event    = threading.Event()
        # Feedback: motor_id → position in turns (from latest Moteus response)
        self._feedback_lock     = threading.Lock()
        self._feedback_turns    = {}    # {motor_id: turns}
        self._feedback_vel      = {}    # {motor_id: turns/s}
        self._feedback_ready    = threading.Event()

        self.loop   = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self._run_loop, daemon=True)

    def start(self):
        self.thread.start()

    def _run_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_until_complete(self._async_main())

    def enable_all(self):
        self.enabled = True

    def disable_all(self):
        self.enabled = False
        with self.commands_lock:
            self.pending_commands = [c.make_stop() for c in self.controllers.values()]

    def get_feedback_q_rad(self) -> list:
        """Return current joint positions [J1..J6] in rad (from Moteus feedback).
        
        Applies a jump filter: if a joint position changes by more than MAX_JUMP_RAD
        in a single call (typically ~10ms), the previous value is kept.
        This rejects CAN noise spikes that could destabilize the control loop.
        """
        MAX_JUMP_RAD = math.radians(90.0)  # 90° max change per 10ms - only blocks obvious CAN glitches
        q = [0.0] * 6
        with self._feedback_lock:
            for i, motor_id in enumerate(sorted(self.motor_ids)):
                turns = self._feedback_turns.get(motor_id, 0.0)
                d = MOTOR_DIR.get(motor_id, 1)
                new_q = turns * 2.0 * math.pi * d
                old_q = self._last_valid_q[i] if hasattr(self, '_last_valid_q') else new_q
                if abs(new_q - old_q) > MAX_JUMP_RAD:
                    q[i] = old_q  # reject spike, keep last valid
                else:
                    q[i] = new_q
        if not hasattr(self, '_last_valid_q'):
            self._last_valid_q = q[:]
        else:
            self._last_valid_q = q[:]
        return q

    def apply_joint_commands(self, commands):
        """
        commands: list of
          (motor_id, pos_turns, vel_turns_s, ff_torque_nm, accel_limit_val)
        """
        if not self.enabled:
            return

        kp_scale = self.kp_des / CONFIG_KP_BASE
        kd_scale = self.kd_des / CONFIG_KD_BASE

        moteus_cmds = []
        for motor_id, pos_turns, vel_turns_s, ff_torque_nm, accel in commands:
            if motor_id in self.controllers:
                c  = self.controllers[motor_id]
                ff = ff_torque_nm if (ff_torque_nm is not None
                                      and not math.isnan(ff_torque_nm)) else 0.0
                raw_a = accel if (accel is not None
                                  and not math.isnan(accel)) else self.accel_limit
                m_accel = None  # Disable trajectory profiler in lead-through

                #   Pure torque (kp=0) is UNSTABLE: gravity FF positive feedback gain
                # (~0.175 Nm/rad rotor) exceeds max Kd damping (~0.072 Nm/(rad/s)).
                # Min stable kp_scale = 0.024. Use 0.05 for good stability margin.
                # With position tracking (hold_q = actual_q), spring error ≈ 0,
                # so arm stays where released (no spring return to initial).
                if kp_scale == 0.0:
                    kp_scale_actual = 0.05   # 5% of 900 = comfortable stability margin
                    kd_scale_actual = 0.08   # 8% of KD_base = very light drag feel
                else:
                    kp_scale_actual = kp_scale
                    kd_scale_actual = max(kd_scale, 0.10)

                moteus_cmds.append(c.make_position(
                    position=pos_turns,  # Always provide tracked position
                    velocity=0.0,
                    kp_scale=kp_scale_actual,
                    kd_scale=kd_scale_actual,
                    feedforward_torque=ff,
                    accel_limit=None,
                    query=True,
                ))

        if moteus_cmds:
            with self.commands_lock:
                self.pending_commands = moteus_cmds

    async def _async_main(self):
        try:
            self.transport = moteus.get_singleton_transport()
            if self.logger:
                self.logger.info("[Dynamics] Moteus transport initialized")
        except Exception as e:
            if self.logger:
                self.logger.error(
                    f"[Dynamics] Transport init failed: {e}\n"
                    "  → Make sure usb2can_node is NOT running.\n"
                    "  → Check USB-CAN adapter: ls /dev/ttyUSB*")
            self._shutdown_event.set()
            return

        qr = moteus.QueryResolution()
        qr.q_current = moteus.INT16

        for nid in self.motor_ids:
            self.controllers[nid] = moteus.Controller(
                id=nid, transport=self.transport, query_resolution=qr)

        # Clear faults
        try:
            await self.transport.cycle(
                [c.make_stop() for c in self.controllers.values()])
        except Exception:
            pass
        await asyncio.sleep(0.1)

        # ── Read initial motor positions ──────────────────────
        motor_id_order = sorted(self.motor_ids)
        try:
            query_cmds = [self.controllers[nid].make_query()
                          for nid in motor_id_order]
            results = await asyncio.wait_for(
                self.transport.cycle(query_cmds), timeout=1.0)
            if results:
                with self._feedback_lock:
                    for r in results:
                        if r is not None:
                            nid = getattr(r, 'id', getattr(r, 'source', None))
                            pos = r.values.get(moteus.Register.POSITION)
                            if nid is not None and pos is not None:
                                self._feedback_turns[nid] = float(pos)
                if self.logger:
                    turns_str = {nid: round(self._feedback_turns.get(nid, 0), 4)
                                 for nid in motor_id_order}
                    self.logger.info(
                        f"[Dynamics] Initial motor positions (turns): {turns_str}")
            self._feedback_ready.set()
        except Exception as e:
            if self.logger:
                self.logger.warn(
                    f"[Dynamics] Could not read initial positions: {e} – using zeros")
            with self._feedback_lock:
                for nid in self.motor_ids:
                    self._feedback_turns.setdefault(nid, 0.0)
            self._feedback_ready.set()

        _CYCLE_PERIOD = 0.010  # 100 Hz
        _target_t     = time.monotonic() + _CYCLE_PERIOD
        cycle_count   = 0
        empty_count   = 0

        while not self._shutdown_event.is_set():
            with self.commands_lock:
                cmds = list(self.pending_commands)

            if cmds:
                empty_count = 0
                try:
                    results = await asyncio.wait_for(
                        self.transport.cycle(cmds), timeout=0.5)
                    cycle_count += 1

                    # Update feedback positions from response
                    if results:
                        with self._feedback_lock:
                            for r in results:
                                if r is not None:
                                    nid = getattr(r, 'id', getattr(r, 'source', None))
                                    if nid is not None:
                                        pos = r.values.get(moteus.Register.POSITION)
                                        vel = r.values.get(moteus.Register.VELOCITY)
                                        if pos is not None:
                                            self._feedback_turns[nid] = float(pos)
                                        if vel is not None:
                                            self._feedback_vel[nid] = float(vel)

                    if cycle_count % 500 == 1 and self.logger:
                        self.logger.info(
                            f"[Dynamics] CAN cycle #{cycle_count} OK "
                            f"({len(cmds)} cmds)")

                except asyncio.TimeoutError:
                    if self.logger:
                        self.logger.warn("[Dynamics] CAN timeout >0.5s")
                except asyncio.CancelledError:
                    raise
                except Exception as e:
                    err = str(e)
                    if 'multiple access' in err or 'disconnected' in err:
                        if self.logger:
                            self.logger.error(
                                "[Dynamics] CAN port conflict – "
                                "stop usb2can_node first, then restart.")
                        self._shutdown_event.set()
                        return
                    if self.logger:
                        self.logger.error(f"[Dynamics] CAN error: {e}")
            else:
                empty_count += 1
                if empty_count >= 200 and self.logger:
                    self.logger.warn(
                        "[Dynamics] No commands generated yet – "
                        "check initial position read ...")
                    empty_count = 0

            _now   = time.monotonic()
            _sleep = max(0.0, _target_t - _now)
            await asyncio.sleep(_sleep)
            _target_t += _CYCLE_PERIOD

    def shutdown(self):
        self._shutdown_event.set()


# ─────────────────────────────────────────────────────────────
# Main ROS2 node
# ─────────────────────────────────────────────────────────────
class GravityControllerNode(Node):
    """
    Pinocchio gravity / full-dynamics controller.

    Autonomous mode (no MoveIt2 required):
      1. Reads motor positions at startup.
      2. Runs 100 Hz timer → computes gravity → holds position.

    Trajectory mode (MoveIt2 running):
      /joint_states updates the target positions real-time.

    Published topics:
      /gravity_torque  Float64MultiArray [J1..J6] Nm — for debugging.
    """

    def __init__(self, args_ns):
        super().__init__('gravity_controller_node')

        # ── ROS2 parameters: only grav_scale & filter (YAML writable) ──
        # grav_scale[i]: per-joint scale applied to Pinocchio gravity output.
        #   0.0 = disabled   1.0 = full Pinocchio torque   -1.0 = flip sign
        # J1: vertical axis  → gravity torque ≈0          → 0.0
        # J2: shoulder       → main gravity joint         → 1.0
        # J3: elbow          → significant gravity        → 1.0
        # J4: forearm rotate → link4(0.41kg)+link5(0.36kg)→ 1.0
        # J5: wrist bend     → link5(0.36kg) contribution → 1.0
        # J6: wrist rotate   → link6(0.003kg), negligible → 0.0
        _dbl  = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE)
        _darr = ParameterDescriptor(type=ParameterType.PARAMETER_DOUBLE_ARRAY)
        self.declare_parameter('grav_scale',
                               [0.0, 1.0, 1.0, 1.0, 1.0, 0.0], _darr)
        self.declare_parameter('ff_sign_override',
                               [1.0, 1.0, 1.0, 1.0, 1.0, 1.0], _darr)
        self.declare_parameter('accel_filter_hz', 5.0, _dbl)

        # Values from argparse (CLI / launch arguments)
        # 30:1 dual encoder Moteus PD tuning guide:
        #   kp_des: stiffness in Nm/rev at OUTPUT shaft
        #     - With good gravity comp, 200-400 is usually sufficient
        #     - Too high (>900): vibration on harmonic drive resonance
        #     - Too low (<50): arm drifts under disturbances
        #   kd_des: damping in Nm/(rev/s) at OUTPUT shaft
        #     - 4-12 is typical; match to kp for ~0.7 damping ratio
        self.comp_mode   = args_ns.mode
        self.kp_des      = float(args_ns.kp_des)
        self.kd_des      = float(args_ns.kd_des)
        self.vel_limit   = float(args_ns.vel_limit)
        self.accel_limit = float(args_ns.accel_limit)

        # Values from ROS2 params (can be overridden by YAML)
        self.grav_scale       = list(self.get_parameter('grav_scale').value)
        self.ff_sign_override = list(self.get_parameter('ff_sign_override').value)
        accel_filter_hz       = float(self.get_parameter('accel_filter_hz').value)

        self.get_logger().info(
            f"[Dynamics] mode={self.comp_mode}  kp={self.kp_des}  "
            f"kd={self.kd_des}  accel_limit={self.accel_limit}  "
            f"grav_scale={self.grav_scale}  ff_sign={self.ff_sign_override}")


        # ── URDF ─────────────────────────────────────────────
        urdf_path = self._find_urdf()
        self.get_logger().info(f"[Dynamics] URDF: {urdf_path}")

        # ── Pinocchio ─────────────────────────────────────────
        self.dyn = DummyX2Dynamics(urdf_path)

        # Acceleration estimation (full_dynamics mode)
        self._prev_v  = [0.0] * 6
        self._filt_a  = [0.0] * 6
        self._dt      = 0.010
        rc            = 1.0 / (2.0 * math.pi * accel_filter_hz)
        self._alpha_a = self._dt / (rc + self._dt)

        # Target joint positions in rad [J1..J6] — initialized from Moteus feedback
        self._hold_q = [0.0] * 6
        self._traj_active = False   # True when /joint_states arrives
        self._loop_counter = 0      # Counts _ctrl_loop calls for diagnostic logging

        # ── Moteus worker ─────────────────────────────────────
        motor_ids = list(range(1, 7))
        self._moteus = DynamicsMoteus(
            motor_ids=motor_ids,
            vel_limit=self.vel_limit,
            accel_limit=self.accel_limit,
            kp_des=self.kp_des,
            kd_des=self.kd_des,
            logger=self.get_logger(),
        )
        self._moteus.start()
        self.get_logger().info("[Dynamics] Waiting for initial motor positions…")

        # Wait (up to 3s) for initial position read
        if self._moteus._feedback_ready.wait(timeout=3.0):
            self._hold_q = self._moteus.get_feedback_q_rad()
            self.get_logger().info(
                f"[Dynamics] Initial joint pos (deg): "
                f"{[round(math.degrees(q),1) for q in self._hold_q]}")
        else:
            self.get_logger().warn(
                "[Dynamics] Timeout reading initial positions — using zeros")

        self._moteus.enable_all()
        self.get_logger().info("[Dynamics] Motors enabled, starting control loop")

        # ── 100 Hz autonomous control timer ──────────────────
        self._ctrl_timer = self.create_timer(self._dt, self._ctrl_loop)

        # ── Optional /joint_states subscription ──────────────
        self._js_sub = self.create_subscription(
            JointState, 'joint_states', self._on_joint_state, 1)

        # ── Gravity torque publisher (debug) ──────────────────
        self._grav_pub = self.create_publisher(
            Float64MultiArray, 'gravity_torque', 1)

        self.get_logger().info(
            "[Dynamics] Ready — gravity compensation active.\n"
            "  -> To freely drag the arm (Zero Impedance / Lead-Through), start with:\n"
            "     ros2 run dummyx_dynamics gravity_controller --kp-des 0 --kd-des 0\n"
            "  -> Connect /joint_states for trajectory tracking.")

    # ── URDF finder ───────────────────────────────────────────
    def _find_urdf(self) -> str:
        try:
            from ament_index_python.packages import get_package_share_directory
            p = os.path.join(get_package_share_directory('dummyx_dynamics'),
                             'urdf', 'dummyx2.urdf')
            if os.path.isfile(p):
                return p
        except Exception:
            pass
        src = os.path.dirname(os.path.abspath(__file__))
        for rel in ['../urdf/dummyx2.urdf', '../../urdf/dummyx2.urdf']:
            p = os.path.normpath(os.path.join(src, rel))
            if os.path.isfile(p):
                return p
        raise FileNotFoundError(
            "dummyx2.urdf not found – run: "
            "colcon build --packages-select dummyx_dynamics")

    # ── 100 Hz control loop ───────────────────────────────────
    def _ctrl_loop(self):
        """Autonomous 100 Hz gravity compensation loop."""
        # 1. Get actual measured positions
        actual_q = self._moteus.get_feedback_q_rad()

        # 2. Compute gravity torque using ACTUAL positions (vital for dragging)
        if self.comp_mode == 'gravity':
            tau_ros = self.dyn.gravity(actual_q)
        elif self.comp_mode == 'full_dynamics':
            v = [0.0] * 6  # zero velocity (hold mode)
            tau_ros = self.dyn.full_dynamics(actual_q, v, self._filt_a)
        else:
            tau_ros = [0.0] * 6

        # Apply per-joint scale
        tau_ros = [tau_ros[i] * self.grav_scale[i] for i in range(6)]

        # Publish for debug
        msg = Float64MultiArray()
        msg.data = tau_ros
        self._grav_pub.publish(msg)

        # Build Moteus commands
        cmds = []
        for i, jname in enumerate(ROS_JOINT_NAMES):
            mid        = i + 1
            d          = MOTOR_DIR.get(mid, 1)

            # ── Lead-Through Position Tracking ─────────────────────────────
            # CRITICAL: When kp_des=0 (lead-through mode), ALWAYS update hold position
            # to actual position, regardless of _traj_active flag.
            # The _traj_active flag from stale /joint_states can cause return-to-start bug.
            if self.kp_des == 0.0:
                self._hold_q[i] = actual_q[i]  # Always track actual in lead-through
            elif not self._traj_active:
                pass  # Hold the last commanded position

            target_q   = self._hold_q[i]
            pos_turns  = target_q / (2.0 * math.pi) * d
            vel_turns  = 0.0

            torque_d = -d
            # Apply per-joint sign override (from config "ff_sign_override").
            # Default: 1.0 (no change). Set to -1.0 to flip a joint's gravity FF.
            ff_sign = self.ff_sign_override[i] if hasattr(self, 'ff_sign_override') else 1.0
            ff = (tau_ros[i] / MOTOR_REDUCTN[mid]) * torque_d * ff_sign

            cmds.append((mid, pos_turns, vel_turns, ff, self.accel_limit))

        # ── Diagnostic log (every 0.5 seconds) ──────────
        if self._loop_counter % 50 == 0:
            ff_str = ", ".join([f"J{c[0]}:{c[3]:.3f}" for c in cmds])
            q_str = ", ".join([f"{math.degrees(a):.1f}°" for a in actual_q])
            self.get_logger().info(f"[Dynamics] FF Torques: {ff_str} | Angles: {q_str}")

        # ── Gravity direction debug (every 5 seconds) ──────────
        if self._loop_counter % 500 == 0:
            vel_lines = []
            for i, (mid, pos_t, vel_t, ff_cmd, _) in enumerate(cmds):
                d = MOTOR_DIR.get(mid, 1)
                tau_pin = tau_ros[i]
                ff_sign = self.ff_sign_override[i] if hasattr(self, 'ff_sign_override') else 1.0
                torque_d = -d
                # Motor velocity (turns/s from Moteus)
                with self._moteus._feedback_lock:
                    mot_vel = self._moteus._feedback_vel.get(mid, 0.0)
                vel_degs = math.degrees(mot_vel * 2.0 * math.pi * d)
                drift = 'STILL' if abs(vel_degs) < 1.0 else (
                    f'DRIFT+{vel_degs:+.1f}°/s' if vel_degs > 0 else f'DRIFT{vel_degs:+.1f}°/s')
                # Direction consistency check:
                # tau_pin > 0 means Pinocchio says: apply + torque to support arm.
                # ff_cmd = tau_pin / 30 * (-d) * ff_sign
                # Expected: ff and tau_pin should have OPPOSITE signs if torque_d * ff_sign = -1
                # (which it is by default when d=1). So ff_cmd < 0 when tau_pin > 0.
                # If the joint is drifting in the SAME direction as ff_cmd:
                #   FF is pushing the joint away rather than holding it → WRONG direction.
                ff_beneficial = (ff_cmd * vel_degs) <= 0  # FF opposes drift = good
                if abs(vel_degs) < 1.0:
                    status = '✅ STABLE'
                elif ff_beneficial:
                    status = '✅ FF opposes drift (CORRECT direction)'
                else:
                    status = '❌ FF aids drift  (WRONG direction — flip ff_sign_override[%d])' % i
                vel_lines.append(
                    f"  J{mid}: τ_pin={tau_pin:+.3f}Nm | d={d:+d} torque_d={torque_d:+d} "
                    f"ff_sign={ff_sign:+.1f} | ff_rotor={ff_cmd:+.4f}Nm | vel={drift} | {status}")
            self.get_logger().warn(
                "[GravDebug] Gravity FF direction diagnostics:\n" + "\n".join(vel_lines) +
                "\n  [Tip] To flip joint N's FF: set ff_sign_override[N-1]=-1.0 in dynamics_config.yaml")

        self._loop_counter += 1
        self._moteus.apply_joint_commands(cmds)

    # ── /joint_states callback ────────────────────────────────
    def _on_joint_state(self, msg: JointState):
        """Update target positions from MoveIt2 / ros2_control trajectory."""
        updated = False
        vel = [0.0] * 6
        for i, jname in enumerate(ROS_JOINT_NAMES):
            if jname in msg.name:
                idx = msg.name.index(jname)
                if idx < len(msg.position):
                    self._hold_q[i] = msg.position[idx]
                    updated = True
                if idx < len(msg.velocity):
                    vel[i] = msg.velocity[idx]
        if updated:
            if not self._traj_active:
                self.get_logger().info(
                    "[Dynamics] /joint_states received — trajectory tracking active")
                self._traj_active = True

            # Update velocity history for full_dynamics acceleration estimate
            if self.comp_mode == 'full_dynamics':
                for i in range(6):
                    a_raw = (vel[i] - self._prev_v[i]) / self._dt
                    self._filt_a[i] = (self._alpha_a * a_raw
                                       + (1.0 - self._alpha_a) * self._filt_a[i])
                self._prev_v = vel[:]

    # ── Shutdown ──────────────────────────────────────────────
    def on_shutdown(self):
        self.get_logger().info("[Dynamics] Shutting down...")
        if hasattr(self, '_ctrl_timer'):
            self._ctrl_timer.cancel()
        self._moteus.disable_all()
        time.sleep(0.05)
        self._moteus.shutdown()
        self.get_logger().info("[Dynamics] Shutdown complete")


# ─────────────────────────────────────────────────────────────
# CLI entry point
# ─────────────────────────────────────────────────────────────
def _parse_args():
    parser = argparse.ArgumentParser(
        description='DummyX2 Pinocchio gravity / dynamics controller',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  ros2 run dummyx_dynamics gravity_controller
  ros2 run dummyx_dynamics gravity_controller --kp-des 2600.0 --kd-des 28.0
  ros2 run dummyx_dynamics gravity_controller --mode full_dynamics
  ros2 run dummyx_dynamics gravity_controller --mode none
        """
    )
    parser.add_argument('--mode', choices=['gravity', 'full_dynamics', 'none'],
                        default='gravity')
    # 30:1 dual encoder, gravity comp mode defaults:
    # kp=400 → moderate stiffness; gravity feedforward handles static load
    # kd=6   → damping ratio ~0.7 for typical arm inertia
    parser.add_argument('--kp-des',      type=float, default=400.0)
    parser.add_argument('--kd-des',      type=float, default=6.0)
    parser.add_argument('--vel-limit',   type=float, default=0.4)
    parser.add_argument('--accel-limit', type=float, default=0.08)
    return parser.parse_known_args()


def main(args=None):
    parsed, remaining = _parse_args()
    rclpy.init(args=remaining)
    node = None
    try:
        node = GravityControllerNode(parsed)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        msg = f"Fatal: {e}\n{traceback.format_exc()}"
        if node:
            try:
                node.get_logger().error(msg)
            except Exception:
                print(msg)
        else:
            print(msg)
    finally:
        if node:
            node.on_shutdown()
            try:
                node.destroy_node()
            except Exception:
                pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
