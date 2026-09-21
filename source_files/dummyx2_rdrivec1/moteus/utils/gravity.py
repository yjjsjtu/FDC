#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
单关节重力补偿程序 (moteus C1 + 谐波减速器 + Pinocchio)

使用 Pinocchio 计算单关节连杆的重力矩，并通过 moteus 的扭矩前馈
进行实时补偿。

硬件配置:
  - 控制器: moteus C1
  - 减速器: 谐波减速器, 减速比 1:30
  - 连杆质量: 0.358 kg
  - 连杆惯量: ixx=0.000232, iyy=0.000174, izz=0.000267

依赖安装:
  pip install pin moteus numpy

使用方法:
  python3 gravity_compensation.py

  运行后电机将进入重力补偿模式，可自由用手拖动关节。
  按 Ctrl+C 退出，电机会自动停止。
"""

import asyncio
import math
import signal
import sys
import time

import numpy as np

try:
    import pinocchio as pin
except ImportError:
    print("请安装 pinocchio: pip install pin")
    sys.exit(1)

try:
    import moteus
except ImportError:
    print("请安装 moteus: pip install moteus")
    sys.exit(1)


# ============================================================
# 配置参数 (根据实际硬件修改)
# ============================================================

# moteus 控制器 CAN ID
MOTEUS_ID = 1

# 谐波减速器减速比 (电机:输出 = 30:1)
GEAR_RATIO = 30.0

# 控制循环频率 (Hz)
CONTROL_RATE_HZ = 50

# 最大补偿扭矩限制 (Nm, 输出端), 安全保护
MAX_TORQUE_NM = 3.0

# ---------- 连杆参数 ----------

# 连杆质量 (kg)
LINK_MASS = 0.35801694858484556

# 连杆惯量 (kg·m?)
LINK_IXX = 0.000232
LINK_IYY = 0.000174
LINK_IZZ = 0.000267
LINK_IXY = -4.2e-05
LINK_IYZ = -0.0
LINK_IXZ = 0.0

# 连杆质心相对于关节的位置 (m)
# *** 请根据实际连杆 CAD 修改 ***
# 来自 URDF link5_1 inertial origin
LINK_COM_X = 0.021577
LINK_COM_Y = 0.062406
LINK_COM_Z = 0.0

# 连杆长度 (m), 用于构建模型
LINK_LENGTH = 0.20

# 关节轴方向: Y 轴旋转 (水平关节, 受重力影响最大)
# 如果关节绕其他轴旋转, 请修改 JOINT_AXIS
# 来自 URDF Joint5 axis="-1.0 0.0 0.0"
JOINT_AXIS = np.array([1, 0, 0])  # X 轴 (符号在扭矩输出时处理)

# 关节零位时连杆方向:
#   "horizontal" - 零位时连杆水平
#   "vertical_down" - 零位时连杆竖直朝下
ZERO_POSITION = "horizontal"

# 补偿增益 (0.0~1.0), 1.0 = 完全补偿, 可适当降低避免过补偿
COMPENSATION_GAIN = 1.0

# ---------- 手动偏移 ----------
# 可在运行时通过键盘调整
manual_offset_torque = 0.0


# ============================================================
# Pinocchio 模型构建
# ============================================================

def build_model():
    """构建单关节 Pinocchio 模型"""
    model = pin.Model()

    # 关节到父坐标系的变换
    if ZERO_POSITION == "horizontal":
        # 零位时连杆沿 X 轴水平
        joint_placement = pin.SE3.Identity()
    elif ZERO_POSITION == "vertical_down":
        # 零位时连杆沿 -Z 竖直朝下
        joint_placement = pin.SE3.Identity()
    else:
        joint_placement = pin.SE3.Identity()

    # 添加旋转关节
    joint_id = model.addJoint(
        0,  # 父关节 (0 = universe/base)
        pin.JointModelRY() if np.allclose(JOINT_AXIS, [0, 1, 0]) else
        pin.JointModelRX() if np.allclose(JOINT_AXIS, [1, 0, 0]) else
        pin.JointModelRZ(),
        joint_placement,
        "joint_1"
    )

    # 连杆惯量矩阵
    inertia = pin.Inertia(
        LINK_MASS,
        np.array([LINK_COM_X, LINK_COM_Y, LINK_COM_Z]),
        np.array([
            [LINK_IXX, LINK_IXY, LINK_IXZ],
            [LINK_IXY, LINK_IYY, LINK_IYZ],
            [LINK_IXZ, LINK_IYZ, LINK_IZZ]
        ])
    )

    # 添加连杆
    model.appendBodyToJoint(joint_id, inertia, pin.SE3.Identity())

    # 设置重力方向 (Z 轴向上)
    model.gravity = pin.Motion(np.array([0, 0, -9.81, 0, 0, 0]))

    # 创建数据
    data = model.createData()

    return model, data


def compute_gravity_torque(model, data, joint_angle_rad):
    """
    计算给定关节角度下的重力矩

    Args:
        model: Pinocchio 模型
        data: Pinocchio 数据
        joint_angle_rad: 关节角度 (弧度, 输出端)

    Returns:
        gravity_torque: 重力矩 (Nm, 输出端)
    """
    q = np.array([joint_angle_rad])
    gravity_torque = pin.computeGeneralizedGravity(model, data, q)
    return gravity_torque[0]


# ============================================================
# moteus 控制
# ============================================================

async def main():
    global manual_offset_torque

    print("=" * 60)
    print("  单关节重力补偿 (moteus C1 + Pinocchio)")
    print("=" * 60)
    print(f"  CAN ID:     {MOTEUS_ID}")
    print(f"  减速比:     1:{GEAR_RATIO:.0f}")
    print(f"  连杆质量:   {LINK_MASS:.4f} kg")
    print(f"  质心位置:   ({LINK_COM_X}, {LINK_COM_Y}, {LINK_COM_Z}) m")
    print(f"  控制频率:   {CONTROL_RATE_HZ} Hz")
    print(f"  最大扭矩:   {MAX_TORQUE_NM:.1f} Nm")
    print(f"  补偿增益:   {COMPENSATION_GAIN:.2f}")
    print("=" * 60)

    # 构建 Pinocchio 模型
    model, data = build_model()
    print("[OK] Pinocchio 模型已构建")

    # 零位验证
    test_torque = compute_gravity_torque(model, data, 0.0)
    print(f"[验证] 零位重力矩: {test_torque:.4f} Nm")
    test_torque_90 = compute_gravity_torque(model, data, math.pi / 2)
    print(f"[验证] 90°重力矩:  {test_torque_90:.4f} Nm")

    # 创建 moteus 控制器 (等效于 --can-disable-brs)
    # 注意: 需要 socketcan 接口已配置 fd on:
    #   sudo ip link set can0 down
    #   sudo ip link set can0 type can bitrate 1000000 dbitrate 5000000 fd on
    #   sudo ip link set can0 up
    from moteus.pythoncan_device import PythonCanDevice
    device = PythonCanDevice(
        interface='socketcan',
        channel='can0',
        disable_brs=True,
    )
    transport = moteus.Transport([device])
    controller = moteus.Controller(id=MOTEUS_ID, transport=transport)
    print(f"[OK] moteus 控制器已连接 (ID={MOTEUS_ID})")

    # 优雅退出
    running = True

    def signal_handler(sig, frame):
        nonlocal running
        running = False
        print("\n[!] 收到停止信号, 正在退出...")

    signal.signal(signal.SIGINT, signal_handler)

    try:
        # 先停止电机并清除故障
        await controller.set_stop()
        await asyncio.sleep(0.1)

        print("\n[运行中] 重力补偿已启动, 按 Ctrl+C 停止\n")

        cycle_time = 1.0 / CONTROL_RATE_HZ
        iteration = 0

        while running:
            loop_start = time.monotonic()

            # 读取当前状态
            state = await controller.query()

            if state is None:
                print("[警告] 无法读取状态, 跳过...")
                await asyncio.sleep(cycle_time)
                continue

            # moteus position 单位: 输出端转数 (revolutions)
            position_rev = state.values.get(moteus.Register.POSITION, 0.0)
            velocity_rev_s = state.values.get(moteus.Register.VELOCITY, 0.0)
            mode = state.values.get(moteus.Register.MODE, 0)

            # 转换为弧度
            # rotor_to_output_ratio=1 时, position 是电机转数
            # 输出角度 = 电机转数 / 减速比 * 2π
            joint_angle_rad = (position_rev / GEAR_RATIO) * 2.0 * math.pi

            # 计算重力补偿扭矩 (输出端 Nm)
            gravity_torque = compute_gravity_torque(model, data, joint_angle_rad)

            # 总补偿扭矩 = 重力补偿 + 手动偏移
            compensation_torque = (
                COMPENSATION_GAIN * gravity_torque + manual_offset_torque
            )

            # 限幅保护
            compensation_torque = max(-MAX_TORQUE_NM,
                                     min(MAX_TORQUE_NM, compensation_torque))

            # moteus 扭矩命令单位:
            #   如果设置了 rotor_to_output_ratio = 1/30,
            #   则扭矩命令是输出端 Nm (moteus 内部会换算)
            #
            #   如果 rotor_to_output_ratio = 1 (默认),
            #   则需要手动除以减速比:
            #     motor_torque = compensation_torque / GEAR_RATIO

            # *** 根据你的 moteus 配置选择 ***
            # 方式1: rotor_to_output_ratio 已设置为 1/30
            # feedforward_torque = compensation_torque

            # 方式2: rotor_to_output_ratio = 1 (默认, 取消下行注释)
            feedforward_torque = compensation_torque / GEAR_RATIO

            # 发送位置模式命令:
            #   position=NaN (不控位置)
            #   velocity=0 (目标速度为0)
            #   feedforward_torque 提供重力补偿
            #   kp_scale=0 (关闭位置环, 允许自由拖动)
            #   kd_scale=0.05 (微弱阻尼, 防止振荡)
            await controller.set_position(
                position=math.nan,
                velocity=0.0,
                feedforward_torque=feedforward_torque,
                kp_scale=0.0,
                kd_scale=0.05,
                maximum_torque=MAX_TORQUE_NM,
                watchdog_timeout=0.5,
                query=True,
            )

            # 打印状态
            iteration += 1
            if iteration % CONTROL_RATE_HZ == 0:
                print(
                    f"  角度: {math.degrees(joint_angle_rad):7.2f}°  "
                    f"重力矩: {gravity_torque:7.4f} Nm  "
                    f"补偿: {feedforward_torque:7.4f} Nm  "
                    f"速度: {velocity_rev_s:6.3f} r/s  "
                    f"模式: {mode}"
                )

            # 控制循环定时
            elapsed = time.monotonic() - loop_start
            sleep_time = cycle_time - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    except Exception as e:
        print(f"\n[错误] {e}")
    finally:
        # 停止电机
        print("[停止] 正在停止电机...")
        try:
            await controller.set_stop()
        except Exception:
            pass
        print("[完成] 电机已停止")


if __name__ == "__main__":
    asyncio.run(main())

