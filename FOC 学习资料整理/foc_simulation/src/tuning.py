"""根据电机参数估算控制环 PI 初值。

这些公式不是最终调参结果，而是一个比“凭经验填数”更可解释的起点：
电流环按 RL 一阶对象配置，速度环按二阶闭环期望极点配置。
"""

from __future__ import annotations

from dataclasses import dataclass
import math

from .motor_model import MotorParams


@dataclass(frozen=True)
class PIGains:
    kp: float
    ki: float


def current_loop_gains(
    resistance_ohm: float,
    inductance_h: float,
    bandwidth_hz: float = 300.0,
) -> PIGains:
    """电流环 PI 初值。

    对电机电流对象 L*di/dt + R*i = v，常用近似是：
    Kp = L * wc，Ki = R * wc，其中 wc = 2*pi*bandwidth。
    """
    if resistance_ohm <= 0.0:
        raise ValueError("resistance_ohm must be positive")
    if inductance_h <= 0.0:
        raise ValueError("inductance_h must be positive")
    if bandwidth_hz <= 0.0:
        raise ValueError("bandwidth_hz must be positive")

    omega_c = 2.0 * math.pi * bandwidth_hz
    return PIGains(kp=inductance_h * omega_c, ki=resistance_ohm * omega_c)


def velocity_loop_gains(
    inertia_kg_m2: float,
    viscous_nm_per_rad_s: float,
    kt_nm_per_a: float,
    bandwidth_hz: float = 20.0,
    damping_ratio: float = 0.9,
) -> PIGains:
    """速度环 PI 初值。

    速度环 PI 输出 Iq，电机近似为 J*domega/dt + B*omega = Kt*Iq。
    令闭环特征项接近 s^2 + 2*zeta*wn*s + wn^2，可得到：
    Kp = (2*zeta*wn*J - B) / Kt，Ki = J*wn^2 / Kt。
    """
    if inertia_kg_m2 <= 0.0:
        raise ValueError("inertia_kg_m2 must be positive")
    if kt_nm_per_a <= 0.0:
        raise ValueError("kt_nm_per_a must be positive")
    if bandwidth_hz <= 0.0:
        raise ValueError("bandwidth_hz must be positive")
    if damping_ratio <= 0.0:
        raise ValueError("damping_ratio must be positive")

    omega_n = 2.0 * math.pi * bandwidth_hz
    kp = (2.0 * damping_ratio * omega_n * inertia_kg_m2 - viscous_nm_per_rad_s) / kt_nm_per_a
    ki = inertia_kg_m2 * omega_n * omega_n / kt_nm_per_a
    return PIGains(kp=max(0.0, kp), ki=max(0.0, ki))


def tuned_gains_from_motor(
    params: MotorParams,
    current_bandwidth_hz: float = 300.0,
    velocity_bandwidth_hz: float = 20.0,
    damping_ratio: float = 0.9,
) -> tuple[PIGains, PIGains]:
    """同时返回 current_gains, velocity_gains，便于例程一行完成整定。"""
    current = current_loop_gains(params.resistance_ohm, params.inductance_h, current_bandwidth_hz)
    velocity = velocity_loop_gains(
        params.inertia_kg_m2,
        params.viscous_nm_per_rad_s,
        params.kt_nm_per_a,
        velocity_bandwidth_hz,
        damping_ratio,
    )
    return current, velocity
