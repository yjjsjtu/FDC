"""用于 FOC 实验的简化表贴式 PMSM/BLDC dq 模型。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

from .foc_math import dq_to_phase


@dataclass(frozen=True)
class MotorParams:
    # 默认参数参考 Drivex2/4310 资料，主要用于教学仿真，不代表最终硬件标定值。
    pole_pairs: int = 14
    resistance_ohm: float = 5.37
    inductance_h: float = 0.00326
    kt_nm_per_a: float = 0.23
    ke_v_per_rad_s: float = 0.23
    inertia_kg_m2: float = 2.02e-5
    viscous_nm_per_rad_s: float = 2.0e-4
    coulomb_friction_nm: float = 0.015
    v_bus: float = 24.0
    friction_smoothing_rad_s: float = 0.08


@dataclass
class MotorState:
    # theta/omega 是机械侧位置和速度；id/iq 是转子同步坐标系下的电流。
    theta_rad: float = 0.0
    omega_rad_s: float = 0.0
    id_a: float = 0.0
    iq_a: float = 0.0
    torque_nm: float = 0.0


class PMSMMotorModel:
    """教学用 dq 轴电机模型。

    这里故意保持简化：Ld=Lq=L，转矩为 Kt*Iq，q 轴反电动势近似为
    Ke*mechanical_speed。
    """

    def __init__(self, params: MotorParams | None = None) -> None:
        self.params = params or MotorParams()
        self.state = MotorState()

    def reset(self, state: MotorState | None = None) -> None:
        self.state = state or MotorState()

    def friction_torque(self, omega_rad_s: float) -> float:
        p = self.params
        # 用 tanh 平滑 sign(omega)，避免零速附近库仑摩擦出现硬跳变。
        return (
            p.viscous_nm_per_rad_s * omega_rad_s
            + p.coulomb_friction_nm * math.tanh(omega_rad_s / p.friction_smoothing_rad_s)
        )

    def phase_currents(self, angle_el_rad: float) -> Tuple[float, float, float]:
        return dq_to_phase(self.state.id_a, self.state.iq_a, angle_el_rad)

    def step(
        self,
        vd_v: float,
        vq_v: float,
        dt: float,
        load_torque_nm: float = 0.0,
    ) -> MotorState:
        if dt <= 0.0:
            raise ValueError("dt must be positive")

        p = self.params
        s = self.state
        # 电角速度 = 机械角速度 * 极对数。
        omega_e = p.pole_pairs * s.omega_rad_s

        # dq 电流微分方程：包含电阻压降、电感耦合项和 q 轴反电动势。
        did = (vd_v - p.resistance_ohm * s.id_a + omega_e * p.inductance_h * s.iq_a) / p.inductance_h
        diq = (
            vq_v
            - p.resistance_ohm * s.iq_a
            - omega_e * p.inductance_h * s.id_a
            - p.ke_v_per_rad_s * s.omega_rad_s
        ) / p.inductance_h

        # 欧拉积分更新电流。仿真步长较小，因此这里足够用于教学演示。
        s.id_a += did * dt
        s.iq_a += diq * dt

        # q 轴电流产生电磁转矩，转矩再通过机械方程更新速度和位置。
        s.torque_nm = p.kt_nm_per_a * s.iq_a
        friction = self.friction_torque(s.omega_rad_s)
        domega = (s.torque_nm - friction - load_torque_nm) / p.inertia_kg_m2
        s.omega_rad_s += domega * dt
        s.theta_rad += s.omega_rad_s * dt
        return s
