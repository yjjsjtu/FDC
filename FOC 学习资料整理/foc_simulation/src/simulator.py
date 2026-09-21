"""闭环 FOC 仿真主循环。

这个模块把控制器、SVPWM 和电机模型串起来，模拟一次完整的
“目标速度 -> FOC 控制 -> 三相 PWM -> 电机响应 -> 反馈”的过程。
"""

from __future__ import annotations

from dataclasses import dataclass
import csv
from pathlib import Path
from typing import Callable, Mapping

import numpy as np

from .compensation import CompensationParams, feedforward_current
from .controllers import PIController, PController, clamp
from .estimators import LowPassVelocityEstimator, PLLVelocityEstimator, QuantizedEncoder
from .foc_math import clarke, electrical_angle, inverse_park, park
from .motor_model import MotorParams, PMSMMotorModel
from .svpwm import calculate_svpwm, duties_to_alpha_beta
from .trajectory import VelocityTrajectoryPlanner

VelocityProfile = Callable[[float], float]
PositionProfile = Callable[[float], float]
LoadProfile = Callable[[float], float]


@dataclass(frozen=True)
class SimulationConfig:
    # dt_s 越小，仿真越接近连续系统，但运行时间也越长。
    dt_s: float = 5.0e-5
    duration_s: float = 2.0
    zero_electric_angle_rad: float = 0.0
    # current_limit_a 和 voltage_limit_v 分别模拟驱动器限流、母线/调制限压。
    current_limit_a: float = 1.8
    voltage_limit_v: float = 13.5
    # 外环是速度环，内环是 d/q 电流环。
    velocity_kp: float = 0.055
    velocity_ki: float = 0.45
    # 完整三环控制时，位置 P 环输出速度目标，再交给速度 PI 环。
    position_kp: float = 12.0
    position_velocity_limit_rad_s: float = 18.0
    use_position_velocity_feedforward: bool = True
    current_kp: float = 2.0
    current_ki: float = 800.0
    # 轨迹层把速度命令转换成受加速度/加加速度限制的速度参考。
    use_trajectory: bool = False
    max_accel_rad_s2: float = 60.0
    max_jerk_rad_s3: float | None = None
    # 电流环电压前馈用于补偿 R*i、反电动势和 d/q 轴交叉耦合。
    use_voltage_feedforward: bool = False
    current_back_calculation_gain: float = 0.0
    rollback_on_voltage_saturation: bool = True
    # 速度反馈可以从理想速度切换为模拟编码器测速，便于提前测试噪声和量化。
    velocity_estimator: str = "none"
    encoder_counts_per_rev: int = 4096
    encoder_noise_std_rad: float = 0.0
    velocity_filter_tau_s: float = 0.002
    pll_kp: float = 400.0
    pll_ki: float = 40000.0


CSV_FIELDS = [
    # CSV 字段固定下来后，仿真数据和未来真实硬件采集数据可以复用同一套辨识脚本。
    "time_s",
    "command_velocity_rad_s",
    "target_position_rad",
    "position_rad",
    "target_velocity_rad_s",
    "target_acceleration_rad_s2",
    "velocity_rad_s",
    "feedback_velocity_rad_s",
    "target_iq_a",
    "id_a",
    "iq_a",
    "vd_v",
    "vq_v",
    "duty_a",
    "duty_b",
    "duty_c",
    "torque_nm",
    "load_torque_nm",
    "voltage_saturated",
]


def run_foc_simulation(
    velocity_profile: VelocityProfile,
    motor_params: MotorParams | None = None,
    config: SimulationConfig | None = None,
    load_profile: LoadProfile | None = None,
    compensation: CompensationParams | None = None,
) -> dict[str, np.ndarray]:
    """运行一次速度闭环 FOC 仿真，并返回所有记录量。"""
    return _run_closed_loop_simulation(
        velocity_profile=velocity_profile,
        position_profile=None,
        motor_params=motor_params,
        config=config,
        load_profile=load_profile,
        compensation=compensation,
    )


def run_position_foc_simulation(
    position_profile: PositionProfile,
    motor_params: MotorParams | None = None,
    config: SimulationConfig | None = None,
    load_profile: LoadProfile | None = None,
    compensation: CompensationParams | None = None,
) -> dict[str, np.ndarray]:
    """运行一次位置-速度-电流三环 FOC 仿真，并返回所有记录量。"""
    return _run_closed_loop_simulation(
        velocity_profile=None,
        position_profile=position_profile,
        motor_params=motor_params,
        config=config,
        load_profile=load_profile,
        compensation=compensation,
    )


def _run_closed_loop_simulation(
    velocity_profile: VelocityProfile | None,
    position_profile: PositionProfile | None,
    motor_params: MotorParams | None = None,
    config: SimulationConfig | None = None,
    load_profile: LoadProfile | None = None,
    compensation: CompensationParams | None = None,
) -> dict[str, np.ndarray]:
    if (velocity_profile is None) == (position_profile is None):
        raise ValueError("provide exactly one of velocity_profile or position_profile")

    params = motor_params or MotorParams()
    cfg = config or SimulationConfig()
    motor = PMSMMotorModel(params)
    if cfg.velocity_estimator not in {"none", "lpf", "pll"}:
        raise ValueError("velocity_estimator must be 'none', 'lpf', or 'pll'")

    # 位置 P 输出目标速度，速度 PI 输出目标 Iq，d/q 电流 PI 输出 Vd/Vq。
    position_p = PController(cfg.position_kp, cfg.position_velocity_limit_rad_s)
    velocity_pi = PIController(cfg.velocity_kp, cfg.velocity_ki, cfg.current_limit_a)
    d_current_pi = PIController(
        cfg.current_kp,
        cfg.current_ki,
        cfg.voltage_limit_v,
        back_calculation_gain=cfg.current_back_calculation_gain,
    )
    q_current_pi = PIController(
        cfg.current_kp,
        cfg.current_ki,
        cfg.voltage_limit_v,
        back_calculation_gain=cfg.current_back_calculation_gain,
    )
    trajectory = (
        VelocityTrajectoryPlanner(cfg.max_accel_rad_s2, cfg.max_jerk_rad_s3)
        if cfg.use_trajectory
        else None
    )
    encoder = None
    velocity_estimator = None
    if cfg.velocity_estimator != "none":
        encoder = QuantizedEncoder(cfg.encoder_counts_per_rev, cfg.encoder_noise_std_rad)
        if cfg.velocity_estimator == "lpf":
            velocity_estimator = LowPassVelocityEstimator(cfg.velocity_filter_tau_s)
        else:
            velocity_estimator = PLLVelocityEstimator(cfg.pll_kp, cfg.pll_ki)

    steps = int(cfg.duration_s / cfg.dt_s)
    arrays: dict[str, list[float]] = {field: [] for field in CSV_FIELDS}

    target_position = position_profile(0.0) if position_profile else 0.0
    previous_command_position = target_position
    previous_target_velocity = velocity_profile(0.0) if velocity_profile else 0.0

    for index in range(steps):
        time_s = index * cfg.dt_s

        # load_profile 用于模拟外部负载扰动，比如机械臂关节上的周期负载。
        load_torque = load_profile(time_s) if load_profile else 0.0
        feedback_theta = motor.state.theta_rad
        feedback_velocity = motor.state.omega_rad_s
        if encoder and velocity_estimator:
            feedback_theta = encoder.measure(motor.state.theta_rad)
            feedback_velocity = velocity_estimator.update(feedback_theta, cfg.dt_s)

        true_angle_el = electrical_angle(
            motor.state.theta_rad,
            params.pole_pairs,
            cfg.zero_electric_angle_rad,
        )
        feedback_angle_el = electrical_angle(feedback_theta, params.pole_pairs, cfg.zero_electric_angle_rad)

        # 真实硬件中是先采三相电流，再用“测得的电角度”变换到 d/q 轴。
        # 默认角度无误差时，这与直接读取电机模型内部 id/iq 完全一致。
        ia, ib, ic = motor.phase_currents(true_angle_el)
        i_alpha, i_beta = clarke(ia, ib, ic)
        id_meas, iq_meas = park(i_alpha, i_beta, feedback_angle_el)

        # command_velocity 是上层给出的原始速度命令或位置命令的差分速度；
        # target_velocity 是速度环实际跟踪的参考。位置三环中它由位置 P 环生成。
        if position_profile:
            command_position = position_profile(time_s)
            command_velocity = (command_position - previous_command_position) / cfg.dt_s if index else 0.0
            previous_command_position = command_position
            position_velocity = position_p.update(command_position - feedback_theta)
            velocity_feedforward = command_velocity if cfg.use_position_velocity_feedforward else 0.0
            target_velocity = clamp(
                position_velocity + velocity_feedforward,
                -cfg.position_velocity_limit_rad_s,
                cfg.position_velocity_limit_rad_s,
            )
            target_position = command_position
            target_accel = (target_velocity - previous_target_velocity) / cfg.dt_s if index else 0.0
        else:
            command_velocity = velocity_profile(time_s)  # type: ignore[union-attr]
            if trajectory:
                reference = trajectory.update(command_velocity, cfg.dt_s)
                target_position = reference.position_rad
                target_velocity = reference.velocity_rad_s
                target_accel = reference.acceleration_rad_s2
            else:
                target_velocity = command_velocity
                target_accel = (target_velocity - previous_target_velocity) / cfg.dt_s if index else 0.0
                target_position += target_velocity * cfg.dt_s
        previous_target_velocity = target_velocity

        # 可选前馈补偿：根据目标速度/加速度提前给一部分 Iq，减轻速度 PI 压力。
        ff_current = feedforward_current(target_velocity, target_accel, compensation) if compensation else 0.0
        target_iq = velocity_pi.update(target_velocity - feedback_velocity, cfg.dt_s, ff_current)
        target_iq = clamp(target_iq, -cfg.current_limit_a, cfg.current_limit_a)

        # d 轴目标为 0，q 轴目标为速度环输出的 target_iq。
        vd_cmd = d_current_pi.update(0.0 - id_meas, cfg.dt_s)
        vq_cmd = q_current_pi.update(target_iq - iq_meas, cfg.dt_s)
        if cfg.use_voltage_feedforward:
            omega_e = params.pole_pairs * feedback_velocity
            vd_cmd += -omega_e * params.inductance_h * iq_meas
            vq_cmd += (
                params.resistance_ohm * target_iq
                + omega_e * params.inductance_h * id_meas
                + params.ke_v_per_rad_s * feedback_velocity
            )

        # 把电压矢量限制在线性调制范围内，避免 SVPWM 占空比越界。
        vector_mag = float(np.hypot(vd_cmd, vq_cmd))
        voltage_saturated = vector_mag > cfg.voltage_limit_v
        if vector_mag > cfg.voltage_limit_v:
            scale = cfg.voltage_limit_v / vector_mag
            vd_cmd *= scale
            vq_cmd *= scale
            if cfg.rollback_on_voltage_saturation:
                d_current_pi.rollback_integrator()
                q_current_pi.rollback_integrator()

        # FOC 心脏流程：Vd/Vq -> Ualpha/Ubeta -> SVPWM -> 三相占空比。
        u_alpha, u_beta = inverse_park(vd_cmd, vq_cmd, feedback_angle_el)
        pwm = calculate_svpwm(u_alpha, u_beta, params.v_bus)
        # 离线仿真中，再把占空比还原成等效电压喂给电机模型。
        applied_alpha, applied_beta = duties_to_alpha_beta(*pwm.duties(), params.v_bus)
        vd_applied, vq_applied = park(applied_alpha, applied_beta, true_angle_el)

        # 电机模型更新后得到下一时刻的电流、速度、位置。
        state = motor.step(vd_applied, vq_applied, cfg.dt_s, load_torque)

        # 保存一行“虚拟遥测数据”，格式尽量接近未来硬件采集 CSV。
        values = {
            "time_s": time_s,
            "command_velocity_rad_s": command_velocity,
            "target_position_rad": target_position,
            "position_rad": state.theta_rad,
            "target_velocity_rad_s": target_velocity,
            "target_acceleration_rad_s2": target_accel,
            "velocity_rad_s": state.omega_rad_s,
            "feedback_velocity_rad_s": feedback_velocity,
            "target_iq_a": target_iq,
            "id_a": id_meas,
            "iq_a": iq_meas,
            "vd_v": vd_applied,
            "vq_v": vq_applied,
            "duty_a": pwm.duty_a,
            "duty_b": pwm.duty_b,
            "duty_c": pwm.duty_c,
            "torque_nm": state.torque_nm,
            "load_torque_nm": load_torque,
            "voltage_saturated": 1.0 if voltage_saturated else 0.0,
        }
        for field in CSV_FIELDS:
            arrays[field].append(float(values[field]))

    return {key: np.asarray(value, dtype=float) for key, value in arrays.items()}


def save_csv(path: str | Path, data: Mapping[str, np.ndarray]) -> None:
    """保存仿真结果，后续辨识脚本直接读取这个 CSV。"""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.DictWriter(fh, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for row_index in range(len(data["time_s"])):
            writer.writerow({field: f"{float(data[field][row_index]):.10g}" for field in CSV_FIELDS})


def step_velocity_profile(
    first: float = 0.0,
    second: float = 15.0,
    third: float = -8.0,
    t1: float = 0.1,
    t2: float = 1.2,
) -> VelocityProfile:
    """一个简单速度阶跃轨迹：先静止，再正转，最后反转。"""
    def profile(time_s: float) -> float:
        if time_s < t1:
            return first
        if time_s < t2:
            return second
        return third

    return profile
