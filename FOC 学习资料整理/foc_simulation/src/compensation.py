"""摩擦和惯量前馈补偿工具。"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Mapping

import numpy as np


@dataclass(frozen=True)
class CompensationParams:
    # 这些参数来自离线辨识，也可以将来替换成真实硬件辨识结果。
    inertia_kg_m2: float
    viscous_nm_per_rad_s: float
    coulomb_friction_nm: float
    kt_nm_per_a: float
    smoothing_rad_s: float = 0.08


def feedforward_torque(
    target_velocity_rad_s: float,
    target_accel_rad_s2: float,
    params: CompensationParams,
) -> float:
    """根据目标速度和加速度计算需要提前补偿的力矩。"""
    # 摩擦补偿：粘滞摩擦 B*omega + 平滑库仑摩擦 Fc*tanh(omega/eps)。
    friction = (
        params.viscous_nm_per_rad_s * target_velocity_rad_s
        + params.coulomb_friction_nm * math.tanh(target_velocity_rad_s / params.smoothing_rad_s)
    )
    # 惯量补偿：如果目标轨迹要求加速，就提前给 J*alpha 的力矩。
    inertia = params.inertia_kg_m2 * target_accel_rad_s2
    return inertia + friction


def feedforward_current(
    target_velocity_rad_s: float,
    target_accel_rad_s2: float,
    params: CompensationParams,
) -> float:
    """把前馈力矩换算成 q 轴电流前馈，供速度环叠加使用。"""
    if params.kt_nm_per_a <= 0.0:
        raise ValueError("kt_nm_per_a must be positive")
    return feedforward_torque(target_velocity_rad_s, target_accel_rad_s2, params) / params.kt_nm_per_a


def rms(values: np.ndarray) -> float:
    """均方根误差，用来衡量整体跟踪误差。"""
    return float(np.sqrt(np.mean(np.square(values))))


def comparison_metrics(data: Mapping[str, np.ndarray]) -> dict[str, float]:
    """计算速度/位置误差指标，便于比较补偿前后效果。"""
    velocity_error = data["target_velocity_rad_s"] - data["velocity_rad_s"]
    position_error = data["target_position_rad"] - data["position_rad"]
    return {
        "velocity_rms_rad_s": rms(velocity_error),
        "position_rms_rad": rms(position_error),
        "velocity_peak_abs_rad_s": float(np.max(np.abs(velocity_error))),
        "position_peak_abs_rad": float(np.max(np.abs(position_error))),
    }


def control_performance_metrics(data: Mapping[str, np.ndarray]) -> dict[str, float]:
    """计算控制性能指标，便于比较 baseline 和 optimized 两组仿真。"""
    metrics = comparison_metrics(data)
    target_velocity = data["target_velocity_rad_s"]
    velocity_error = target_velocity - data["velocity_rad_s"]
    command_velocity = data.get("command_velocity_rad_s", target_velocity)
    command_error = command_velocity - data["velocity_rad_s"]
    trajectory_error = command_velocity - target_velocity
    id_current = data["id_a"]
    iq_current = data["iq_a"]
    target_iq = data["target_iq_a"]
    voltage_saturated = data.get("voltage_saturated", np.zeros_like(target_velocity))

    threshold = max(0.2, 0.03 * float(np.max(np.abs(target_velocity))))
    settled_time = float(data["time_s"][-1])
    abs_error = np.abs(velocity_error)
    for index in range(len(abs_error)):
        if np.all(abs_error[index:] <= threshold):
            settled_time = float(data["time_s"][index])
            break

    metrics.update(
        {
            "id_rms_a": rms(id_current),
            "iq_rms_a": rms(iq_current),
            "iq_peak_abs_a": float(np.max(np.abs(iq_current))),
            "target_iq_peak_abs_a": float(np.max(np.abs(target_iq))),
            "voltage_saturation_ratio": float(np.mean(voltage_saturated > 0.5)),
            "settling_time_s": settled_time,
            "command_velocity_rms_rad_s": rms(command_error),
            "command_velocity_peak_abs_rad_s": float(np.max(np.abs(command_error))),
            "trajectory_shaping_rms_rad_s": rms(trajectory_error),
        }
    )
    return metrics
