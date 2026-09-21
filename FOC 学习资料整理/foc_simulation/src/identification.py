"""从 CSV 数据离线辨识电机参数。

仿真数据和未来真实硬件采集数据只要字段一致，就能共用这些函数。
"""

from __future__ import annotations

import csv
import json
from pathlib import Path
from typing import Mapping

import numpy as np


def read_csv(path: str | Path) -> dict[str, np.ndarray]:
    """读取 CSV，并把每一列转成 numpy 数组，方便后续拟合计算。"""
    with Path(path).open("r", newline="", encoding="utf-8") as fh:
        reader = csv.DictReader(fh)
        rows = list(reader)
        if not rows:
            raise ValueError(f"empty CSV: {path}")
        fields = reader.fieldnames or []
        data = {field: [] for field in fields}
        for row in rows:
            for field in fields:
                data[field].append(float(row[field]))
    return {key: np.asarray(value, dtype=float) for key, value in data.items()}


def write_json(path: str | Path, payload: Mapping[str, float | dict]) -> None:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)


def estimate_resistance(data: Mapping[str, np.ndarray]) -> float:
    """根据阶跃实验末端的稳态 V/I 估计相电阻 R。"""
    voltage = data.get("voltage_v", data.get("vq_v"))
    current = data.get("current_a", data.get("iq_a"))
    if voltage is None or current is None:
        raise ValueError("CSV needs voltage_v/current_a or vq_v/iq_a")

    n = len(current)
    # 取后 25% 的数据作为近似稳态段，避免启动瞬态影响 R 的估计。
    start = int(n * 0.75)
    mask = np.abs(current[start:]) > 1e-6
    ratios = np.abs(voltage[start:][mask] / current[start:][mask])
    if ratios.size == 0:
        raise ValueError("not enough nonzero current samples for R estimate")
    return float(np.median(ratios))


def estimate_inductance(data: Mapping[str, np.ndarray], resistance_ohm: float | None = None) -> float:
    """根据 RL 一阶阶跃响应拟合相电感 L。"""
    time = data["time_s"]
    voltage = data.get("voltage_v", data.get("vq_v"))
    current = data.get("current_a", data.get("iq_a"))
    if voltage is None or current is None:
        raise ValueError("CSV needs voltage_v/current_a or vq_v/iq_a")

    resistance = resistance_ohm if resistance_ohm is not None else estimate_resistance(data)
    steady_current = float(np.median(current[int(len(current) * 0.8) :]))
    if abs(steady_current) < 1e-9:
        raise ValueError("steady current is too small")

    # i(t)=I_final*(1-exp(-t/tau))，所以 log(1-i/I_final) 对 t 是直线。
    normalized = current / steady_current
    mask = (normalized > 0.08) & (normalized < 0.85)
    if np.count_nonzero(mask) < 3:
        raise ValueError("not enough transient samples for L estimate")

    x = time[mask] - time[mask][0]
    y = np.log(np.clip(1.0 - normalized[mask], 1e-9, 1.0))
    slope, _ = np.polyfit(x, y, 1)
    tau = -1.0 / slope
    return float(resistance * tau)


def identify_rl(data: Mapping[str, np.ndarray]) -> dict[str, float]:
    """同时辨识 R 和 L。"""
    resistance = estimate_resistance(data)
    inductance = estimate_inductance(data, resistance)
    return {
        "resistance_ohm": resistance,
        "inductance_h": inductance,
    }


def identify_friction_inertia(
    data: Mapping[str, np.ndarray],
    kt_nm_per_a: float = 0.23,
    steady_accel_threshold: float = 3.0,
) -> dict[str, float]:
    """辨识库仑摩擦 Fc、粘滞摩擦 B 和惯量 J。"""
    time = data["time_s"]
    velocity = data["velocity_rad_s"]
    dt = float(np.median(np.diff(time)))
    accel = np.gradient(velocity, dt)

    if "torque_nm" in data:
        torque = data["torque_nm"]
    elif "iq_a" in data:
        # 如果没有直接力矩数据，就用 tau=Kt*Iq 从 q 轴电流换算。
        torque = kt_nm_per_a * data["iq_a"]
    else:
        raise ValueError("CSV needs torque_nm or iq_a")

    # 恒速段加速度较小，主要用于拟合摩擦项。
    moving = np.abs(velocity) > 0.15
    steady = moving & (np.abs(accel) < steady_accel_threshold)
    if np.count_nonzero(steady) < 5:
        steady = moving

    # 最小二乘拟合：tau = Fc*sign(omega) + B*omega。
    signs = np.sign(velocity[steady])
    design = np.column_stack([signs, velocity[steady]])
    coeffs, *_ = np.linalg.lstsq(design, torque[steady], rcond=None)
    coulomb = float(abs(coeffs[0]))
    viscous = float(max(coeffs[1], 0.0))

    # 动态段加速度较大，用 tau - friction = J*alpha 拟合惯量。
    dynamic = np.abs(accel) > max(steady_accel_threshold, np.percentile(np.abs(accel), 65))
    friction_est = coulomb * np.sign(velocity) + viscous * velocity
    residual = torque - friction_est
    denom = float(np.dot(accel[dynamic], accel[dynamic]))
    if denom < 1e-12:
        inertia = 0.0
    else:
        inertia = float(np.dot(accel[dynamic], residual[dynamic]) / denom)

    return {
        "coulomb_friction_nm": max(coulomb, 0.0),
        "viscous_nm_per_rad_s": max(viscous, 0.0),
        "inertia_kg_m2": max(inertia, 0.0),
    }
