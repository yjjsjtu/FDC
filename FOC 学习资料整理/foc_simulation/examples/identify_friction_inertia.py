#!/usr/bin/env python3
"""从速度/力矩数据中辨识库仑摩擦、粘滞摩擦和惯量。"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.identification import identify_friction_inertia, read_csv, write_json
from src.plotting import plot_lines


def moving_average(values: np.ndarray, window: int = 121) -> np.ndarray:
    """中心滑动平均，用来显示力矩主趋势，不用于篡改原始辨识数据。"""
    if window < 3:
        return values.copy()
    if window % 2 == 0:
        window += 1
    pad = window // 2
    padded = np.pad(values, (pad, pad), mode="edge")
    kernel = np.ones(window, dtype=float) / window
    return np.convolve(padded, kernel, mode="valid")


def generate_friction_sweep(
    path: Path,
    inertia: float = 2.02e-5,
    viscous: float = 2.0e-4,
    coulomb: float = 0.015,
) -> None:
    """生成一份包含正反向和加减速的仿真数据，用来测试辨识算法。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    dt = 0.001
    time = np.arange(0.0, 8.0, dt)
    velocity = 8.0 * np.sin(2.0 * math.pi * time / 4.0) + 2.0 * np.sin(2.0 * math.pi * time / 1.2)
    accel = np.gradient(velocity, dt)
    # 真实模型：tau = J*alpha + B*omega + Fc*sign(omega)，再叠加一点小扰动。
    torque = inertia * accel + viscous * velocity + coulomb * np.sign(velocity)
    torque += 0.0006 * np.sin(37.0 * time)

    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time_s", "velocity_rad_s", "torque_nm"])
        for row in zip(time, velocity, torque):
            writer.writerow([f"{float(value):.10g}" for value in row])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input CSV with time_s, velocity_rad_s, torque_nm")
    args = parser.parse_args()

    input_path = ROOT / args.input if not Path(args.input).is_absolute() else Path(args.input)
    if not input_path.exists():
        # 没有真实 CSV 时自动生成仿真数据，便于离线演示。
        generate_friction_sweep(input_path)
        print(f"Generated synthetic friction/inertia data: {input_path}")

    data = read_csv(input_path)
    result = identify_friction_inertia(data)
    write_json(ROOT / "outputs" / "friction_inertia_params.json", result)

    # predicted 是用辨识参数重建出的力矩，用来和原始 torque 对比。
    velocity = data["velocity_rad_s"]
    dt = float(np.median(np.diff(data["time_s"])))
    accel = np.gradient(velocity, dt)
    predicted = (
        result["inertia_kg_m2"] * accel
        + result["viscous_nm_per_rad_s"] * velocity
        + result["coulomb_friction_nm"] * np.sign(velocity)
    )
    filtered_torque = moving_average(data["torque_nm"])
    residual = data["torque_nm"] - predicted
    filtered_residual = moving_average(residual)
    residual_rms = float(np.sqrt(np.mean(np.square(residual))))
    filtered_residual_rms = float(np.sqrt(np.mean(np.square(filtered_residual))))
    write_json(
        ROOT / "outputs" / "friction_fit_metrics.json",
        {
            "raw_residual_rms_nm": residual_rms,
            "filtered_residual_rms_nm": filtered_residual_rms,
            "max_abs_residual_nm": float(np.max(np.abs(residual))),
        },
    )

    plot_lines(
        ROOT / "outputs" / "friction_fit.png",
        data["time_s"],
        [
            ("raw_torque", data["torque_nm"]),
            ("filtered_torque", filtered_torque),
            ("predicted", predicted),
        ],
    )
    plot_lines(
        ROOT / "outputs" / "friction_residual.png",
        data["time_s"],
        [
            ("raw_residual", residual),
            ("filtered_residual", filtered_residual),
        ],
    )

    print(f"Fc={result['coulomb_friction_nm']:.6f} Nm")
    print(f"B={result['viscous_nm_per_rad_s']:.8f} Nm/(rad/s)")
    print(f"J={result['inertia_kg_m2']:.9f} kg*m^2")
    print(f"raw_residual_rms={residual_rms:.8f} Nm")
    print(f"filtered_residual_rms={filtered_residual_rms:.8f} Nm")
    print(f"Results written to {ROOT / 'outputs' / 'friction_inertia_params.json'}")


if __name__ == "__main__":
    main()
