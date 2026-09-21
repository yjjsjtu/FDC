#!/usr/bin/env python3
"""比较无补偿和加入摩擦/惯量前馈后的速度跟踪效果。"""

from __future__ import annotations

import json
import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.compensation import CompensationParams, comparison_metrics
from src.identification import write_json
from src.motor_model import MotorParams
from src.plotting import plot_lines
from src.simulator import SimulationConfig, run_foc_simulation, save_csv


def smooth_profile(time_s: float) -> float:
    """正反向平滑速度轨迹，用来考察换向和加减速时的跟踪效果。"""
    if time_s < 0.2:
        return 0.0
    return 6.0 * math.sin(2.0 * math.pi * (time_s - 0.2) / 1.4)


def load_profile(time_s: float) -> float:
    """模拟一个周期性外部负载扰动。"""
    return 0.006 * math.sin(2.0 * math.pi * time_s / 0.9)


def load_compensation_params(params: MotorParams) -> CompensationParams:
    """优先使用辨识结果；没有辨识文件时使用电机模型默认参数。"""
    path = ROOT / "outputs" / "friction_inertia_params.json"
    if path.exists():
        payload = json.loads(path.read_text(encoding="utf-8"))
        return CompensationParams(
            inertia_kg_m2=float(payload["inertia_kg_m2"]),
            viscous_nm_per_rad_s=float(payload["viscous_nm_per_rad_s"]),
            coulomb_friction_nm=float(payload["coulomb_friction_nm"]),
            kt_nm_per_a=params.kt_nm_per_a,
        )
    return CompensationParams(
        inertia_kg_m2=params.inertia_kg_m2,
        viscous_nm_per_rad_s=params.viscous_nm_per_rad_s,
        coulomb_friction_nm=params.coulomb_friction_nm,
        kt_nm_per_a=params.kt_nm_per_a,
    )


def main() -> None:
    outputs = ROOT / "outputs"
    data_dir = ROOT / "data"
    outputs.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    params = MotorParams()
    config = SimulationConfig(duration_s=2.8, velocity_kp=0.035, velocity_ki=0.22)

    # 第一组：只有反馈 PI，没有摩擦/惯量前馈。
    no_comp = run_foc_simulation(smooth_profile, params, config, load_profile=load_profile)
    comp_params = load_compensation_params(params)
    # 第二组：在速度 PI 上叠加前馈电流 Iq_ff。
    with_comp = run_foc_simulation(
        smooth_profile,
        params,
        config,
        load_profile=load_profile,
        compensation=comp_params,
    )

    save_csv(data_dir / "compensation_no_ff.csv", no_comp)
    save_csv(data_dir / "compensation_with_ff.csv", with_comp)

    no_metrics = comparison_metrics(no_comp)
    with_metrics = comparison_metrics(with_comp)
    # ratio 小于 1 表示加入补偿后误差下降。
    payload = {
        "without_compensation": no_metrics,
        "with_compensation": with_metrics,
        "improvement": {
            "velocity_rms_ratio": with_metrics["velocity_rms_rad_s"] / no_metrics["velocity_rms_rad_s"],
            "position_rms_ratio": with_metrics["position_rms_rad"] / no_metrics["position_rms_rad"],
        },
    }
    write_json(outputs / "compensation_metrics.json", payload)

    plot_lines(
        outputs / "compensation_compare.png",
        no_comp["time_s"],
        [
            ("target_velocity", no_comp["target_velocity_rad_s"]),
            ("no_ff_velocity", no_comp["velocity_rad_s"]),
            ("with_ff_velocity", with_comp["velocity_rad_s"]),
        ],
    )

    print(f"No FF velocity RMS: {no_metrics['velocity_rms_rad_s']:.5f}")
    print(f"With FF velocity RMS: {with_metrics['velocity_rms_rad_s']:.5f}")
    print(f"Metrics written to {outputs / 'compensation_metrics.json'}")


if __name__ == "__main__":
    main()
