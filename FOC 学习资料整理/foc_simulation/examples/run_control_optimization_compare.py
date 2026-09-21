#!/usr/bin/env python3
"""比较基础控制和优化控制的闭环效果。

baseline 使用原始速度命令、经验 PI、无电压前馈。
smooth 使用轨迹规划，优先降低电流尖峰和平滑运动。
precision 直接跟踪原始速度命令，优先提高目标速度跟随精度。
"""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.compensation import control_performance_metrics
from src.identification import write_json
from src.motor_model import MotorParams
from src.plotting import plot_lines
from src.simulator import SimulationConfig, run_foc_simulation, save_csv
from src.tuning import tuned_gains_from_motor


def command_velocity_profile(time_s: float) -> float:
    """带多次换向的速度命令，用来暴露突变、超调和电流峰值问题。"""
    if time_s < 0.15:
        return 0.0
    if time_s < 0.75:
        return 14.0
    if time_s < 1.35:
        return -9.0
    if time_s < 1.95:
        return 10.0
    return -5.0


def load_profile(time_s: float) -> float:
    """小幅负载扰动，避免测试只覆盖完全理想空载工况。"""
    return 0.004 if 1.0 <= time_s < 1.4 else 0.0


def main() -> None:
    outputs = ROOT / "outputs"
    data_dir = ROOT / "data"
    outputs.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    params = MotorParams()
    current_gains, velocity_gains = tuned_gains_from_motor(params)

    baseline_config = SimulationConfig(duration_s=2.4)
    smooth_config = SimulationConfig(
        duration_s=2.4,
        velocity_kp=velocity_gains.kp,
        velocity_ki=velocity_gains.ki,
        current_kp=current_gains.kp,
        current_ki=current_gains.ki,
        use_trajectory=True,
        max_accel_rad_s2=45.0,
        max_jerk_rad_s3=650.0,
        use_voltage_feedforward=True,
        current_back_calculation_gain=120.0,
        velocity_estimator="lpf",
        encoder_counts_per_rev=4096,
        encoder_noise_std_rad=0.0004,
        velocity_filter_tau_s=0.0025,
    )
    precision_config = SimulationConfig(
        duration_s=2.4,
        velocity_kp=0.075,
        velocity_ki=0.8,
        current_kp=current_gains.kp,
        current_ki=current_gains.ki,
        use_trajectory=False,
        use_voltage_feedforward=True,
        current_back_calculation_gain=120.0,
    )

    baseline = run_foc_simulation(command_velocity_profile, params, baseline_config, load_profile=load_profile)
    smooth = run_foc_simulation(command_velocity_profile, params, smooth_config, load_profile=load_profile)
    precision = run_foc_simulation(command_velocity_profile, params, precision_config, load_profile=load_profile)

    save_csv(data_dir / "control_baseline.csv", baseline)
    save_csv(data_dir / "control_smooth.csv", smooth)
    save_csv(data_dir / "control_optimized.csv", precision)
    save_csv(data_dir / "control_precision.csv", precision)

    baseline_metrics = control_performance_metrics(baseline)
    smooth_metrics = control_performance_metrics(smooth)
    precision_metrics = control_performance_metrics(precision)
    payload = {
        "baseline": baseline_metrics,
        "smooth": smooth_metrics,
        "precision": precision_metrics,
        "gains": {
            "current_kp": current_gains.kp,
            "current_ki": current_gains.ki,
            "smooth_velocity_kp": velocity_gains.kp,
            "smooth_velocity_ki": velocity_gains.ki,
            "precision_velocity_kp": precision_config.velocity_kp,
            "precision_velocity_ki": precision_config.velocity_ki,
        },
        "improvement": {
            "smooth_target_velocity_rms_ratio": smooth_metrics["velocity_rms_rad_s"]
            / baseline_metrics["velocity_rms_rad_s"],
            "precision_command_velocity_rms_ratio": precision_metrics["command_velocity_rms_rad_s"]
            / baseline_metrics["command_velocity_rms_rad_s"],
            "precision_iq_peak_ratio": precision_metrics["iq_peak_abs_a"]
            / max(baseline_metrics["iq_peak_abs_a"], 1.0e-12),
        },
    }
    write_json(outputs / "control_optimization_metrics.json", payload)

    plot_lines(
        outputs / "control_optimization_compare.png",
        baseline["time_s"],
        [
            ("command_velocity", baseline["command_velocity_rad_s"]),
            ("baseline_velocity", baseline["velocity_rad_s"]),
            ("precision_velocity", precision["velocity_rad_s"]),
        ],
    )
    plot_lines(
        outputs / "trajectory_compare.png",
        smooth["time_s"],
        [
            ("command_velocity", smooth["command_velocity_rad_s"]),
            ("smooth_target_velocity", smooth["target_velocity_rad_s"]),
            ("smooth_velocity", smooth["velocity_rad_s"]),
        ],
    )
    plot_lines(
        outputs / "current_decoupling_compare.png",
        baseline["time_s"],
        [
            ("baseline_id", baseline["id_a"]),
            ("precision_id", precision["id_a"]),
            ("baseline_iq", baseline["iq_a"]),
            ("precision_iq", precision["iq_a"]),
        ],
    )

    print("Control optimization comparison complete")
    print(f"baseline_command_velocity_rms={baseline_metrics['command_velocity_rms_rad_s']:.5f}")
    print(f"precision_command_velocity_rms={precision_metrics['command_velocity_rms_rad_s']:.5f}")
    print(f"smooth_target_velocity_rms={smooth_metrics['velocity_rms_rad_s']:.5f}")
    print(f"baseline_iq_peak={baseline_metrics['iq_peak_abs_a']:.5f}")
    print(f"precision_iq_peak={precision_metrics['iq_peak_abs_a']:.5f}")
    print(f"Metrics written to {outputs / 'control_optimization_metrics.json'}")


if __name__ == "__main__":
    main()
