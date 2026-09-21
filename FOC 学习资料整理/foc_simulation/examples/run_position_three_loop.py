#!/usr/bin/env python3
"""运行位置-速度-电流三环 FOC 仿真。"""

from __future__ import annotations

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.compensation import control_performance_metrics
from src.identification import write_json
from src.motor_model import MotorParams
from src.plotting import plot_lines
from src.simulator import SimulationConfig, run_position_foc_simulation, save_csv
from src.tuning import tuned_gains_from_motor


def smoothstep(value: float) -> float:
    """0~1 平滑插值，避免位置命令出现速度突变。"""
    value = max(0.0, min(1.0, value))
    return 0.5 - 0.5 * math.cos(math.pi * value)


def segment(time_s: float, start_s: float, end_s: float, start_rad: float, end_rad: float) -> float:
    ratio = (time_s - start_s) / (end_s - start_s)
    return start_rad + (end_rad - start_rad) * smoothstep(ratio)


def position_profile(time_s: float) -> float:
    """正反向位置轨迹，用来验证完整三环控制。"""
    if time_s < 0.2:
        return 0.0
    if time_s < 1.0:
        return segment(time_s, 0.2, 1.0, 0.0, 2.0)
    if time_s < 1.3:
        return 2.0
    if time_s < 2.2:
        return segment(time_s, 1.3, 2.2, 2.0, -1.2)
    if time_s < 2.5:
        return -1.2
    if time_s < 3.4:
        return segment(time_s, 2.5, 3.4, -1.2, 1.0)
    return 1.0


def main() -> None:
    outputs = ROOT / "outputs"
    data_dir = ROOT / "data"
    outputs.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    params = MotorParams()
    current_gains, _ = tuned_gains_from_motor(params)
    config = SimulationConfig(
        duration_s=3.8,
        position_kp=16.0,
        position_velocity_limit_rad_s=18.0,
        velocity_kp=0.075,
        velocity_ki=0.8,
        current_kp=current_gains.kp,
        current_ki=current_gains.ki,
        use_voltage_feedforward=True,
        current_back_calculation_gain=120.0,
    )

    data = run_position_foc_simulation(position_profile, params, config)
    metrics = control_performance_metrics(data)
    save_csv(data_dir / "position_three_loop.csv", data)
    write_json(outputs / "position_three_loop_metrics.json", metrics)

    # 位置图：蓝色目标位置，红色实际位置。
    plot_lines(
        outputs / "position_three_loop_response.png",
        data["time_s"],
        [
            ("target_position", data["target_position_rad"]),
            ("position", data["position_rad"]),
        ],
    )
    # 速度图：蓝色位置环输出速度目标，红色实际速度。
    plot_lines(
        outputs / "position_three_loop_velocity.png",
        data["time_s"],
        [
            ("target_velocity", data["target_velocity_rad_s"]),
            ("velocity", data["velocity_rad_s"]),
        ],
    )
    # 电流图：蓝色目标 Iq，红色实际 Iq，绿色实际 Id。
    plot_lines(
        outputs / "position_three_loop_current.png",
        data["time_s"],
        [
            ("target_iq", data["target_iq_a"]),
            ("iq", data["iq_a"]),
            ("id", data["id_a"]),
        ],
    )

    print("Position three-loop simulation complete")
    print(f"position_rms_rad={metrics['position_rms_rad']:.5f}")
    print(f"velocity_rms_rad_s={metrics['velocity_rms_rad_s']:.5f}")
    print(f"iq_peak_abs_a={metrics['iq_peak_abs_a']:.5f}")
    print(f"Metrics written to {outputs / 'position_three_loop_metrics.json'}")


if __name__ == "__main__":
    main()
