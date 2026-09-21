#!/usr/bin/env python3
"""运行速度闭环 FOC 仿真。

输出速度响应、d/q 电流响应和完整 CSV，主要用来验证闭环控制链路是否能
稳定跟踪目标速度。
"""

from __future__ import annotations

import math
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.compensation import comparison_metrics
from src.motor_model import MotorParams
from src.plotting import plot_lines
from src.simulator import SimulationConfig, run_foc_simulation, save_csv


def smoothstep(value: float) -> float:
    """0~1 平滑过渡函数，起点和终点斜率都为 0。"""
    value = max(0.0, min(1.0, value))
    return 0.5 - 0.5 * math.cos(math.pi * value)


def smooth_velocity_profile(time_s: float) -> float:
    """更接近真实测试的速度轨迹：静止、平滑加速、保持、平滑反转。"""
    if time_s < 0.2:
        return 0.0
    if time_s < 0.8:
        return 15.0 * smoothstep((time_s - 0.2) / 0.6)
    if time_s < 1.4:
        return 15.0
    if time_s < 2.1:
        return 15.0 + (-8.0 - 15.0) * smoothstep((time_s - 1.4) / 0.7)
    return -8.0


def main() -> None:
    outputs = ROOT / "outputs"
    data_dir = ROOT / "data"
    outputs.mkdir(parents=True, exist_ok=True)
    data_dir.mkdir(parents=True, exist_ok=True)

    params = MotorParams()
    config = SimulationConfig(duration_s=2.4)
    # 使用平滑速度轨迹替代硬阶跃，避免目标速度无限加速度导致 target_iq 出现不真实尖峰。
    data = run_foc_simulation(smooth_velocity_profile, params, config)

    csv_path = data_dir / "velocity_foc.csv"
    save_csv(csv_path, data)
    metrics = comparison_metrics(data)

    # 速度图：蓝色目标速度，红色实际速度。
    plot_lines(
        outputs / "velocity_response.png",
        data["time_s"],
        [
            ("target_velocity", data["target_velocity_rad_s"]),
            ("velocity", data["velocity_rad_s"]),
        ],
    )
    # 电流图：蓝色目标 Iq，红色实际 Iq，绿色实际 Id。
    plot_lines(
        outputs / "current_loop.png",
        data["time_s"],
        [
            ("target_iq", data["target_iq_a"]),
            ("iq", data["iq_a"]),
            ("id", data["id_a"]),
        ],
    )

    print(f"Velocity FOC CSV written to {csv_path}")
    print(f"velocity_rms_rad_s={metrics['velocity_rms_rad_s']:.4f}")
    print(f"position_rms_rad={metrics['position_rms_rad']:.4f}")
    print(f"Plots written to {outputs}")


if __name__ == "__main__":
    main()
