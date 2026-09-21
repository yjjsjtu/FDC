#!/usr/bin/env python3
"""从 RL 阶跃数据中辨识相电阻 R 和相电感 L。"""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.identification import identify_rl, read_csv, write_json
from src.plotting import plot_lines


def generate_rl_step(path: Path, resistance: float = 5.37, inductance: float = 0.00326, voltage: float = 3.0) -> None:
    """生成一份理想 RL 阶跃数据，用于没有硬件时验证辨识流程。"""
    path.parent.mkdir(parents=True, exist_ok=True)
    dt = 2.0e-5
    time = np.arange(0.0, 0.012, dt)
    current = voltage / resistance * (1.0 - np.exp(-time * resistance / inductance))
    with path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["time_s", "voltage_v", "current_a"])
        for t, i in zip(time, current):
            writer.writerow([f"{float(t):.10g}", f"{voltage:.10g}", f"{float(i):.10g}"])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input CSV with time_s, voltage_v, current_a")
    args = parser.parse_args()

    input_path = ROOT / args.input if not Path(args.input).is_absolute() else Path(args.input)
    if not input_path.exists():
        # 如果用户还没有真实数据，先生成仿真数据，让流程可以完整跑通。
        generate_rl_step(input_path)
        print(f"Generated synthetic RL step data: {input_path}")

    data = read_csv(input_path)
    result = identify_rl(data)
    write_json(ROOT / "outputs" / "rl_params.json", result)

    # 用辨识出来的 R/L 重新生成拟合曲线，和原始电流阶跃曲线对比。
    resistance = result["resistance_ohm"]
    inductance = result["inductance_h"]
    voltage = data.get("voltage_v", data.get("vq_v"))
    current_fit = voltage[0] / resistance * (1.0 - np.exp(-data["time_s"] * resistance / inductance))

    plot_lines(
        ROOT / "outputs" / "rl_identification.png",
        data["time_s"],
        [
            ("measured_current", data.get("current_a", data.get("iq_a"))),
            ("fit_current", current_fit),
        ],
    )
    print(f"R={resistance:.5f} ohm")
    print(f"L={inductance:.8f} H")
    print(f"Results written to {ROOT / 'outputs' / 'rl_params.json'}")


if __name__ == "__main__":
    main()
