#!/usr/bin/env python3
"""生成 SVPWM 占空比演示数据和图片。

这个脚本固定一个电压矢量幅值，让电压矢量旋转一圈，观察 A/B/C 三相
占空比如何随电角度变化。
"""

from __future__ import annotations

import csv
import math
from pathlib import Path
import sys

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.plotting import plot_lines
from src.svpwm import calculate_svpwm


def main() -> None:
    out_dir = ROOT / "outputs"
    data_dir = ROOT / "data"
    data_dir.mkdir(parents=True, exist_ok=True)
    out_dir.mkdir(parents=True, exist_ok=True)

    v_bus = 24.0
    voltage_mag = 7.0
    # angles 表示电压矢量在 alpha/beta 平面内转一整圈。
    angles = np.linspace(0.0, 2.0 * math.pi, 721)
    duty_a, duty_b, duty_c, sectors = [], [], [], []

    for angle in angles:
        # 把目标电压矢量投给 SVPWM，得到当前角度下的三相占空比。
        result = calculate_svpwm(voltage_mag * math.cos(angle), voltage_mag * math.sin(angle), v_bus)
        duty_a.append(result.duty_a)
        duty_b.append(result.duty_b)
        duty_c.append(result.duty_c)
        sectors.append(result.sector)

    # 保存 CSV，方便后续用表格查看每个角度对应的扇区和占空比。
    csv_path = data_dir / "svpwm_demo.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        writer = csv.writer(fh)
        writer.writerow(["angle_rad", "sector", "duty_a", "duty_b", "duty_c"])
        for row in zip(angles, sectors, duty_a, duty_b, duty_c):
            writer.writerow([f"{float(value):.10g}" for value in row])

    # 画出三相占空比曲线：蓝色 duty_a，红色 duty_b，绿色 duty_c。
    plot_lines(
        out_dir / "svpwm_sector.png",
        angles,
        [
            ("duty_a", np.asarray(duty_a)),
            ("duty_b", np.asarray(duty_b)),
            ("duty_c", np.asarray(duty_c)),
        ],
    )
    print(f"SVPWM demo written to {csv_path}")
    print(f"Plot written to {out_dir / 'svpwm_sector.png'}")


if __name__ == "__main__":
    main()
