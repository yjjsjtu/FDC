#!/usr/bin/env python3
"""Run stress scenarios for the migrated RDrive three-loop simulation."""

from __future__ import annotations

import copy
import csv
import json
import math
from pathlib import Path
from typing import Callable, Dict, List, Tuple

import numpy as np
from PIL import Image, ImageDraw

try:
    from .plot_results import (
        AXIS,
        BG,
        BLUE,
        CYAN,
        FONT_BODY,
        FONT_PANEL,
        FONT_SMALL,
        FONT_TITLE,
        GREEN,
        GRID,
        MUTED,
        ORANGE,
        PANEL_BG,
        PURPLE,
        RED,
        TEXT,
    )
    from .simulate import simulate
except ImportError:  # Allows running this file directly from rdrive_three_loop/.
    from plot_results import (  # type: ignore[no-redef]
        AXIS,
        BG,
        BLUE,
        CYAN,
        FONT_BODY,
        FONT_PANEL,
        FONT_SMALL,
        FONT_TITLE,
        GREEN,
        GRID,
        MUTED,
        ORANGE,
        PANEL_BG,
        PURPLE,
        RED,
        TEXT,
    )
    from simulate import simulate  # type: ignore[no-redef]


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "output"
PLOT_DIR = OUTPUT / "plots"
STRESS_DIR = OUTPUT / "stress"
WIDTH = 1600
HEIGHT = 1000


CaseMutator = Callable[[Dict], None]


def _draw_panel(draw: ImageDraw.ImageDraw, bounds: Tuple[int, int, int, int], title: str) -> Tuple[int, int, int, int]:
    left, top, right, bottom = bounds
    draw.rounded_rectangle(bounds, radius=18, fill=PANEL_BG, outline="#e2e8f0", width=2)
    draw.text((left + 26, top + 18), title, font=FONT_PANEL, fill=TEXT)
    return left + 98, top + 72, right - 36, bottom - 68


def _nice_range(values: np.ndarray) -> Tuple[float, float]:
    finite = values[np.isfinite(values)]
    ymin = float(np.min(finite))
    ymax = float(np.max(finite))
    if math.isclose(ymin, ymax):
        ymin -= 1.0
        ymax += 1.0
    margin = 0.1 * (ymax - ymin)
    return ymin - margin, ymax + margin


def _draw_axes(
    draw: ImageDraw.ImageDraw,
    area: Tuple[int, int, int, int],
    xmin: float,
    xmax: float,
    ymin: float,
    ymax: float,
    ylabel: str,
    draw_x_tick_labels: bool = True,
) -> Tuple[Callable[[float], float], Callable[[float], float]]:
    left, top, right, bottom = area
    for tick in range(6):
        y = top + tick * (bottom - top) / 5
        value = ymax - tick * (ymax - ymin) / 5
        draw.line((left, y, right, y), fill=GRID, width=1)
        draw.text((left - 70, y - 9), f"{value:.3g}", font=FONT_SMALL, fill=MUTED)
    for tick in range(7):
        x = left + tick * (right - left) / 6
        value = xmin + tick * (xmax - xmin) / 6
        draw.line((x, top, x, bottom), fill=GRID, width=1)
        if draw_x_tick_labels:
            draw.text((x - 16, bottom + 10), f"{value:.2f}", font=FONT_SMALL, fill=MUTED)
    draw.line((left, top, left, bottom), fill=AXIS, width=2)
    draw.line((left, bottom, right, bottom), fill=AXIS, width=2)
    draw.text((left, top - 26), ylabel, font=FONT_SMALL, fill=MUTED)

    def sx(value: float) -> float:
        return left + (value - xmin) / max(1e-12, xmax - xmin) * (right - left)

    def sy(value: float) -> float:
        return bottom - (value - ymin) / max(1e-12, ymax - ymin) * (bottom - top)

    return sx, sy


def _draw_line_chart(
    image: Image.Image,
    bounds: Tuple[int, int, int, int],
    title: str,
    ylabel: str,
    time: np.ndarray,
    series: List[Tuple[str, np.ndarray, str]],
) -> None:
    draw = ImageDraw.Draw(image)
    area = _draw_panel(draw, bounds, title)
    y_values = np.concatenate([values for _, values, _ in series])
    ymin, ymax = _nice_range(y_values)
    sx, sy = _draw_axes(draw, area, float(time[0]), float(time[-1]), ymin, ymax, ylabel)
    if ymin < 0.0 < ymax:
        draw.line((area[0], sy(0.0), area[2], sy(0.0)), fill=AXIS, width=1)
    step = max(1, len(time) // 2200)
    legend_x = area[0] + 12
    for name, values, color in series:
        points = [(sx(float(x)), sy(float(y))) for x, y in zip(time[::step], values[::step])]
        if len(points) >= 2:
            draw.line(points, fill=color, width=3, joint="curve")
        draw.line((legend_x, area[1] + 20, legend_x + 30, area[1] + 20), fill=color, width=4)
        draw.text((legend_x + 38, area[1] + 8), name, font=FONT_SMALL, fill=TEXT)
        legend_x += 175


def _position_loop_sample_indices(result: Dict[str, np.ndarray]) -> np.ndarray:
    """Pick one log sample per position-loop update to avoid drawing aliasing."""
    time = result["time_s"]
    if len(time) < 3:
        return np.arange(len(time))
    log_dt = float(np.median(np.diff(time)))
    ref = result["position_ref_rev"]
    changed = np.flatnonzero(np.abs(np.diff(ref)) > 1.0e-12) + 1
    if len(changed) >= 3:
        stride = int(round(float(np.median(np.diff(changed)))))
    else:
        stride = int(round(0.001 / max(log_dt, 1.0e-12)))
    stride = max(1, stride)
    indices = np.arange(0, len(time), stride, dtype=int)
    if indices[-1] != len(time) - 1:
        indices = np.append(indices, len(time) - 1)
    return indices


def _draw_bar_chart(
    image: Image.Image,
    bounds: Tuple[int, int, int, int],
    title: str,
    ylabel: str,
    values: List[float],
    labels: List[str],
    color: str,
    limit: float | None = None,
) -> None:
    draw = ImageDraw.Draw(image)
    area = _draw_panel(draw, bounds, title)
    left, top, right, bottom = area
    ymax = max(values + ([limit] if limit is not None else [0.0])) * 1.2
    ymax = ymax if ymax > 0.0 else 1.0
    sx, sy = _draw_axes(draw, area, 0.0, float(len(values)), 0.0, ymax, ylabel, draw_x_tick_labels=False)
    if limit is not None:
        y = sy(limit)
        draw.line((left, y, right, y), fill=RED, width=2)
        draw.text((right - 120, y - 22), "建议上限", font=FONT_SMALL, fill=RED)
    bar_w = (right - left) / max(1, len(values)) * 0.58
    for idx, value in enumerate(values):
        cx = sx(idx + 0.5)
        draw.rectangle((cx - bar_w / 2, sy(value), cx + bar_w / 2, bottom), fill=color)
        draw.text((cx - 14, bottom + 10), str(idx + 1), font=FONT_SMALL, fill=MUTED)
        draw.text((cx - 26, sy(value) - 22), f"{value:.2f}", font=FONT_SMALL, fill=TEXT)
    legend_y = bounds[3] - 42
    for idx, label in enumerate(labels):
        x = bounds[0] + 32 + (idx % 4) * 355
        y = legend_y + (idx // 4) * 20
        draw.text((x, y), f"{idx + 1}. {label}", font=FONT_SMALL, fill=TEXT)


def _case_configs(base: Dict) -> List[Tuple[str, str, Dict]]:
    def cfg_with(mutator: CaseMutator) -> Dict:
        cfg = copy.deepcopy(base)
        mutator(cfg)
        return cfg

    return [
        ("nominal_ff", "基准+前馈", cfg_with(lambda cfg: None)),
        (
            "nominal_no_motion_ff",
            "基准无运动前馈",
            cfg_with(lambda cfg: cfg["controller"].update({
                "enable_inertia_feedforward": False,
                "enable_friction_feedforward": False,
            })),
        ),
        (
            "fast_move",
            "高速大行程",
            cfg_with(lambda cfg: (
                cfg["simulation"].update({"duration_s": 1.1}),
                cfg["scenario"].update({"target_position_rev": 0.45, "load_start_s": 0.62, "load_end_s": 0.9, "load_torque_nm": 0.10}),
                cfg["controller"].update({"speed_limit_rev_s": 4.0, "acceleration_limit_rev_s2": 28.0}),
            )),
        ),
        (
            "heavy_load",
            "强负载",
            cfg_with(lambda cfg: cfg["scenario"].update({"load_torque_nm": 0.16, "load_start_s": 0.50, "load_end_s": 0.82})),
        ),
        (
            "inertia_mismatch",
            "惯量失配",
            cfg_with(lambda cfg: (
                cfg["motor"].update({"inertia_kg_m2": 0.000095}),
                cfg["scenario"].update({"target_position_rev": 0.35, "load_torque_nm": 0.08}),
                cfg["controller"].update({"speed_limit_rev_s": 2.8, "acceleration_limit_rev_s2": 16.0}),
            )),
        ),
        (
            "friction_mismatch",
            "摩擦失配",
            cfg_with(lambda cfg: (
                cfg["motor"].update({"coulomb_friction_nm": 0.008, "viscous_friction_nm_per_rad_s": 0.00008}),
                cfg["scenario"].update({"target_position_rev": 0.18, "load_torque_nm": 0.04}),
            )),
        ),
        (
            "reverse_multi_step",
            "多段反向",
            cfg_with(lambda cfg: (
                cfg["simulation"].update({"duration_s": 1.5}),
                cfg["scenario"].clear(),
                cfg["scenario"].update({
                    "target_events_rev": [[0.08, 0.22], [0.48, -0.18], [0.94, 0.16]],
                    "load_events_nm": [[0.22, 0.34, 0.06], [0.70, 0.86, -0.05], [1.08, 1.28, 0.08]],
                }),
                cfg["controller"].update({"speed_limit_rev_s": 3.0, "acceleration_limit_rev_s2": 22.0}),
            )),
        ),
        (
            "sine_load",
            "正弦扰动",
            cfg_with(lambda cfg: (
                cfg["simulation"].update({"duration_s": 1.4}),
                cfg["scenario"].update({
                    "target_position_rev": 0.30,
                    "load_start_s": 0.25,
                    "load_end_s": 1.15,
                    "load_torque_nm": 0.02,
                    "load_sine_nm": {
                        "start_s": 0.25,
                        "end_s": 1.15,
                        "amplitude_nm": 0.055,
                        "frequency_hz": 7.0,
                    },
                }),
            )),
        ),
    ]


def _metrics_for(name: str, label: str, cfg: Dict, result: Dict[str, np.ndarray], summary: Dict[str, float]) -> Dict[str, float | str | bool]:
    position_error_deg = (result["position_ref_rev"] - result["position_rev"]) * 360.0
    speed_error = result["speed_ref_rev_s"] - result["speed_rev_s"]
    allowed_voltage = float(summary["allowed_voltage_vector_v"])
    current_limit = float(cfg["inverter"]["current_limit_a"])
    voltage_utilization = float(summary["max_voltage_vector_v"] / allowed_voltage) if allowed_voltage else 0.0
    peak_position_error_deg = float(np.max(np.abs(position_error_deg)))
    final_error_deg = float(abs(summary["final_position_error_rev"]) * 360.0)
    pass_case = (
        peak_position_error_deg < 6.0
        and final_error_deg < 1.0
        and float(summary["max_abs_iq_a"]) < current_limit * 0.95
        and float(summary["voltage_saturation_fraction"]) < 0.01
    )
    return {
        "name": name,
        "label": label,
        "pass": pass_case,
        "final_error_deg": final_error_deg,
        "peak_position_error_deg": peak_position_error_deg,
        "rms_position_error_deg": float(np.sqrt(np.mean(position_error_deg * position_error_deg))),
        "rms_speed_error_rev_s": float(np.sqrt(np.mean(speed_error * speed_error))),
        "max_abs_iq_a": float(summary["max_abs_iq_a"]),
        "max_abs_id_a": float(summary["max_abs_id_a"]),
        "voltage_utilization_percent": voltage_utilization * 100.0,
        "voltage_saturation_percent": float(summary["voltage_saturation_fraction"]) * 100.0,
        "max_abs_motion_feedforward_nm": float(summary["max_abs_motion_feedforward_nm"]),
        "move_peak_error_deg": float(summary["move_peak_error_deg"]),
        "load_peak_error_deg": float(summary["load_peak_error_deg"]),
        "unload_peak_error_deg": float(summary["unload_peak_error_deg"]),
    }


def _save_summary_csv(path: Path, rows: List[Dict[str, float | str | bool]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = list(rows[0])
    with path.open("w", newline="", encoding="utf-8-sig") as fh:
        writer = csv.DictWriter(fh, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _plot_stress_bars(rows: List[Dict[str, float | str | bool]]) -> None:
    labels = [str(row["label"]) for row in rows]
    peak_errors = [float(row["peak_position_error_deg"]) for row in rows]
    iq_peaks = [float(row["max_abs_iq_a"]) for row in rows]
    voltage = [float(row["voltage_utilization_percent"]) for row in rows]

    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.text((55, 28), "图 7　复杂工况位置跟踪压力测试", font=FONT_TITLE, fill=TEXT)
    pass_count = sum(1 for row in rows if bool(row["pass"]))
    draw.text((57, 76), f"{len(rows)} 组工况；通过 {pass_count} 组；柱越低表示拟合/跟踪效果越好", font=FONT_BODY, fill=MUTED)
    _draw_bar_chart(image, (45, 120, 1555, 520), "峰值位置误差", "误差 / °", peak_errors, labels, PURPLE, limit=6.0)
    rms_errors = [float(row["rms_position_error_deg"]) for row in rows]
    _draw_bar_chart(image, (45, 555, 1555, 955), "RMS 位置误差", "误差 / °", rms_errors, labels, BLUE)
    image.save(PLOT_DIR / "07_stress_position_error.png", format="PNG", optimize=True)

    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.text((55, 28), "图 8　复杂工况电流与电压余量", font=FONT_TITLE, fill=TEXT)
    draw.text((57, 76), "观察是否接近电流限制或 SVPWM 电压边界", font=FONT_BODY, fill=MUTED)
    _draw_bar_chart(image, (45, 120, 1555, 520), "峰值 Iq", "电流 / A", iq_peaks, labels, ORANGE)
    _draw_bar_chart(image, (45, 555, 1555, 955), "电压利用率", "利用率 / %", voltage, labels, GREEN)
    image.save(PLOT_DIR / "08_stress_current_voltage.png", format="PNG", optimize=True)


def _plot_selected_timeseries(case_data: Dict[str, Tuple[Dict[str, np.ndarray], Dict[str, float]]]) -> None:
    for name, title in [("reverse_multi_step", "多段反向工况"), ("sine_load", "正弦扰动负载工况")]:
        result, summary = case_data[name]
        t = result["time_s"]
        position_error_deg = (result["position_ref_rev"] - result["position_rev"]) * 360.0
        sampled = _position_loop_sample_indices(result)
        image = Image.new("RGB", (WIDTH, 1120), BG)
        draw = ImageDraw.Draw(image)
        draw.text((55, 28), f"图 9　{title}跟踪结果" if name == "reverse_multi_step" else f"图 10　{title}跟踪结果", font=FONT_TITLE, fill=TEXT)
        draw.text(
            (57, 76),
            f"峰值误差 {summary['position_peak_error_deg']:.2f}°；RMS 误差 {summary['position_rms_error_deg']:.2f}°；峰值 Iq {summary['max_abs_iq_a']:.2f} A；误差按位置环采样显示",
            font=FONT_BODY,
            fill=MUTED,
        )
        _draw_line_chart(
            image,
            (45, 120, 1555, 465),
            "位置参考与实际位置",
            "单位: rev",
            t,
            [
                ("命令位置", result["command_position_rev"], RED),
                ("轨迹位置", result["position_ref_rev"], PURPLE),
                ("实际位置", result["position_rev"], BLUE),
            ],
        )
        _draw_line_chart(
            image,
            (45, 500, 1555, 790),
            "位置误差",
            "单位: deg",
            t[sampled],
            [
                ("位置环采样误差", position_error_deg[sampled], PURPLE),
            ],
        )
        _draw_line_chart(
            image,
            (45, 825, 1555, 1090),
            "负载扰动与观测",
            "单位: N*m",
            t,
            [
                ("负载转矩", result["load_nm"], ORANGE),
                ("估计负载", result["load_estimate_nm"], GREEN),
            ],
        )
        image.save(PLOT_DIR / ("09_stress_reverse_tracking.png" if name == "reverse_multi_step" else "10_stress_sine_load_tracking.png"), format="PNG", optimize=True)


def run() -> Tuple[List[Dict[str, float | str | bool]], Dict[str, Tuple[Dict[str, np.ndarray], Dict[str, float]]]]:
    base_config = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
    rows: List[Dict[str, float | str | bool]] = []
    case_data: Dict[str, Tuple[Dict[str, np.ndarray], Dict[str, float]]] = {}
    STRESS_DIR.mkdir(parents=True, exist_ok=True)
    PLOT_DIR.mkdir(parents=True, exist_ok=True)

    for name, label, cfg in _case_configs(base_config):
        result, summary = simulate(cfg)
        case_data[name] = (result, summary)
        rows.append(_metrics_for(name, label, cfg, result, summary))

    _save_summary_csv(STRESS_DIR / "stress_summary.csv", rows)
    (STRESS_DIR / "stress_metrics.json").write_text(
        json.dumps(rows, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    _plot_stress_bars(rows)
    _plot_selected_timeseries(case_data)
    return rows, case_data


def main() -> None:
    rows, _ = run()
    print(f"Stress scenarios complete: {sum(1 for row in rows if row['pass'])}/{len(rows)} passed")
    for row in rows:
        print(
            f"{row['label']}: peak={float(row['peak_position_error_deg']):.3f} deg, "
            f"rms={float(row['rms_position_error_deg']):.3f} deg, "
            f"Iq={float(row['max_abs_iq_a']):.3f} A, "
            f"Vutil={float(row['voltage_utilization_percent']):.2f}%, "
            f"pass={row['pass']}"
        )
    print(f"Metrics: {STRESS_DIR / 'stress_metrics.json'}")


if __name__ == "__main__":
    main()
