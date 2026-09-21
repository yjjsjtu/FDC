#!/usr/bin/env python3
"""Render standalone PNG figures from the three-loop simulation CSV."""

import json
import math
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image, ImageDraw, ImageFont


ROOT = Path(__file__).resolve().parent
OUTPUT = ROOT / "output"
PLOT_DIR = OUTPUT / "plots"

WIDTH = 1600
HEIGHT = 1000
BG = "#f5f7fb"
PANEL_BG = "#ffffff"
GRID = "#dce3ed"
AXIS = "#64748b"
TEXT = "#172033"
MUTED = "#64748b"
BLUE = "#2563eb"
RED = "#dc2626"
GREEN = "#16a34a"
PURPLE = "#7c3aed"
ORANGE = "#ea580c"
CYAN = "#0891b2"


def load_font(size: int, bold: bool = False):
    regular_candidates = [
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"),
        Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
        Path("/usr/share/fonts/truetype/arphic/uming.ttc"),
        Path("C:/Windows/Fonts/msyh.ttc"),
        Path("C:/Windows/Fonts/simhei.ttf"),
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
        Path("C:/Windows/Fonts/arial.ttf"),
    ]
    bold_candidates = [
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
        Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"),
        Path("/usr/share/fonts/truetype/droid/DroidSansFallbackFull.ttf"),
        Path("/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"),
        Path("C:/Windows/Fonts/msyhbd.ttc"),
        Path("C:/Windows/Fonts/simhei.ttf"),
        Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"),
        Path("C:/Windows/Fonts/arialbd.ttf"),
    ]
    candidates = bold_candidates if bold else regular_candidates
    for candidate in candidates:
        if candidate.exists():
            return ImageFont.truetype(str(candidate), size=size)
    return ImageFont.load_default()


FONT_TITLE = load_font(34, True)
FONT_PANEL = load_font(23, True)
FONT_BODY = load_font(18)
FONT_SMALL = load_font(15)


def nice_number(value: float) -> str:
    magnitude = abs(value)
    if magnitude >= 1000 or (magnitude != 0 and magnitude < 0.001):
        return f"{value:.2e}"
    if magnitude >= 10:
        return f"{value:.1f}"
    if magnitude >= 1:
        return f"{value:.2f}"
    return f"{value:.4f}"


def dashed_line(draw: ImageDraw.ImageDraw, xy: Tuple[float, float, float, float], fill: str,
                width: int = 2, dash: int = 10) -> None:
    x1, y1, x2, y2 = xy
    length = math.hypot(x2 - x1, y2 - y1)
    if length == 0:
        return
    dx = (x2 - x1) / length
    dy = (y2 - y1) / length
    cursor = 0.0
    while cursor < length:
        end = min(cursor + dash, length)
        draw.line((x1 + dx * cursor, y1 + dy * cursor,
                   x1 + dx * end, y1 + dy * end), fill=fill, width=width)
        cursor += 2 * dash


def panel(
    image: Image.Image,
    bounds: Tuple[int, int, int, int],
    time: np.ndarray,
    series: Sequence[Tuple[str, np.ndarray, str]],
    title: str,
    ylabel: str,
    shade: Optional[Tuple[float, float, str]] = None,
    events: Sequence[Tuple[float, str]] = (),
    y_range: Optional[Tuple[float, float]] = None,
    hlines: Sequence[Tuple[float, str, str]] = (),
) -> None:
    draw = ImageDraw.Draw(image)
    left, top, right, bottom = bounds
    draw.rounded_rectangle(bounds, radius=18, fill=PANEL_BG, outline="#e2e8f0", width=2)
    draw.text((left + 26, top + 18), title, font=FONT_PANEL, fill=TEXT)

    plot_left = left + 118
    plot_right = right - 32
    plot_top = top + 68
    plot_bottom = bottom - 64
    plot_w = plot_right - plot_left
    plot_h = plot_bottom - plot_top

    all_y = np.concatenate([values[np.isfinite(values)] for _, values, _ in series])
    if y_range is None:
        ymin = float(np.min(all_y))
        ymax = float(np.max(all_y))
        span = ymax - ymin
        margin = 0.09 * (span if span > 1e-12 else max(1.0, abs(ymax)))
        ymin -= margin
        ymax += margin
    else:
        ymin, ymax = y_range
    xmin = float(time[0])
    xmax = float(time[-1])

    def px(x: float) -> float:
        return plot_left + (x - xmin) / (xmax - xmin) * plot_w

    def py(y: float) -> float:
        return plot_bottom - (y - ymin) / (ymax - ymin) * plot_h

    if shade:
        start, end, label = shade
        sx1 = max(plot_left, px(start))
        sx2 = min(plot_right, px(end))
        draw.rectangle((sx1, plot_top, sx2, plot_bottom), fill="#fff1e6")
        draw.text((sx1 + 8, plot_top + 6), label, font=FONT_SMALL, fill=ORANGE)

    for tick in range(6):
        y = plot_top + tick * plot_h / 5
        value = ymax - tick * (ymax - ymin) / 5
        draw.line((plot_left, y, plot_right, y), fill=GRID, width=1)
        label = nice_number(value)
        tw = draw.textbbox((0, 0), label, font=FONT_SMALL)[2]
        draw.text((plot_left - tw - 10, y - 9), label, font=FONT_SMALL, fill=MUTED)
    for tick in range(7):
        x = plot_left + tick * plot_w / 6
        value = xmin + tick * (xmax - xmin) / 6
        draw.line((x, plot_top, x, plot_bottom), fill=GRID, width=1)
        label = f"{value:.2f}"
        tw = draw.textbbox((0, 0), label, font=FONT_SMALL)[2]
        draw.text((x - tw / 2, plot_bottom + 10), label, font=FONT_SMALL, fill=MUTED)

    draw.line((plot_left, plot_top, plot_left, plot_bottom), fill=AXIS, width=2)
    draw.line((plot_left, plot_bottom, plot_right, plot_bottom), fill=AXIS, width=2)
    draw.text((plot_left, plot_top - 26), ylabel, font=FONT_SMALL, fill=MUTED)
    time_label = "时间 / s"
    tw = draw.textbbox((0, 0), time_label, font=FONT_SMALL)[2]
    draw.text(((plot_left + plot_right - tw) / 2, bottom - 32), time_label, font=FONT_SMALL, fill=MUTED)

    for event_time, event_label in events:
        x = px(event_time)
        dashed_line(draw, (x, plot_top, x, plot_bottom), "#94a3b8", 2, 7)
        draw.text((x + 6, plot_bottom - 24), event_label, font=FONT_SMALL, fill=MUTED)

    for value, color, label in hlines:
        if ymin <= value <= ymax:
            y = py(value)
            dashed_line(draw, (plot_left, y, plot_right, y), color, 2, 9)
            draw.text((plot_right - 150, y - 20), label, font=FONT_SMALL, fill=color)

    stride = max(1, len(time) // 2200)
    sampled_t = time[::stride]
    legend_x = plot_left + 12
    for name, values, color in series:
        sampled_y = values[::stride]
        points = [(px(float(x)), py(float(y))) for x, y in zip(sampled_t, sampled_y)
                  if math.isfinite(float(y))]
        if len(points) >= 2:
            draw.line(points, fill=color, width=3, joint="curve")
        draw.line((legend_x, plot_top + 20, legend_x + 30, plot_top + 20), fill=color, width=4)
        draw.text((legend_x + 38, plot_top + 9), name, font=FONT_SMALL, fill=TEXT)
        legend_x += 145


def position_loop_sample_indices(time: np.ndarray, reference: np.ndarray) -> np.ndarray:
    """Pick one log sample per position-loop update for readable error plots."""
    if len(time) < 3:
        return np.arange(len(time))
    log_dt = float(np.median(np.diff(time)))
    changed = np.flatnonzero(np.abs(np.diff(reference)) > 1.0e-12) + 1
    if len(changed) >= 3:
        stride = int(round(float(np.median(np.diff(changed)))))
    else:
        stride = int(round(0.001 / max(log_dt, 1.0e-12)))
    stride = max(1, stride)
    indices = np.arange(0, len(time), stride, dtype=int)
    if indices[-1] != len(time) - 1:
        indices = np.append(indices, len(time) - 1)
    return indices


def new_figure(title: str, subtitle: str) -> Image.Image:
    image = Image.new("RGB", (WIDTH, HEIGHT), BG)
    draw = ImageDraw.Draw(image)
    draw.text((55, 28), title, font=FONT_TITLE, fill=TEXT)
    draw.text((57, 73), subtitle, font=FONT_BODY, fill=MUTED)
    return image


def save_figure(image: Image.Image, name: str) -> None:
    PLOT_DIR.mkdir(parents=True, exist_ok=True)
    image.save(PLOT_DIR / name, format="PNG", optimize=True)


def main() -> None:
    data = np.genfromtxt(OUTPUT / "simulation.csv", delimiter=",", names=True, encoding="utf-8-sig")
    config = json.loads((ROOT / "config.json").read_text(encoding="utf-8"))
    summary = json.loads((OUTPUT / "summary.json").read_text(encoding="utf-8"))
    t = data["time_s"]
    scenario = config["scenario"]
    load_shade = (scenario["load_start_s"], scenario["load_end_s"], "0.08 N·m 负载区间")
    event = ((scenario["position_step_time_s"], "位置指令"),)

    position_error_deg = (data["position_ref_rev"] - data["position_rev"]) * 360.0
    position_error_indices = position_loop_sample_indices(t, data["position_ref_rev"])
    position_series = [
        ("轨迹位置", data["position_ref_rev"], PURPLE),
        ("实际位置", data["position_rev"], BLUE),
    ]
    if "command_position_rev" in data.dtype.names:
        position_series.insert(0, ("命令位置", data["command_position_rev"], RED))
    image = new_figure(
        "图 1　位置环跟踪结果",
        f"目标 0.25 rev；最终误差 {summary['final_position_error_rev']:.3e} rev；"
        f"超调 {summary['overshoot_rev']*360:.2f}°；误差按位置环采样显示",
    )
    panel(image, (45, 115, 1555, 525), t,
          position_series,
          "位置响应", "单位: rev", load_shade, event)
    panel(image, (45, 550, 1555, 955), t[position_error_indices],
          (("位置环采样误差", position_error_deg[position_error_indices], PURPLE),),
          "位置误差", "单位: deg", load_shade, event, hlines=((0.0, AXIS, "零误差"),))
    save_figure(image, "01_position_tracking.png")

    speed_error = data["speed_ref_rev_s"] - data["speed_rev_s"]
    image = new_figure(
        "图 2　速度环跟踪结果",
        f"速度环 3 kHz；峰值实际速度 {summary['max_abs_speed_rev_s']:.3f} rev/s",
    )
    panel(image, (45, 115, 1555, 525), t,
          (("速度给定", data["speed_ref_rev_s"], RED), ("实际速度", data["speed_rev_s"], BLUE)),
          "速度响应", "速度 / (rev/s)", load_shade, event)
    panel(image, (45, 550, 1555, 955), t,
          (("速度误差", speed_error, ORANGE),),
          "速度误差", "误差 / (rev/s)", load_shade, event, hlines=((0.0, AXIS, "零误差"),))
    save_figure(image, "02_speed_tracking.png")

    iq_error = data["iq_ref_a"] - data["iq_a"]
    id_error = data["id_ref_a"] - data["id_a"]
    image = new_figure(
        "图 3　D/Q 电流内环",
        f"电流环 30 kHz；峰值 |Iq|={summary['max_abs_iq_a']:.3f} A，峰值 |Id|={summary['max_abs_id_a']:.3e} A",
    )
    panel(image, (45, 115, 1555, 525), t,
          (("Iq 给定", data["iq_ref_a"], RED), ("Iq 实际", data["iq_a"], BLUE), ("Id 实际", data["id_a"], GREEN)),
          "电流跟踪", "电流 / A", load_shade, event)
    panel(image, (45, 550, 1555, 955), t,
          (("Iq 误差", iq_error, PURPLE), ("Id 误差", id_error, GREEN)),
          "电流误差", "误差 / A", load_shade, event, hlines=((0.0, AXIS, "零误差"),))
    save_figure(image, "03_dq_current.png")

    allowed = float(summary["allowed_voltage_vector_v"])
    image = new_figure(
        "图 4　D/Q 电压与 SVPWM",
        f"24 V 母线；允许电压矢量 {allowed:.3f} V；饱和占比 {summary['voltage_saturation_fraction']*100:.2f}%",
    )
    panel(image, (45, 115, 1555, 525), t,
          (("Vd", data["vd_v"], GREEN), ("Vq", data["vq_v"], PURPLE)),
          "D/Q 电压输出", "电压 / V", load_shade, event,
          y_range=(-allowed * 1.08, allowed * 1.08),
          hlines=((allowed, RED, "+电压边界"), (-allowed, RED, "-电压边界")))
    panel(image, (45, 550, 1555, 955), t,
          (("Duty A", data["duty_a"], RED), ("Duty B", data["duty_b"], GREEN), ("Duty C", data["duty_c"], BLUE)),
          "三相 SVPWM 占空比", "占空比", load_shade, event, y_range=(0.0, 1.0))
    save_figure(image, "04_voltage_and_svpwm.png")

    image = new_figure(
        "图 5　负载扰动与恢复",
        f"0.55～0.80 s 加载 0.08 N·m；撤载后最大位置误差 {summary['post_load_recovery_error_rev']*360:.3f}°",
    )
    panel(image, (45, 115, 1555, 525), t,
          (("电磁转矩", data["torque_nm"], BLUE), ("负载转矩", data["load_nm"], ORANGE),
           ("估计负载", data["load_estimate_nm"], GREEN), ("运动前馈", data["motion_feedforward_nm"], PURPLE)),
          "电磁转矩与外部负载", "转矩 / N·m", load_shade, event)
    panel(image, (45, 550, 1555, 955), t[position_error_indices],
          (("位置环采样误差", position_error_deg[position_error_indices], PURPLE),),
          "扰动期间的位置误差", "单位: deg", load_shade, event,
          hlines=((0.0, AXIS, "零线"),))
    save_figure(image, "05_load_disturbance.png")

    image = new_figure(
        "图 6　惯性与摩擦前馈补偿",
        f"峰值惯性前馈 {summary['max_abs_inertia_feedforward_nm']:.4f} N·m；"
        f"峰值摩擦前馈 {summary['max_abs_friction_feedforward_nm']:.4f} N·m",
    )
    panel(image, (45, 115, 1555, 525), t,
          (("惯性前馈", data["inertia_feedforward_nm"], BLUE),
           ("摩擦前馈", data["friction_feedforward_nm"], GREEN),
           ("合计前馈", data["motion_feedforward_nm"], PURPLE)),
          "运动模型前馈转矩", "转矩 / N·m", load_shade, event,
          hlines=((0.0, AXIS, "零线"),))
    motion_iq = data["motion_feedforward_nm"] / config["motor"]["flux_linkage_wb"] / (1.5 * config["motor"]["pole_pairs"])
    panel(image, (45, 550, 1555, 955), t,
          (("Iq 给定", data["iq_ref_a"], RED), ("运动前馈 Iq", motion_iq, CYAN), ("Iq 实际", data["iq_a"], BLUE)),
          "前馈折算到 q 轴电流", "电流 / A", load_shade, event,
          hlines=((0.0, AXIS, "零线"),))
    save_figure(image, "06_friction_inertia_feedforward.png")

    for path in sorted(PLOT_DIR.glob("*.png")):
        print(path)


if __name__ == "__main__":
    main()
