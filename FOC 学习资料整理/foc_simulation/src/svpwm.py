"""SVPWM 空间矢量 PWM 计算。

输入是定子坐标系下的目标电压矢量 Ualpha/Ubeta，输出是三相桥
A/B/C 的占空比。真实硬件中这些占空比会交给 MCU 定时器生成 PWM。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Tuple

from .foc_math import TWO_PI, SQRT3, normalize_angle, phase_to_alpha_beta


@dataclass(frozen=True)
class SVPWMResult:
    # sector: 当前电压矢量位于 6 个 60 度扇区中的哪一个。
    # t1/t2: 相邻两个有效矢量在一个 PWM 周期内的作用时间比例。
    # t0: 零矢量作用时间比例。
    # duty_a/b/c: 最终三相占空比。
    sector: int
    t1: float
    t2: float
    t0: float
    duty_a: float
    duty_b: float
    duty_c: float
    modulation: float

    def duties(self) -> Tuple[float, float, float]:
        return self.duty_a, self.duty_b, self.duty_c


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def calculate_svpwm(
    u_alpha: float,
    u_beta: float,
    v_bus: float,
    centered: bool = True,
) -> SVPWMResult:
    """计算 SVPWM 扇区、矢量作用时间和三相占空比。

    modulation 是电压矢量幅值除以母线电压后的归一化结果，并限制在
    SVPWM 线性调制区域，避免占空比越界。
    """
    if v_bus <= 0.0:
        raise ValueError("v_bus must be positive")

    # atan2 得到电压矢量方向；每 60 度对应一个扇区。
    angle = float(normalize_angle(math.atan2(u_beta, u_alpha)))
    sector = int(math.floor(angle / (math.pi / 3.0))) + 1
    sector = max(1, min(6, sector))

    # 电压矢量幅值越大，T1/T2 越长；幅值被限制后就不会超过母线能力。
    magnitude = math.hypot(u_alpha, u_beta)
    modulation = _clamp(magnitude / v_bus, 0.0, 0.999 / SQRT3)

    # T1 和 T2 是目标矢量在当前扇区两个相邻有效矢量上的投影时间。
    t1 = SQRT3 * math.sin(sector * math.pi / 3.0 - angle) * modulation
    t2 = SQRT3 * math.sin(angle - (sector - 1.0) * math.pi / 3.0) * modulation
    t1 = max(0.0, t1)
    t2 = max(0.0, t2)
    # centered=True 时把零矢量平均分到 PWM 周期两端，波形更对称。
    t0 = max(0.0, 1.0 - t1 - t2) if centered else 0.0

    half_zero = t0 / 2.0
    # 不同扇区下，T1/T2/T0 分配到 A/B/C 三相的顺序不同。
    if sector == 1:
        ta, tb, tc = t1 + t2 + half_zero, t2 + half_zero, half_zero
    elif sector == 2:
        ta, tb, tc = t1 + half_zero, t1 + t2 + half_zero, half_zero
    elif sector == 3:
        ta, tb, tc = half_zero, t1 + t2 + half_zero, t2 + half_zero
    elif sector == 4:
        ta, tb, tc = half_zero, t1 + half_zero, t1 + t2 + half_zero
    elif sector == 5:
        ta, tb, tc = t2 + half_zero, half_zero, t1 + t2 + half_zero
    else:
        ta, tb, tc = t1 + t2 + half_zero, half_zero, t1 + half_zero

    return SVPWMResult(
        sector=sector,
        t1=t1,
        t2=t2,
        t0=t0,
        duty_a=_clamp(ta, 0.0, 1.0),
        duty_b=_clamp(tb, 0.0, 1.0),
        duty_c=_clamp(tc, 0.0, 1.0),
        modulation=modulation,
    )


def duties_to_alpha_beta(duty_a: float, duty_b: float, duty_c: float, v_bus: float) -> Tuple[float, float]:
    """把占空比换算回等效 alpha/beta 电压，用于离线仿真电机模型。"""
    return phase_to_alpha_beta(duty_a * v_bus, duty_b * v_bus, duty_c * v_bus)
