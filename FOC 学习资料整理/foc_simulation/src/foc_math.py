"""FOC 中使用的坐标变换。

符号约定和 SimpleFOC 常用写法保持一致：
    Ualpha = cos(theta) * Ud - sin(theta) * Uq
    Ubeta  = sin(theta) * Ud + cos(theta) * Uq
"""

from __future__ import annotations

import math
from typing import Tuple

import numpy as np

TWO_PI = 2.0 * math.pi
SQRT3 = math.sqrt(3.0)
SQRT3_BY_2 = SQRT3 / 2.0


def normalize_angle(angle: float | np.ndarray) -> float | np.ndarray:
    """把角度归一化到 [0, 2*pi)，避免角度无限增大导致计算不稳定。"""
    return np.mod(angle, TWO_PI)


def electrical_angle(
    mechanical_angle_rad: float,
    pole_pairs: int,
    zero_electric_angle_rad: float = 0.0,
) -> float:
    """把编码器读到的机械角度换算成 FOC 使用的电角度。"""
    return float(normalize_angle(pole_pairs * mechanical_angle_rad - zero_electric_angle_rad))


def clarke(phase_a: float, phase_b: float, phase_c: float) -> Tuple[float, float]:
    """Clarke 变换：三相静止坐标 A/B/C -> 二相静止坐标 alpha/beta。"""
    # 平衡三相里 A+B+C=0，因此用 A 相作为 alpha，B 相推导 beta。
    alpha = phase_a
    beta = (phase_a + 2.0 * phase_b) / SQRT3
    return alpha, beta


def inverse_clarke(alpha: float, beta: float) -> Tuple[float, float, float]:
    """反 Clarke 变换：alpha/beta -> 平衡三相 A/B/C。"""
    phase_a = alpha
    phase_b = -0.5 * alpha + SQRT3_BY_2 * beta
    phase_c = -0.5 * alpha - SQRT3_BY_2 * beta
    return phase_a, phase_b, phase_c


def park(alpha: float, beta: float, angle_el_rad: float) -> Tuple[float, float]:
    """Park 变换：静止坐标 alpha/beta -> 跟随转子旋转的 d/q 坐标。"""
    ca = math.cos(angle_el_rad)
    sa = math.sin(angle_el_rad)
    # d 轴和转子磁链方向对齐，q 轴和 d 轴正交，主要负责产生转矩。
    d_axis = ca * alpha + sa * beta
    q_axis = -sa * alpha + ca * beta
    return d_axis, q_axis


def inverse_park(d_axis: float, q_axis: float, angle_el_rad: float) -> Tuple[float, float]:
    """反 Park 变换：控制器算出的 Ud/Uq -> 定子侧 Ualpha/Ubeta。"""
    ca = math.cos(angle_el_rad)
    sa = math.sin(angle_el_rad)
    alpha = ca * d_axis - sa * q_axis
    beta = sa * d_axis + ca * q_axis
    return alpha, beta


def phase_to_alpha_beta(phase_a: float, phase_b: float, phase_c: float) -> Tuple[float, float]:
    """去掉三相公共模电压，再转换成 alpha/beta 等效电压。"""
    # SVPWM 输出的是三相对地占空比，公共模不产生线电压，也不影响电机相间电压。
    common = (phase_a + phase_b + phase_c) / 3.0
    return clarke(phase_a - common, phase_b - common, phase_c - common)


def dq_to_phase(d_axis: float, q_axis: float, angle_el_rad: float) -> Tuple[float, float, float]:
    """把 d/q 坐标量转换成三相量，常用于观察相电流或相电压。"""
    alpha, beta = inverse_park(d_axis, q_axis, angle_el_rad)
    return inverse_clarke(alpha, beta)
