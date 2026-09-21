"""模拟编码器和速度估计器。

真实控制器通常不能直接拿到“完美速度”，而是从编码器角度差分得到速度。
这个模块用于在离线仿真里加入量化和滤波影响，提前观察控制器鲁棒性。
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import random


@dataclass
class QuantizedEncoder:
    """把连续机械角度转换成编码器可读到的量化角度。"""

    counts_per_rev: int = 4096
    noise_std_rad: float = 0.0
    seed: int = 1

    def __post_init__(self) -> None:
        if self.counts_per_rev <= 0:
            raise ValueError("counts_per_rev must be positive")
        if self.noise_std_rad < 0.0:
            raise ValueError("noise_std_rad must be non-negative")
        self._rng = random.Random(self.seed)

    @property
    def resolution_rad(self) -> float:
        return 2.0 * math.pi / self.counts_per_rev

    def measure(self, theta_rad: float) -> float:
        measured = round(theta_rad / self.resolution_rad) * self.resolution_rad
        if self.noise_std_rad > 0.0:
            measured += self._rng.gauss(0.0, self.noise_std_rad)
        return measured


@dataclass
class LowPassVelocityEstimator:
    """由角度差分得到速度，并用一阶低通滤波抑制量化噪声。"""

    tau_s: float = 0.002

    def __post_init__(self) -> None:
        if self.tau_s <= 0.0:
            raise ValueError("tau_s must be positive")
        self._last_theta: float | None = None
        self.velocity_rad_s = 0.0

    def reset(self) -> None:
        self._last_theta = None
        self.velocity_rad_s = 0.0

    def update(self, measured_theta_rad: float, dt_s: float) -> float:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if self._last_theta is None:
            self._last_theta = measured_theta_rad
            return self.velocity_rad_s

        raw_velocity = (measured_theta_rad - self._last_theta) / dt_s
        self._last_theta = measured_theta_rad
        alpha = dt_s / (self.tau_s + dt_s)
        self.velocity_rad_s += alpha * (raw_velocity - self.velocity_rad_s)
        return self.velocity_rad_s


@dataclass
class PLLVelocityEstimator:
    """简化 PLL 速度估计器，可作为低通差分以外的可选方案。"""

    kp: float = 400.0
    ki: float = 40000.0

    def __post_init__(self) -> None:
        if self.kp <= 0.0 or self.ki <= 0.0:
            raise ValueError("PLL gains must be positive")
        self.position_rad = 0.0
        self.velocity_rad_s = 0.0
        self._initialized = False

    def reset(self) -> None:
        self.position_rad = 0.0
        self.velocity_rad_s = 0.0
        self._initialized = False

    def update(self, measured_theta_rad: float, dt_s: float) -> float:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if not self._initialized:
            self.position_rad = measured_theta_rad
            self._initialized = True
            return self.velocity_rad_s

        error = measured_theta_rad - self.position_rad
        self.position_rad += (self.velocity_rad_s + self.kp * error) * dt_s
        self.velocity_rad_s += self.ki * error * dt_s
        return self.velocity_rad_s
