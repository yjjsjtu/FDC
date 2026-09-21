"""FOC 仿真中使用的小型控制器模块。"""

from __future__ import annotations

from dataclasses import dataclass


def clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


@dataclass
class PIController:
    """带输出限幅和简单抗积分饱和的 PI 控制器。"""

    kp: float
    ki: float
    limit: float
    integrator_limit: float | None = None
    back_calculation_gain: float = 0.0

    def __post_init__(self) -> None:
        self.integrator = 0.0
        self._previous_integrator = 0.0

    def reset(self) -> None:
        self.integrator = 0.0
        self._previous_integrator = 0.0

    def rollback_integrator(self) -> None:
        """回退到上一次 update 前的积分值。

        单个 PI 输出没有饱和时，后面的电压矢量限幅仍可能整体缩小 Vd/Vq。
        这时调用本函数，相当于冻结本周期积分，避免积分项继续向饱和方向堆积。
        """
        self.integrator = self._previous_integrator

    def update(self, error: float, dt: float, feedforward: float = 0.0) -> float:
        if dt <= 0.0:
            raise ValueError("dt must be positive")
        if self.back_calculation_gain < 0.0:
            raise ValueError("back_calculation_gain must be non-negative")

        # 先尝试积分，但积分项本身也要限幅，否则长时间误差会把积分推得过大。
        integ_limit = self.integrator_limit if self.integrator_limit is not None else self.limit
        self._previous_integrator = self.integrator
        candidate_integrator = clamp(
            self.integrator + self.ki * error * dt,
            -integ_limit,
            integ_limit,
        )
        unclamped = self.kp * error + candidate_integrator + feedforward
        output = clamp(unclamped, -self.limit, self.limit)

        if self.back_calculation_gain > 0.0:
            corrected = candidate_integrator + self.back_calculation_gain * (output - unclamped) * dt
            self.integrator = clamp(corrected, -integ_limit, integ_limit)
            return output

        # 抗积分饱和：如果输出已经顶到上/下限，并且误差还在继续把它往外推，
        # 就暂时不接受新的积分，避免解除饱和后出现很大的过冲。
        saturated_high = unclamped > self.limit and error > 0.0
        saturated_low = unclamped < -self.limit and error < 0.0
        if not (saturated_high or saturated_low):
            self.integrator = candidate_integrator

        return output


@dataclass
class PController:
    """只带比例项的控制器，适合最外层位置环等简单场景。"""

    kp: float
    limit: float

    def update(self, error: float) -> float:
        return clamp(self.kp * error, -self.limit, self.limit)
