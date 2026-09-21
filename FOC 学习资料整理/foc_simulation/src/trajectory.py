"""速度参考轨迹规划。

仿真中直接给速度阶跃会等价于要求电机瞬间产生无限加速度，实际硬件做不到，
也会让电流和电压指令出现不真实尖峰。这里把“用户命令速度”变成受限的
position/velocity/acceleration 参考量，更接近真实伺服控制器里的轨迹层。
"""

from __future__ import annotations

from dataclasses import dataclass

from .controllers import clamp


@dataclass(frozen=True)
class ReferenceState:
    """轨迹规划器每个控制周期输出的参考状态。"""

    position_rad: float
    velocity_rad_s: float
    acceleration_rad_s2: float


@dataclass
class VelocityTrajectoryPlanner:
    """把速度命令转换成加速度/加加速度受限的速度参考。

    max_accel_rad_s2 限制速度斜率，相当于梯形速度规划。
    max_jerk_rad_s3 限制加速度变化率；设为 None 时就是普通梯形规划，
    设为正数时会得到更平滑的 S 曲线规划。
    """

    max_accel_rad_s2: float
    max_jerk_rad_s3: float | None = None
    position_rad: float = 0.0
    velocity_rad_s: float = 0.0
    acceleration_rad_s2: float = 0.0

    def reset(self) -> None:
        self.position_rad = 0.0
        self.velocity_rad_s = 0.0
        self.acceleration_rad_s2 = 0.0

    def update(self, command_velocity_rad_s: float, dt_s: float) -> ReferenceState:
        if dt_s <= 0.0:
            raise ValueError("dt_s must be positive")
        if self.max_accel_rad_s2 <= 0.0:
            raise ValueError("max_accel_rad_s2 must be positive")
        if self.max_jerk_rad_s3 is not None and self.max_jerk_rad_s3 <= 0.0:
            raise ValueError("max_jerk_rad_s3 must be positive or None")

        old_velocity = self.velocity_rad_s
        error = command_velocity_rad_s - old_velocity
        desired_accel = clamp(error / dt_s, -self.max_accel_rad_s2, self.max_accel_rad_s2)

        # S 曲线收尾时要提前把加速度拉回 0，否则到达目标速度那一刻会出现
        # “速度正好命中，但加速度突然归零”的 jerk 尖峰。
        if self.max_jerk_rad_s3 is not None and abs(error) > 1.0e-12:
            direction = 1.0 if error > 0.0 else -1.0
            accel_along_error = self.acceleration_rad_s2 * direction
            stopping_delta = self.acceleration_rad_s2 * self.acceleration_rad_s2 / (
                2.0 * self.max_jerk_rad_s3
            )
            if accel_along_error > 0.0 and abs(error) <= stopping_delta + abs(self.acceleration_rad_s2) * dt_s:
                desired_accel = 0.0

        if self.max_jerk_rad_s3 is None:
            next_accel = desired_accel
            accel_step = float("inf")
        else:
            accel_step = self.max_jerk_rad_s3 * dt_s
            next_accel = self.acceleration_rad_s2 + clamp(
                desired_accel - self.acceleration_rad_s2,
                -accel_step,
                accel_step,
            )

        next_velocity = old_velocity + next_accel * dt_s
        # 如果一步积分越过了命令值，直接落到命令值并清零加速度，避免零点附近抖动。
        if (
            error != 0.0
            and (command_velocity_rad_s - next_velocity) * error <= 0.0
            and abs(next_accel) <= accel_step + 1.0e-12
            and (
                self.max_jerk_rad_s3 is None
                or abs(error) >= abs(next_accel) * dt_s - 1.0e-12
            )
        ):
            next_velocity = command_velocity_rad_s
            next_accel = 0.0

        self.velocity_rad_s = next_velocity
        self.acceleration_rad_s2 = next_accel
        self.position_rad += next_velocity * dt_s
        return ReferenceState(self.position_rad, self.velocity_rad_s, self.acceleration_rad_s2)
