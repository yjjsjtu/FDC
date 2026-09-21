from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.compensation import control_performance_metrics
from src.motor_model import MotorParams
from src.simulator import SimulationConfig, run_foc_simulation
from src.trajectory import VelocityTrajectoryPlanner
from src.tuning import tuned_gains_from_motor


class ControlOptimizationTests(unittest.TestCase):
    def test_trajectory_limits_acceleration_and_jerk(self) -> None:
        planner = VelocityTrajectoryPlanner(max_accel_rad_s2=20.0, max_jerk_rad_s3=200.0)
        dt = 0.01
        states = [planner.update(10.0, dt) for _ in range(80)]
        accelerations = np.asarray([state.acceleration_rad_s2 for state in states])
        jerks = np.diff(accelerations) / dt
        self.assertLessEqual(float(np.max(np.abs(accelerations))), 20.0 + 1.0e-9)
        self.assertLessEqual(float(np.max(np.abs(jerks))), 200.0 + 1.0e-9)
        self.assertGreater(states[-1].position_rad, 0.0)

    def test_tuned_gains_are_positive(self) -> None:
        current_gains, velocity_gains = tuned_gains_from_motor(MotorParams())
        self.assertGreater(current_gains.kp, 0.0)
        self.assertGreater(current_gains.ki, 0.0)
        self.assertGreater(velocity_gains.kp, 0.0)
        self.assertGreater(velocity_gains.ki, 0.0)

    def test_optimized_control_reduces_error_and_iq_peak(self) -> None:
        params = MotorParams()
        current_gains, velocity_gains = tuned_gains_from_motor(params)

        def profile(time_s: float) -> float:
            if time_s < 0.05:
                return 0.0
            if time_s < 0.45:
                return 10.0
            return -6.0

        baseline = run_foc_simulation(profile, params, SimulationConfig(duration_s=0.9))
        optimized = run_foc_simulation(
            profile,
            params,
            SimulationConfig(
                duration_s=0.9,
                velocity_kp=velocity_gains.kp,
                velocity_ki=velocity_gains.ki,
                current_kp=current_gains.kp,
                current_ki=current_gains.ki,
                use_trajectory=True,
                max_accel_rad_s2=45.0,
                max_jerk_rad_s3=650.0,
                use_voltage_feedforward=True,
                current_back_calculation_gain=120.0,
                velocity_estimator="lpf",
                encoder_noise_std_rad=0.0004,
                velocity_filter_tau_s=0.0025,
            ),
        )
        baseline_metrics = control_performance_metrics(baseline)
        optimized_metrics = control_performance_metrics(optimized)

        self.assertLess(optimized_metrics["velocity_rms_rad_s"], baseline_metrics["velocity_rms_rad_s"])
        self.assertLess(optimized_metrics["iq_peak_abs_a"], baseline_metrics["iq_peak_abs_a"])
        self.assertTrue(np.all(optimized["duty_a"] >= 0.0))
        self.assertTrue(np.all(optimized["duty_a"] <= 1.0))


if __name__ == "__main__":
    unittest.main()
