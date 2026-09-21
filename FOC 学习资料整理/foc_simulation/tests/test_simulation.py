from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.motor_model import MotorParams
from src.simulator import SimulationConfig, run_foc_simulation, run_position_foc_simulation
from src.tuning import tuned_gains_from_motor


class SimulationTests(unittest.TestCase):
    def test_velocity_step_converges_and_duties_are_valid(self) -> None:
        data = run_foc_simulation(
            lambda t: 8.0 if t > 0.05 else 0.0,
            config=SimulationConfig(duration_s=0.8, dt_s=5.0e-5),
        )
        final_error = abs(float(data["target_velocity_rad_s"][-1] - data["velocity_rad_s"][-1]))
        self.assertLess(final_error, 1.5)
        self.assertTrue(np.all(data["duty_a"] >= 0.0))
        self.assertTrue(np.all(data["duty_a"] <= 1.0))
        self.assertTrue(np.all(data["duty_b"] >= 0.0))
        self.assertTrue(np.all(data["duty_b"] <= 1.0))
        self.assertTrue(np.all(data["duty_c"] >= 0.0))
        self.assertTrue(np.all(data["duty_c"] <= 1.0))
        self.assertLess(float(np.mean(np.abs(data["id_a"][-2000:]))), 0.15)

    def test_position_three_loop_tracks_position_reference(self) -> None:
        params = MotorParams()
        current_gains, _ = tuned_gains_from_motor(params)

        def position_profile(time_s: float) -> float:
            return 1.0 if time_s > 0.1 else 0.0

        data = run_position_foc_simulation(
            position_profile,
            params,
            SimulationConfig(
                duration_s=1.0,
                position_kp=14.0,
                position_velocity_limit_rad_s=14.0,
                velocity_kp=0.075,
                velocity_ki=0.8,
                current_kp=current_gains.kp,
                current_ki=current_gains.ki,
                use_voltage_feedforward=True,
                current_back_calculation_gain=120.0,
            ),
        )
        final_error = abs(float(data["target_position_rad"][-1] - data["position_rad"][-1]))
        self.assertLess(final_error, 0.08)
        self.assertTrue(np.all(data["duty_a"] >= 0.0))
        self.assertTrue(np.all(data["duty_a"] <= 1.0))
        self.assertLess(float(np.mean(np.abs(data["id_a"][-2000:]))), 0.15)


if __name__ == "__main__":
    unittest.main()
