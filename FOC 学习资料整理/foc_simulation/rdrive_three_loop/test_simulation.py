import json
import unittest
from pathlib import Path

try:
    from .simulate import simulate
except ImportError:  # Allows running this file directly from rdrive_three_loop/.
    from simulate import simulate


class ThreeLoopSimulationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        config_path = Path(__file__).with_name("config.json")
        cls.config = json.loads(config_path.read_text(encoding="utf-8"))
        cls.result, cls.summary = simulate(cls.config)

    def test_reaches_position_target(self):
        self.assertLess(abs(self.summary["final_position_error_rev"]), 0.005)

    def test_recovers_from_load_step(self):
        self.assertLess(self.summary["post_load_recovery_error_rev"], 0.008)

    def test_motion_profile_does_not_overshoot(self):
        self.assertLess(self.summary["reference_overshoot_deg"], 1e-6)

    def test_disturbance_transients_are_reduced(self):
        self.assertLess(self.summary["load_peak_error_deg"], 2.0)
        self.assertLess(self.summary["unload_peak_error_deg"], 2.0)

    def test_current_limit_is_respected(self):
        limit = self.config["inverter"]["current_limit_a"]
        self.assertLessEqual(self.summary["max_abs_iq_a"], limit * 1.03)

    def test_voltage_limit_is_respected(self):
        self.assertLessEqual(
            self.summary["max_voltage_vector_v"],
            self.summary["allowed_voltage_vector_v"] + 1e-9,
        )

    def test_simulation_remains_finite(self):
        for values in self.result.values():
            self.assertTrue((abs(values) < 1e9).all())


if __name__ == "__main__":
    unittest.main()
