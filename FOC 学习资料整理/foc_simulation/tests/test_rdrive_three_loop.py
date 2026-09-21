from __future__ import annotations

import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdrive_three_loop.simulate import simulate


class RDriveThreeLoopMigrationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        config_path = ROOT / "rdrive_three_loop" / "config.json"
        cls.config = json.loads(config_path.read_text(encoding="utf-8"))
        cls.result, cls.summary = simulate(cls.config)

    def test_migrated_simulation_tracks_position(self) -> None:
        self.assertLess(abs(self.summary["final_position_error_rev"]), 0.005)
        self.assertLess(self.summary["post_load_recovery_error_rev"], 0.008)

    def test_migrated_limits_are_respected(self) -> None:
        current_limit = self.config["inverter"]["current_limit_a"]
        self.assertLessEqual(self.summary["max_abs_iq_a"], current_limit * 1.03)
        self.assertLessEqual(
            self.summary["max_voltage_vector_v"],
            self.summary["allowed_voltage_vector_v"] + 1.0e-9,
        )

    def test_migrated_motion_feedforward_is_logged(self) -> None:
        self.assertGreater(self.summary["max_abs_motion_feedforward_nm"], 0.0)
        self.assertIn("inertia_feedforward_nm", self.result)
        self.assertIn("friction_feedforward_nm", self.result)
        self.assertIn("command_position_rev", self.result)

    def test_migrated_outputs_are_finite(self) -> None:
        for values in self.result.values():
            self.assertTrue((abs(values) < 1.0e9).all())

    def test_migrated_simulation_supports_complex_scenarios(self) -> None:
        config = json.loads(json.dumps(self.config))
        config["simulation"]["duration_s"] = 0.55
        config["scenario"].clear()
        config["scenario"].update(
            {
                "target_events_rev": [[0.05, 0.08], [0.28, -0.04]],
                "load_events_nm": [[0.16, 0.24, 0.03], [0.36, 0.45, -0.02]],
                "load_sine_nm": {
                    "start_s": 0.12,
                    "end_s": 0.50,
                    "amplitude_nm": 0.01,
                    "frequency_hz": 5.0,
                },
            }
        )
        result, summary = simulate(config)
        self.assertIn("command_position_rev", result)
        self.assertLess(summary["position_peak_error_deg"], 8.0)
        self.assertLess(summary["voltage_saturation_fraction"], 0.01)
        self.assertLessEqual(
            summary["max_abs_iq_a"],
            config["inverter"]["current_limit_a"] * 1.03,
        )


if __name__ == "__main__":
    unittest.main()
