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

    def test_migrated_outputs_are_finite(self) -> None:
        for values in self.result.values():
            self.assertTrue((abs(values) < 1.0e9).all())


if __name__ == "__main__":
    unittest.main()
