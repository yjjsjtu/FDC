from __future__ import annotations

from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.identification import identify_friction_inertia, identify_rl


class IdentificationTests(unittest.TestCase):
    def test_identify_rl_from_synthetic_step(self) -> None:
        resistance = 5.37
        inductance = 0.00326
        voltage = 3.0
        time = np.arange(0.0, 0.012, 2.0e-5)
        current = voltage / resistance * (1.0 - np.exp(-time * resistance / inductance))
        result = identify_rl({"time_s": time, "voltage_v": np.full_like(time, voltage), "current_a": current})
        self.assertLess(abs(result["resistance_ohm"] - resistance) / resistance, 0.03)
        self.assertLess(abs(result["inductance_h"] - inductance) / inductance, 0.08)

    def test_identify_friction_inertia_from_synthetic_data(self) -> None:
        inertia = 2.02e-5
        viscous = 2.0e-4
        coulomb = 0.015
        dt = 0.001
        time = np.arange(0.0, 8.0, dt)
        velocity = 8.0 * np.sin(2.0 * np.pi * time / 4.0) + 2.0 * np.sin(2.0 * np.pi * time / 1.2)
        accel = np.gradient(velocity, dt)
        torque = inertia * accel + viscous * velocity + coulomb * np.sign(velocity)
        result = identify_friction_inertia({"time_s": time, "velocity_rad_s": velocity, "torque_nm": torque})
        self.assertLess(abs(result["coulomb_friction_nm"] - coulomb) / coulomb, 0.08)
        self.assertLess(abs(result["viscous_nm_per_rad_s"] - viscous) / viscous, 0.10)
        self.assertLess(abs(result["inertia_kg_m2"] - inertia) / inertia, 0.10)


if __name__ == "__main__":
    unittest.main()

