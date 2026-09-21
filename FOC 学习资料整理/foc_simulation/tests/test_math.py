from __future__ import annotations

import math
from pathlib import Path
import sys
import unittest

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from src.foc_math import clarke, inverse_clarke, inverse_park, normalize_angle, park
from src.svpwm import calculate_svpwm


class MathTests(unittest.TestCase):
    def test_clarke_inverse_clarke_roundtrip(self) -> None:
        a, b, c = 1.2, -0.7, -0.5
        alpha, beta = clarke(a, b, c)
        aa, bb, cc = inverse_clarke(alpha, beta)
        self.assertAlmostEqual(a, aa, places=9)
        self.assertAlmostEqual(b, bb, places=9)
        self.assertAlmostEqual(c, cc, places=9)

    def test_park_inverse_park_roundtrip(self) -> None:
        d, q, angle = 0.4, 1.7, 1.2
        alpha, beta = inverse_park(d, q, angle)
        dd, qq = park(alpha, beta, angle)
        self.assertAlmostEqual(d, dd, places=9)
        self.assertAlmostEqual(q, qq, places=9)

    def test_normalize_angle_range(self) -> None:
        values = normalize_angle(np.array([-7.0, -0.1, 0.0, 7.0, 99.0]))
        self.assertTrue(np.all(values >= 0.0))
        self.assertTrue(np.all(values < 2.0 * math.pi))

    def test_svpwm_duty_range_and_sector_coverage(self) -> None:
        sectors = set()
        for angle in np.linspace(0.0, 2.0 * math.pi, 241, endpoint=False):
            result = calculate_svpwm(6.0 * math.cos(angle), 6.0 * math.sin(angle), 24.0)
            sectors.add(result.sector)
            self.assertGreaterEqual(result.duty_a, 0.0)
            self.assertLessEqual(result.duty_a, 1.0)
            self.assertGreaterEqual(result.duty_b, 0.0)
            self.assertLessEqual(result.duty_b, 1.0)
            self.assertGreaterEqual(result.duty_c, 0.0)
            self.assertLessEqual(result.duty_c, 1.0)
        self.assertEqual(sectors, {1, 2, 3, 4, 5, 6})


if __name__ == "__main__":
    unittest.main()

