#!/usr/bin/env python3
"""运行 RDrive/moteus 三环 FOC 的复杂工况压力测试。"""

from __future__ import annotations

from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdrive_three_loop.stress_scenarios import main


if __name__ == "__main__":
    main()
