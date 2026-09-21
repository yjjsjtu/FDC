#!/usr/bin/env python3
"""运行迁移后的 RDrive/moteus 风格三环 FOC 仿真。"""

from __future__ import annotations

import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from rdrive_three_loop.simulate import simulate, write_csv, write_svg


def main() -> None:
    sim_root = ROOT / "rdrive_three_loop"
    config_path = sim_root / "config.json"
    output_dir = sim_root / "output"
    output_dir.mkdir(parents=True, exist_ok=True)

    config = json.loads(config_path.read_text(encoding="utf-8"))
    result, summary = simulate(config)
    write_csv(output_dir / "simulation.csv", result)
    (output_dir / "summary.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    write_svg(output_dir / "three_loop_response.svg", result, summary)

    try:
        from rdrive_three_loop import plot_results

        plot_results.main()
        plot_note = "PNG plots regenerated"
    except ModuleNotFoundError as exc:
        if exc.name != "PIL":
            raise
        plot_note = "PNG plots skipped because Pillow is not installed"

    print("RDrive three-loop simulation complete")
    print(f"final_position_error_rev={summary['final_position_error_rev']:.6g}")
    print(f"max_abs_iq_a={summary['max_abs_iq_a']:.6g}")
    print(f"voltage_saturation_fraction={summary['voltage_saturation_fraction']:.6g}")
    print(f"Output directory: {output_dir}")
    print(plot_note)


if __name__ == "__main__":
    main()
