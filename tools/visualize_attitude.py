#!/usr/bin/env python3
"""Convenience wrapper for the IMU attitude visualizer.

This repo keeps the actual tool under:
  apps/examples/uav_states_v1/tools/visualize_attitude.py

So you can run from the workspace root:
  python3 tools/visualize_attitude.py /dev/ttyUSB0 115200
"""

from __future__ import annotations

import runpy
from pathlib import Path


def main() -> None:
    root = Path(__file__).resolve().parent.parent
    target = root / "apps" / "examples" / "uav_states_v1" / "tools" / "visualize_attitude.py"

    if not target.exists():
        raise SystemExit(f"Error: tool not found: {target}")

    # Execute the target script as if it were run directly.
    runpy.run_path(str(target), run_name="__main__")


if __name__ == "__main__":
    main()
