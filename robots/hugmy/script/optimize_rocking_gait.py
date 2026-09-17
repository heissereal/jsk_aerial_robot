#!/usr/bin/env python3
"""Run the common MuJoCo optimizer with the alternating-pivot gait config."""

import os
from pathlib import Path
import sys


SCRIPT = Path(__file__).with_name("optimize_inchworm_gait.py")
CONFIG = Path(__file__).parents[1] / "config" / "rocking_optimization.yaml"


def main():
    arguments = [sys.executable, str(SCRIPT), *sys.argv[1:]]
    if "--config" not in arguments:
        arguments.extend(["--config", str(CONFIG)])
    os.execv(sys.executable, arguments)


if __name__ == "__main__":
    main()
