#!/usr/bin/env python3
"""Run Microchip power/clock host and target-compilation regression suites.

Target compilation requires an Arm GNU toolchain. Eclipse application linking
and physical sleep/wake behavior are separate validation steps.
"""
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
SUITES = (
    "tests/microchip/compile_power.py",
    "tests/sam4l/run_power.py",
    "tests/samd/run_power.py",
    "tests/samd/run_standby.py",
)


def main():
    for relative in SUITES:
        path = ROOT / relative
        print("\n=== " + relative + " ===", flush=True)
        subprocess.run([sys.executable, str(path)], cwd=ROOT, check=True)
    print("\nMicrochip power/clock regression suites PASS", flush=True)


if __name__ == "__main__":
    main()
