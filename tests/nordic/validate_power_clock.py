#!/usr/bin/env python3
"""Run the Nordic 0.14 power/clock regression suite and optional Arm syntax matrix.

The model and source checks run without an Arm toolchain. If --compiler is
given, production power, clock and I2C sources are compiled for the selected
targets; this does not replace Eclipse managed-project linking or board tests.
"""
import argparse
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]

CHECKS = (
	"tests/nordic/check_clock_hfxo_source.py",
	"tests/nordic/check_power_source.py",
	"tests/nordic/check_delay_calibration.py",
	"tests/nordic/run_system_sleep.py",
	"tests/nordic/check_i2c_lifecycle.py",
)


def run(command):
	print("RUN:", " ".join(command), flush=True)
	return subprocess.run(command, cwd=ROOT, check=False).returncode


def main():
	parser = argparse.ArgumentParser(description=__doc__)
	parser.add_argument("--compiler", help="Arm G++ compiler path for focused target syntax builds")
	parser.add_argument("--target", action="append",
		help="Nordic target name, repeat to build more targets; omitted means all supported targets")
	args = parser.parse_args()
	failures = 0

	for check in CHECKS:
		if run([sys.executable, str(ROOT / check)]) != 0:
			failures += 1

	if args.compiler:
		command = [sys.executable, str(ROOT / "tests/nordic/compile_power_clock.py"),
			"--compiler", args.compiler, "--with-i2c"]
		for target in args.target or ():
			command += ["--target", target]
		if run(command) != 0:
			failures += 1

	print("Nordic power/clock validation:",
		"PASS" if failures == 0 else str(failures) + " FAIL")
	return 1 if failures else 0


if __name__ == "__main__":
	sys.exit(main())
