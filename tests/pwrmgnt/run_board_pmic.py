#!/usr/bin/env python3
"""Build and run the board PMIC drivers against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/pwrmgnt/board_pmic_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/device_intrf.cpp",
    ROOT / "src/pwrmgnt/pm_adp5360.cpp",
    ROOT / "src/pwrmgnt/pm_npm6001.cpp",
]

with tempfile.TemporaryDirectory(prefix="board-pmic-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("board_pmic_" + opt[1:])
        cmd = [
            CXX, "-std=gnu++23", opt,
            "-Wall", "-Wextra", "-Werror", "-Wpedantic",
            "-Wno-vla", "-fno-rtti", "-fno-exceptions",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            # Shared headers use GNU extensions and optional unused parameters.
            # Keep strict diagnostics on the driver and test source files.
            "-isystem", str(ROOT / "include"),
        ]
        subprocess.run(cmd + [str(p) for p in SOURCES] + ["-lm", "-o", str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        if result.returncode != 0 or "Board PMIC lifecycle tests: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "Board PMIC lifecycle tests: PASS", flush=True)
