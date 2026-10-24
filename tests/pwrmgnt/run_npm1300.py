#!/usr/bin/env python3
"""Build and run the nPM1300 production driver against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/pwrmgnt/npm1300_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/device_intrf.cpp",
    ROOT / "src/pwrmgnt/pm_npm1300.cpp",
    ROOT / "src/pwrmgnt/pm_npm1300_fuel.cpp",
]

with tempfile.TemporaryDirectory(prefix="npm1300-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("npm1300_" + opt[1:])
        cmd = [
            CXX, "-std=gnu++23", opt, "-DNPM1300_FUEL_TEST",
            "-I", str(ROOT / "tests/pwrmgnt/shim"),
            "-Wall", "-Wextra", "-Werror", "-Wpedantic",
            "-Wno-vla", "-fno-rtti", "-fno-exceptions",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            # Shared headers use GNU extensions and optional unused parameters.
            # Keep strict diagnostics on the driver and test source files.
            "-isystem", str(ROOT / "include"),
        ]
        subprocess.run(cmd + [str(p) for p in SOURCES] + ["-lm", "-o", str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        if result.returncode != 0 or "nPM1300 production-driver tests: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "nPM1300 production-driver tests: PASS", flush=True)

