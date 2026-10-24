#!/usr/bin/env python3
"""Build and run the BMI270 production driver against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/sensors/bmi270_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/sensors/accel_sensor.cpp",
    ROOT / "src/sensors/gyro_sensor.cpp",
    ROOT / "src/sensors/ag_bmi270.cpp",
]

with tempfile.TemporaryDirectory(prefix="bmi270-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("bmi270_" + opt[1:])
        cmd = [
            CXX, "-std=gnu++23", opt,
            "-Wall", "-Wextra", "-Werror", "-Wpedantic",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I" + str(ROOT / "tests/sensors/shim"),
            "-I" + str(ROOT / "include"),
        ]
        subprocess.run(cmd + [str(p) for p in SOURCES] + ["-lm", "-o", str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True, check=True)
        if "BMI270 production-driver tests: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "BMI270 production-driver tests: PASS", flush=True)
