#!/usr/bin/env python3
"""Build and run the BMM350 production driver against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/sensors/bmm350_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/device_intrf.cpp",
    ROOT / "src/sensors/mag_sensor.cpp",
    ROOT / "src/sensors/mag_bmm350.cpp",
]

with tempfile.TemporaryDirectory(prefix="bmm350-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("bmm350_" + opt[1:])
        cmd = [
            CXX, "-std=gnu++23", opt,
            "-Wall", "-Wextra", "-Werror", "-Wpedantic",
            "-Wno-vla", "-fno-rtti", "-fno-exceptions",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I" + str(ROOT / "tests/sensors/shim"),
            # Shared headers use GNU extensions and optional unused parameters.
            # Keep strict diagnostics on the driver and test source files.
            "-isystem", str(ROOT / "include"),
        ]
        subprocess.run(cmd + [str(p) for p in SOURCES] + ["-lm", "-o", str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        if result.returncode != 0 or "BMM350 production-driver tests: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "BMM350 production-driver tests: PASS", flush=True)
