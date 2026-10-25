#!/usr/bin/env python3
"""Build and run the Thingy52 production driver against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/sensors/thingy52_devices_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/device_intrf.cpp",
    ROOT / "src/sensors/color_bh1745.cpp",
    ROOT / "src/sensors/gas_ccs811.cpp",
    ROOT / "src/miscdev/ioexp_sx1509.cpp",
]

with tempfile.TemporaryDirectory(prefix="thingy52_devices-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("thingy52_devices_" + opt[1:])
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
        env = dict(os.environ)
        env.setdefault("ASAN_OPTIONS", "detect_leaks=0")
        result = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
        if result.returncode != 0 or "Thingy52 production-driver tests: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "Thingy52 production-driver tests: PASS", flush=True)

