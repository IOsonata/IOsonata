#!/usr/bin/env python3
"""Build and run the BH1749 production driver against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/sensors/bh1749_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/device_intrf.cpp",
    ROOT / "src/sensors/color_bh1749.cpp",
]

with tempfile.TemporaryDirectory(prefix="bh1749-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("bh1749_" + opt[1:])
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
        if result.returncode != 0 or "BH1749 Device lifecycle and transfers: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "BH1749 Device lifecycle and transfers: PASS", flush=True)

