#!/usr/bin/env python3
"""Run the production nRF70 tasklet dispatcher with a bounded queue fixture."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/net/nrf70_tasklet_test.cpp",
    ROOT / "src/net/nrf70_tasklet.cpp",
]

with tempfile.TemporaryDirectory(prefix="nrf70-tasklet-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("nrf70-tasklet_" + opt[1:])
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
        env = dict(os.environ)
        env.setdefault("ASAN_OPTIONS", "detect_leaks=0")
        result = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
        if result.returncode != 0 or "nRF70 tasklet lifecycle: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "nRF70 tasklet lifecycle: PASS", flush=True)

