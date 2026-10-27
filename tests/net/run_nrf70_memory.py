#!/usr/bin/env python3
"""Run the nRF70 memory and lock callbacks with caller-owned fixed blocks."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/net/nrf70_memory_test.cpp",
    ROOT / "src/net/nrf70_osal_memory.cpp",
    ROOT / "src/net/nrf70_osal_sync.cpp",
]

with tempfile.TemporaryDirectory(prefix="nrf70-memory-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("nrf70-memory_" + opt[1:])
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
        if result.returncode != 0 or "nRF70 fixed memory and locks: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "nRF70 fixed memory and locks: PASS", flush=True)


# Inspect production objects independently of sanitizer/runtime dependencies.
with tempfile.TemporaryDirectory(prefix="nrf70-memory-symbols-") as td:
    for source in SOURCES[1:]:
        obj = Path(td) / (source.stem + ".o")
        subprocess.run([
            CXX, "-std=gnu++23", "-O2", "-I" + str(ROOT / "include"),
            "-c", str(source), "-o", str(obj),
        ], check=True)
        symbols = subprocess.check_output([
            os.environ.get("NM", "nm"), "-u", str(obj),
        ], text=True)
        for line in symbols.splitlines():
            symbol = line.split()[-1]
            if symbol in {"malloc", "calloc", "realloc", "free"} or symbol.startswith("_Zn"):
                raise RuntimeError("Unexpected heap dependency: " + symbol)
    print("Production memory/lock objects: no heap references", flush=True)
