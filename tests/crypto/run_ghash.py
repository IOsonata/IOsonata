#!/usr/bin/env python3
"""GHASH differential checks and isolated host timing (not MCU timing)."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["src/device.cpp", "src/device_intrf.cpp",
           "src/crypto/crypto_aes_soft.cpp", "tests/crypto/ghash_test.cpp"]
with tempfile.TemporaryDirectory(prefix="crypto-ghash-") as temp:
    exe = Path(temp) / "test"
    for opt in ("-O0", "-Os", "-O2"):
        for timing in (False, True):
            if timing and opt == "-O0":
                continue
            flags = (["-DGHASH_BENCHMARK"] if timing else
                     ["-fsanitize=address,undefined", "-fno-omit-frame-pointer"])
            subprocess.run(["g++", "-std=gnu++23", opt, "-Wall", "-Wextra", "-Werror",
                            "-fno-rtti", "-fno-exceptions", "-fno-pie", "-no-pie",
                            *flags, "-I" + str(ROOT / "include"),
                            *(str(ROOT / path) for path in SOURCES), "-o", str(exe)], check=True)
            print(opt, "host timing" if timing else "sanitizers", flush=True)
            subprocess.run([str(exe)], check=True,
                           env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
