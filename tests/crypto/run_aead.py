#!/usr/bin/env python3
"""CCM/GCM differential tests; requires GCC and OpenSSL development files."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["src/device.cpp", "src/device_intrf.cpp",
           "src/crypto/crypto_aes_modes.cpp", "src/crypto/crypto_aes_soft.cpp",
           "tests/crypto/aead_test.cpp"]

with tempfile.TemporaryDirectory(prefix="crypto-aead-") as temp:
    exe = Path(temp) / "test"
    for opt in ("-O0", "-Os", "-O2"):
        subprocess.run(["g++", "-std=gnu++23", opt, "-g", "-Wall", "-Wextra", "-Werror",
                        "-fno-rtti", "-fno-exceptions", "-fsanitize=address,undefined",
                        "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
                        "-I" + str(ROOT / "include"),
                        *(str(ROOT / path) for path in SOURCES), "-lcrypto", "-o", str(exe)], check=True)
        print(opt, flush=True)
        subprocess.run([str(exe)], check=True,
                       env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
