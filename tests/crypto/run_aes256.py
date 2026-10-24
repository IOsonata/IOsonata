#!/usr/bin/env python3
"""AES-256 differential and inheritance tests; requires GCC and OpenSSL headers."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["src/device.cpp", "src/device_intrf.cpp", "src/crypto/crypto.cpp",
           "src/crypto/crypto_aes_modes.cpp", "src/crypto/crypto_aes_soft.cpp",
           "src/crypto/crypto_softsha256.cpp", "src/crypto/crypto_softrng.cpp",
           "src/crypto/crypto_uecc.cpp", "src/crypto/crypto_p256.cpp"]


def run(args, **kwargs):
    subprocess.run([str(arg) for arg in args], check=True, **kwargs)


with tempfile.TemporaryDirectory(prefix="aes256-") as temp:
    out = Path(temp)
    includes = ["-I" + str(ROOT / p) for p in ("include", "micro-ecc")]
    for opt in ("-O0", "-O2"):
        flags = [opt, "-g", "-fsanitize=address,undefined", "-DuECC_ENABLE_VLI_API=1",
                 "-ffunction-sections", "-fdata-sections"]
        run(["gcc", *flags, *includes, "-c", ROOT / "micro-ecc/uECC.c", "-o", out / "uecc.o"])
        run(["g++", "-std=gnu++23", "-Wall", "-Wextra", "-Werror", "-fno-rtti",
             "-fno-exceptions", *flags, *includes, *(ROOT / p for p in SOURCES),
             ROOT / "tests/crypto/aes256_test.cpp", out / "uecc.o",
             "-Wl,--gc-sections", "-lcrypto", "-o", out / "test"])
        print(opt, flush=True)
        run([out / "test"], env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
