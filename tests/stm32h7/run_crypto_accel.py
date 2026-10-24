#!/usr/bin/env python3
"""Run H753 CRYP/HASH driver against an OpenSSL-backed host register model.
Requires g++, gcc and OpenSSL development headers. No hardware timing claims.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent

def run(args, **kwargs):
    subprocess.run([str(a) for a in args], check=True, **kwargs)

with tempfile.TemporaryDirectory(prefix="h7-crypto-") as temp:
    out = Path(temp)
    cmsis = (ROOT / "ARM/ST/STM32H7xx/include/stm32h753xx.h").read_text()
    # Use bit definitions from the real device header, not duplicated constants.
    defines = [line for line in cmsis.splitlines() if re.match(
        r"#define (CRYP_(CR|SR)_|HASH_(CR|SR|STR)_|RCC_AHB2(ENR|RSTR)_)", line)]
    (out / "stm32.h").write_text('#define IOSONATA_STM32_HAS_RNG 1\n#include "crypto_accel_model.h"\n' + "\n".join(defines))
    inc = ["-I" + str(p) for p in (out, HERE, ROOT / "include", ROOT / "ARM/ST/include", ROOT / "micro-ecc")]
    for opt in ("-O0", "-O2"):
        flags = [opt, "-g", "-fsanitize=address,undefined", "-DuECC_ENABLE_VLI_API=1",
                 "-DSTM32H753xx", "-DuECC_WORD_SIZE=4", "-DCRYPTO_STM32H7_POLL_MAX=32", "-DCRYPTO_TEST_NO_MAIN"]
        run(["gcc", *flags, *inc, "-c", ROOT / "micro-ecc/uECC.c", "-o", out / "uecc.o"])
        sources = ["src/crypto/crypto_aes_modes.cpp", "src/crypto/crypto_aes_soft.cpp",
                   "src/crypto/crypto_softsha256.cpp", "src/crypto/crypto_softrng.cpp",
                   "src/crypto/crypto_uecc.cpp", "src/crypto/crypto_p256.cpp",
                   "src/crypto/crypto.cpp", "ARM/ST/src/crypto_default_stm32.cpp",
                   "src/device.cpp", "src/device_intrf.cpp",
                   "ARM/ST/STM32H7xx/src/crypto_stm32h7.cpp"]
        cxx = ["g++", "-std=gnu++23", "-Wall", "-Wextra", "-Werror", "-fno-rtti",
               "-fno-exceptions", "-ffunction-sections", "-fdata-sections", *flags, *inc]
        objects = [out / "uecc.o"]
        for source in sources:
            obj = out / (Path(source).stem + ".o")
            run([*cxx, "-c", ROOT / source, "-o", obj])
            objects.append(obj)
        archive = out / "libcrypto-target.a"
        run(["ar", "rcs", archive, *objects])
        run([*cxx, ROOT / "exemples/crypto/crypto_engine_test.cpp",
             HERE / "crypto_accel_model.cpp", archive, "-Wl,--gc-sections",
             "-lcrypto", "-o", out / "test"])
        print(opt, flush=True)
        run([out / "test"], env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
