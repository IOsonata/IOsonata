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
    (out / "stm32.h").write_text('#include "crypto_accel_model.h"\n' + "\n".join(defines))
    (out / "board.h").write_text("""#include "crypto_stm32h7.h"
#define CRYPTO_TEST_AES256
#define CRYPTO_TEST_AES_TYPE CryptoStm32H7Aes
#define CRYPTO_TEST_AES_MEMSIZE sizeof(CryptoStm32H7Aes)
#define CRYPTO_TEST_AES_CREATE(m,s) CryptoStm32H7AesCreate(m,s)
#define CRYPTO_TEST_HASH_TYPE CryptoStm32H7Hash
#define CRYPTO_TEST_HASH_MEMSIZE sizeof(CryptoStm32H7Hash)
#define CRYPTO_TEST_HASH_CREATE(m,s) CryptoStm32H7HashCreate(m,s)
""")
    inc = ["-I" + str(p) for p in (out, HERE, ROOT / "include", ROOT / "ARM/ST/include", ROOT / "micro-ecc")]
    for opt in ("-O0", "-O2"):
        flags = [opt, "-g", "-fsanitize=address,undefined", "-DuECC_ENABLE_VLI_API=1",
                 "-DuECC_WORD_SIZE=4", "-DCRYPTO_STM32H7_POLL_MAX=32", "-DCRYPTO_TEST_NO_MAIN"]
        run(["gcc", *flags, *inc, "-c", ROOT / "micro-ecc/uECC.c", "-o", out / "uecc.o"])
        sources = ["exemples/crypto/crypto_engine_test.cpp", "src/crypto/crypto_softaes.cpp",
                   "src/crypto/crypto_softsha256.cpp", "src/crypto/crypto_softrng.cpp",
                   "src/crypto/crypto_uecc.cpp", "src/crypto/crypto_p256.cpp",
                   "src/device.cpp", "src/device_intrf.cpp", "ARM/ST/STM32H7xx/src/crypto_stm32h7.cpp",
                   "tests/stm32h7/crypto_accel_model.cpp"]
        run(["g++", "-std=gnu++23", "-Wall", "-Wextra", "-Werror", "-fno-rtti", "-fno-exceptions",
             "-ffunction-sections", "-fdata-sections", *flags, *inc,
             *(ROOT / p for p in sources), out / "uecc.o", "-Wl,--gc-sections", "-lcrypto", "-o", out / "test"])
        print(opt, flush=True)
        run([out / "test"], env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
