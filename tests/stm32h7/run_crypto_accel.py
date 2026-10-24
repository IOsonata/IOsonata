#!/usr/bin/env python3
"""Run H753 CRYP/HASH driver against an OpenSSL-backed host register model.
Requires g++, gcc and OpenSSL development headers. No hardware timing claims.
"""
from pathlib import Path
import os
import sys
import re
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
L4 = "--l4s9" in sys.argv

def run(args, **kwargs):
    subprocess.run([str(a) for a in args], check=True, **kwargs)

with tempfile.TemporaryDirectory(prefix="h7-crypto-") as temp:
    out = Path(temp)
    cmsis = (ROOT / "ARM/ST/STM32H7xx/include/stm32h753xx.h").read_text()
    # Use bit definitions from the real device header, not duplicated constants.
    defines = [line for line in cmsis.splitlines() if re.match(
        r"#define (CRYP_(CR|SR)_|HASH_(CR|SR|STR)_|RCC_AHB2(ENR|RSTR)_)", line)]
    if L4:
        defines = [line for line in cmsis.splitlines() if re.match(
            r"#define (CRYP_(CR|SR)_|RCC_AHB2(?:ENR|RSTR)_CRYP)", line)]
        l4 = (ROOT / "ARM/ST/STM32L4xx/include/stm32l4s9xx.h").read_text()
        defines += [line for line in l4.splitlines() if re.match(
            r"#define (AES_(CR|SR)_|HASH_(CR|SR|STR)_|RCC_AHB2(?:ENR|RSTR)_)", line)]
    (out / "stm32.h").write_text('#define IOSONATA_STM32_HAS_RNG 1\n#include "crypto_accel_model.h"\n' + "\n".join(defines))
    inc = ["-I" + str(p) for p in (out, HERE, ROOT / "include", ROOT / "ARM/ST/include", ROOT / "micro-ecc")]
    for opt, minimal in (("-O0", False), ("-O2", False), ("-O0", True), ("-Os", True)):
        flags = [opt, "-g", "-fsanitize=address,undefined", "-DuECC_ENABLE_VLI_API=1",
                 "-DSTM32L4S9xx" if L4 else "-DSTM32H753xx", "-DuECC_WORD_SIZE=4", "-DCRYPTO_STM32H7_POLL_MAX=32", "-DCRYPTO_TEST_NO_MAIN"]
        if minimal:
            flags.remove("-fsanitize=address,undefined")
            flags.append("-DCRYPTO_LINK_TEST")
        run(["gcc", *flags, *inc, "-c", ROOT / "micro-ecc/uECC.c", "-o", out / "uecc.o"])
        sources = ["src/coredev/timer.cpp", "tests/crypto/timer_fixture.cpp",
                   "src/crypto/crypto_aes_modes.cpp", "src/crypto/crypto_aes_soft.cpp",
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
        reference = [] if minimal else [ROOT / "exemples/crypto/crypto_engine_test.cpp"]
        run([*cxx, *reference,
             HERE / "crypto_accel_model.cpp", archive, "-Wl,--gc-sections",
             "-lcrypto", "-o", out / "test"])
        print(opt, flush=True)
        run([out / "test"], env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})

        if minimal:
            symbols = subprocess.check_output(["nm", "-C", out / "test"], text=True)
            for forbidden in ("CryptoSoftAes", "CryptoAesSoft", "CryptoSoftRng"):
                assert forbidden not in symbols, forbidden
            assert "CryptoStm32H7Aes::Cipher" in symbols
            assert "CryptoStm32H7Hash::Hash" in symbols
            # Streaming SHA has no target override yet; its software is required.
            assert "CryptoSoftSha256Hash::HashUpdate" in symbols
            print("Minimal target archive: no software AES or PRNG; streaming SHA retained")
