#!/usr/bin/env python3
"""Production CC3xx/BA414EP drivers with OpenSSL host register arithmetic.
Requires gcc, g++, ar and OpenSSL development headers. Not hardware validation.
"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["src/coredev/timer.cpp", "tests/crypto/timer_fixture.cpp", "src/device.cpp", "src/device_intrf.cpp", "src/crypto/crypto.cpp",
           "ARM/Nordic/src/crypto_default_nrfx.cpp", "src/crypto/crypto_aes_modes.cpp",
           "src/crypto/crypto_aes_soft.cpp", "src/crypto/crypto_softsha256.cpp",
           "src/crypto/crypto_softrng.cpp", "src/crypto/crypto_uecc.cpp",
           "src/crypto/crypto_p256.cpp", "src/crypto/ba414ep.cpp", "ARM/src/crypto_cc3xx.cpp"]


def run(args, **kwargs):
    subprocess.run([str(arg) for arg in args], check=True, **kwargs)


with tempfile.TemporaryDirectory(prefix="pka-models-") as temp:
    out = Path(temp)
    (out / "nrf.h").write_text("")
    (out / "cmsis_compiler.h").write_text(
        "#include <stdint.h>\nstatic inline uint32_t __get_IPSR(void) { return 0; }\n")
    inc = ["-I" + str(p) for p in (out, ROOT / "include", ROOT / "ARM/include",
                                  ROOT / "ARM/Nordic/include", ROOT / "micro-ecc")]
    for opt in ("-O0", "-O2"):
        flags = [opt, "-g", "-fsanitize=address,undefined", "-DuECC_ENABLE_VLI_API=1",
                 "-DuECC_WORD_SIZE=4", "-DNRF5340_XXAA_APPLICATION", "-DCRYPTO_TEST_NO_MAIN",
                 "-DCC3XX_ACQUIRE_SPINS=4", "-DCC3XX_PKA_WAIT_SPINS=4", "-ffunction-sections", "-fdata-sections"]
        run(["gcc", *flags, *inc, "-c", ROOT / "micro-ecc/uECC.c", "-o", out / "uecc.o"])
        objects = [out / "uecc.o"]
        cxx = ["g++", "-std=gnu++23", "-Wall", "-Wextra", "-Werror", "-fno-rtti",
               "-fno-exceptions", *flags, *inc]
        for source in SOURCES:
            obj = out / (Path(source).stem + ".o")
            run([*cxx, "-c", ROOT / source, "-o", obj])
            objects.append(obj)
        archive = out / "libcrypto-target.a"
        run(["ar", "rcs", archive, *objects])
        run([*cxx, ROOT / "tests/crypto/pka_model_test.cpp", archive,
             "-Wl,--gc-sections", "-lcrypto", "-o", out / "test"])
        print(opt, flush=True)
        run([out / "test"], env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
