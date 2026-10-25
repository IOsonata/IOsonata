#!/usr/bin/env python3
"""Build real software crypto as an archive; test direct object construction and initialization.
Requires gcc, g++ and ar. No MCU toolchain or hardware is exercised.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = ["src/coredev/timer.cpp", "tests/crypto/timer_fixture.cpp", "src/device.cpp", "src/device_intrf.cpp", "src/crypto/crypto.cpp",
           "src/crypto/crypto_default.cpp", "src/crypto/crypto_aes_modes.cpp",
           "src/crypto/crypto_aes_soft.cpp", "src/crypto/crypto_softsha256.cpp",
           "src/crypto/crypto_softrng.cpp", "src/crypto/crypto_uecc.cpp",
           "src/crypto/crypto_p256.cpp", "micro-ecc/uECC.c"]


def run(args):
    subprocess.run([str(arg) for arg in args], check=True)


with tempfile.TemporaryDirectory(prefix="crypto-objects-") as temp:
    out = Path(temp)
    includes = ["-I" + str(ROOT / p) for p in ("include", "micro-ecc")]
    for opt in ("-O0", "-O2"):
        objects = []
        flags = [opt, "-DuECC_ENABLE_VLI_API=1", "-ffunction-sections", "-fdata-sections"]
        for source in SOURCES:
            obj = out / (Path(source).stem + ".o")
            compiler = ["gcc", "-std=gnu17"] if source.endswith(".c") else [
                "g++", "-std=gnu++23", "-fno-rtti", "-fno-exceptions"]
            run([*compiler, *flags, *includes, "-c", ROOT / source, "-o", obj])
            objects.append(obj)
        archive = out / "libcrypto-test.a"
        run(["ar", "rcs", archive, *objects])
        for source in ("tests/crypto/object_test.cpp", "tests/crypto/soak_failure_test.cpp",
                       "exemples/crypto/crypto_engine_test.cpp"):
            executable = out / Path(source).stem
            run(["g++", "-std=gnu++23", "-fno-rtti", "-fno-exceptions", *flags,
                 *includes, ROOT / source, archive, "-Wl,--gc-sections", "-o", executable])
            print(opt, source, flush=True)
            result = subprocess.run([str(executable)], check=True, text=True, capture_output=True)
            print(result.stdout, end="")
            if source.endswith("crypto_engine_test.cpp"):
                assert "Benchmark timer: 32768 Hz" in result.stdout
                assert "Crypto performance (nanoseconds per operation" in result.stdout
                assert result.stdout.count("ns/op") == 28
                assert "target 3337, software 3337 ns/op" in result.stdout
        executable = out / "timer_failure_test"
        run(["g++", "-std=gnu++23", "-fno-rtti", "-fno-exceptions", *flags,
             "-DCRYPTO_TEST_NO_MAIN", *includes,
             ROOT / "exemples/crypto/crypto_engine_test.cpp",
             ROOT / "tests/crypto/timer_failure_test.cpp", archive,
             "-Wl,--gc-sections", "-o", executable])
        result = subprocess.run([str(executable)], check=True, text=True, capture_output=True)
        assert "[FAIL] benchmark timer initializes" in result.stdout
        assert "Crypto performance" not in result.stdout
        print("Timer failure reported; no benchmark output", flush=True)
