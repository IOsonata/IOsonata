#!/usr/bin/env python3
"""Compile the actual ESP32-C3 RNG backend against a mock register/ROM model."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = Path(__file__).resolve().parent
SOURCE = ROOT / "RISCV/Espressif/src/crypto_rng_esp32c3.cpp"

with tempfile.TemporaryDirectory(prefix="esp32c3-rng-") as name:
    for optimization in ("-O0", "-O2"):
        exe = Path(name) / ("test_" + optimization[1:])
        subprocess.run([
            "g++", "-std=gnu++23", optimization, "-Wall", "-Wextra", "-Werror",
            "-fno-rtti", "-fno-exceptions",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-DESP32C3", "-DESP32_RNG_TEST", "-DESP32_RNG_TRACE=0",
            "-DESP32_RNG_WAIT_MAX_POLLS=1024",
            "-I", str(ROOT / "include"),
            "-I", str(ROOT / "RISCV/Espressif/include"),
            "-I", str(TEST),
            "-include", str(TEST / "esp32c3_rng_model.h"),
            str(SOURCE), str(TEST / "esp32c3_rng_model.cpp"),
            "-o", str(exe)
        ], check=True)
        for scenario in ("good", "analog", "foreign", "cycle", "stuck"):
            subprocess.run([str(exe), scenario], check=True)
print("ESP32-C3 secure RNG register model: PASS")
