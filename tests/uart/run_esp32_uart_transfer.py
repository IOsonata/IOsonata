#!/usr/bin/env python3
"""Build ESP32 UART transfer tests against the real driver and CFIFO.

Copyright (c) 2026 I-SYST inc. MIT License; see repository LICENSE.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="esp32-uart-") as directory:
    for chip in ("ESP32C3", "ESP32C5", "ESP32C6"):
        for optimization in ("-O0", "-O2"):
            binary = Path(directory) / (chip + optimization)
            subprocess.run([
                "g++", "-std=gnu++17", optimization, "-g",
                "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
                "-fsanitize=undefined", "-fno-sanitize-recover=all",
                "-D" + chip, "-I" + str(ROOT / "include"),
                "-I" + str(ROOT / "RISCV/Espressif/include"),
                "-I" + str(ROOT / "RISCV/include"),
                str(ROOT / "tests/uart/esp32_uart_transfer_test.cpp"),
                str(ROOT / "src/cfifo.c"), "-o", str(binary),
            ], check=True)
            subprocess.run([str(binary)], check=True)
            print(chip, optimization, "PASS")
