#!/usr/bin/env python3
"""Test UART stdio short writes and bounded stalls.

Copyright (c) 2026 I-SYST inc. MIT License; see repository LICENSE.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix="uart-retarget-") as directory:
    for optimization in ("-O0", "-O2"):
        binary = Path(directory) / "uart_retarget"
        subprocess.run([
            "gcc", "-std=gnu11", optimization, "-g", "-Wall", "-Wextra",
            "-ffunction-sections", "-fdata-sections", "-Wl,--gc-sections",
            "-fsanitize=undefined", "-fno-sanitize-recover=all",
            "-I" + str(ROOT / "include"),
            str(ROOT / "tests/uart/uart_retarget_test.c"), "-o", str(binary),
        ], check=True)
        subprocess.run([str(binary)], check=True)
        print("UART stdio", optimization, "PASS")
