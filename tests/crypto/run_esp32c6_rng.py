#!/usr/bin/env python3
"""-------------------------------------------------------------------------
@file   run_esp32c6_rng.py

@brief  Build and execute ESP32-C6 RNG register-model tests.

@author IOsonata contributors
@date   Nov. 10, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

-------------------------------------------------------------------------"""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix="esp32c6-rng-") as name:
    for opt in ("-O0", "-O2"):
        exe = Path(name) / ("test_" + opt[1:])
        subprocess.run([
            "g++", "-std=gnu++23", opt, "-Wall", "-Wextra", "-Werror",
            "-fno-rtti", "-fno-exceptions", "-fsanitize=address,undefined",
            "-fno-omit-frame-pointer", "-fno-pie", "-no-pie",
            "-DESP32C6", "-DESP32_RNG_TEST", "-DESP32_RNG_WAIT_MAX_POLLS=1024",
            "-I", str(ROOT / "include"), "-I", str(ROOT / "RISCV/Espressif/include"),
            str(TEST / "esp32c6_rng_model.cpp"), "-o", str(exe)
        ], check=True)
        for scenario in ("good", "pmu1", "pmu2", "pmu3", "powered", "master0", "owner", "foreign", "clock", "bus", "analog", "stuck", "nested", "restore"):
            subprocess.run([str(exe), scenario], check=True,
                           env={**os.environ, "ASAN_OPTIONS": "detect_leaks=0"})
            print(opt, scenario, "PASS", flush=True)
