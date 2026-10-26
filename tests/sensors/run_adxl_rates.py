#!/usr/bin/env python3
"""Build and run the ADXL367/ADXL372 production drivers against a simulated DeviceIntrf."""

# MIT License
#
# Copyright (c) 2019 I-SYST inc. All rights reserved.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
	ROOT / "tests/sensors/adxl_rate_test.cpp",
	ROOT / "src/device.cpp",
	ROOT / "src/device_intrf.cpp",
	ROOT / "src/sensors/accel_sensor.cpp",
	ROOT / "src/sensors/accel_adxl367.cpp",
	ROOT / "src/sensors/accel_adxl372.cpp",
]

with tempfile.TemporaryDirectory(prefix="adxl_rate-") as td:
	for opt in ("-O0", "-O2"):
		exe = Path(td) / ("adxl_rate_" + opt[1:])
		cmd = [
			CXX, "-std=gnu++23", opt,
			"-Wall", "-Wextra", "-Werror", "-Wpedantic",
			"-Wno-vla", "-Wno-unused-variable", "-Wno-unused-but-set-variable", "-Wno-unused-parameter", "-fno-rtti", "-fno-exceptions",
			"-fsanitize=address,undefined", "-fno-omit-frame-pointer",
			"-I" + str(ROOT / "tests/sensors/shim"),
			"-I" + str(ROOT / "tests/sensors/thingy52_shim"),
			# Shared headers use GNU extensions and optional unused parameters.
			# Keep strict diagnostics on the driver and test source files.
			"-isystem", str(ROOT / "include"),
		]
		subprocess.run(cmd + [str(p) for p in SOURCES] + ["-lm", "-o", str(exe)], check=True)
		env = dict(os.environ)
		env.setdefault("ASAN_OPTIONS", "detect_leaks=0")
		result = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
		if result.returncode != 0 or "ADXL367/ADXL372 rate tests: PASS" not in result.stdout:
			raise RuntimeError(result.stdout + result.stderr)
		print(opt, "ADXL367/ADXL372 rate tests: PASS", flush=True)
