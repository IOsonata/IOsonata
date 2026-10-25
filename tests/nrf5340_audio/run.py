#!/usr/bin/env python3
"""Build and run the nRF5340 Audio DK generic device-driver host tests."""

import os
import pathlib
import subprocess
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]

with tempfile.TemporaryDirectory() as temp:
    exe = pathlib.Path(temp) / "nrf5340_audio_devices"
    cmd = [
        os.environ.get("CXX", "g++"),
        "-std=gnu++23",
        "-O2",
        "-Wall",
        "-Wextra",
        "-Werror",
        "-fsanitize=address,undefined",
        "-I" + str(ROOT / "tests/nrf5340_audio/hostport"),
        "-I" + str(ROOT / "include"),
        str(ROOT / "tests/nrf5340_audio/device_test.cpp"),
        str(ROOT / "src/audio/cs47l63.cpp"),
        str(ROOT / "src/sensors/ina230.cpp"),
        str(ROOT / "src/pwrmgnt/pm_npm1100.cpp"),
        str(ROOT / "src/device.cpp"),
        str(ROOT / "src/device_intrf.cpp"),
        "-o",
        str(exe),
    ]
    subprocess.run(cmd, check=True)
    subprocess.run([str(exe)], check=True)
