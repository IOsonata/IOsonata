#!/usr/bin/env python3
"""Build and run the Thingy52 production driver against a simulated DeviceIntrf."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
CXX = os.environ.get("CXX", "g++")
SOURCES = [
    ROOT / "tests/sensors/thingy52_board_test.cpp",
    ROOT / "src/device.cpp",
    ROOT / "src/device_intrf.cpp",
    ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/common/thingy52_board.cpp",
    ROOT / "src/sensors/gas_ccs811.cpp",
    ROOT / "src/miscdev/ioexp_sx1509.cpp",
]

with tempfile.TemporaryDirectory(prefix="thingy52_board-") as td:
    for opt in ("-O0", "-O2"):
        exe = Path(td) / ("thingy52_board_" + opt[1:])
        cmd = [
            CXX, "-std=gnu++23", opt,
            "-Wall", "-Wextra", "-Werror", "-Wpedantic",
            "-Wno-vla", "-fno-rtti", "-fno-exceptions",
            "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
            "-I" + str(ROOT / "tests/sensors/shim"),
            "-I" + str(ROOT / "tests/sensors/thingy52_shim"),
            "-I" + str(ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/common"),
            # Shared headers use GNU extensions and optional unused parameters.
            # Keep strict diagnostics on the driver and test source files.
            "-isystem", str(ROOT / "include"),
        ]
        subprocess.run(cmd + [str(p) for p in SOURCES] + ["-lm", "-o", str(exe)], check=True)
        env = dict(os.environ)
        env.setdefault("ASAN_OPTIONS", "detect_leaks=0")
        result = subprocess.run([str(exe)], capture_output=True, text=True, env=env)
        if result.returncode != 0 or "Thingy52 board tests: PASS" not in result.stdout:
            raise RuntimeError(result.stdout + result.stderr)
        print(opt, "Thingy52 board tests: PASS", flush=True)


# Compile the actual example configuration and API calls, with GPIO/delay shims.
common = ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/common"
example = ROOT / "ARM/Nordic/nRF52/nRF52832/exemples/Thingy52PeripheralTest"
subprocess.run([
    CXX, "-std=gnu++23", "-Wall", "-Wextra", "-Werror", "-fno-rtti", "-fno-exceptions",
    "-I" + str(ROOT / "tests/sensors/shim"),
    "-I" + str(ROOT / "tests/sensors/thingy52_shim"),
    "-I" + str(common), "-I" + str(example / "src"),
    "-isystem", str(ROOT / "include"), "-fsyntax-only",
    str(common / "thingy52_board.cpp"), str(example / "src/main.cpp"),
], check=True)
print("Thingy52PeripheralTest host syntax: PASS")
