#!/usr/bin/env python3
"""Compile the actual ESP32-C3 RNG backend against a mock register/ROM model."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TEST = Path(__file__).resolve().parent
SOURCE = ROOT / "RISCV/Espressif/src/crypto_rng_esp32c3.cpp"

# Static compatibility check: ESP32-C3 lacks CSR cycle/mcycle.
# The production RNG must use the existing CPU counter selector, not
# a standalone rdcycle instruction that traps at the first wait.
driver_src = SOURCE.read_text()
delay_src = (ROOT / "RISCV/include/idelay.h").read_text()
system_src = (ROOT / "RISCV/Espressif/src/system_esp32_system.c").read_text()
assert '#include "idelay.h"' in driver_src
assert "return IOsonataRiscvCycle32();" in driver_src
assert not any(unsupported in driver_src for unsupported in
               ('csrr %0, 0xC00', 'csrr %0, 0xB00', 'rdcycle'))
assert "IOSONATA_RISCV_CYCLE_CSR        0x7E2U" in delay_src
assert 'csrw 0x7E2' in system_src and 'csrw 0x7E1' in system_src


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
