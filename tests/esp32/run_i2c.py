#!/usr/bin/env python3
"""Run production ESP32 I2C and generic transfer code with modeled MMIO."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='esp32-i2c-') as directory:
    for chip in ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61'):
        for optimization in ('-O0', '-O2'):
            binary = Path(directory) / (chip + optimization)
            subprocess.run(['g++', '-std=gnu++17', optimization, '-g',
                '-fsanitize=undefined', '-fno-sanitize-recover=all', '-D' + chip,
                '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'RISCV/include'),
                '-I' + str(ROOT / 'RISCV/Espressif/include'),
                str(ROOT / 'tests/esp32/i2c_master_test.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
            print(chip, optimization, 'I2C master PASS', flush=True)
