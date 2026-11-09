#!/usr/bin/env python3
"""Execute production ESP32 Timer Group and IRQ code using host MMIO and UBSan."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='esp32-timer-') as directory:
    for chip in ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61'):
        for optimization in ('-O0', '-O2'):
            binary = Path(directory) / (chip + optimization)
            subprocess.run(['g++', '-std=gnu++17', optimization, '-g',
                '-fsanitize=undefined', '-fno-sanitize-recover=all', '-D' + chip,
                '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'RISCV/include'),
                '-I' + str(ROOT / 'RISCV/Espressif/include'),
                str(ROOT / 'tests/esp32/timer_test.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
            print(chip, optimization, 'timer PASS', flush=True)
