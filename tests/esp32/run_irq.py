#!/usr/bin/env python3
"""Run production ESP32 routing/dispatch with host MMIO and UBSan."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='esp32-irq-') as directory:
    for chip in ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61'):
        for optimization in ('-O0', '-O2'):
            binary = Path(directory) / (chip + optimization)
            subprocess.run(['g++', '-std=gnu++17', optimization, '-g',
                '-fsanitize=undefined', '-fno-sanitize-recover=all', '-D' + chip,
                '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'RISCV/include'),
                '-I' + str(ROOT / 'RISCV/Espressif/include'),
                str(ROOT / 'tests/esp32/irq_test.cpp'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)
            print(chip, optimization, 'IRQ PASS')

# Exercise the real UART initializer, clock helper, matrix routing and ISR.
with tempfile.TemporaryDirectory(prefix='esp32-uart-irq-') as directory:
    for chip in ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61'):
        binary = Path(directory) / chip
        subprocess.run(['g++', '-std=gnu++17', '-O2', '-g',
            '-fsanitize=undefined', '-fno-sanitize-recover=all',
            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
            '-DESP32_UART_REAL_IRQ', '-D' + chip,
            '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'RISCV/include'),
            '-I' + str(ROOT / 'RISCV/Espressif/include'),
            str(ROOT / 'tests/uart/esp32_uart_init_test.cpp'),
            str(ROOT / 'src/cfifo.c'), '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
        print(chip, 'UART IRQ integration PASS')
