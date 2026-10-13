#!/usr/bin/env python3
"""LPC1769 USB clock preparation checks. MIT License, I-SYST, 2026."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'hw17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/startup/system17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        str(root / 'tests/lpc/usb/hw17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_hw_lpc17xx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)
