#!/usr/bin/env python3
"""Run LPC11Uxx clock decoding against vendor CMSIS peripheral declarations."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
includes = ['tests/lpc_uart/shim', 'include', 'ARM/NXP/LPC11xx/include']
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'clock_test')
        subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-unused-parameter', '-Wno-missing-field-initializers',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc_clock/clock_test.c'),
                        str(root / 'ARM/NXP/LPC11xx/src/system_LPC11Uxx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)
