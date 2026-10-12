#!/usr/bin/env python3
"""Run LPC11Uxx GPIO and LPC17xx GPIO control regressions."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
includes = ['tests/lpc/uart/shim', 'include', 'ARM/NXP/LPC11xx/include',
            'ARM/NXP/LPC11xx/LPC11U35/exemples/UartPrbsTxTest/src']
for compiler, standard in [('gcc', '-std=gnu11'), ('g++', '-std=c++17')]:
    for opt in ['-O0', '-O2']:
        for test in ['gpio_test.c', 'irq_test.c']:
            with tempfile.TemporaryDirectory() as tmp:
                exe = str(Path(tmp) / 'gpio_test')
                subprocess.run([compiler, standard, opt, '-Wall', '-Wextra', '-Werror',
                                '-fsanitize=undefined', '-fno-sanitize-recover=all',
                                *['-I' + str(root / p) for p in includes],
                                str(root / 'tests/lpc/gpio' / test),
                                str(root / 'ARM/NXP/LPC11xx/src/iopincfg_lpc11uxx.c'),
                                '-o', exe], check=True)
                subprocess.run([exe], check=True)

includes = ['tests/lpc/uart/shim', 'include', 'ARM/NXP/LPC17xx/Include']
for compiler, standard in [('gcc', '-std=gnu11'), ('g++', '-std=c++17')]:
    for opt in ['-O0', '-O2']:
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(Path(tmp) / 'lpc17_control_test')
            subprocess.run([compiler, standard, opt, '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            *['-I' + str(root / p) for p in includes],
                            str(root / 'tests/lpc/gpio/lpc17_control_test.c'),
                            '-o', exe], check=True)
            subprocess.run([exe], check=True)
