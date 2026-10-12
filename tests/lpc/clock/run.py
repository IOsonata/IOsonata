#!/usr/bin/env python3
"""Run LPC11Uxx clock decoding against vendor CMSIS peripheral declarations."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
includes = ['tests/lpc/uart/shim', 'include', 'ARM/NXP/LPC11xx/include']
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'clock_test')
        subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-unused-parameter', '-Wno-missing-field-initializers',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc/clock/clock_test.c'),
                        str(root / 'ARM/NXP/LPC11xx/src/system_LPC11Uxx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)

# Compile the production source as C, then link callers in both languages.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        obj = str(Path(tmp) / 'system.o')
        flags = [opt, '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                 '-Wno-missing-field-initializers', '-fsanitize=undefined',
                 '-fno-sanitize-recover=all',
                 *['-I' + str(root / p) for p in includes]]
        subprocess.run(['gcc', '-std=gnu11', *flags, '-c',
                        str(root / 'ARM/NXP/LPC11xx/src/system_LPC11Uxx.c'),
                        '-o', obj], check=True)
        for compiler, standard in [('gcc', '-std=gnu11'), ('g++', '-std=c++17')]:
            exe = str(Path(tmp) / 'clock_link_test')
            subprocess.run([compiler, standard, *flags,
                            str(root / 'tests/lpc/clock/clock_link_test.c'),
                            obj, '-o', exe], check=True)
            subprocess.run([exe], check=True, timeout=5)

# Decode LPC17xx clocks using the real system source and vendor peripheral types.
# Unused startup/setter sections are discarded; no PLL transitions are simulated.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'clock17_test')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-sign-compare', '-Wno-unused-but-set-variable',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                        '-include', str(root / 'tests/lpc/startup/system17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        str(root / 'tests/lpc/clock/clock17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/system_LPC17xx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)

# Compile the LPC17xx source and register fixture as C; link public C/C++ callers.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        flags = [opt, '-Wall', '-Wextra', '-Werror',
                 '-Wno-sign-compare', '-Wno-unused-but-set-variable',
                 '-fsanitize=undefined', '-fno-sanitize-recover=all',
                 '-ffunction-sections', '-fdata-sections',
                 '-I' + str(root / 'tests/lpc/uart/shim'),
                 '-I' + str(root / 'ARM/NXP/LPC17xx/Include')]
        obj = str(Path(tmp) / 'system17.o')
        fixture = str(Path(tmp) / 'flash17.o')
        subprocess.run(['gcc', '-std=gnu17', *flags,
                        '-include', str(root / 'tests/lpc/clock/flash17_shim.h'),
                        '-c', str(root / 'ARM/NXP/LPC17xx/src/system_LPC17xx.c'),
                        '-o', obj], check=True)
        subprocess.run(['gcc', '-std=gnu17', *flags, '-Dmain=TestClock17Run',
                        '-c', str(root / 'tests/lpc/clock/flash17_test.c'),
                        '-o', fixture], check=True)
        for compiler, standard in [('gcc', '-std=gnu17'), ('g++', '-std=c++17')]:
            exe = str(Path(tmp) / 'clock17_link_test')
            subprocess.run([compiler, standard, *flags, '-Wl,--gc-sections',
                            str(root / 'tests/lpc/clock/clock17_link_test.c'),
                            obj, fixture, '-o', exe], check=True)
            subprocess.run([exe], check=True, timeout=10)
