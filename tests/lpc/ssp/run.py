#!/usr/bin/env python3
"""Run legacy SSP rate and polling-transfer regressions."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
for family, directory, define in [('LPC11xx', 'include', []),
                                  ('LPC17xx', 'Include', ['-DTEST_LPC17'])]:
    includes = ['tests/lpc/uart/shim', 'include', 'ARM/NXP/include',
                f'ARM/NXP/{family}/{directory}']
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'rate_test')
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                        '-Wno-missing-field-initializers', '-Wno-unused-parameter',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *define, *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc/ssp/rate_test.cpp'), '-o', exe], check=True)
        subprocess.run([exe], check=True)

# Exercise the transfer model in both optimization modes.
for opt in ['-O0', '-O2']:
    includes = ['tests/lpc/uart/shim', 'include', 'ARM/NXP/include',
                'ARM/NXP/LPC11xx/include']
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'transfer_test')
        subprocess.run(['g++', '-std=c++17', opt, '-Wall', '-Wextra',
                        '-Wno-missing-field-initializers', '-Wno-unused-parameter',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc/ssp/transfer_test.cpp'), '-o', exe], check=True)
        subprocess.run([exe], check=True)

# LPC17xx family setup with the shared transfer code and real pin configuration.
for opt in ['-O0', '-O2']:
    includes = ['tests/lpc/uart/shim', 'include', 'ARM/NXP/include',
                'ARM/NXP/LPC17xx/Include']
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'init17_test')
        subprocess.run(['g++', '-std=c++17', opt, '-Wall', '-Wextra',
                        '-Wno-missing-field-initializers', '-Wno-unused-parameter',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc/ssp/init17_test.cpp'), '-o', exe], check=True)
        subprocess.run([exe], check=True)
