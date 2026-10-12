#!/usr/bin/env python3
"""Exercise LPC11Uxx and LPC17xx I2C through the generic interface and real driver source."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
includes = ['tests/lpc546xx/shim', 'tests/lpc_uart/shim', 'include',
            'ARM/NXP/include', 'ARM/NXP/LPC11xx/include']
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'regression')
        subprocess.run(['g++', '-std=c++17', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-missing-field-initializers', '-Wno-unused-parameter',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc_i2c/master_test.cpp'), '-o', exe], check=True)
        subprocess.run([exe], check=True)

# Run the same bus/error scenarios on each LPC17xx descriptor.
includes = ['tests/lpc546xx/shim', 'tests/lpc_uart/shim', 'include',
            'ARM/NXP/include', 'ARM/NXP/LPC17xx/Include']
for opt in ['-O0', '-O2']:
    for devno in range(3):
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(Path(tmp) / 'regression')
            subprocess.run(['g++', '-std=c++17', opt, '-Wall', '-Wextra', '-Werror',
                            '-Wno-missing-field-initializers', '-Wno-unused-parameter',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            '-DTEST_DEVNO=' + str(devno),
                            *['-I' + str(root / p) for p in includes],
                            str(root / 'tests/lpc_i2c/master17_test.cpp'), '-o', exe], check=True)
            subprocess.run([exe], check=True)
