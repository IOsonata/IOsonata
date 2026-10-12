#!/usr/bin/env python3
"""Check the LPC11Uxx USB interface from C and C++; no controller is executed."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
for compiler, standard in [('gcc', 'c11'), ('g++', 'c++17')]:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'header_test')
        subprocess.run([compiler, '-std=' + standard, '-Wall', '-Wextra',
                        '-Werror', '-I' + str(root / 'include'),
                        '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                        str(root / 'tests/lpc/usb/header_test.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)
print('LPC11Uxx USB C/C++ header checks: PASS')
