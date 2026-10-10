#!/usr/bin/env python3
"""Run the LPC546xx host register regressions with the repository's NXP headers."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
includes = ['tests/lpc546xx/shim', 'include', 'ARM/NXP/LPC546xx/include',
            'ARM/NXP/LPC546xx/LPC54628/lib/include']
for test in ['timer_test.cpp']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'regression')
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-DLPC54628_SERIES', *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc546xx' / test), '-o', exe], check=True)
        subprocess.run([exe], check=True)
