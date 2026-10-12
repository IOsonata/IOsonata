#!/usr/bin/env python3
"""Run shared legacy SSP rate regressions against both vendor device headers."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
for family, directory, define in [('LPC11xx', 'include', []),
                                  ('LPC17xx', 'Include', ['-DTEST_LPC17'])]:
    includes = ['tests/lpc_uart/shim', 'include', 'ARM/NXP/include',
                f'ARM/NXP/{family}/{directory}']
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'rate_test')
        subprocess.run(['g++', '-std=c++17', '-O2', '-Wall', '-Wextra',
                        '-Wno-missing-field-initializers', '-Wno-unused-parameter',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *define, *['-I' + str(root / p) for p in includes],
                        str(root / 'tests/lpc_ssp/rate_test.cpp'), '-o', exe], check=True)
        subprocess.run([exe], check=True)
