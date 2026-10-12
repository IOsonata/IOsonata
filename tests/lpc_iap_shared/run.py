#!/usr/bin/env python3
"""Link C and C++ callers against the shared LPC IAP C implementation."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        obj = str(Path(tmp) / 'iap.o')
        flags = [opt, '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                 '-fsanitize=undefined', '-fno-sanitize-recover=all',
                 '-I' + str(root / 'ARM/NXP/include')]
        subprocess.run(['gcc', '-std=gnu11', *flags, '-Wno-pointer-to-int-cast',
                        '-DLPC54628_SERIES', '-c',
                        str(root / 'ARM/NXP/src/lpc_iap.c'), '-o', obj], check=True)
        for test in ['signature_test', 'eeprom_page_test']:
            for compiler, standard in [('gcc', '-std=gnu11'), ('g++', '-std=c++17')]:
                exe = str(Path(tmp) / test)
                subprocess.run([compiler, standard, *flags,
                                str(root / 'tests/lpc_iap_shared' / (test + '.c')),
                                obj, '-o', exe], check=True)
                subprocess.run([exe], check=True)
