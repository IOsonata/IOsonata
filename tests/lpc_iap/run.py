#!/usr/bin/env python3
"""Test production LPC11Uxx IAP wrappers with a substitute ROM entry."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'iap_test')
        subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-pointer-to-int-cast', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all',
                        '-I' + str(root / 'tests/lpc_iap/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                        str(root / 'tests/lpc_iap/iap_test.c'),
                        str(root / 'ARM/NXP/LPC11xx/src/iap_lpc11uxx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)
