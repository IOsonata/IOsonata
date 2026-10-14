#!/usr/bin/env python3
"""Test production LPC11Uxx IAP wrappers with a substitute ROM entry."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'iap_test')
        subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                        '-Wno-pointer-to-int-cast', '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-DLPC11XX_SERIES',
                        '-I' + str(root / 'tests/lpc/iap_shared/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                        '-I' + str(root / 'ARM/NXP/include'),
                        str(root / 'tests/lpc/iap/iap_test.c'),
                        str(root / 'ARM/NXP/src/lpc_iap.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)
