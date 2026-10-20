#!/usr/bin/env python3
"""Linux host register checks; requires g++. Does not model flash timing/ECC."""
from pathlib import Path
import subprocess
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[2]
with tempfile.TemporaryDirectory() as tmp:
    exe = str(Path(tmp) / 'check')
    includes = [HERE / 'shim', ROOT / 'ARM/ST/include',
                ROOT / 'ARM/ST/STM32H7xx/include', ROOT / 'include']
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-DSTM32H753xx', *['-I' + str(p) for p in includes],
                    str(HERE / 'check.cpp'), '-o', exe], check=True)
    subprocess.run([exe], check=True)
