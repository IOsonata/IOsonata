#!/usr/bin/env python3
"""Check F401 startup selects linked vectors rather than FLASH_BASE."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory() as tmp:
    exe = str(Path(tmp) / 'vectors')
    subprocess.run(['gcc', '-D_GNU_SOURCE', '-DSTM32F401xC', '-std=c11', '-O2',
                    '-I' + str(root / 'tests/stm32f4/shim'),
                    '-I' + str(root / 'ARM/ST/STM32F4xx/include'),
                    str(root / 'ARM/ST/STM32F4xx/src/system_stm32f4xx.c'),
                    str(root / 'tests/stm32f4/vector_test.c'), '-o', exe], check=True)
    subprocess.run([exe], check=True)
    print('STM32F401xC linked vectors: PASS')
