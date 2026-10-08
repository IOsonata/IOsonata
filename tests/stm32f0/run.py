#!/usr/bin/env python3
"""Run the STM32F0 host register regressions with the repository's ST headers."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
includes = ['tests/stm32f0/shim', 'include', 'ARM/ST/STM32F0xx/include',
            'ARM/ST/STM32F0xx/STM32F030x8/lib/include']
for device in ['STM32F030x6', 'STM32F030x8', 'STM32F030xC', 'STM32F070x6', 'STM32F070xB']:
    with tempfile.TemporaryDirectory() as tmp:
        opts = ['-D' + device, '-O2'] + ['-I' + str(root / p) for p in includes]
        objects = []
        for i, src in enumerate(['src/cfifo.c', 'ARM/ST/STM32F0xx/src/iopincfg_stm32f0x.c',
                                 'ARM/ST/STM32F0xx/src/system_stm32f0xx.c',
                                 'ARM/ST/STM32F0xx/src/uart_stm32f0x.cpp',
                                 'tests/stm32f0/register_test.cpp']):
            obj = str(Path(tmp) / (str(i) + '.o'))
            cpp = src.endswith('.cpp')
            subprocess.run(['g++' if cpp else 'gcc', '-std=c++17' if cpp else '-std=c11',
                            *opts, '-c', str(root / src), '-o', obj], check=True)
            objects.append(obj)
        exe = str(Path(tmp) / 'regression')
        subprocess.run(['g++', *objects, '-o', exe], check=True)
        subprocess.run([exe], check=True)
        print(device + ': PASS', flush=True)
