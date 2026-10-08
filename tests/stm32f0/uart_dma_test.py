#!/usr/bin/env python3
"""Run STM32F030x8 UART DMA ownership and interrupt regressions."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
includes = ['tests/stm32f0/shim', 'include', 'ARM/ST/STM32F0xx/include',
            'ARM/ST/STM32F0xx/STM32F030x8/lib/include']
for opt in ['-O0', '-O2']:
    device = 'STM32F030x8'
    with tempfile.TemporaryDirectory() as tmp:
        opts = ['-D' + device, opt, '-fsanitize=undefined', '-fno-sanitize-recover=all'] + ['-I' + str(root / p) for p in includes]
        objects = []
        for i, src in enumerate(['src/cfifo.c', 'ARM/ST/STM32F0xx/src/iopincfg_stm32f0x.c',
                                 'ARM/ST/STM32F0xx/src/system_stm32f0xx.c',
                                 'ARM/ST/STM32F0xx/src/uart_stm32f0x.cpp',
                                 'src/device_intrf.cpp', 'tests/stm32f0/uart_dma_test.cpp']):
            obj = str(Path(tmp) / (str(i) + '.o'))
            cpp = src.endswith('.cpp')
            subprocess.run(['g++' if cpp else 'gcc', '-std=c++17' if cpp else '-std=c11',
                            *opts, '-c', str(root / src), '-o', obj], check=True)
            objects.append(obj)
        exe = str(Path(tmp) / 'regression')
        subprocess.run(['g++', '-no-pie', '-fsanitize=undefined', *objects, '-o', exe], check=True)
        subprocess.run([exe], check=True)
        print(device + ' UART DMA ' + opt + ': PASS', flush=True)
