#!/usr/bin/env python3
"""STM32 UART host test and GNU Arm compile check of ARM/ST/src/uart_stm32.cpp.

Run: python3 tests/stm32/run_uart.py

uart_test.cpp runs the driver with the ST device header of each part below,
the host core header of tests/stm32/shim and the real CFIFO and DeviceIntrf
code, at -O0 and -O2 with the undefined behavior sanitizer. The USART, DMA
and RCC registers are modelled by trapping their accesses, see uart_test.cpp.
Linux x86-64 only.

When arm-none-eabi-g++ is found, the driver is also compiled for each part
with the CMSIS core headers at -O0 and -Os, warnings as errors.

Not hardware validation: no timing, no electrical test.

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import os
from pathlib import Path
import platform
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
DRIVER = ROOT / 'ARM/ST/src/uart_stm32.cpp'

# Part, family directory, Cortex-M core and floating point flags
PARTS = [
    ('STM32F030x6', 'STM32F0xx', 'cortex-m0', []),
    ('STM32F030x8', 'STM32F0xx', 'cortex-m0', []),
    ('STM32F030xC', 'STM32F0xx', 'cortex-m0', []),
    ('STM32F070x6', 'STM32F0xx', 'cortex-m0', []),
    ('STM32F070xB', 'STM32F0xx', 'cortex-m0', []),
    ('STM32F091xC', 'STM32F0xx', 'cortex-m0', []),
    ('STM32L432xx', 'STM32L4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32L476xx', 'STM32L4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32L496xx', 'STM32L4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32L4S9xx', 'STM32L4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32F401xC', 'STM32F4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32F405xx', 'STM32F4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32F413xx', 'STM32F4xx', 'cortex-m4', ['-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']),
    ('STM32WBA65xx', 'STM32WBAxx', 'cortex-m33', ['-mfloat-abi=hard', '-mfpu=fpv5-sp-d16']),
]


def run(args):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        raise SystemExit(f'FAIL ({p.returncode}): {" ".join(map(str, args))}\n{p.stdout}')
    return p.stdout


def main():
    host = sys.platform.startswith('linux') and platform.machine() in ('x86_64', 'AMD64')
    cxx = os.environ.get('CXX') or shutil.which('g++')
    cc = os.environ.get('CC') or shutil.which('gcc')

    if not host:
        print('NOT RUN: the host test needs Linux x86-64')
    elif not cxx or not cc:
        raise SystemExit('A host C and C++ compiler is required.')
    else:
        with tempfile.TemporaryDirectory(prefix='stm32uart-') as tmp:
            tmp = Path(tmp)
            for part, family, _, _ in PARTS:
                inc = ['-I' + str(HERE / 'shim'), '-I' + str(ROOT / 'include'),
                       '-I' + str(ROOT / 'ARM/ST/include'), '-I' + str(ROOT / 'ARM/ST' / family / 'include')]
                for label, flags in (('O0', ['-O0']), ('O2', ['-O2'])):
                    opt = ['-D' + part, *flags, '-g', '-fsanitize=undefined', '-fno-sanitize-recover=all']
                    objs = []
                    for src, lang in ((ROOT / 'src/cfifo.c', 'c'), (ROOT / 'src/device_intrf.cpp', 'c++'),
                                      (DRIVER, 'c++'), (HERE / 'uart_test.cpp', 'c++')):
                        obj = tmp / f'{part}_{label}_{src.stem}.o'
                        if lang == 'c':
                            run([cc, '-std=gnu17', *opt, *inc, '-c', src, '-o', obj])
                        else:
                            run([cxx, '-std=gnu++23', '-fno-exceptions', '-fno-rtti', *opt, *inc,
                                 '-c', src, '-o', obj])
                        objs.append(obj)
                    exe = tmp / f'{part}_{label}'
                    run([cxx, '-no-pie', '-fsanitize=undefined', *objs, '-o', exe])
                    out = run([exe])
                    print(f'{part} {label}: {out.strip().splitlines()[-1]}', flush=True)

    arm = shutil.which('arm-none-eabi-g++')
    if arm:
        with tempfile.TemporaryDirectory(prefix='stm32uart-arm-') as tmp:
            for part, family, cpu, fpu in PARTS:
                inc = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
                       '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/ST/include'),
                       '-I' + str(ROOT / 'ARM/ST' / family / 'include')]
                for label in ('O0', 'Os'):
                    run([arm, f'-mcpu={cpu}', '-mthumb', *fpu, '-D' + part, f'-{label}', '-std=gnu++23',
                         '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', '-ffunction-sections',
                         '-fdata-sections', *inc, '-c', DRIVER, '-o', Path(tmp) / f'{part}_{label}.o'])
        print(f'GNU Arm compile of uart_stm32.cpp for {len(PARTS)} parts at -O0 and -Os: PASS')
    else:
        print('NOT RUN: arm-none-eabi-g++ not found, no Arm compile')
    print('STM32 UART checks complete. Hardware not tested.')


if __name__ == '__main__':
    main()
