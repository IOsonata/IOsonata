#!/usr/bin/env python3
"""RA4M1 UART host test and GNU Arm compile check of uart_ra4m1.cpp.

Run: python3 tests/ra4m1/run_uart_validation.py

uart_test.cpp compiles the driver with the SCI, PRCR, module stop and ICU
model of tests/ra4m1/uart_include, the real CFIFO and DeviceIntrf code, at
-O0, -Os, -O2 and with the address and undefined behavior sanitizers. The
SCI, DMAC, DELSR and PRCR bit definitions and the SCI and DMAC event ids are
copied from the RA4M1 headers into build/uart. When arm-none-eabi-g++ is found, the driver is also
compiled for the Cortex-M4 with the real headers at -O0 and -Os.

The ICU model takes RXI, TXI and ERI as edge events that set the IR flag of
their slot, as the handler order of the Renesas FSP assumes, and the line can
move while the handler runs. Not hardware validation: no exact timing, no
electrical test.

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
import re
import shutil
import subprocess
import sys

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TARGET = ROOT / 'ARM/Renesas/RA4M1'
BUILD = HERE / 'build/uart'


def run(args):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        raise SystemExit(f'FAIL ({p.returncode}): {" ".join(map(str, args))}\n{p.stdout}')
    return p.stdout


def header(name, lines):
    guard = '__' + name.upper().replace('.', '_') + '__'
    text = f'// Copied from the RA4M1 headers by run_uart_validation.py\n'
    text += f'#ifndef {guard}\n#define {guard}\n' + '\n'.join(lines) + f'\n#endif\n'
    (BUILD / name).write_text(text)


def main():
    BUILD.mkdir(parents=True, exist_ok=True)

    cmsis = (TARGET / 'include/R7FA4M1AB.h').read_text()
    header('sci_bits.h', [l for l in cmsis.splitlines()
                          if re.match(r'\s*#define (R_SCI0_|R_SYSTEM_PRCR_|R_DMAC0_|R_DMA_DMAST_|R_ICU_DELSR_)\w+\s', l)])
    icu = (TARGET / 'include/interrupt_ra4m1.h').read_text()
    header('evt_ids.h', [l for l in icu.splitlines()
                         if re.match(r'#define RA4M1_EVTID_(SCI\d_\w+|DMAC\d_INT)\s', l)])

    cxx = os.environ.get('CXX') or shutil.which('g++') or shutil.which('clang++')
    cc = os.environ.get('CC') or shutil.which('gcc') or shutil.which('clang')
    if not cxx or not cc:
        raise SystemExit('A host C and C++ compiler is required.')

    inc = ['-I' + str(HERE / 'uart_include'), '-I' + str(BUILD), '-I' + str(ROOT / 'include')]
    warn = ['-Wall', '-Wextra', '-Wno-unused-parameter', '-Wno-missing-field-initializers', '-Werror']
    modes = [('O0', ['-O0']), ('Os', ['-Os']), ('O2', ['-O2']),
             ('sanitized', ['-O1', '-g', '-fsanitize=address,undefined',
                            '-fno-sanitize-recover=all', '-fno-omit-frame-pointer'])]
    for label, flags in modes:
        cfifo = BUILD / f'cfifo_{label}.o'
        devintrf = BUILD / f'device_intrf_{label}.o'
        test = BUILD / f'uart_test_{label}'
        run([cc, '-std=gnu17', *warn, *flags, *inc, '-c', ROOT / 'src/cfifo.c', '-o', cfifo])
        run([cxx, '-std=gnu++20', *warn, *flags, *inc, '-c', ROOT / 'src/device_intrf.cpp', '-o', devintrf])
        run([cxx, '-std=gnu++20', *warn, *flags, *inc, HERE / 'uart_test.cpp', cfifo, devintrf, '-o', test])
        out = run([test])
        print(label, out.strip().splitlines()[-1])

    arm = shutil.which('arm-none-eabi-g++')
    if arm:
        real = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
                '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(TARGET / 'include')]
        for label in ('O0', 'Os'):
            run([arm, '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16',
                 '-std=gnu++23', f'-{label}', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti',
                 '-ffunction-sections', '-fdata-sections', *real, '-c',
                 TARGET / 'src/uart_ra4m1.cpp', '-o', BUILD / f'uart_ra4m1_arm_{label}.o'])
        print('GNU Arm Cortex-M4 compile of uart_ra4m1.cpp at -O0 and -Os: PASS')
    else:
        print('NOT RUN: arm-none-eabi-g++ not found, no Cortex-M4 compile')
    print('UART checks complete. Hardware not tested.')


if __name__ == '__main__':
    main()
