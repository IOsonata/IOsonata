#!/usr/bin/env python3
"""STM32WBA host test of the LPTIM1 and LPTIM2 timers.

Run: python3 tests/stm32wba/run.py

wba_timer_test.cpp runs the shared STM32 timer driver (ARM/ST/src
timer_stm32.cpp, timer_lptim_stm32.cpp, timer_tim_stm32.cpp) with the ST
STM32WBA65 device header and the host core header of tests/stm32/shim, at
-O0 and -O2 with the undefined behavior sanitizer. Peripheral registers are
plain memory mapped at their addresses. Linux only.

When arm-none-eabi-g++ is found, the sources are also compiled for the
STM32WBA65 at -O0 and -Os, warnings as errors. tests/stm32/run_timer.py
compiles them for every STM32 part.

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
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
TIMER = [ROOT / 'ARM/ST/src' / f for f in ('timer_stm32.cpp', 'timer_lptim_stm32.cpp', 'timer_tim_stm32.cpp')]


def run(args):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        raise SystemExit(f'FAIL ({p.returncode}): {" ".join(map(str, args))}\n{p.stdout}')
    return p.stdout


def main():
    cxx = os.environ.get('CXX') or shutil.which('g++')
    cc = os.environ.get('CC') or shutil.which('gcc')

    if not sys.platform.startswith('linux'):
        print('NOT RUN: the host test needs Linux')
    elif not cxx or not cc:
        raise SystemExit('A host C and C++ compiler is required.')
    else:
        inc = ['-I' + str(ROOT / 'tests/stm32/shim'), '-I' + str(ROOT / 'include'),
               '-I' + str(ROOT / 'ARM/ST/include'), '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include')]
        with tempfile.TemporaryDirectory(prefix='stm32wba-') as tmp:
            tmp = Path(tmp)
            for label, flags in (('O0', ['-O0']), ('O2', ['-O2'])):
                opt = ['-DSTM32WBA65xx', *flags, '-g', '-fsanitize=undefined', '-fno-sanitize-recover=all']
                objs = []
                for src, lang in [(f, 'c++') for f in TIMER] + [(HERE / 'wba_timer_test.cpp', 'c++')]:
                    obj = tmp / f'{label}_{src.stem}.o'
                    if lang == 'c':
                        run([cc, '-std=gnu17', *opt, *inc, '-c', src, '-o', obj])
                    else:
                        run([cxx, '-std=gnu++23', '-fno-exceptions', '-fno-rtti', *opt, *inc, '-c', src, '-o', obj])
                    objs.append(obj)
                exe = tmp / f'wba_{label}'
                run([cxx, '-no-pie', '-fsanitize=undefined', *objs, '-o', exe])
                print(f'STM32WBA65 {label}: {run([exe]).strip().splitlines()[-1]}', flush=True)

    arm = shutil.which('arm-none-eabi-g++')
    if arm:
        inc = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
               '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/ST/include'),
               '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include')]
        with tempfile.TemporaryDirectory(prefix='stm32wba-arm-') as tmp:
            for label in ('O0', 'Os'):
                for src in TIMER:
                    lang = ['-x', 'c', '-std=gnu17'] if src.suffix == '.c' else ['-std=gnu++23', '-fno-exceptions', '-fno-rtti']
                    run([arm, '-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv5-sp-d16', '-DSTM32WBA65xx',
                         f'-{label}', *lang, '-Wall', '-Werror', '-ffunction-sections', '-fdata-sections', *inc,
                         '-c', src, '-o', Path(tmp) / f'{src.stem}_{label}.o'])
        print('GNU Arm compile of the timer sources for the WBA65 at -O0 and -Os: PASS')
    else:
        print('NOT RUN: arm-none-eabi-g++ not found, no Arm compile')
    print('STM32WBA checks complete. Hardware not tested.')


if __name__ == '__main__':
    main()
