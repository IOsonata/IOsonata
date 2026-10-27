#!/usr/bin/env python3
"""RA4M1 I2C host test and GNU Arm compile check of i2c_ra4m1.cpp.

Run: python3 tests/ra4m1/run_i2c_validation.py

i2c_test.cpp compiles the driver with the IIC, PRCR and module stop model of
tests/ra4m1/i2c_include, the real DeviceIntrf code, at -O0, -Os, -O2
and with the address and undefined behavior sanitizers. The PRCR bit
definitions and the IIC event ids are copied from the RA4M1 headers into
build/i2c. When arm-none-eabi-g++ is found, the driver is also compiled for
the Cortex-M4 with the real headers at -O0 and -Os.

Not hardware validation: the model follows the Renesas FSP r_iic_master and
r_iic_slave flows, with no exact timing and no electrical test.

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
TARGET = ROOT / 'ARM/Renesas/RA4/RA4M1'
BUILD = HERE / 'build/i2c'


def run(args):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode != 0:
        raise SystemExit(f'FAIL ({p.returncode}): {" ".join(map(str, args))}\n{p.stdout}')
    return p.stdout


def header(name, lines):
    guard = '__' + name.upper().replace('.', '_') + '__'
    text = f'// Copied from the RA4M1 headers by run_i2c_validation.py\n'
    text += f'#ifndef {guard}\n#define {guard}\n' + '\n'.join(lines) + f'\n#endif\n'
    (BUILD / name).write_text(text)


def main():
    BUILD.mkdir(parents=True, exist_ok=True)

    cmsis = (TARGET / 'include/R7FA4M1AB.h').read_text()
    header('sys_bits.h', [l for l in cmsis.splitlines()
                          if re.match(r'\s*#define (R_SYSTEM_PRCR_)\w+\s', l)])
    icu = (TARGET / 'include/interrupt_ra4m1.h').read_text()
    header('evt_ids.h', [l for l in icu.splitlines()
                         if re.match(r'#define RA4M1_EVTID_IIC\d_\w+\s', l)])

    cxx = os.environ.get('CXX') or shutil.which('g++') or shutil.which('clang++')
    if not cxx:
        raise SystemExit('A host C++ compiler is required.')

    inc = ['-I' + str(HERE / 'i2c_include'), '-I' + str(BUILD), '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/Renesas/RA4/RA4M1/include')]
    warn = ['-Wall', '-Wextra', '-Wno-unused-parameter', '-Wno-missing-field-initializers', '-Werror']
    modes = [('O0', ['-O0']), ('Os', ['-Os']), ('O2', ['-O2']),
             ('sanitized', ['-O1', '-g', '-fsanitize=address,undefined',
                            '-fno-sanitize-recover=all', '-fno-omit-frame-pointer'])]
    for label, flags in modes:
        objs = []
        for src in (ROOT / 'src/device_intrf.cpp', TARGET / 'src/i2c_ra4m1.cpp'):
            obj = BUILD / f'{src.stem}_{label}.o'
            run([cxx, '-std=gnu++20', *warn, *flags, *inc, '-c', src, '-o', obj])
            objs.append(obj)
        test = BUILD / f'i2c_test_{label}'
        run([cxx, '-std=gnu++20', *warn, *flags, *inc, HERE / 'i2c_test.cpp', *objs, '-o', test])
        out = run([test])
        print(label, out.strip().splitlines()[-1])

    real = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
            '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(TARGET / 'include')]
    flags = ['-std=gnu++23', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti',
             '-ffunction-sections', '-fdata-sections']
    arm = shutil.which('arm-none-eabi-g++')
    zig = None
    if not arm:
        try:
            import ziglang      # noqa: F401
            zig = [sys.executable, '-m', 'ziglang', 'c++', '-target', 'thumb-linux-musleabihf',
                   '-mcpu=cortex_m4']
        except ImportError:
            zig = None
    if arm:
        cmd = [arm, '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=hard', '-mfpu=fpv4-sp-d16']
        name = 'GNU Arm'
    elif zig:
        cmd = zig
        name = 'clang (ziglang package)'
    else:
        cmd = None
    if cmd:
        for label in ('O0', 'Os'):
            run([*cmd, *flags, f'-{label}', *real, '-c',
                 TARGET / 'src/i2c_ra4m1.cpp', '-o', BUILD / f'i2c_ra4m1_arm_{label}.o'])
        print(f'{name} Cortex-M4 compile of i2c_ra4m1.cpp at -O0 and -Os: PASS')
    else:
        print('NOT RUN: no arm-none-eabi-g++ and no ziglang package, no Cortex-M4 compile')
    print('I2C checks complete. Hardware not tested.')


if __name__ == '__main__':
    main()
