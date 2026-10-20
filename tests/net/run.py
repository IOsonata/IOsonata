#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2026, I-SYST inc., all rights reserved
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Run the real lwIP packet tests; LWIP_ROOT selects the SDK checkout."""
import os
from pathlib import Path
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
LWIP = Path(os.environ.get('LWIP_ROOT', ROOT.parent / 'external/lwip'))


def run(args):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode:
        raise SystemExit(' '.join(map(str, args)) + '\n' + p.stdout)
    return p.stdout


def main():
    if not (LWIP / 'src/include/lwip/init.h').is_file():
        raise SystemExit('Set LWIP_ROOT to the SDK lwIP checkout (tested with 2.2.0).')
    sources = sorted((LWIP / 'src/core').glob('*.c'))
    sources += sorted((LWIP / 'src/core/ipv4').glob('*.c'))
    sources += [LWIP / 'src/netif/ethernet.c', ROOT / 'src/device_intrf.cpp',
                ROOT / 'src/net/ip_lwip.cpp', ROOT / 'tests/net/packet_test.cpp']
    inc = ['-I' + str(ROOT / p) for p in ('tests/net/hostport', 'include', 'include/net/lwip')]
    inc += ['-I' + str(LWIP / 'src/include')]
    with tempfile.TemporaryDirectory(prefix='iosonata-net-') as td:
        out = Path(td)
        for opt in ('O0', 'O2'):
            objects = []
            for i, source in enumerate(sources):
                obj = out / (str(i) + '_' + opt + '.o')
                cpp = source.suffix == '.cpp'
                compiler = 'g++' if cpp else 'gcc'
                lang = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
                run([compiler, *lang, '-' + opt, '-g', '-fsanitize=address,undefined',
                     '-fno-sanitize-recover=all', *inc, '-c', source, '-o', obj])
                objects.append(obj)
            exe = out / opt
            run(['g++', '-fsanitize=address,undefined', *objects, '-o', exe])
            os.environ['ASAN_OPTIONS'] = 'detect_leaks=0'
            print(opt + ': ' + run([exe]).strip(), flush=True)
    arm = shutil.which('arm-none-eabi-g++')
    if arm:
        inc = ['-I' + str(ROOT / p) for p in ('include', 'include/net/lwip', 'ARM/include',
               'ARM/CMSIS/Core/Include', 'ARM/ST/STM32H7xx/include')]
        inc += ['-I' + str(LWIP / 'src/include')]
        with tempfile.TemporaryDirectory(prefix='net-arm-') as td:
            for opt in ('O0', 'Os'):
                for source in ('src/net/ip_lwip.cpp', 'ARM/ST/STM32H7xx/src/ethernet_stm32h7.cpp'):
                    run([arm, '-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx', '-std=gnu++23',
                         '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', '-' + opt, *inc,
                         '-c', ROOT / source, '-o', Path(td) / (Path(source).stem + '.o')])
        print('H753 Ethernet and generic IP Arm compile O0/Os: PASS')


if __name__ == '__main__':
    main()
