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

"""Run HttpServer against its socket model at O0 and O2 with ASan and UBSan."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def run(args):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    if p.returncode:
        raise SystemExit(' '.join(map(str, args)) + '\n' + p.stdout)
    return p.stdout


def main():
    sources = [ROOT / 'src/device_intrf.cpp', ROOT / 'src/net/http_server.cpp',
               ROOT / 'tests/net/http_server_test.cpp']
    with tempfile.TemporaryDirectory(prefix='iosonata-http-') as td:
        for opt in ('O0', 'O2'):
            exe = Path(td) / opt
            run(['g++', '-std=gnu++23', '-' + opt, '-g', '-Wall', '-Wextra', '-Werror',
                 '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                 '-I' + str(ROOT / 'include'), *sources, '-o', exe])
            os.environ['ASAN_OPTIONS'] = 'detect_leaks=0'
            print(opt + ': ' + run([exe]).strip(), flush=True)


if __name__ == '__main__':
    main()
