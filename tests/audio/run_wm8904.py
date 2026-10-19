#!/usr/bin/env python3
"""Exercise production WM8904 control using a modeled register transport."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

with tempfile.TemporaryDirectory(prefix='wm8904-') as temp:
    for opt in ('-O0', '-Os'):
        exe = Path(temp) / 'wm8904'
        cmd = [os.environ.get('CXX', 'g++'), '-std=gnu++23', opt,
               '-Wall', '-Wextra', '-Werror', '-Wno-overloaded-virtual',
               '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
               '-I' + str(ROOT / 'tests/audio/hostport'),
               '-I' + str(ROOT / 'tests/display/hostport'),
               '-I' + str(ROOT / 'include')]
        sources = ['tests/audio/wm8904_test.cpp', 'src/audio/wm8904.cpp',
                   'src/device.cpp', 'src/device_intrf.cpp']
        subprocess.run(cmd + [str(ROOT / p) for p in sources] + ['-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(opt + ': WM8904 checks passed', flush=True)
