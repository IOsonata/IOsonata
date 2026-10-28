#!/usr/bin/env python3
"""Run the touch panel drivers (FT5406, GT911) on the I2C and pin model."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

with tempfile.TemporaryDirectory(prefix='touch-') as temp:
    for opt in ('-O0', '-Os'):
        exe = Path(temp) / 'touch'
        cmd = [os.environ.get('CXX', 'g++'), '-std=gnu++23', opt,
               '-Wall', '-Wextra', '-Werror', '-Wno-overloaded-virtual',
               '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer',
               '-I' + str(ROOT / 'tests/display/touch_hostport'),
               '-I' + str(ROOT / 'tests/display/hostport'),
               '-I' + str(ROOT / 'include')]
        sources = ['tests/display/touch_test.cpp', 'src/display/touch.cpp', 'src/display/touch_ft5406.cpp',
                   'src/display/touch_gt911.cpp', 'src/device.cpp', 'src/device_intrf.cpp']
        subprocess.run(cmd + [str(ROOT / p) for p in sources] + ['-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(opt + ': touch checks passed', flush=True)
