#!/usr/bin/env python3
"""Exercise the production SSD1315 driver over a modeled SPI controller."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

with tempfile.TemporaryDirectory(prefix='ssd1315-') as td:
    for opt in ('-O0', '-Os'):
        exe = Path(td) / 'ssd1315_test'
        cmd = [os.environ.get('CXX', 'g++'), '-std=gnu++23', opt,
               '-Wall', '-Wextra', '-Werror', '-Wno-overloaded-virtual',
               '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
               '-I' + str(ROOT / 'tests/display/hostport'),
               '-I' + str(ROOT / 'include')]
        sources = ['tests/display/ssd1315_test.cpp', 'src/display/display_ssd1315.cpp',
                   'src/display/display.cpp', 'src/device.cpp', 'src/device_intrf.cpp']
        # Compile font as C, preserving its C linkage declaration in the header.
        font = Path(td) / 'font.o'
        subprocess.run([os.environ.get('CC', 'gcc'), '-I' + str(ROOT / 'include'),
                        '-c', str(ROOT / 'src/display/fonts/syst5x7.c'), '-o', str(font)], check=True)
        subprocess.run(cmd + [str(ROOT / p) for p in sources] +
                       [str(font), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(opt + ': SSD1315 checks passed', flush=True)
