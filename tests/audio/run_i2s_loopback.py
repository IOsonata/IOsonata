#!/usr/bin/env python3
"""Check the forwarding, startup and configuration of the actual loopback example."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]

with tempfile.TemporaryDirectory(prefix='i2s-loopback-') as temp:
    for opt in ('-O0', '-Os'):
        exe = Path(temp) / 'loopback'
        flags = ['-std=gnu++23', opt, '-Wall', '-Wextra', '-Werror',
                 '-Wno-overloaded-virtual', '-Wno-missing-field-initializers',
                 '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                 '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                 '-DSTM32WBA65xx']
        flags += ['-I' + str(ROOT / p) for p in
                  ('tests/display/hostport', 'include', 'ARM/ST/include',
                   'ARM/ST/STM32WBAxx/STM32WBA65/exemples/I2sLoopbackDemo/src')]
        sources = ['tests/audio/i2s_loopback_test.cpp', 'src/display/display_ssd1315.cpp',
                   'src/display/display.cpp', 'src/device.cpp', 'src/device_intrf.cpp', 'src/audio/wm8904.cpp', 'src/coredev/timer.cpp']
        font = Path(temp) / 'font.o'
        subprocess.run([os.environ.get('CC', 'gcc'), '-I' + str(ROOT / 'include'),
                        '-c', str(ROOT / 'src/display/fonts/syst5x7.c'), '-o', str(font)], check=True)
        subprocess.run([os.environ.get('CXX', 'g++'), *flags,
                        *[str(ROOT / p) for p in sources], str(font), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(opt + ': I2S loopback checks passed', flush=True)
