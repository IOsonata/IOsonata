#!/usr/bin/env python3
"""Run the generic SD card driver on the SPI mode and SD bus card models."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]

with tempfile.TemporaryDirectory(prefix='sdcard-') as temp:
    for opt in ('-O0', '-Os'):
        exe = Path(temp) / 'sdcard'
        cmd = [os.environ.get('CXX', 'g++'), '-std=gnu++17', opt,
               '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
               '-fsanitize=address,undefined', '-fno-sanitize-recover=all', '-fno-omit-frame-pointer',
               '-I' + str(ROOT / 'tests/storage/sdcard/hostport'),
               '-I' + str(ROOT / 'include')]
        sources = ['tests/storage/sdcard/sdcard_test.cpp', 'src/storage/sdcard_impl.cpp',
                   'src/storage/diskio_impl.cpp', 'src/device_intrf.cpp', 'src/crc.c']
        subprocess.run(cmd + ['-x', 'c++'] + [str(ROOT / p) for p in sources] + ['-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(opt + ': SD card checks passed', flush=True)
