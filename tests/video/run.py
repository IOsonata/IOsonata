#!/usr/bin/env python3
"""Exercise the production ArduCAM driver with synchronous and IRQ transports."""
from pathlib import Path
import os
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
with tempfile.TemporaryDirectory(prefix='camera-irq-') as tmp:
    for opt in ('-O0', '-O2'):
        exe = Path(tmp) / 'camera'
        subprocess.run(['g++', '-std=gnu++23', opt, '-g', '-fno-rtti', '-fno-exceptions',
                        '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
                        '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                        '-I' + str(HERE / 'hostport'), '-I' + str(ROOT / 'include'),
                        str(HERE / 'camera_interrupt_test.cpp'),
                        str(ROOT / 'src/video/cam_arducam_mega.cpp'),
                        str(ROOT / 'src/device.cpp'), str(ROOT / 'src/device_intrf.cpp'),
                        '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True,
                       env={**os.environ, 'ASAN_OPTIONS': 'detect_leaks=0'})
