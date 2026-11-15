#!/usr/bin/env python3
"""Run SAM4L runtime clock register tests on a Linux host."""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='sam4l-clock-') as tmp:
    for part in ('SAM4LC8C', 'SAM4LS8C'):
        for opt in ('-O0', '-O2'):
            binary = Path(tmp) / (part + opt)
            subprocess.run([
                'gcc', '-std=gnu17', opt, '-Wall', '-Wextra',
                '-Wno-pointer-to-int-cast', '-Wno-attributes',
                '-Wno-missing-field-initializers', '-D__' + part + '__',
                '-fsanitize=undefined', '-fno-sanitize-recover=all',
                '-I' + str(ROOT / 'tests/sam4l/clock_shim'),
                '-I' + str(ROOT / 'include'),
                '-I' + str(ROOT / 'ARM/Microchip/SAM4L/include'),
                str(ROOT / 'tests/sam4l/clock_test.c'), '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True, timeout=30)
