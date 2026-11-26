#!/usr/bin/env python3
"""Build and run the SAM4L I2C master interrupt host check."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
includes = ['-I' + str(root / path) for path in
            ('tests/sam4l/power_shim', 'include', 'include/coredev',
             'ARM/Microchip/SAM4L/include', 'ARM/include')]
sources = [str(root / 'tests/sam4l/i2c_irq_test.cpp'), str(root / 'src/device_intrf.cpp')]

with tempfile.TemporaryDirectory() as tmp:
    for opt in ('-O0', '-O2'):
        binary = Path(tmp) / 'i2c_irq'
        subprocess.run(['g++', '-std=c++17', '-Wall', opt, '-g', '-fno-pie', '-no-pie',
                        '-fpermissive', '-fsanitize=address,undefined',
                        '-fno-sanitize-recover=all', '-D__SAM4LS8C__', *includes,
                        *sources, '-o', str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True)
        print(opt, result.stdout.strip())
        if result.returncode:
            raise RuntimeError(result.stderr or 'SAM4L I2C interrupt check failed')
