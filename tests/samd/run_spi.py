#!/usr/bin/env python3
"""Run the complete SAMD SPI implementation with host register/DMAC stand-ins.

Only its hardware includes are replaced. Tests cover mode selection, transfer
completion and resource release; they do not validate electrical timing.
"""
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/Microchip/SAMD/src/spi_samd.cpp').read_text()
with tempfile.TemporaryDirectory(prefix='samd-spi-') as directory:
    out = Path(directory)
    (out / 'spi_driver.h').write_text('\n'.join(
        line for line in source.splitlines() if not line.startswith('#include')))
    for opt in ('-O0', '-Os'):
        binary = out / ('spi' + opt)
        subprocess.run(['g++', '-std=gnu++23', opt, '-fno-rtti',
                        '-Wall', '-Wextra', '-Werror', '-Wno-class-memaccess',
                        '-Wno-missing-field-initializers',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(out), '-I' + str(ROOT / 'include'),
                        '-I' + str(ROOT / 'ARM/Microchip/SAMD/SAMD11/include'),
                        str(ROOT / 'tests/samd/spi_test.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
