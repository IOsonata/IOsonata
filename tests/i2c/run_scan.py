#!/usr/bin/env python3
"""Test Nordic scan selection with ACK/NACK replies and always-successful starts."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='i2cscan-') as td:
    Path(td, 'nrf.h').write_text('void ScanIdle();\n#define __WFE() ScanIdle()\n')
    for mcu in ('nRF52832', 'nRF52840'):
        for opt in ('-O0', '-Os'):
            exe = str(Path(td, 'scan'))
            cmd = [os.environ.get('CXX', 'g++'), '-std=gnu++23', opt,
                   '-ffunction-sections', '-fdata-sections', '-I' + td,
                   '-I' + str(ROOT / 'include'),
                   '-I' + str(ROOT / 'ARM/Nordic/include'),
                   '-I' + str(ROOT / 'ARM/Nordic/nRF52' / mcu / 'exemples/I2CScan/src'),
                   '-DIRQ_PRIO_NORMAL=3', '-DIRQ_PRIO_LOW=5']
            sources = ['tests/i2c/scan_test.cpp', 'src/device_intrf.cpp', 'src/cfifo.c']
            subprocess.run(cmd + [str(ROOT / p) for p in sources] +
                           ['-Wl,--gc-sections', '-o', exe], check=True)
            result = subprocess.run([exe], capture_output=True, text=True, check=True)
            assert result.stdout.count('Device at') == 3, result.stdout
            for address in ('08', '48', '77'):
                assert 'Device at 0x' + address in result.stdout, result.stdout
            assert '3 devices found' in result.stdout, result.stdout
            print(mcu, opt, 'ACK/NACK scan PASS', flush=True)
