#!/usr/bin/env python3
"""Run the shared WBA EEPROM demo through the production generic Nvm driver."""
import os
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
with tempfile.TemporaryDirectory(prefix='wba-eeprom-') as td:
    for opt in ('-O0', '-Os'):
        exe = str(Path(td) / 'test')
        cmd = [os.environ.get('CXX', 'g++'), '-std=gnu++23', opt,
               '-Wall', '-Wextra', '-Werror', '-Wno-overloaded-virtual',
               '-Wno-missing-field-initializers', '-DEEPROM_DEMO_UART=0',
               '-DSTM32WBA65xx', '-ffunction-sections', '-fdata-sections',
               '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
               '-I' + str(ROOT / 'tests/display/hostport'),
               '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/ST/include'),
               '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/exemples/SeepDemo/src')]
        sources = ['tests/storage/wba_eeprom_test.cpp', 'src/storage/nvm.cpp',
                   'src/device.cpp', 'src/device_intrf.cpp']
        subprocess.run(cmd + [str(ROOT / p) for p in sources] +
                       ['-Wl,--gc-sections', '-o', exe], check=True)
        subprocess.run([exe], check=True)
        print(opt + ': PASS', flush=True)
