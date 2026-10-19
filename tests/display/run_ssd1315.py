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
        fonts = []
        for name in ('syst5x7.c', 'FreeMono/FreeMono8pt.c', 'FreeSans/FreeSansBold24pt.c'):
            font = Path(td) / (Path(name).stem + '.o')
            subprocess.run([os.environ.get('CC', 'gcc'), '-I' + str(ROOT / 'include'),
                            '-c', str(ROOT / 'src/display/fonts' / name), '-o', str(font)], check=True)
            fonts.append(str(font))
        subprocess.run(cmd + [str(ROOT / p) for p in sources] +
                       fonts + ['-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        print(opt + ': SSD1315 checks passed', flush=True)
        # Exercise the configuration supplied to SPIInit by the real console
        # example, not just a manually configured mock display interface.
        console = Path(td) / 'ssd1315_console_test'
        console_sources = ['tests/display/ssd1315_console_test.cpp',
                           'src/display/display_ssd1315.cpp', 'src/display/display.cpp',
                           'src/device.cpp', 'src/device_intrf.cpp']
        subprocess.run(cmd + ['-DSTM32WBA65xx', '-ffunction-sections', '-fdata-sections',
                             '-Wno-unused-but-set-variable',
                             '-Wno-missing-field-initializers',
                             '-I' + str(ROOT / 'ARM/ST/include'),
                             '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/STM32WBA65/exemples/LCDConsoleDisplayDemo/src'),
                             '-Wl,--gc-sections'] +
                       [str(ROOT / p) for p in console_sources] + fonts +
                       ['-o', str(console)], check=True)
        subprocess.run([str(console)], check=True)
