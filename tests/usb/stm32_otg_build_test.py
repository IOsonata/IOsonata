#!/usr/bin/env python3
"""Compile the shared STM32 OTG driver, CDC stack and examples for each MCU.

Requires arm-none-eabi-g++. This is an object compilation check, not a complete
board firmware link or hardware test. ARM_CXXFLAGS may supply toolchain paths.
"""
from pathlib import Path
import os
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TARGETS = (
    ('STM32H753xx', 'STM32H7xx', 'cortex-m7'),
    ('STM32WBA65xx', 'STM32WBAxx', 'cortex-m33'),
    ('STM32F401xC', 'STM32F4xx', 'cortex-m4'),
    ('STM32L476xx', 'STM32L4xx', 'cortex-m4'),
    ('STM32L496xx', 'STM32L4xx', 'cortex-m4'),
)
SOURCES = (
    'ARM/ST/src/usb_ctrlr_stm32.cpp',
    'src/usb/usb.cpp',
    'src/usb/usb_intrf.cpp',
    'src/usb/usb_int.cpp',
    'src/usb/usbd_epalloc.cpp',
    'src/usb/usbd_cdc.cpp',
    'src/usb/usbd_cdc_desc.cpp',
    'exemples/usb/usb_cdc_loopback.cpp',
    'exemples/usb/usb_cdc_prbs_tx.cpp',
)
with tempfile.TemporaryDirectory() as tmp:
    for device, family, cpu in TARGETS:
        for optimization in ('-O0', '-Os'):
            flags = [os.environ.get('ARM_CXX', 'arm-none-eabi-g++'),
                     '-std=gnu++23', '-c', optimization, '-mcpu=' + cpu,
                     '-mthumb', '-mfloat-abi=hard', '-mfpu=' +
                     ('fpv5-d16' if cpu == 'cortex-m7' else
                      'fpv5-sp-d16' if cpu == 'cortex-m33' else 'fpv4-sp-d16'),
                     '-fno-exceptions', '-fno-rtti', '-Wno-volatile',
                     '-D' + device, '-D__PROGRAM_START']
            flags += shlex.split(os.environ.get('ARM_CXXFLAGS', ''))
            for directory in ('include', 'ARM/include', 'ARM/ST/include',
                              'ARM/ST/' + family + '/include',
                              'ARM/CMSIS/Core/Include'):
                flags += ['-I' + str(ROOT / directory)]
            for index, source in enumerate(SOURCES):
                example = []
                if device == 'STM32WBA65xx' and source.startswith('exemples/'):
                    name = ('UsbCdcLoopback' if 'loopback' in source else
                            'UsbCdcPrbsTx')
                    example = ['-I' + str(ROOT / 'ARM/ST/STM32WBAxx' /
                                         'STM32WBA65/exemples' / name / 'src')]
                subprocess.run(flags + example + [str(ROOT / source), '-o',
                                        str(Path(tmp) / (str(index) + '.o'))],
                               check=True)
            print(device, optimization, 'controller, CDC and examples PASS')

