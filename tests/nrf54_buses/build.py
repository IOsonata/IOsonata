#!/usr/bin/env python3
"""Compile the Nordic bus drivers, then run the nRF54 register tests."""
import argparse
from pathlib import Path
import subprocess
import sys
import tempfile

root = Path(__file__).resolve().parents[2]
p = argparse.ArgumentParser()
p.add_argument('--cxx', default='arm-none-eabi-g++')
p.add_argument('--mdk', type=Path, required=True, help='Nordic MDK directory containing nrf.h')
p.add_argument('--cmsis', type=Path, default=root / 'ARM/CMSIS/Core/Include')
p.add_argument('--support-root', type=Path, default=root, help=argparse.SUPPRESS)
a = p.parse_args()
includes = [a.mdk, root/'include', a.support_root/'include', a.support_root/'include/coredev',
            root/'ARM/Nordic/include', a.support_root/'ARM/Nordic/include',
            a.support_root/'ARM/include', a.cmsis]
targets = ['NRF54L15_XXAA', 'NRF54LM20A_XXAA', 'NRF54LM20B_XXAA',
           'NRF52832_XXAA', 'NRF52840_XXAA', 'NRF5340_XXAA_APPLICATION',
           'NRF5340_XXAA_NETWORK', 'NRF9160_XXAA', 'NRF9120_XXAA']
with tempfile.TemporaryDirectory() as tmp:
    for chip in targets:
        cpu = 'cortex-m4' if chip.startswith('NRF52') else 'cortex-m33+nodsp' if chip.endswith('NETWORK') else 'cortex-m33'
        flags = [a.cxx, '-std=gnu++17', '-mcpu='+cpu, '-mthumb', '-O1', '-g',
                 '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections', '-D'+chip]
        if chip.startswith('NRF54'):
            flags += ['-DNRF_APPLICATION']
        flags += ['-I'+str(i) for i in includes]
        for bus in ['i2c', 'spi']:
            subprocess.run(flags + ['-c', str(root/f'ARM/Nordic/src/{bus}_nrfx.cpp'),
                                   '-o', f'{tmp}/{chip}_{bus}.o'], check=True)
        print(chip + ': drivers compile', flush=True)
        if chip.startswith('NRF54'):
            output = f'{tmp}/{chip}.elf'
            sources = [root/'tests/nrf54_buses/fixture.cpp', a.support_root/'src/device_intrf.cpp',
                       a.support_root/'src/coredev/shared_intrf.cpp', root/'src/coredev/i2c.cpp']
            args = flags + [str(s) for s in sources] + ['-nostartfiles',
                '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,init']
            args += ['-Wl,-u,'+s for s in ['rate', 'address', 'slave_test', 'transfer', 'offsets', 'register_read', 'timeout_test']]
            subprocess.run(args + ['-o', output], check=True)
            subprocess.run([sys.executable, str(root/'tests/nrf54_buses/run.py'), output,
                            '5' if chip == 'NRF54L15_XXAA' else '7'], check=True)
