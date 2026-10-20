#!/usr/bin/env python3
"""Build and run the nRF9160 master interrupt checks."""
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
a = p.parse_args()
includes = [a.mdk, root/'include', root/'include/coredev', root/'ARM/Nordic/include',
            root/'ARM/include', a.cmsis]
flags = [a.cxx, '-std=gnu++17', '-mcpu=cortex-m33', '-mthumb', '-O1', '-g',
         '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections', '-DNRF9160_XXAA']
flags += ['-I'+str(i) for i in includes]
with tempfile.TemporaryDirectory() as tmp:
    output = f'{tmp}/master_irq.elf'
    sources = [root/'tests/nordic_master_irq/fixture.cpp', root/'src/device_intrf.cpp',
               root/'src/coredev/shared_intrf.cpp', root/'src/coredev/i2c.cpp']
    args = flags + [str(s) for s in sources] + ['-nostartfiles',
        '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,init']
    args += ['-Wl,-u,'+s for s in ['isr', 'tx', 'rx', 'read_reg', 'busy', 'tx_ready', 'events',
                                   'address', 'dma_max', 'offsets', 'buffer_address']]
    subprocess.run(args + ['-o', output], check=True)
    subprocess.run([sys.executable, str(root/'tests/nordic_master_irq/run.py'), output], check=True)
