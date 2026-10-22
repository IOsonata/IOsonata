#!/usr/bin/env python3
"""Build and run the Nordic master interrupt checks for each MCU."""
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
p.add_argument('--chip', action='append', help='MCU define, can be repeated (default: all)')
a = p.parse_args()
includes = [a.mdk, root/'include', root/'include/coredev', root/'ARM/Nordic/include',
            root/'ARM/include', a.cmsis]
chips = a.chip or ['NRF9160_XXAA', 'NRF52832_XXAA', 'NRF54L15_XXAA', 'NRF54LM20A_XXAA']
with tempfile.TemporaryDirectory() as tmp:
    for chip in chips:
        cpu = 'cortex-m4' if chip.startswith('NRF52') else 'cortex-m33'
        flags = [a.cxx, '-std=gnu++17', '-mcpu='+cpu, '-mthumb', '-O1', '-g',
                 '-fno-inline-functions-called-once',
                 '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections', '-D'+chip]
        if chip.startswith('NRF54'):
            flags += ['-DNRF_APPLICATION']
        flags += ['-I'+str(i) for i in includes]
        output = f'{tmp}/{chip}.elf'
        sources = [root/'tests/nordic_master_irq/fixture.cpp', root/'src/device_intrf.cpp',
                   root/'src/coredev/shared_intrf.cpp', root/'src/coredev/i2c.cpp']
        args = flags + [str(s) for s in sources] + ['-nostartfiles',
            '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,init']
        args += ['-Wl,-u,'+s for s in ['isr', 'tx', 'rx', 'read_reg', 'read_command', 'write_data', 'busy', 'tx_ready', 'events',
                                       'address', 'dma_max', 'offsets', 'buffer_address', 'twim', 'twi', 'spi_legacy', 'txbuf_size', 'txbuf_address']]
        subprocess.run(args + ['-o', output], check=True)
        subprocess.run([sys.executable, str(root/'tests/nordic_master_irq/run.py'), output, chip], check=True)
