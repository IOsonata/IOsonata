#!/usr/bin/env python3
"""Cross-build the R9A02G021 SPI slave emulator firmware and run the check."""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='riscv-none-elf-')
parser.add_argument('--build-only', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
out = tests / 'build' / 'spi_slave'
out.mkdir(parents=True, exist_ok=True)
port = 'RISCV/Renesas/R9A02'
includes = ['include', 'RISCV/include', port + '/include']
# No Zba, Zbb or Zbs: unicorn does not decode them
arch = ['-march=rv32imac_zicsr_zifencei', '-mabi=ilp32']
flags = arch + ['-DR9A02G021', '-Os', '-g', '-ffunction-sections', '-fdata-sections',
                '-Wall'] + ['-I' + str(root / p) for p in includes]
sources = [port + '/src/spi_r9a02.cpp', port + '/src/sau_r9a02.cpp', port + '/src/dtc_r9a02.cpp',
           port + '/src/interrupt_r9a02.cpp', port + '/src/iopincfg_r9a02.c',
           port + '/R9A02G021/lib/src/system_r9a02.c', port + '/R9A02G021/lib/src/Vectors_R9A02.c',
           'RISCV/src/ResetEntry.c', 'src/coredev/spi.cpp', 'src/device_intrf.cpp']
sources = [root / p for p in sources] + [tests / 'spi_slave_fixture.cpp']
objects = []
for i, source in enumerate(sources):
    cpp = source.suffix == '.cpp'
    opts = ['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
    obj = out / (str(i) + '_' + source.stem + '.o')
    subprocess.run([args.tool_prefix + ('g++' if cpp else 'gcc'), *flags, *opts,
                    '-c', str(source), '-o', str(obj)], check=True)
    objects.append(obj)
elf = out / 'spi_slave.elf'
exports = ['EmuInit', 'EmuInit20', 'EmuSnooze', 'EmuOtherSnooze', 'EmuStandby', 'EmuReset', 'EmuDisable',
           'EmuEnable', 'EmuPowerOff', 'EmuRate', 'EmuMasterTx', 'R9a02IrqDispatch',
           'SystemPeriphClockGet']
subprocess.run([args.tool_prefix + 'g++', *arch, '--specs=nano.specs', '--specs=nosys.specs',
                '-Wl,--gc-sections', *['-Wl,--undefined=' + name for name in exports],
                '-L' + str(root / port / 'ldscript'), '-L' + str(root / 'RISCV/ldscript'),
                '-Tgcc_R9A02G021.ld', *map(str, objects), '-o', str(elf)], check=True)
print(elf.name + ': R9A02G021 SPI slave emulator firmware linked', flush=True)
if not args.build_only:
    subprocess.run([sys.executable, str(tests / 'spi_slave_emu.py'), str(elf)], check=True)
