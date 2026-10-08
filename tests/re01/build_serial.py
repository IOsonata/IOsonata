#!/usr/bin/env python3
"""Cross-build RE01 serial regression firmware and run the register model."""
import argparse
from pathlib import Path
import subprocess
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='arm-none-eabi-')
parser.add_argument('--package', choices=['DBN', 'CFB', 'CFP'], default='CFB')
parser.add_argument('--build-only', action='store_true')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
out = tests / 'build' / ('serial_' + args.package)
out.mkdir(parents=True, exist_ok=True)
port = root / 'ARM/Renesas/RE01/src'
lddir = root / 'ARM/Renesas/RE01/ldscript'
includes = ['include', 'ARM/include', 'ARM/CMSIS/Core/Include',
            'ARM/Renesas/RE01/include', 'ARM/Renesas/RE01/RE01_1500KB/lib/include']
flags = ['-mcpu=cortex-m0plus', '-mthumb', '-DRE01_1500KB',
         '-DRE01_1500KB_' + args.package, '-O2', '-g', '-ffunction-sections',
         '-fdata-sections', '-Wall', '-Wextra', '-Wno-unused-parameter',
         '-Wno-missing-field-initializers'] + ['-I' + str(root / p) for p in includes]
sources = sorted(p for p in port.iterdir()
                 if p.suffix in ('.c', '.cpp') and p.name != 'dfu_re01.cpp')
common = ['ARM/src/ResetEntry.c', 'ARM/src/iatomic.c', 'src/cfifo.c',
          'src/coredev/uart.cpp', 'src/coredev/timer.cpp', 'src/coredev/spi.cpp',
          'src/coredev/i2c.cpp', 'src/device_intrf.cpp']
sources += [root / p for p in common] + [tests / 'serial_fixture.cpp']
objects = []
for i, source in enumerate(sources):
    cpp = source.suffix == '.cpp'
    opts = ['-std=gnu++17', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu11']
    obj = out / (str(i) + '_' + source.stem + '.o')
    subprocess.run([args.tool_prefix + ('g++' if cpp else 'gcc'), *flags, *opts,
                    '-c', str(source), '-o', str(obj)], check=True)
    objects.append(obj)
elf = out / 'serial.elf'
exports = ['SerialInit', 'SerialCall', 'SerialCppProbe', 'SerialTx', 'SerialRx',
           'SystemPeriphClockGet']
subprocess.run([args.tool_prefix + 'g++', '-mcpu=cortex-m0plus', '-mthumb',
                '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections',
                *['-Wl,--undefined=' + name for name in exports],
                '-L' + str(root / 'ARM/ldscript'),
                '-T' + str(lddir / 'gcc_re01_1500kb.ld'),
                *map(str, objects), '-o', str(elf)], check=True)
print(elf.name + ': RE01 ' + args.package + ' serial firmware linked', flush=True)
if not args.build_only:
    subprocess.run([sys.executable, str(tests / 'serial_emu.py'), str(elf)], check=True)
