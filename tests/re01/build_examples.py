#!/usr/bin/env python3
"""Build RE01 example images and inspect their option-memory layout."""
import argparse
from pathlib import Path
import re
import subprocess

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='arm-none-eabi-')
parser.add_argument('--package', choices=['DBN', 'CFB', 'CFP'], default='CFB')
args = parser.parse_args()
root = Path(__file__).resolve().parents[2]
out = Path(__file__).resolve().parent / 'build' / ('arm_' + args.package)
out.mkdir(parents=True, exist_ok=True)
includes = ['include', 'ARM/include', 'ARM/CMSIS/Core/Include',
            'ARM/Renesas/RE01/include', 'ARM/Renesas/RE01/RE01_1500KB/lib/include']
flags = ['-mcpu=cortex-m0plus', '-mthumb', '-DRE01_1500KB',
         '-DRE01_1500KB_' + args.package, '-O2', '-g', '-ffunction-sections',
         '-fdata-sections', '-Wall', '-Wextra', '-Wno-unused-parameter',
         '-Wno-missing-field-initializers'] + ['-I' + str(root / p) for p in includes]
port = root / 'ARM/Renesas/RE01/src'
lddir = root / 'ARM/Renesas/RE01/ldscript'


def run(cmd):
    return subprocess.check_output(cmd, text=True)


def compile_source(source, name, board=None):
    cpp = source.suffix == '.cpp'
    opts = ['-std=gnu++17', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu11']
    if board:
        opts.append('-I' + str(board))
    obj = out / (name + '.o')
    run([args.tool_prefix + ('g++' if cpp else 'gcc'), *flags, *opts,
         '-c', str(source), '-o', str(obj)])
    return obj


def check_image(elf):
    sections = {}
    for line in run([args.tool_prefix + 'objdump', '-h', str(elf)]).splitlines():
        m = re.match(r'\s*\d+\s+(\S+)\s+([0-9a-f]+)\s+([0-9a-f]+)\s+([0-9a-f]+)', line)
        if m:
            sections[m[1]] = tuple(int(v, 16) for v in m.groups()[1:])
    assert sections['.ivector'][1] == 0 and sections['.ivector'][0] <= 0x400
    assert sections['.osm'][:2] == (0x40, 0x400)
    assert sections['.Version'][1] >= 0x440
    for name, (size, addr, lma) in sections.items():
        if size and 0 < addr < 0x180000 and name not in ('.ivector', '.osm'):
            assert addr >= 0x440, (name, addr)
        if size and 0 < lma < 0x180000 and name not in ('.ivector', '.osm'):
            assert lma >= 0x440, (name, lma)
    osm = out / (elf.stem + '.osm.bin')
    run([args.tool_prefix + 'objcopy', '--only-section=.osm', '-O', 'binary', str(elf), str(osm)])
    assert osm.read_bytes() == bytes([255]) * 64
    symbols = {}
    for line in run([args.tool_prefix + 'nm', '-n', str(elf)]).splitlines():
        parts = line.split()
        if len(parts) == 3:
            symbols[parts[2]] = int(parts[0], 16)
    assert symbols['__StackLimit'] < symbols['__StackTop'] == 0x20040000
    assert symbols['__heap_start__'] < symbols['__heap_end__'] < symbols['__StackTop']
    print(elf.name + ': link and memory layout passed')


objects = [compile_source(p, p.stem) for p in sorted(port.iterdir())
           if p.suffix in ('.c', '.cpp') and p.name != 'dfu_re01.cpp']
common = ['ARM/src/ResetEntry.c', 'ARM/src/iatomic.c', 'src/cfifo.c',
          'src/coredev/uart.cpp', 'src/coredev/timer.cpp', 'src/coredev/spi.cpp',
          'src/coredev/i2c.cpp', 'src/device_intrf.cpp',
          'src/prbs.c', 'src/pulse_train.c']
objects += [compile_source(root / p, Path(p).stem + '_generic') for p in common]
examples = [('Blinky', 'exemples/misc/blinky.c'),
            ('TimerDemo', 'exemples/timer/timer_demo.cpp'),
            ('UartPrbsTxTest', 'exemples/uart/uart_prbs_tx.cpp')]
for name, source in examples:
    board = root / 'ARM/Renesas/RE01/RE01_1500KB/exemples' / name / 'src'
    obj = compile_source(root / source, name, board)
    for script in ('gcc_re01_1500kb.ld', 'RE01_1500KB.ld'):
        elf = out / (name + '_' + Path(script).stem + '.elf')
        run([args.tool_prefix + 'g++', '-mcpu=cortex-m0plus', '-mthumb',
             '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections',
             '-L' + str(root / 'ARM/ldscript'), '-T' + str(lddir / script),
             *map(str, objects), str(obj), '-o', str(elf)])
        check_image(elf)
