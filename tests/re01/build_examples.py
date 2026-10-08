#!/usr/bin/env python3
"""Build RE01 example images and inspect their option-memory layout."""
import argparse
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET
from urllib.parse import unquote

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='arm-none-eabi-')
parser.add_argument('--package', choices=['DBN', 'CFB', 'CFP'], default='CFB')
parser.add_argument('--timer-devno', type=int, choices=range(9), default=3)
parser.add_argument('--all-timers', action='store_true',
                    help='Build TimerDemo for each of the nine timer devices')
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


def compile_source(source, name, board=None, defines=()):
    cpp = source.suffix == '.cpp'
    opts = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu11']
    if board:
        opts.append('-I' + str(board))
    opts.extend('-D' + value for value in defines)
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
          'src/prbs.c', 'src/pulse_train.c', 'src/uart_retarget.c', 'src/stddev.c']
objects += [compile_source(root / p, Path(p).stem + '_generic') for p in common]
examples = ['Blinky', 'TimerDemo', 'UartPrbsTxTest', 'PulseTrain',
            'UartRetargetDemo', 'I2CMasterDemo', 'SPIMasterDemo']
for name in examples:
    project = root / 'ARM/Renesas/RE01/RE01_1500KB/exemples' / name / 'ioc'
    links = ET.parse(project / '.project').findall('.//linkedResources/link')
    paths = []
    for link in links:
        uri = unquote(link.findtext('locationURI'))
        match = re.fullmatch(r'(?:\$\{)?PARENT-(\d+)-PROJECT_LOC(?:\})?/(.+)', uri)
        if match:
            paths.append(project.parents[int(match[1]) - 1] / match[2])
    board = next(p.parent for p in paths if p.name == 'board.h')
    source = next(p for p in paths if p.suffix in ('.c', '.cpp'))
    assert all(p.is_file() for p in paths), paths
    defines = []
    if name == 'UartRetargetDemo':
        defines = ['UART_INT_MODE=true', 'UART_DMA_MODE=false', 'UART_BAUDRATE=115200']
    if name == 'I2CMasterDemo':
        defines = ['I2C_MASTER_DMA_ENABLE=false', 'I2C_MASTER_INT_ENABLE=false']
    if name == 'TimerDemo':
        defines = ['TIMER_DEMO_FREQ=32768', 'TIMER_DEMO_UART']
    devices = range(9) if name == 'TimerDemo' and args.all_timers else (
              [args.timer_devno] if name == 'TimerDemo' else [None])
    for devno in devices:
        label = name if devno is None else name + '_dev' + str(devno)
        options = defines + (['TIMER_DEMO_DEVNO=' + str(devno)] if devno is not None else [])
        obj = compile_source(source, label, board, options)
        for script in ('gcc_re01_1500kb.ld', 'RE01_1500KB.ld'):
            elf = out / (label + '_' + Path(script).stem + '.elf')
            run([args.tool_prefix + 'g++', '-mcpu=cortex-m0plus', '-mthumb',
                 '--specs=nano.specs', '--specs=nosys.specs', '-Wl,--gc-sections',
                 '-L' + str(root / 'ARM/ldscript'), '-T' + str(lddir / script),
                 *map(str, objects), str(obj), '-o', str(elf)])
            check_image(elf)
