#!/usr/bin/env python3
"""Build the L152RE library and four examples; emulate register tests.

Requires GNU Arm GCC, Python unicorn and pyelftools. Uses the real ST CMSIS
header and driver instructions. Ready flags are modeled, not physical timing.
Run from any directory: python3 tests/stm32l1/run.py
ARM_CPPFLAGS and ARM_LDFLAGS can describe a relocated compiler installation.
"""
import os
from pathlib import Path
import shlex
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

ROOT = Path(__file__).resolve().parents[2]
PORT = ROOT / 'ARM/ST/STM32L1xx'
PROJECT = PORT / 'STM32L152RE'
EXAMPLES = {
    'Blinky': 'exemples/misc/blinky.c',
    'TimerDemo': 'exemples/timer/timer_demo.cpp',
    'UartPrbsTxTest': 'exemples/uart/uart_prbs_tx.cpp',
    'UartLoopback': 'exemples/uart/uart_loopback.cpp',
}


def run(args):
    p = subprocess.run(list(map(str, args)), text=True, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, cwd=ROOT)
    if p.returncode:
        raise RuntimeError(' '.join(map(str, args)) + '\n' + p.stdout)
    return p.stdout


def emulate(path):
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for address, size in ((0x08000000, 0x100000), (0x20000000, 0x14000), (0x40000000, 0x20000000),
                          (0xe0000000, 0x100000)):
        u.mem_map(address, size)
    with path.open('rb') as f:
        elf = ELFFile(f)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD':
                u.mem_write(segment['p_vaddr'], segment.data())
        symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

    def read(address):
        return struct.unpack('<I', u.mem_read(address, 4))[0]

    def write(address, value):
        u.mem_write(address, struct.pack('<I', value))

    def ready(uc, access, address, size, value, data):
        if address == 0x40023800:  # RCC CR: HSI and PLL ready
            value = read(address)
            value = (value | 2) if value & 1 else value & ~2
            value = (value | (1 << 25)) if value & (1 << 24) else value & ~(1 << 25)
            write(address, value)
        elif address == 0x40023808:  # RCC CFGR switch acknowledgement
            value = read(address)
            write(address, (value & ~0x0c) | ((value & 3) << 2))
        elif address == 0x40007004:  # PWR CSR: voltage scaling settled
            write(address, read(address) & ~(1 << 4))

    u.hook_add(UC_HOOK_MEM_READ, ready)
    write(0x40023804, 5 << 13)  # MSI reset range

    def call(name):
        u.reg_write(UC_ARM_REG_SP, 0x20013ff0)
        u.reg_write(UC_ARM_REG_LR, 0x080ff001)
        u.emu_start(symbols[name] | 1, 0x080ff000, count=10000000)
        assert u.reg_read(UC_ARM_REG_PC) == 0x080ff000, name + ' did not return'
        return u.reg_read(UC_ARM_REG_R0)

    call('SystemInit')
    result = call('main')
    assert result == 0, f'register_test.cpp check failed on line {result}'


def main():
    for tool in ('gcc', 'g++', 'ar', 'size'):
        if not shutil.which('arm-none-eabi-' + tool):
            raise SystemExit('Missing arm-none-eabi-' + tool)
    links = ET.parse(PROJECT / 'lib/ioc/.project').findall('.//linkedResources/link')
    sources = [ROOT / link.findtext('locationURI').removeprefix('PARENT-6-PROJECT_LOC/')
               for link in links if link.findtext('name', '').startswith('src/')]
    assert sources and all(p.is_file() for p in sources)
    includes = [ROOT / p for p in ('ARM/ST/include', 'ARM/include', 'ARM/CMSIS/Core/Include', 'include')]
    includes.append(PORT / 'include')
    base = ['-mcpu=cortex-m3', '-mthumb', '-DSTM32L152xE', '-D__PROGRAM_START',
            '-ffunction-sections', '-fdata-sections', '-g']
    base += ['-I' + str(p) for p in includes]
    base += shlex.split(os.environ.get('ARM_CPPFLAGS', ''))
    ldflags = shlex.split(os.environ.get('ARM_LDFLAGS', ''))
    with tempfile.TemporaryDirectory(prefix='stm32l152re-') as tmp:
        tmp = Path(tmp)
        for label, opt, fpu in (('Debug', '-O0', []), ('Release', '-Os', [])):
            out = tmp / label
            out.mkdir()
            flags = [*base, opt, *fpu, '-DDEBUG' if label == 'Debug' else '-DNDEBUG']

            def compile_source(source, obj, extra=()):
                cpp = source.suffix == '.cpp'
                lang = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
                run(['arm-none-eabi-g++' if cpp else 'arm-none-eabi-gcc', *flags, *lang,
                     *extra, '-c', source, '-o', obj])

            objects = []
            for source in sources:
                obj = out / (source.stem + '.o')
                compile_source(source, obj)
                objects.append(obj)
            archive = out / 'libIOsonata_STM32L152RE.a'
            run(['arm-none-eabi-ar', 'rcs', archive, *objects])

            def link(obj, elf, extra=()):
                run(['arm-none-eabi-g++', '-mcpu=cortex-m3', '-mthumb', *fpu, *ldflags,
                     *extra, '-L' + str(ROOT / 'ARM/ldscript'), '-T' + str(PORT / 'ldscript/gcc_stm32l152xe.ld'),
                     '-Wl,--gc-sections', obj, archive, '-lnosys', '-o', elf])

            for name, rel in EXAMPLES.items():
                obj = out / (name + '.o')
                compile_source(ROOT / rel, obj, ['-I' + str(PROJECT / 'exemples' / name / 'src')])
                cfg = next(c for c in ET.parse(PROJECT / 'exemples' / name / 'ioc/.cproject').findall('.//cconfiguration')
                           if c.find('storageModule').get('name') == label)
                linker = next(o.get('value') for o in cfg.findall('.//option')
                              if o.get('superClass', '').endswith('cpp.linker.other'))
                link(obj, out / (name + '.elf'), ['--specs=nano.specs', *shlex.split(linker)])
            obj = out / 'register_test.o'
            compile_source(ROOT / 'tests/stm32l1/register_test.cpp', obj)
            link(obj, out / 'register_test.elf')
            emulate(out / 'register_test.elf')
            print(label + ': library, four examples and register tests PASS', flush=True)
    print('Hardware timing, electrical behavior and data transfers remain untested.')


if __name__ == '__main__':
    main()
