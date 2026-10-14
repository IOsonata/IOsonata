#!/usr/bin/env python3
"""Build LPC UART, SPI and I2C examples in Debug and Release.

Copyright (c) 2026, I-SYST inc. MIT License.
Uses the timer suite's verified startup/runtime source lists and the managed
project board headers. No hardware is exercised.
"""
from pathlib import Path
import argparse
import re
import runpy
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
TARGETS = {
    '11': ('LPC11xx', 'LPC11U35', 'link.py', 'cortex-m0', 'LPC11XX_SERIES', 'gcc_lpc11u35.ld'),
    '17': ('LPC17xx', 'LPC1769', 'link17.py', 'cortex-m3', 'LPC1769', 'gcc_lpc1767_9.ld'),
    '546': ('LPC546xx', 'LPC54628', 'link546.py', 'cortex-m4', 'LPC54628_SERIES', 'gcc_lpc54628.ld'),
}
APPLICATIONS = {
    'UartLoopback': 'exemples/uart/uart_loopback.cpp',
    'SpiLoopback': 'exemples/spi/spi_loopback.cpp',
    'I2CScan': 'exemples/i2c/i2c_scan.cpp',
}


def build(target, prefix, directory):
    family, mcu, script, cpu, define, linker = TARGETS[target]
    timer = runpy.run_path(str(ROOT / 'tests/lpc/timer' / script))
    sources = timer['SOURCES'] + ['src/coredev/i2c.cpp', 'src/coredev/spi.cpp']
    if target == '546':
        sources += ['ARM/NXP/LPC546xx/src/i2c_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/spi_lpc546xx.cpp']
    else:
        sources += ['ARM/NXP/src/i2c_lpc_master.cpp', 'ARM/NXP/src/spi_lpcxx.c']
        sources += (['ARM/NXP/LPC11xx/src/i2c_lpc11uxx.cpp', 'ARM/NXP/LPC11xx/src/spi_lpc11uxx.c']
                    if target == '11' else
                    ['ARM/NXP/LPC17xx/src/i2c_lpc17xx.c', 'ARM/NXP/LPC17xx/src/spi_lpc17xx.c'])
    project = ROOT / 'ARM/NXP' / family / mcu
    linked = timer['project_sources'](project / 'lib/ioc/.project')
    for source in sources:
        assert (ROOT / source).resolve() in linked, source
    for name, application in APPLICATIONS.items():
        example = project / 'exemples' / name
        app_links = timer['project_sources'](example / 'ioc/.project')
        assert ROOT / application in app_links
        assert example / 'src/board.h' in app_links
        for config in ET.parse(example / 'ioc/.cproject').findall('.//configuration'):
            text = ET.tostring(config, encoding='unicode')
            debug = config.get('name') == 'Debug'
            assert '--specs=' + ('rdimon' if debug else 'nosys') + '.specs' in text
            assert ('--specs=rdimon.specs' in text) == debug
    for mode in ['Debug', 'Release']:
        output = directory / target / mode
        output.mkdir(parents=True)
        flags = ['-mcpu=' + cpu, '-mthumb', '-fsigned-char', '-D' + define,
                 '-ffunction-sections', '-fdata-sections']
        if target == '546':
            flags += ['-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
        flags += ['-O0', '-g3', '-DDEBUG'] if mode == 'Debug' else ['-Os', '-DNDEBUG']
        includes = ['-I' + str(ROOT / path) for path in timer['INCLUDES']]
        objects = []
        for index, source in enumerate(sources):
            cpp = source.endswith('.cpp')
            language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
            obj = output / f'{index}.o'
            subprocess.run([prefix + ('g++' if cpp else 'gcc'), *flags, *includes, *language,
                            '-c', str(ROOT / source), '-o', str(obj)], check=True)
            objects.append(str(obj))
        archive = output / 'libIOsonata.a'
        subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects], check=True)
        for name, application in APPLICATIONS.items():
            obj = output / (name + '.o')
            subprocess.run([prefix + 'g++', *flags, *includes,
                            '-I' + str(project / 'exemples' / name / 'src'),
                            '-std=gnu++23', '-fno-exceptions', '-fno-rtti',
                            '-c', str(ROOT / application), '-o', str(obj)], check=True)
            image = output / (name + '.elf')
            subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                            '--specs=' + ('rdimon' if mode == 'Debug' else 'nosys') + '.specs',
                            '-L' + str(ROOT / 'ARM/ldscript'),
                            '-T' + str(ROOT / 'ARM/NXP' / family / 'ldscript' / linker),
                            '-Wl,--gc-sections', str(obj), str(archive), '-o', str(image)], check=True)
            binary = image.with_suffix('.bin')
            subprocess.run([prefix + 'objcopy', '-O', 'binary', str(image), str(binary)], check=True)
            assert sum(struct.unpack_from('<8I', binary.read_bytes())) & 0xffffffff == 0
            symbols = subprocess.check_output([prefix + 'nm', str(image)], text=True)
            assert ('initialise_monitor_handles' in symbols) == (mode == 'Debug')
            symbol = {'UartLoopback': 'UARTInit', 'SpiLoopback': 'SPIInit', 'I2CScan': 'I2CInit'}[name]
            assert re.search(r'^\w+ T ' + symbol + '$', symbols, re.M), symbol
            print(f'{mcu} {name} {mode}: vector checksum, driver link and semihosting PASS', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', choices=TARGETS)
    parser.add_argument('--prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        for target in [args.target] if args.target else TARGETS:
            build(target, args.prefix, Path(tmp))


if __name__ == '__main__':
    main()
