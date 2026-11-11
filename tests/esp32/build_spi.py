#!/usr/bin/env python3
"""Cross-link C/C++ SPI API with each chip's startup; no hardware execution."""
import argparse
from pathlib import Path
import subprocess
import re
import xml.etree.ElementTree as ET
from urllib.parse import unquote
from build_startup import ROOT, BASE, run

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='riscv-none-elf-')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
tool = args.tool_prefix
for chip in ('ESP32C2', 'ESP32C3', 'ESP32C5', 'ESP32C6', 'ESP32C61'):
    names = {'ResetEntry.c', 'iatomic.c', 'timer.cpp', 'timer_esp32.cpp', 'spi.cpp', 'spi_esp32.cpp',
             'esp32xx_irq.c', 'CppRuntimeOverload.cpp', 'uart.cpp',
             'uart_esp32.cpp', 'system_esp32_uart_clock.c', 'iopincfg_esp32.c',
             'device.cpp', 'device_intrf.cpp', 'cfifo.c', 'uart_retarget.c', 'stddev.c'}
    if chip in ('ESP32C3', 'ESP32C6'):
        names.update({'system_' + chip.lower() + '.c',
                      'Vectors_' + chip.lower() + '.c'})
    else:
        names.update({'system_esp32.c', 'Vectors_esp32.c'})
    names.add('system_esp32_system.c' if chip in ('ESP32C2', 'ESP32C3')
              else 'system_esp32_pcr.c')
    project = BASE / chip / 'lib/ioc'
    selected = []
    for link in ET.parse(project / '.project').findall('.//linkedResources/link'):
        uri = unquote(link.findtext('locationURI'))
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
        if match and Path(match[2]).name in names:
            source = project.parents[int(match[1]) - 1] / match[2]
            assert source.is_file(), source
            selected.append(source)
    assert {p.name for p in selected} == names
    for config, opt in (('Debug', '-O0'), ('Release', '-Os')):
        directory = args.output.resolve() / chip / config
        directory.mkdir(parents=True, exist_ok=True)
        flags = ['-march=rv32imc_zicsr', '-mabi=ilp32', opt,
                 '-ffunction-sections', '-fdata-sections', '-D' + chip]
        flags += ['-I' + str(ROOT / p) for p in
                  ('include', 'RISCV/include', 'RISCV/Espressif/include')]
        objects = []
        for source in selected:
            cpp = source.suffix == '.cpp'
            obj = directory / (source.stem + '.o')
            run(tool + ('g++' if cpp else 'gcc'), *flags,
                *(['-std=gnu++20', '-fno-rtti', '-fno-exceptions'] if cpp else ['-std=gnu11']),
                '-c', source, '-o', obj)
            objects.append(obj)
        library = directory / 'libspi.a'
        run(tool + 'ar', 'rcs', library, *objects)
        for api in ('c', 'cpp'):
            obj = directory / ('main-' + api + '.o')
            elf = obj.with_suffix('.elf')
            run(tool + ('gcc' if api == 'c' else 'g++'), *flags,
                *(['-std=gnu11'] if api == 'c' else
                  ['-x', 'c++', '-std=gnu++20', '-fno-rtti', '-fno-exceptions']),
                '-c', ROOT / 'tests/esp32/spi_link_test.c', '-o', obj)
            run(tool + 'g++', *flags, obj, library,
                '-L' + str(ROOT / 'RISCV/ldscript'), '-Wl,--undefined=ResetEntry',
                '-Wl,--gc-sections', '--specs=nano.specs', '--specs=nosys.specs',
                '-T', BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld'), '-o', elf)
            symbols = subprocess.check_output([tool + 'nm', str(elf)], text=True)
            assert ' SPIInit' in symbols
            assert 'Pei' not in symbols
            print(chip, config, api, 'SPI link PASS', flush=True)

        app = BASE / chip / 'exemples/SpiLoopback'
        links = ET.parse(app / 'ioc/.project').findall('.//linkedResources/link')
        app_sources = [x.findtext('locationURI') for x in links
                       if x.findtext('name').endswith('.cpp')]
        assert app_sources == ['PARENT-7-PROJECT_LOC/exemples/spi/spi_loopback.cpp']
        obj = directory / 'SpiLoopback.o'
        elf = obj.with_suffix('.elf')
        run(tool + 'g++', *flags, '-std=gnu++20', '-fno-rtti', '-fno-exceptions',
            '-I' + str(app / 'src'), '-c', ROOT / 'exemples/spi/spi_loopback.cpp', '-o', obj)
        run(tool + 'g++', *flags, obj, library, '-L' + str(ROOT / 'RISCV/ldscript'),
            '-Wl,--undefined=ResetEntry', '-Wl,--gc-sections',
            '--specs=nano.specs', '--specs=nosys.specs',
            '-T', BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld'), '-o', elf)
        symbols = subprocess.check_output([tool + 'nm', str(elf)], text=True)
        for name in ('SPIInit', 'UARTRetargetEnable', 'UARTStdDevWrite', '_write'):
            assert re.search(r' T ' + name + r'$', symbols, re.MULTILINE), name
        assert 'Pei' not in symbols
        run(tool + 'objcopy', '-O', 'binary', elf, elf.with_suffix('.bin'))
        assert elf.with_suffix('.bin').read_bytes()[:8] == bytes.fromhex('1d04dbae1d04dbae')
        print(chip, config, 'SpiLoopback PASS', flush=True)
