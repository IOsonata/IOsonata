#!/usr/bin/env python3
"""Cross-build startup/IRQ/UART integration; this does not emulate hardware."""
import argparse
from pathlib import Path
import re
import subprocess
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / 'RISCV/Espressif/ESP32C'
CHIPS = ('ESP32C2', 'ESP32C5', 'ESP32C61')
STACK_TOP = {'ESP32C2': 0x3FCCC000, 'ESP32C5': 0x4084C000, 'ESP32C61': 0x4083C000}


def run(*command):
    subprocess.run([str(x) for x in command], check=True)


def sources(project):
    result = []
    for link in ET.parse(project / '.project').findall('.//link'):
        uri = link.findtext('locationURI')
        if uri == 'virtual:/virtual':
            continue
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
        assert match, uri
        path = project.parents[int(match[1]) - 1] / match[2]
        assert path.is_file(), path
        result.append(path)
    return result


def host_checks(chip, output):
    includes = ['-I' + str(ROOT / p) for p in
                ('.', 'include', 'RISCV/include', 'RISCV/Espressif/include')]
    for name in ('startup_clock_test', 'gpio_uart_test'):
        binary = output / (chip + '-' + name)
        run('gcc', '-std=gnu11', '-D' + chip, '-ffunction-sections',
            '-Wl,--gc-sections', *includes, ROOT / 'tests/esp32' / (name + '.c'),
            '-o', binary)
        run(binary)
        print(chip, name, 'PASS', flush=True)


def inspect_image(tool, chip, elf):
    binary = elf.with_suffix('.bin')
    run(tool + 'objcopy', '-O', 'binary', elf, binary)
    assert binary.read_bytes()[:8] == bytes.fromhex('1d04dbae1d04dbae')
    symbols = {}
    for line in subprocess.check_output([tool + 'nm', str(elf)], text=True).splitlines():
        fields = line.split()
        if len(fields) == 3:
            symbols[fields[2]] = int(fields[0], 16)
    assert symbols['ResetEntry'] == 0x42000008
    assert symbols['__StackTop'] == STACK_TOP[chip]
    alias = 0x3FCA0000 if chip == 'ESP32C2' else symbols['__iram_start__']
    assert symbols['__iram_data_start__'] == alias
    assert 'Esp32InitVectors' in symbols
    assert 'RISCV_TrapHandler' in symbols
    if chip == 'ESP32C2':
        assert symbols['RISCV_VectorTable'] % 256 == 0
    else:
        assert symbols['RISCV_TrapHandler'] % 64 == 0


def build(chip, tool, output):
    target = BASE / chip
    library_sources = sources(target / 'lib/ioc')
    for config, optimization in (('Debug', '-O0'), ('Release', '-Os')):
        directory = output / chip / config
        directory.mkdir(parents=True, exist_ok=True)
        flags = ['-march=rv32imc_zicsr', '-mabi=ilp32', optimization, '-g3',
                 '-ffunction-sections', '-fdata-sections', '-D' + chip]
        flags += ['-I' + str(ROOT / p) for p in
                  ('include', 'RISCV/include', 'RISCV/Espressif/include')]
        objects = []
        for source in library_sources:
            cpp = source.suffix == '.cpp'
            obj = directory / (source.stem + '.o')
            language = (['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if cpp
                        else ['-std=gnu11'])
            run(tool + ('g++' if cpp else 'gcc'), *flags, *language,
                '-c', source, '-o', obj)
            objects.append(obj)
        library = directory / ('libIOsonata_' + chip + '.a')
        run(tool + 'ar', 'rcs', library, *objects)
        for app, source in (('UartPrbsTxTest', 'uart_prbs_tx.cpp'),
                            ('UartLoopback', 'uart_loopback.cpp')):
            # Both API variants use the generic .cpp example, as in IOcomposer.
            for api in ('cpp', 'c-api'):
                obj = directory / (app + '-' + api + '.o')
                elf = obj.with_suffix('.elf')
                defines = ['-DDEMO_C'] if api == 'c-api' else []
                run(tool + 'g++', *flags, '-std=gnu++20', '-fno-exceptions',
                    '-fno-rtti', '-I' + str(target / 'exemples' / app / 'src'),
                    *defines, '-c', ROOT / 'exemples/uart' / source, '-o', obj)
                run(tool + 'g++', *flags, obj, library, '-L' + str(ROOT / 'RISCV/ldscript'),
                    '-Wl,--undefined=ResetEntry', '-Wl,--gc-sections',
                    '--specs=nano.specs', '--specs=nosys.specs',
                    '-T', BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld'), '-o', elf)
                inspect_image(tool, chip, elf)
                print(chip, config, app, api, 'PASS', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--tool-prefix', default='riscv-none-elf-')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    for chip in CHIPS:
        host_checks(chip, output)
        build(chip, args.tool_prefix, output)


if __name__ == '__main__':
    main()
