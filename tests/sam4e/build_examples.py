#!/usr/bin/env python3
"""Link SAM4E16E startup/UART examples against selected library sources.

Uses managed-project source links, defines and linker scripts. This is an
Arm compile/link check, not a full IOcomposer build or a hardware test.
"""
import argparse
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from urllib.parse import unquote

ROOT = Path(__file__).resolve().parents[2]
PART = ROOT / 'ARM/Microchip/SAM4E/SAM4E16E'
SOURCES = (
    'src/CppRuntimeOverload.cpp', 'ARM/src/ResetEntry.c',
    'ARM/Microchip/SAM4E/src/Vectors_SAM4E.c',
    'ARM/Microchip/SAM4E/src/system_sam4e.c',
    'ARM/Microchip/SAM4E/src/system_clock_sam4e.cpp',
    'ARM/Microchip/SAM4E/src/system_power_sam4e.cpp',
    'ARM/Microchip/SAM4E/src/iopincfg_sam4e.c',
    'ARM/Microchip/SAM4E/src/uart_sam4e.cpp',
    'src/coredev/uart.cpp', 'src/device_intrf.cpp', 'src/device.cpp',
    'src/pulse_train.c', 'src/cfifo.c', 'src/prbs.c', 'src/stddev.c', 'src/uart_retarget.c',
)


def source_links(project):
    sources = []
    for link in ET.parse(project / '.project').findall('.//link'):
        uri = link.findtext('locationURI')
        if uri.startswith('virtual:'):
            continue
        prefix, relative = uri.split('-PROJECT_LOC/', 1)
        parent = project
        for _ in range(int(prefix.split('-')[1])):
            parent = parent.parent
        source = (parent / unquote(relative)).resolve()
        if source.suffix in ('.c', '.cpp'):
            sources.append(source)
    return sources


def config(project, mode):
    cfg = next(c for c in ET.parse(project / '.cproject').findall('.//cconfiguration')
               if c.find('.//configuration').get('name') == mode)
    defines = set()
    scripts = set()
    for option in cfg.findall('.//option'):
        kind = option.get('superClass', '')
        if kind.endswith(('.c.compiler.defs', '.cpp.compiler.defs')):
            defines.update('-D' + v.get('value') for v in option)
        if kind.endswith(('.c.linker.scriptfile', '.cpp.linker.scriptfile')):
            scripts.update((project / mode / v.get('value').strip('"')).resolve()
                           for v in option)
    return sorted(defines), scripts


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain-prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    cc = args.toolchain_prefix
    library = PART / 'lib/ioc'
    linked = set(source_links(library))
    assert all(ROOT / source in linked for source in SOURCES)
    includes = ['-I' + str(ROOT / path) for path in
                ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
                 'ARM/Microchip/SAM4E/include')]
    with tempfile.TemporaryDirectory(prefix='sam4e-examples-') as tmp:
        for mode, opt in (('Debug', '-O0'), ('Release', '-Os')):
            out = Path(tmp) / mode
            out.mkdir()
            flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt,
                     '-fsigned-char', '-ffunction-sections', '-fdata-sections']

            def compile(source, dest, extra):
                cpp = source.suffix == '.cpp'
                language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
                obj = dest / (source.name + '.o')
                subprocess.run([cc + ('g++' if cpp else 'gcc'), *flags,
                                *language, *includes, *extra, '-c', str(source),
                                '-o', str(obj)], check=True)
                return str(obj)

            libdefs, _ = config(library, mode)
            objects = [compile(ROOT / source, out, libdefs) for source in SOURCES]
            archive = out / 'libIOsonata_SAM4E16E.a'
            subprocess.run([cc + 'ar', 'rcs', str(archive), *objects], check=True)
            for name in ('Blinky', 'UartRetargetDemo', 'UartPrbsTxTest'):
                project = PART / 'exemples' / name / 'ioc'
                defines, scripts = config(project, mode)
                assert len(scripts) == 1 and all(s.is_file() for s in scripts)
                dest = out / name
                dest.mkdir()
                app = [compile(source, dest, [*defines, '-I' + str(project.parent / 'src')])
                       for source in source_links(project)]
                elf = dest / (name + '.elf')
                subprocess.run([cc + 'g++', *flags, '--specs=nano.specs',
                                '--specs=nosys.specs', '-Wl,--gc-sections',
                                '-L' + str(ROOT / 'ARM/ldscript'),
                                '-T' + str(next(iter(scripts))), *app,
                                str(archive), '-o', str(elf)], check=True)
                symbols = subprocess.check_output([cc + 'nm', str(elf)], text=True)
                for symbol in ('SystemInit', 'SystemResetReason', 'SystemLowFreqClockSelect'):
                    assert ' T ' + symbol + '\n' in symbols, symbol
                print(mode + ': ' + name + ' linked with SAM4E clock/reset implementation', flush=True)


if __name__ == '__main__':
    main()
