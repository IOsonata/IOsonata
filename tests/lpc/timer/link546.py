#!/usr/bin/env python3
"""Link the real LPC54628 TimerDemo in Debug and Release.

Copyright (c) 2026, I-SYST inc. MIT License.
Requires an Arm GNU toolchain with newlib-nano and C++23 support.
"""
from pathlib import Path
import argparse
import re
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
SOURCES = [
    'ARM/src/ResetEntry.c',
    'ARM/NXP/LPC546xx/src/Vectors_LPC546xx.c',
    'ARM/NXP/LPC546xx/src/system_LPC546xx.c',
    'ARM/NXP/LPC546xx/src/iopincfg_lpc546xx.cpp',
    'src/cfifo.c',
    'src/device_intrf.cpp',
    'src/coredev/timer.cpp',
    'ARM/NXP/src/timer_lpcxx.cpp',
    'ARM/NXP/LPC546xx/src/timer_lpc546xx.cpp',
    'src/coredev/uart.cpp',
    'src/coredev/shared_intrf.cpp',
    'ARM/NXP/LPC546xx/src/flexcomm_lpc546xx.cpp',
    'ARM/NXP/LPC546xx/src/dma_lpc546xx.cpp',
    'ARM/NXP/LPC546xx/src/uart_lpc546xx.cpp',
    'src/uart_retarget.c',
    'src/stddev.c',
]
INCLUDES = [
    'include', 'ARM/include', 'ARM/CMSIS/Core/Include',
    'ARM/NXP/include', 'ARM/NXP/LPC546xx/include',
    'ARM/NXP/LPC546xx/LPC54628/lib/include',
]

EXAMPLES = {'timer': 'TimerDemo'}


def project_sources(project):
    linked = set()
    names = set()
    for link in ET.parse(project).findall('.//link'):
        name = link.findtext('name')
        assert name not in names, name
        names.add(name)
        uri = link.findtext('locationURI')
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.*)', uri)
        if match:
            linked.add((project.parent.parents[int(match[1]) - 1] / match[2]).resolve())
    return linked


def check_project():
    mcu = ROOT / 'ARM/NXP/LPC546xx/LPC54628'
    linked = project_sources(mcu / 'lib/ioc/.project')
    for source in SOURCES:
        assert (ROOT / source).resolve() in linked, 'Missing library project link: ' + source
    for kind, project_name in EXAMPLES.items():
        example = mcu / 'exemples' / project_name
        linked = project_sources(example / 'ioc/.project')
        assert ROOT / 'exemples/timer/timer_demo.cpp' in linked
        assert example / 'src/board.h' in linked
        for config in ET.parse(example / 'ioc/.cproject').findall('.//configuration'):
            target = {option.get('superClass', '').rsplit('.', 1)[-1]: option.get('value', '')
                      for option in config.findall('./folderInfo/toolChain/option')}
            assert target['family'].endswith('.cortex-m4')
            assert target['abi'].endswith('.hard')
            assert target['unit'].endswith('.fpv4spd16')
            mode = config.get('name')
            specs = 'rdimon' if mode == 'Debug' else 'nosys'
            for tool in config.findall('.//tool'):
                name = tool.get('superClass', '')
                options = {option.get('superClass').split('.option.', 1)[-1]: option
                           for option in tool.findall('option')}
                if name.endswith('.cpp.compiler'):
                    assert options['cpp.compiler.std'].get('value').endswith('.gnucpp23')
                if name.endswith(('.c.linker', '.cpp.linker')):
                    language = 'cpp' if name.endswith('.cpp.linker') else 'c'
                    flags = options[language + '.linker.other'].get('value')
                    assert '--specs=' + specs + '.specs' in flags
                    assert ('rdimon' in flags) == (mode == 'Debug')


def build(prefix, output, mode, kind):
    project_name = EXAMPLES[kind]
    board_name = project_name
    directory = output / project_name / mode
    directory.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
             '-fsigned-char',
             '-ffunction-sections', '-fdata-sections', '-DLPC54628_SERIES']
    flags += ['-O0', '-g3', '-DDEBUG'] if mode == 'Debug' else ['-Os', '-DNDEBUG']
    includes = ['-I' + str(ROOT / path) for path in INCLUDES]
    includes += ['-I' + str(ROOT / 'ARM/NXP/LPC546xx/LPC54628/exemples' / board_name / 'src')]
    objects = []
    application = 'exemples/timer/timer_demo.cpp'
    sources = SOURCES + [application]
    for index, source in enumerate(sources):
        cpp = source.endswith('.cpp')
        compiler = prefix + ('g++' if cpp else 'gcc')
        language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        obj = directory / (str(index) + '.o')
        subprocess.run([compiler, *flags, *includes, *language,
                        '-c', str(ROOT / source), '-o', str(obj)], check=True)
        objects.append(str(obj))

    # Match applications linking a separate MCU library. This also checks
    # archive extraction of startup, vector, timer and UART objects.
    archive = directory / 'libIOsonata_LPC54628.a'
    archive.unlink(missing_ok=True)
    subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects[:-1]], check=True)
    image = directory / f'{kind}_demo.elf'
    specs = 'rdimon' if mode == 'Debug' else 'nosys'
    subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                    '--specs=' + specs + '.specs',
                    '-L' + str(ROOT / 'ARM/ldscript'),
                    '-T' + str(ROOT / 'ARM/NXP/LPC546xx/ldscript/gcc_lpc54628.ld'),
                    '-Wl,--gc-sections,-Map,' + str(directory / f'{kind}_demo.map'),
                    objects[-1], str(archive), '-o', str(image)], check=True)
    binary = directory / f'{kind}_demo.bin'
    subprocess.run([prefix + 'objcopy', '-O', 'binary', str(image), str(binary)], check=True)
    data = binary.read_bytes()
    assert len(data) <= 512 * 1024, 'Flash overflow'
    vectors = struct.unpack_from('<8I', data)
    assert sum(vectors) & 0xffffffff == 0, 'Invalid LPC vector checksum'
    assert vectors[0] == 0x20010000 - 32, 'IAP SRAM reservation lost'
    symbols = subprocess.check_output([prefix + 'nm', str(image)], text=True)
    addresses = {name: int(addr, 16) for addr, kind, name in
                 re.findall(r'^([0-9a-fA-F]+)\s+(\w)\s+(\S+)$', symbols, re.M)}
    assert vectors[1] == addresses['ResetEntry'] | 1
    assert ('initialise_monitor_handles' in addresses) == (mode == 'Debug')
    assert 'TimerInit' in addresses
    assert addresses['__Vectors'] == 0
    for irq, name in [(10, 'CTIMER0_IRQHandler'), (11, 'CTIMER1_IRQHandler'),
                      (36, 'CTIMER2_IRQHandler'), (13, 'CTIMER3_IRQHandler'),
                      (37, 'CTIMER4_IRQHandler')]:
        assert re.search(r'^[0-9a-fA-F]+ T ' + name + '$', symbols, re.M)
        vector = struct.unpack_from('<I', data, (16 + irq) * 4)[0]
        assert vector == addresses[name] | 1
    assert ('UARTRetargetEnable' in addresses) == (mode == 'Release')
    attributes = subprocess.check_output([prefix + 'readelf', '-A', str(image)], text=True)
    assert 'Tag_ABI_VFP_args: VFP registers' in attributes
    sections = subprocess.check_output([prefix + 'readelf', '-SW', str(image)], text=True)
    assert '.usb_ram' not in sections
    ram = addresses['__bss_end__'] - addresses['__data_start__']
    stack = addresses['__StackTop'] - addresses['__StackLimit']
    assert ram < 64 * 1024 - 32 and stack > 0
    print(f'{project_name} {mode}: flash {len(data)} / 524288 bytes; main RAM {ram} / 65504 bytes; '
          f'USB SRAM 0 bytes; linker stack allowance {stack} bytes', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='arm-none-eabi-', help='GNU tool prefix, optionally with a path')
    parser.add_argument('--output', type=Path, help='Keep ELF, map and binary files in this directory')
    args = parser.parse_args()
    check_project()
    with tempfile.TemporaryDirectory(prefix='lpc546-timer-link-') as temporary:
        output = args.output.resolve() if args.output else Path(temporary)
        for kind in EXAMPLES:
            for mode in ['Debug', 'Release']:
                build(args.prefix, output, mode, kind)
    print('LPC54628 TimerDemo ARM firmware links: PASS')


if __name__ == '__main__':
    main()
