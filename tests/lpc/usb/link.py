#!/usr/bin/env python3
"""Link the real LPC11U35 CDC and HID loopback firmware in Debug and Release.

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
    'ARM/src/iatomic.c',
    'ARM/NXP/src/lpc_iap.c',
    'ARM/NXP/LPC11xx/src/Vectors_LPC11Uxx.c',
    'ARM/NXP/LPC11xx/src/system_LPC11Uxx.c',
    'ARM/NXP/LPC11xx/src/iopincfg_lpc11uxx.c',
    'ARM/NXP/LPC11xx/src/usb_lpc11uxx.c',
    'src/cfifo.c',
    'src/device_intrf.cpp',
    'src/app_evt_handler.cpp',
    'src/app_run.cpp',
    'src/app_wait.cpp',
    'src/usb/usb.cpp',
    'src/usb/usb_intrf.cpp',
    'src/usb/usbd_cdc.cpp',
    'src/usb/usbd_cdc_desc.cpp',
    'src/usb/usbd_epalloc.cpp',
    'src/usb/usbd_hid.cpp',
    'src/usb/usb_int.cpp',
]
INCLUDES = [
    'include', 'ARM/include', 'ARM/CMSIS/Core/Include',
    'ARM/NXP/include', 'ARM/NXP/LPC11xx/include',
]

EXAMPLES = {'cdc': 'UsbCdcLoopback', 'hid': 'UsbHidLoopback'}


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
    mcu = ROOT / 'ARM/NXP/LPC11xx/LPC11U35'
    linked = project_sources(mcu / 'lib/ioc/.project')
    for source in SOURCES:
        assert (ROOT / source).resolve() in linked, 'Missing library project link: ' + source
    for kind, project_name in EXAMPLES.items():
        example = mcu / 'exemples' / project_name
        linked = project_sources(example / 'ioc/.project')
        assert ROOT / f'exemples/usb/usb_{kind}_loopback.cpp' in linked
        assert example / 'src/board.h' in linked
        for config in ET.parse(example / 'ioc/.cproject').findall('.//configuration'):
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
    directory = output / EXAMPLES[kind] / mode
    directory.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m0', '-mthumb', '-fsigned-char',
             '-ffunction-sections', '-fdata-sections', '-DLPC11XX_SERIES']
    flags += ['-O0', '-g3', '-DDEBUG'] if mode == 'Debug' else ['-Os', '-DNDEBUG']
    includes = ['-I' + str(ROOT / path) for path in INCLUDES]
    includes += ['-I' + str(ROOT / 'ARM/NXP/LPC11xx/LPC11U35/exemples' / EXAMPLES[kind] / 'src')]
    objects = []
    sources = SOURCES + [f'exemples/usb/usb_{kind}_loopback.cpp']
    for index, source in enumerate(sources):
        cpp = source.endswith('.cpp')
        compiler = prefix + ('g++' if cpp else 'gcc')
        language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu11']
        obj = directory / (str(index) + '.o')
        subprocess.run([compiler, *flags, *includes, *language,
                        '-c', str(ROOT / source), '-o', str(obj)], check=True)
        objects.append(str(obj))

    # Match applications linking a separate MCU library. This also checks
    # archive extraction of startup, vector, controller and class objects.
    archive = directory / 'libIOsonata_LPC11U35.a'
    archive.unlink(missing_ok=True)
    subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects[:-1]], check=True)
    image = directory / f'{kind}_loopback.elf'
    specs = 'rdimon' if mode == 'Debug' else 'nosys'
    subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                    '--specs=' + specs + '.specs',
                    '-L' + str(ROOT / 'ARM/ldscript'),
                    '-T' + str(ROOT / 'ARM/NXP/LPC11xx/ldscript/gcc_lpc11u35.ld'),
                    '-Wl,--gc-sections,-Map,' + str(directory / f'{kind}_loopback.map'),
                    objects[-1], str(archive), '-o', str(image)], check=True)
    binary = directory / f'{kind}_loopback.bin'
    subprocess.run([prefix + 'objcopy', '-O', 'binary', str(image), str(binary)], check=True)
    data = binary.read_bytes()
    assert len(data) <= 64 * 1024, 'Flash overflow'
    vectors = struct.unpack_from('<8I', data)
    assert sum(vectors) & 0xffffffff == 0, 'Invalid LPC vector checksum'
    assert vectors[0] == 0x10002000 - 32, 'IAP SRAM reservation lost'
    assert vectors[4:7] == (0, 0, 0), 'Cortex-M0 reserved vectors'
    symbols = subprocess.check_output([prefix + 'nm', str(image)], text=True)
    addresses = {name: int(addr, 16) for addr, kind, name in
                 re.findall(r'^([0-9a-fA-F]+)\s+(\w)\s+(\S+)$', symbols, re.M)}
    assert vectors[1] == addresses['ResetEntry'] | 1
    assert ('initialise_monitor_handles' in addresses) == (mode == 'Debug')
    # A weak default ISR must not silently stand in for the USB controller.
    assert re.search(r'^[0-9a-fA-F]+ T USB_IRQHandler$', symbols, re.M)
    usb_vector = struct.unpack_from('<I', data, (16 + 22) * 4)[0]
    assert usb_vector == addresses['USB_IRQHandler'] | 1
    sections = subprocess.check_output([prefix + 'readelf', '-SW', str(image)], text=True)
    assert re.search(r'\.usb_ram\s+NOBITS\s+20004000\s+\w+\s+000340', sections), sections
    ram = addresses['__bss_end__'] - addresses['__data_start__']
    stack = addresses['__StackTop'] - addresses['__StackLimit']
    assert ram < 8 * 1024 - 32 and stack > 0
    print(f'{EXAMPLES[kind]} {mode}: flash {len(data)} / 65536 bytes; main RAM {ram} / 8160 bytes; '
          f'USB SRAM 832 / 2048 bytes; linker stack allowance {stack} bytes', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='arm-none-eabi-', help='GNU tool prefix, optionally with a path')
    parser.add_argument('--output', type=Path, help='Keep ELF, map and binary files in this directory')
    args = parser.parse_args()
    check_project()
    with tempfile.TemporaryDirectory(prefix='lpc11-usb-link-') as temporary:
        output = args.output.resolve() if args.output else Path(temporary)
        for kind in EXAMPLES:
            for mode in ['Debug', 'Release']:
                build(args.prefix, output, mode, kind)
    print('LPC11U35 CDC and HID ARM firmware links: PASS')


if __name__ == '__main__':
    main()
