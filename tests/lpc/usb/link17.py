#!/usr/bin/env python3
"""LPC1769 public EP0 firmware links. MIT License, I-SYST, 2026."""
from pathlib import Path
import argparse
import re
import runpy
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
timer = runpy.run_path(str(ROOT / 'tests/lpc/timer/link17.py'))
SOURCES = timer['SOURCES'] + [
    'ARM/NXP/LPC17xx/src/usb_hw_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_control_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_irq_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_lpc17xx.c',
    'ARM/NXP/src/lpc_iap.c',
    'src/usb/usb.cpp', 'src/app_evt_handler.cpp', 'src/app_run.cpp', 'src/app_wait.cpp',
]


def build(prefix, directory, debug):
    directory.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m3', '-mthumb', '-DLPC1769', '-ffunction-sections', '-fdata-sections']
    flags += ['-O0', '-g3', '-DDEBUG'] if debug else ['-Os', '-DNDEBUG']
    includes = ['-I' + str(ROOT / p) for p in timer['INCLUDES']]
    objects = []
    for index, source in enumerate(SOURCES + ['tests/lpc/usb/firmware/ep017_firmware.cpp']):
        cpp = source.endswith('.cpp')
        obj = directory / f'{index}.o'
        language = ['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        subprocess.run([prefix + ('g++' if cpp else 'gcc'), *flags, *includes, *language,
                        '-c', str(ROOT / source), '-o', str(obj)], check=True)
        objects.append(str(obj))
    archive = directory / 'libIOsonata_LPC1769.a'
    subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects[:-1]], check=True)
    image = directory / 'ep0.elf'
    subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                    '--specs=' + ('rdimon' if debug else 'nosys') + '.specs',
                    '-L' + str(ROOT / 'ARM/ldscript'),
                    '-T' + str(ROOT / 'ARM/NXP/LPC17xx/ldscript/gcc_lpc1767_9.ld'),
                    '-Wl,--gc-sections', objects[-1], str(archive), '-o', str(image)], check=True)
    binary = image.with_suffix('.bin')
    subprocess.run([prefix + 'objcopy', '-O', 'binary', str(image), str(binary)], check=True)
    data = binary.read_bytes()
    symbols = subprocess.check_output([prefix + 'nm', str(image)], text=True)
    addresses = {name: int(addr, 16) for addr, kind, name in
                 re.findall(r'^([0-9a-fA-F]+)\s+(\w)\s+(\S+)$', symbols, re.M)}
    assert sum(struct.unpack_from('<8I', data)) & 0xffffffff == 0
    assert len(data) < 512 * 1024
    assert addresses['__bss_end__'] - addresses['__data_start__'] < 32 * 1024 - 32
    assert struct.unpack_from('<I', data, 40 * 4)[0] == addresses['USB_IRQHandler'] | 1
    assert re.search(r'^\w+ T USB_IRQHandler$', symbols, re.M)
    assert ('initialise_monitor_handles' in addresses) == debug
    for name in ['IAPReadUID', 'IapEntry', 'UsbInit', 'UsbEnable', 'UsbDevProcessEvent', 'UsbCtrlrStart', 'Lpc17UsbControlInterrupt']:
        assert name in addresses, name
    subprocess.run([prefix + 'size', str(image)], check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    linked = timer['project_sources'](ROOT / 'ARM/NXP/LPC17xx/LPC1769/lib/ioc/.project')
    for source in SOURCES:
        assert (ROOT / source).resolve() in linked, source
    with tempfile.TemporaryDirectory() as tmp:
        for debug in [True, False]:
            build(args.prefix, Path(tmp) / ('Debug' if debug else 'Release'), debug)
    print('LPC1769 EP0 public firmware Debug/Release links: PASS')
