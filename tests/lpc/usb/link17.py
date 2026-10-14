#!/usr/bin/env python3
"""LPC1769 EP0, CDC, HID and PRBS firmware links. MIT License, I-SYST, 2026."""
from pathlib import Path
import argparse
import re
import runpy
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
EXAMPLE = ROOT / 'ARM/NXP/LPC17xx/LPC1769/exemples/UsbEp0Demo'
EXAMPLES = [(EXAMPLE, 'usb_ep0_demo.cpp'),
            (EXAMPLE.parent / 'UsbCdcLoopback', str(ROOT / 'exemples/usb/usb_cdc_loopback.cpp')),
            (EXAMPLE.parent / 'UsbHidLoopback', str(ROOT / 'exemples/usb/usb_hid_loopback.cpp')),
            (EXAMPLE.parent / 'UsbCdcPrbsTx', str(ROOT / 'exemples/usb/usb_cdc_prbs_tx.cpp'))]
timer = runpy.run_path(str(ROOT / 'tests/lpc/timer/link17.py'))
SOURCES = timer['SOURCES'] + [
    'ARM/NXP/LPC17xx/src/usb_hw_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_control_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_irq_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_lpc17xx.c',
    'ARM/NXP/LPC17xx/src/usb_data_lpc17xx.c',
    'ARM/NXP/src/lpc_iap.c',
    'src/prbs.c',
    'src/usb/usb_int.cpp', 'src/usb/usbd_hid.cpp',
    'src/usb/usb_intrf.cpp', 'src/usb/usbd_epalloc.cpp',
    'src/usb/usbd_cdc.cpp', 'src/usb/usbd_cdc_desc.cpp',
    'src/usb/usb.cpp', 'src/app_evt_handler.cpp', 'src/app_run.cpp', 'src/app_wait.cpp',
]


def build(prefix, directory, debug, example, source_name):
    application = example / "src" / source_name
    directory.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m3', '-mthumb', '-DLPC1769', '-ffunction-sections', '-fdata-sections']
    flags += ['-O0', '-g3', '-DDEBUG'] if debug else ['-Os', '-DNDEBUG']
    includes = ['-I' + str(ROOT / p) for p in timer['INCLUDES']]
    includes.append('-I' + str(example / 'src'))
    objects = []
    for index, source in enumerate(SOURCES + [str(application)]):
        cpp = source.endswith('.cpp')
        obj = directory / f'{index}.o'
        language = ['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        subprocess.run([prefix + ('g++' if cpp else 'gcc'), *flags, *includes, *language,
                        '-c', str(ROOT / source), '-o', str(obj)], check=True)
        objects.append(str(obj))
    archive = directory / 'libIOsonata_LPC1769.a'
    subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects[:-1]], check=True)
    image = directory / (example.name + '.elf')
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
    if example.name in ['UsbCdcLoopback', 'UsbCdcPrbsTx']:
        demangled = subprocess.check_output([prefix + 'nm', '-C', str(image)], text=True)
        assert 'UsbdEpAlloc(' in demangled
        assert 'UsbdCdc::SelectConfig(' in demangled
        for name in ['UsbCtrlrEpSend', 'UsbCtrlrEpReceive', 'Lpc17UsbDataDispatch']:
            assert name in addresses, name
    if example.name == 'UsbCdcPrbsTx':
        assert 'Prbs8' in addresses
        assert 'AppRun' in addresses
    if example.name == 'UsbHidLoopback':
        demangled = subprocess.check_output([prefix + 'nm', '-C', str(image)], text=True)
        assert 'UsbdHid::SelectConfig(' in demangled
        assert 'UsbIntIntrfOpen' in demangled
        assert 'Lpc17UsbDataDispatch' in addresses
        assert 'AppRun' in addresses
    subprocess.run([prefix + 'size', str(image)], check=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    linked = timer['project_sources'](ROOT / 'ARM/NXP/LPC17xx/LPC1769/lib/ioc/.project')
    for source in SOURCES:
        assert (ROOT / source).resolve() in linked, source
    for example, source_name in EXAMPLES:
        app_links = timer['project_sources'](example / 'ioc/.project')
        assert app_links == {example / 'src' / source_name, example / 'src/board.h'}
        for config in ET.parse(example / 'ioc/.cproject').findall('.//configuration'):
            mode = config.get('name')
            assert mode in ['Debug', 'Release']
            specs = 'rdimon' if mode == 'Debug' else 'nosys'
            for tool in config.findall('.//tool'):
                if tool.get('superClass', '').endswith(('.c.linker', '.cpp.linker')):
                    options = [option for option in tool.findall('option')
                               if option.get('superClass', '').endswith('.linker.other')]
                    assert len(options) == 1
                    flags = options[0].get('value')
                    assert '--specs=' + specs + '.specs' in flags
                    assert ('rdimon' in flags) == (mode == 'Debug')
    with tempfile.TemporaryDirectory() as tmp:
        for debug in [True, False]:
            for example, source_name in EXAMPLES:
                build(args.prefix, Path(tmp) / example.name / ('Debug' if debug else 'Release'),
                      debug, example, source_name)
    print('LPC1769 EP0, CDC, HID and PRBS firmware Debug/Release links: PASS')
