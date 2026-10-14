#!/usr/bin/env python3
"""Link LPC54628 USB examples, composite and EP0 firmware in Debug and Release.

Copyright (c) 2026, I-SYST inc. MIT License.
Checks the managed projects and real startup; no hardware is exercised.
"""
from pathlib import Path
import argparse
import re
import runpy
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
# Reuse the established startup, runtime and retarget build configuration.
timer = runpy.run_path(str(ROOT / 'tests/lpc/timer/link546.py'))
SOURCES = timer['SOURCES'] + [
    'ARM/NXP/LPC546xx/src/usb_hw_lpc546xx.c',
    'ARM/NXP/LPC546xx/src/usb_lpc546xx.c',
    'ARM/NXP/src/usb_ep0_lpc_fs.c',
    'ARM/NXP/src/usb_data_lpc_fs.c',
    'ARM/NXP/src/lpc_iap.c',
    'src/prbs.c',
    'src/app_evt_handler.cpp', 'src/app_run.cpp', 'src/app_wait.cpp',
    'src/usb/usb.cpp', 'src/usb/usb_intrf.cpp', 'src/usb/usbd_cdc.cpp',
    'src/usb/usbd_cdc_desc.cpp', 'src/usb/usbd_epalloc.cpp',
    'src/usb/usbd_hid.cpp', 'src/usb/usbd_bulk.cpp', 'src/usb/usb_int.cpp',
]

APPLICATIONS = {
    'UsbCdcLoopback': 'exemples/usb/usb_cdc_loopback.cpp',
    'UsbCdcPrbsTx': 'exemples/usb/usb_cdc_prbs_tx.cpp',
    'UsbHidLoopback': 'exemples/usb/usb_hid_loopback.cpp',
    'UsbCustomBulkLoopback': 'exemples/usb/usb_custom_bulk_loopback.cpp',
    'UsbCompositeLoopback': 'exemples/usb/usb_composite_loopback.cpp',
}


def build(prefix, output, mode, example):
    output.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
             '-ffunction-sections', '-fdata-sections', '-DLPC54628_SERIES']
    flags += ['-O0', '-g3', '-DDEBUG'] if mode == 'Debug' else ['-Os', '-DNDEBUG']
    includes = ['-I' + str(ROOT / p) for p in timer['INCLUDES']]
    board = example if example in APPLICATIONS else 'UsbCdcLoopback'
    includes += ['-I' + str(ROOT / 'ARM/NXP/LPC546xx/LPC54628/exemples' / board / 'src')]
    objects = []
    for index, source in enumerate(SOURCES):
        cpp = source.endswith('.cpp')
        obj = output / f'{index}.o'
        language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        subprocess.run([prefix + ('g++' if cpp else 'gcc'), *flags, *includes, *language,
                        '-c', str(ROOT / source), '-o', str(obj)], check=True)
        objects.append(str(obj))
    main = output / 'main.c'
    main.write_text('''#include "usb_control_lpc546xx.h"
static void Event(const UsbCtrlrEvt_t *pEvent)
{
    if (pEvent->Type == USB_CTRLR_EVT_SETUP)
        UsbCtrlrEpStall(0, 0, false);
}
int main(void)
{
    return Lpc546xxUsbControlPrepare(6, Event) ? 0 : 1;
}
''')
    if example:
        main = ROOT / ('tests/lpc/usb/firmware/composite_firmware.cpp'
                       if example == 'Composite' else APPLICATIONS[example])
    subprocess.run([prefix + ('g++' if example else 'gcc'), *flags, *includes,
                    *(['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if example else ['-std=gnu17']), '-c', str(main),
                    '-o', str(output / 'main.o')], check=True)
    archive = output / 'libIOsonata_LPC54628.a'
    subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects], check=True)
    image = output / ((example or 'ep0') + '.elf')
    subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                    '--specs=' + ('rdimon' if mode == 'Debug' else 'nosys') + '.specs',
                    '-L' + str(ROOT / 'ARM/ldscript'),
                    '-T' + str(ROOT / 'ARM/NXP/LPC546xx/ldscript/gcc_lpc54628.ld'),
                    '-Wl,--gc-sections', str(output / 'main.o'), str(archive),
                    '-o', str(image)], check=True)
    binary = image.with_suffix('.bin')
    subprocess.run([prefix + 'objcopy', '-O', 'binary', str(image), str(binary)], check=True)
    data = binary.read_bytes()
    assert len(data) <= 512 * 1024
    assert sum(struct.unpack_from('<8I', data)) & 0xffffffff == 0
    symbols = subprocess.check_output([prefix + 'nm', str(image)], text=True)
    addresses = {name: int(addr, 16) for addr, kind, name in
                 re.findall(r'^([0-9a-fA-F]+)\s+(\w)\s+(\S+)$', symbols, re.M)}
    assert ('initialise_monitor_handles' in addresses) == (mode == 'Debug')
    assert re.search(r'^[0-9a-fA-F]+ T USB0_IRQHandler$', symbols, re.M)
    assert struct.unpack_from('<I', data, (16 + 28) * 4)[0] == addresses['USB0_IRQHandler'] | 1
    if example == 'UsbHidLoopback':
        demangled = subprocess.check_output([prefix + 'nm', '-C', str(image)], text=True)
        assert 'UsbdHid::Init(' in demangled and 'UsbIntIntrfInit' in addresses
    if example == 'UsbCustomBulkLoopback':
        demangled = subprocess.check_output([prefix + 'nm', '-C', str(image)], text=True)
        assert 'UsbdBulk::Init(' in demangled
    if example == 'UsbCdcPrbsTx':
        assert 'Prbs8' in addresses
    if example in ['Composite', 'UsbCompositeLoopback']:
        demangled = subprocess.check_output([prefix + 'nm', '-C', str(image)], text=True)
        for name in ['UsbdCdc', 'UsbdHid', 'UsbdBulk']:
            assert name + '::Init(' in demangled, name + ' initialization discarded'
        assert 'UsbIntIntrfInit' in addresses
    sections = subprocess.check_output([prefix + 'readelf', '-SW', str(image)], text=True)
    assert re.search(r'\.usb_ram\s+NOBITS\s+40100000\s+\w+\s+000340', sections)
    ram = addresses['__bss_end__'] - addresses['__data_start__']
    stack = addresses['__StackTop'] - addresses['__StackLimit']
    assert struct.unpack_from('<I', data)[0] == 0x20010000 - 32
    assert ram < 64 * 1024 - 32 and stack > 0
    print(f'LPC54628 {example or "EP0"} {mode}: {len(data)} bytes flash, 832 bytes USB RAM; '
          f'{ram} bytes main RAM; {stack} bytes linker stack allowance; USB0 vector: PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    linked = timer['project_sources'](ROOT / 'ARM/NXP/LPC546xx/LPC54628/lib/ioc/.project')
    for source in SOURCES:
        assert (ROOT / source).resolve() in linked, source
    for name, source in APPLICATIONS.items():
        example = ROOT / 'ARM/NXP/LPC546xx/LPC54628/exemples' / name
        linked = timer['project_sources'](example / 'ioc/.project')
        assert ROOT / source in linked
        assert example / 'src/board.h' in linked
        for config in ET.parse(example / 'ioc/.cproject').findall('.//configuration'):
            mode = config.get('name')
            text = ET.tostring(config, encoding='unicode')
            assert '.cortex-m4' in text and '.fpv4spd16' in text and '.hard' in text
            assert '.gnucpp23' in text
            assert '--specs=' + ('rdimon' if mode == 'Debug' else 'nosys') + '.specs' in text
            assert ('--specs=rdimon.specs' in text) == (mode == 'Debug')
    with tempfile.TemporaryDirectory() as tmp:
        for mode in ['Debug', 'Release']:
            for example in [None, *APPLICATIONS]:
                build(args.prefix, Path(tmp) / mode / (example or 'EP0'), mode, example)


if __name__ == '__main__':
    main()
