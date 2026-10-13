#!/usr/bin/env python3
"""Link LPC54628 EP0 bring-up with real startup in Debug and Release.

Copyright (c) 2026, I-SYST inc. MIT License.
This is a link fixture, not a USB application or enumeration test.
"""
from pathlib import Path
import argparse
import re
import runpy
import struct
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[3]
# Reuse the established startup, runtime and retarget build configuration.
timer = runpy.run_path(str(ROOT / 'tests/lpc/timer/link546.py'))
SOURCES = timer['SOURCES'] + [
    'ARM/NXP/LPC546xx/src/usb_hw_lpc546xx.c',
    'ARM/NXP/LPC546xx/src/usb_lpc546xx.c',
    'ARM/NXP/src/usb_ep0_lpc_fs.c',
]


def build(prefix, output, mode):
    output.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
             '-ffunction-sections', '-fdata-sections', '-DLPC54628_SERIES']
    flags += ['-O0', '-g3', '-DDEBUG'] if mode == 'Debug' else ['-Os', '-DNDEBUG']
    includes = ['-I' + str(ROOT / p) for p in timer['INCLUDES']]
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
    subprocess.run([prefix + 'gcc', *flags, *includes, '-std=gnu17', '-c', str(main),
                    '-o', str(output / 'main.o')], check=True)
    archive = output / 'libIOsonata_LPC54628.a'
    subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects], check=True)
    image = output / 'ep0.elf'
    subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                    '--specs=' + ('rdimon' if mode == 'Debug' else 'nosys') + '.specs',
                    '-L' + str(ROOT / 'ARM/ldscript'),
                    '-T' + str(ROOT / 'ARM/NXP/LPC546xx/ldscript/gcc_lpc54628.ld'),
                    '-Wl,--gc-sections', str(output / 'main.o'), str(archive),
                    '-o', str(image)], check=True)
    binary = output / 'ep0.bin'
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
    sections = subprocess.check_output([prefix + 'readelf', '-SW', str(image)], text=True)
    assert re.search(r'\.usb_ram\s+NOBITS\s+40100000\s+\w+\s+000340', sections)
    print(f'LPC54628 EP0 {mode}: {len(data)} bytes flash, 832 bytes USB RAM; USB0 vector: PASS')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    linked = timer['project_sources'](ROOT / 'ARM/NXP/LPC546xx/LPC54628/lib/ioc/.project')
    for source in SOURCES:
        assert (ROOT / source).resolve() in linked, source
    with tempfile.TemporaryDirectory() as tmp:
        for mode in ['Debug', 'Release']:
            build(args.prefix, Path(tmp) / mode, mode)


if __name__ == '__main__':
    main()
