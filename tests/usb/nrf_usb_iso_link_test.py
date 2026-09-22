#!/usr/bin/env python3
"""Check optional ISO linkage using the two compiled nRF52 controller objects.

Pass --base-object and --iso-object from the same target build. --tool-prefix
defaults to arm-none-eabi- and may include the toolchain's absolute path.
Relocatable links leave unrelated platform symbols unresolved; the test checks
archive extraction and strong/weak binding, not a complete firmware link.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def symbols(prefix, path):
    output = subprocess.check_output([prefix + 'nm', '-C', str(path)], text=True)
    result = {}
    for line in output.splitlines():
        match = re.match(r'^\s*(?:[0-9a-fA-F]+\s+)?([A-Za-z?])\s+(.+)$', line)
        if match:
            result[match[2].split('(')[0]] = match[1]
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--base-object', required=True, type=Path)
    parser.add_argument('--iso-object', required=True, type=Path)
    parser.add_argument('--tool-prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    hooks = ('nRFUsbdIsoService', 'nRFUsbdIsoFinishDma', 'nRFUsbdIsoSof',
             'nRFUsbdIsoEpClose', 'nRFUsbdIsoXfer', 'UsbCtrlrEpOpen')

    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for order in ((args.base_object, args.iso_object),
                      (args.iso_object, args.base_object)):
            archive = root / 'controller.a'
            archive.unlink(missing_ok=True)
            subprocess.run([args.tool_prefix + 'ar', 'rcs', str(archive),
                            *(str(path.resolve()) for path in order)], check=True)
            for iso in (False, True):
                linked = root / 'controller.o'
                command = [args.tool_prefix + 'ld', '-r', '-u', 'UsbCtrlrInit',
                           '-u', 'USBD_IRQHandler']
                if iso:
                    command += ['-u', 'UsbCtrlrIsoInit']
                subprocess.run(command + [str(archive), '-o', str(linked)], check=True)
                found = symbols(args.tool_prefix, linked)
                assert ('UsbCtrlrIsoInit' in found) == iso, found
                assert found['nRFUsbdIsoStart'] == ('T' if iso else 'w'), found
                for hook in hooks:
                    assert found[hook] == ('T' if iso else 'W'), (hook, found)
                assert 'nRFUsbdIsoEpOpen' not in found, 'obsolete forwarding helper'

    print('PASS: regular-only link omits ISO; IsoInit selects every strong ISO '
          'handler in either archive order')


if __name__ == '__main__':
    main()
