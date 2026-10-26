#!/usr/bin/env python3
"""Build periodic device classes against FS and HS host controller models."""
from pathlib import Path
import os
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCES = [
    'tests/usb/usbd_periodic_test.cpp', 'src/usb/usb.cpp',
    'src/usb/usbd_periodic.cpp', 'src/usb/usbd_int.cpp', 'src/usb/usbd_iso.cpp',
    'src/usb/usbd_epalloc.cpp', 'src/usb/usb_int.cpp', 'src/usb/usb_iso.cpp',
    'src/usb/usb_intrf.cpp', 'src/device_intrf.cpp', 'src/cfifo.c',
    'src/app_evt_handler.cpp', 'src/app_run.cpp',
]
with tempfile.TemporaryDirectory(prefix='iosonata-periodic-') as temp:
    work = Path(temp)
    header = (ROOT / 'tests/usb/hostport/usb_ctrlr.h').read_text()
    header = header.replace('#define USB_CTRLR_ISO_INIT(DevNo) true',
                            '#define USB_CTRLR_ISO_INIT(DevNo) UsbCtrlrIsoInit(DevNo)')
    header = header.replace('bool UsbCtrlrInit(int DevNo,',
                            'bool UsbCtrlrIsoInit(int DevNo);\nbool UsbCtrlrInit(int DevNo,')
    for high_speed in (False, True):
        port = header
        if high_speed:
            port = port.replace('USB_HIGHSPEED_CAPABLE_0 = 0', 'USB_HIGHSPEED_CAPABLE_0 = 1')
            port = port.replace('USB_CTRLR0_INT_PKT_LEN_MAX = 64', 'USB_CTRLR0_INT_PKT_LEN_MAX = 1024')
            port = port.replace('USB_CTRLR0_ISO_PKT_LEN_MAX = 63', 'USB_CTRLR0_ISO_PKT_LEN_MAX = 1024')
        (work / 'usb_ctrlr.h').write_text(port)
        output = work / ('hs' if high_speed else 'fs')
        command = shlex.split(os.environ.get('CXX', 'g++')) + [
            '-std=gnu++17', '-O1', '-g', '-Wall', '-Wextra', '-Werror',
            '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections',
            '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
            '-I' + str(work), '-I' + str(ROOT / 'include'), '-x', 'c++',
            *[str(ROOT / p) for p in SOURCES],
            '-Wl,-dead_strip' if sys.platform == 'darwin' else '-Wl,--gc-sections',
            '-o', str(output),
        ]
        if sys.platform != 'darwin':
            command.insert(1, '-no-pie')
        subprocess.run(command, check=True)
        subprocess.run([str(output)], check=True)

        if not high_speed:
            for iso in (0, 1):
                example = work / ('iso-example' if iso else 'int-example')
                args = [str(ROOT / 'tests/usb/periodic_examples_test.cpp')
                        if arg == str(ROOT / SOURCES[0]) else
                        str(example) if arg == str(output) else arg
                        for arg in command]
                # The included regression main is not invoked by these fixtures.
                args.remove('-Werror')
                args.insert(1, '-DTEST_ISO_EXAMPLE=' + str(iso))
                subprocess.run(args, check=True)
                subprocess.run([str(example)], check=True)
