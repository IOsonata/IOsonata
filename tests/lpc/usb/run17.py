#!/usr/bin/env python3
"""LPC1769 USB clock preparation checks. MIT License, I-SYST, 2026."""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'hw17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/startup/system17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        str(root / 'tests/lpc/usb/hw17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_hw_lpc17xx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

# Model only W1C and command writes; retain vendor register reads/control flow.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp) / 'usb17.c'
        code = (root / 'ARM/NXP/LPC17xx/src/usb_hw_lpc17xx.c').read_text()
        for register, hook in [('USBDevIntClr', 'TestSieClear'), ('USBCmdCode', 'TestSieCode')]:
            code = re.sub(r'LPC_USB->' + register + r' = ([^;]+);',
                          lambda match: hook + '(' + match[1] + ');', code)
        source.write_text(code)
        exe = str(Path(tmp) / 'sie17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/usb/sie17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        str(root / 'tests/lpc/usb/sie17_test.c'), str(source),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

# Model only W1C and command writes; retain vendor register reads/control flow.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp) / 'usb17.c'
        code = (root / 'ARM/NXP/LPC17xx/src/usb_hw_lpc17xx.c').read_text()
        for register, hook in [('USBDevIntClr', 'TestSieClear'), ('USBCmdCode', 'TestSieCode'),
                               ('USBMaxPSize', 'TestPacketSize'), ('USBTxData', 'TestPacketWrite'),
                               ('USBEpIntClr', 'TestEndpointClear')]:
            code = re.sub(r'LPC_USB->' + register + r' = ([^;]+);',
                          lambda match: hook + '(' + match[1] + ');', code)
        code = code.replace('LPC_USB->USBRxData', 'TestPacketRead()')
        code = code.replace('LPC_USB->USBMaxPSize', 'TestPacketMaxSize()')
        source.write_text(code)
        exe = str(Path(tmp) / 'sie17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/usb/packet17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        str(root / 'tests/lpc/usb/packet17_test.c'), str(source),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

# Control event tests use the real shared LPC event definitions and mock only
# the private FIFO/SIE boundary; register access uses the vendor CMSIS header.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'control17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/startup/system17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        '-I' + str(root / 'ARM/NXP/include'),
                        '-I' + str(root / 'include'),
                        str(root / 'tests/lpc/usb/control17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_control_lpc17xx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        source = Path(tmp) / 'irq17.c'
        code = (root / 'ARM/NXP/LPC17xx/src/usb_irq_lpc17xx.c').read_text()
        for register, hook in [('USBDevIntClr', 'TestIrqClear'), ('USBDevIntSet', 'TestIrqSet')]:
            code = re.sub(r'LPC_USB->' + register + r' = ([^;]+);',
                          lambda match: hook + '(' + match[1] + ');', code)
        source.write_text(code)
        exe = str(Path(tmp) / 'irq17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/usb/irq17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        '-I' + str(root / 'ARM/NXP/include'),
                        '-I' + str(root / 'include'),
                        str(root / 'tests/lpc/usb/irq17_test.c'), str(source),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'lifecycle17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-ffunction-sections', '-Wl,--gc-sections',
                        '-DLPC1769', '-Wno-pointer-to-int-cast',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/usb/lifecycle17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        '-I' + str(root / 'ARM/NXP/include'),
                        '-I' + str(root / 'include'),
                        str(root / 'tests/lpc/usb/lifecycle17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_lpc17xx.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_data_lpc17xx.c'),
                        str(root / 'ARM/NXP/src/lpc_iap.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

# Private CPU transfer state uses mocked FIFO helpers and real LPC callbacks.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'data17')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/usb/lifecycle17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        '-I' + str(root / 'ARM/NXP/include'),
                        '-I' + str(root / 'include'),
                        str(root / 'tests/lpc/usb/data17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_data_lpc17xx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)


# Real generic classes/core plus the public LPC1769 driver and transfer state.
# Only hardware FIFO, SIE and EP0 handshakes are replaced by the model.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        flags = [opt, '-Wall', '-Wextra', '-Werror', '-Wno-missing-field-initializers',
                 '-ffunction-sections', '-fdata-sections', '-DLPC1769',
                 '-fsanitize=undefined', '-fno-sanitize-recover=all',
                 '-include', str(root / 'tests/lpc/usb/lifecycle17_shim.h')]
        includes = ['-I' + str(root / p) for p in ['tests/lpc/uart/shim',
                    'ARM/NXP/LPC17xx/Include', 'ARM/NXP/include', 'include']]
        sources = ['tests/lpc/usb/integration17_model.c', 'tests/lpc/usb/classes17_test.cpp',
                   'ARM/NXP/LPC17xx/src/usb_lpc17xx.c',
                   'ARM/NXP/LPC17xx/src/usb_data_lpc17xx.c',
                   'src/usb/usb.cpp', 'src/usb/usb_intrf.cpp', 'src/usb/usb_int.cpp',
                   'src/usb/usbd_cdc.cpp', 'src/usb/usbd_cdc_desc.cpp',
                   'src/usb/usbd_hid.cpp', 'src/usb/usbd_bulk.cpp',
                   'src/usb/usbd_epalloc.cpp', 'src/device_intrf.cpp',
                   'src/app_evt_handler.cpp', 'src/cfifo.c']
        objects = []
        for index, source in enumerate(sources):
            cpp = source.endswith('.cpp')
            obj = tmp / f'{index}.o'
            subprocess.run(['g++' if cpp else 'gcc',
                            '-std=gnu++20' if cpp else '-std=gnu17',
                            *([] if cpp else ['-Wno-pointer-to-int-cast']),
                            *flags, *includes, '-c', str(root / source), '-o', str(obj)], check=True)
            objects.append(str(obj))
        exe = str(tmp / 'classes17')
        subprocess.run(['g++', *flags, '-Wl,--gc-sections', *objects, '-o', exe], check=True)
        for order in range(8):
            subprocess.run([exe, str(order)], check=True, timeout=10)
