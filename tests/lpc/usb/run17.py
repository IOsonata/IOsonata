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
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-include', str(root / 'tests/lpc/usb/lifecycle17_shim.h'),
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        '-I' + str(root / 'ARM/NXP/include'),
                        '-I' + str(root / 'include'),
                        str(root / 'tests/lpc/usb/lifecycle17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/usb_lpc17xx.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)
