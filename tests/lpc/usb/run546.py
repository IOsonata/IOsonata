#!/usr/bin/env python3
"""Check LPC546xx USB preparation with vendor CMSIS registers.

Copyright (c) 2026, I-SYST inc. MIT License.
"""
from pathlib import Path
import re
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
includes = ['tests/lpc/lpc546xx/shim', 'include', 'ARM/NXP/include',
            'ARM/NXP/LPC546xx/include', 'ARM/NXP/LPC546xx/LPC54628/lib/include']
source = (root / 'ARM/NXP/LPC546xx/src/usb_hw_lpc546xx.c').read_text()
# Model the divider's asynchronous REQFLAG; keep vendor register declarations.
source, count = re.subn(r'SYSCON->USB0CLKDIV = ([^;]+);',
                        r'TestUsbDivider(\1);', source)
assert count == 2
source = '#include <stdint.h>\nvoid TestUsbDivider(uint32_t);\n' + source
for opt in ['-O0', '-O2']:
    for good_layout in [True, False]:
        with tempfile.TemporaryDirectory() as tmp:
            tmp = Path(tmp)
            (tmp / 'usb.c').write_text(source)
            exe = tmp / 'usb546'
            layout = ['-Wl,--section-start=.usb_ram=0x40100000'] if good_layout else ['-DTEST_BAD_LAYOUT']
            subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all', '-no-pie',
                            '-DLPC54628_SERIES', *layout,
                            *['-I' + str(root / p) for p in includes],
                            str(tmp / 'usb.c'), str(root / 'tests/lpc/usb/hardware546_test.c'),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=5)

# Check the production linker script: retain only referenced USB memory,
# keep NOLOAD data out of the binary, and reject USB SRAM overflow.
with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    script = (root / 'ARM/NXP/LPC546xx/ldscript/gcc_lpc54628.ld').read_text()
    script = re.sub(r'^OUTPUT_FORMAT.*$', '', script, flags=re.M)
    script = re.sub(r'^GROUP.*$', '', script, flags=re.M)
    (tmp / 'image.ld').write_text(script)
    for size, retained in [(832, True), (832, False), (8193, True)]:
        (tmp / 'image.s').write_text(f'''
.section .vectors,"a"
.long __StackTop, ResetEntry + 1, NMI_Handler + 1, HardFault_Handler + 1
.long MemManage_Handler + 1, BusFault_Handler + 1, UsageFault_Handler + 1
.long __valid_user_code_checksum
.section .text,"ax"
.global ResetEntry, NMI_Handler, HardFault_Handler
.global MemManage_Handler, BusFault_Handler, UsageFault_Handler
ResetEntry:
NMI_Handler:
HardFault_Handler:
MemManage_Handler:
BusFault_Handler:
UsageFault_Handler:
.space 4
.section .usb_ram,"aw",@nobits
.balign 256
.global usb_memory
usb_memory:
.space {size}
''')
        subprocess.run(['as', '--32', 'image.s', '-o', 'image.o'], cwd=tmp, check=True)
        args = ['ld', '-m', 'elf_i386', '-L', str(root / 'ARM/ldscript'),
                '-T', 'image.ld', '--gc-sections', 'image.o', '-o', 'image.elf']
        if retained:
            args += ['-u', 'usb_memory']
        result = subprocess.run(args, cwd=tmp, capture_output=True, text=True)
        if size > 8192:
            assert result.returncode != 0 and 'USBRAM' in result.stderr, result.stderr
            continue
        assert result.returncode == 0, result.stderr
        sections = subprocess.check_output(['readelf', '-SW', 'image.elf'], cwd=tmp, text=True)
        if retained:
            assert re.search(r'\.usb_ram\s+NOBITS\s+40100000\s+\w+\s+000340', sections), sections
        else:
            assert '.usb_ram' not in sections, sections
        subprocess.run(['objcopy', '-O', 'binary', 'image.elf', 'image.bin'], cwd=tmp, check=True)
        assert (tmp / 'image.bin').stat().st_size < 1024
print('LPC54628 USB SRAM placement, NOLOAD, garbage collection and overflow: PASS')
