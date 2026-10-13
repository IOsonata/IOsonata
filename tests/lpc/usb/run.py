#!/usr/bin/env python3
"""Check LPC11Uxx USB headers, hardware preparation and SRAM placement."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
for compiler, standard in [('gcc', 'c11'), ('g++', 'c++17')]:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'header_test')
        subprocess.run([compiler, '-std=' + standard, '-Wall', '-Wextra',
                        '-Werror', '-I' + str(root / 'include'),
                        '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                        str(root / 'tests/lpc/usb/header_test.c'),
                        '-o', exe], check=True)
        subprocess.run([exe], check=True)
print('LPC11Uxx USB C/C++ header checks: PASS')

# Use real vendor peripheral types and the production C implementation.
# The host core shim records NVIC/barriers and can complete an EPSKIP request.
for opt in ['-O0', '-O2']:
    for valid_layout in [True, False]:
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(Path(tmp) / 'hardware_test')
            layout = ['-Wl,--section-start=.usb_ram=0x20004000'] if valid_layout else ['-DTEST_BAD_LAYOUT']
            subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all', '-no-pie',
                            '-I' + str(root / 'tests/lpc/usb/shim'),
                            '-I' + str(root / 'include'),
                            '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                            *layout, str(root / 'tests/lpc/usb/hardware_test.c'),
                            str(root / 'ARM/NXP/LPC11xx/src/usb_lpc11uxx.c'),
                            '-o', exe], check=True)
            subprocess.run([exe], check=True, timeout=5)

# Exercise the production memory map with host binutils, like startup/run.py.
# NOLOAD must keep USB SRAM out of the flash image and startup zeroing;
# unused storage must be discarded, and overflow must fail the link.
import re
with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    script = (root / 'ARM/NXP/LPC11xx/ldscript/gcc_lpc11u35.ld').read_text()
    script = re.sub(r'^OUTPUT_FORMAT.*$', '', script, flags=re.M)
    script = re.sub(r'^GROUP.*$', '', script, flags=re.M)
    (tmp / 'image.ld').write_text(script)
    for size, retained in [(832, True), (832, False), (2049, True)]:
        (tmp / 'image.s').write_text(f'''
.section .vectors,"a"
.long __StackTop, ResetEntry + 1, NMI_Handler + 1, HardFault_Handler + 1
.long 0, 0, 0, __valid_user_code_checksum
.section .text,"ax"
.global ResetEntry, NMI_Handler, HardFault_Handler
ResetEntry:
NMI_Handler:
HardFault_Handler:
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
        if size > 2048:
            assert result.returncode != 0 and 'RAM3' in result.stderr, result.stderr
            continue
        assert result.returncode == 0, result.stderr
        sections = subprocess.check_output(['readelf', '-SW', 'image.elf'], cwd=tmp, text=True)
        if retained:
            assert re.search(r'\.usb_ram\s+NOBITS\s+20004000\s+\w+\s+000340', sections), sections
        else:
            assert '.usb_ram' not in sections, sections
        subprocess.run(['objcopy', '-O', 'binary', 'image.elf', 'image.bin'], cwd=tmp, check=True)
        assert (tmp / 'image.bin').stat().st_size < 1024
print('LPC11U35 USB SRAM placement, NOLOAD, garbage collection and overflow: PASS')
