#!/usr/bin/env python3
"""Check LPC11U35 linker checksum arithmetic using host GNU binutils.

Synthetic 32-bit vector words model Thumb pointers explicitly. This does not
validate ARM relocations or execute startup code; see README.md.
"""
from pathlib import Path
import re
import struct
import subprocess
import tempfile

root = Path(__file__).resolve().parents[3]
scripts = root / 'ARM/NXP/LPC11xx/ldscript'
layouts = ['gcc_lpc11u35.ld', 'gcc_lpc11u35_dfu.ld', 'dfu_boot_lpc11u35.ld']

with tempfile.TemporaryDirectory() as tmp:
    tmp = Path(tmp)
    for name in layouts:
        # Retain the production memory map, sections, assertions and checksum.
        # Only the object format and target runtime libraries are host-specific.
        script = (scripts / name).read_text()
        script = re.sub(r'^OUTPUT_FORMAT.*$', '', script, flags=re.M)
        script = re.sub(r'^GROUP.*$', '', script, flags=re.M)
        (tmp / 'image.ld').write_text(script)
        for padding in [512, 1024]:
            (tmp / 'vectors.s').write_text(f'''
.section .vectors,"a"
.long __StackTop
.long ResetEntry + 1
.long NMI_Handler + 1
.long HardFault_Handler + 1
.long 0, 0, 0
.long __valid_user_code_checksum
.section .text,"ax"
.space {padding}
.balign 4
.global ResetEntry, NMI_Handler, HardFault_Handler, DfuTgtStart
ResetEntry:
.space 4
NMI_Handler:
.space 8
HardFault_Handler:
.space 4
DfuTgtStart:
.space 4
''')
            subprocess.run(['as', '--32', 'vectors.s', '-o', 'vectors.o'],
                           cwd=tmp, check=True)
            subprocess.run(['ld', '-m', 'elf_i386', '-L', str(scripts),
                            '-L', str(root / 'ARM/ldscript'), '-T', 'image.ld',
                            'vectors.o', '-o', 'image.elf'], cwd=tmp, check=True)
            subprocess.run(['objcopy', '-O', 'binary', '-j', '.ivector',
                            'image.elf', 'vectors.bin'], cwd=tmp, check=True)
            words = struct.unpack('<8I', (tmp / 'vectors.bin').read_bytes())
            assert sum(words) & 0xffffffff == 0, (name, padding, words)
            assert words[4:7] == (0, 0, 0)
            assert all(word & 1 for word in words[1:4])
            # Keep the top 32 bytes of SRAM0 available to the IAP ROM.
            assert words[0] == 0x10001fe0
            origin = 0xa000 if name == layouts[1] else 0
            assert origin + padding <= words[1] < origin + padding + 64
        print(name + ': checksum passed at two handler placements')
