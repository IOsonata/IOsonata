#!/usr/bin/env python3
"""Check LPC linker checksum arithmetic using host GNU binutils.

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

# Link the actual LPC17xx vector source with strong application handlers.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        exe = str(Path(tmp) / 'vector17_test')
        subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        '-I' + str(root / 'tests/lpc/uart/shim'),
                        '-I' + str(root / 'include'),
                        '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                        str(root / 'tests/lpc/startup/vector17_test.c'),
                        str(root / 'ARM/NXP/LPC17xx/src/Vectors_LPC17xx.c'),
                        '-o', exe], check=True)
        symbols = subprocess.check_output(['nm', '-S', exe], text=True)
        size = int(next(line.split()[1] for line in symbols.splitlines()
                        if line.split()[-1] == 'g_Vectors'), 16)
        assert size == 51 * struct.calcsize('P'), size
        subprocess.run([exe], check=True)

# LPC17xx Cortex-M3 checksum includes all six core exception addresses.
scripts = root / 'ARM/NXP/LPC17xx/ldscript'
layouts = ['gcc_lpc1767_9.ld', 'gcc_lpc1767_9_dfu.ld', 'dfu_boot_lpc1769.ld']
for name in layouts:
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        script = (scripts / name).read_text()
        script = re.sub(r'^OUTPUT_FORMAT.*$', '', script, flags=re.M)
        script = re.sub(r'^GROUP.*$', '', script, flags=re.M)
        (tmp / 'image.ld').write_text(script)
        for padding in [512, 1024]:
            handlers = ['ResetEntry', 'NMI_Handler', 'HardFault_Handler',
                        'MemManage_Handler', 'BusFault_Handler', 'UsageFault_Handler']
            asm = '.section .vectors,"a"\n.long __StackTop\n'
            asm += ''.join('.long ' + h + ' + 1\n' for h in handlers)
            asm += '.long __valid_user_code_checksum\n'
            asm += '.section .text,"ax"\n.space ' + str(padding) + '\n.balign 4\n'
            for index, handler in enumerate(handlers):
                asm += f'.global {handler}\n{handler}:\n.space {4 * (index + 1)}\n'
            # AHB initialized data must not displace the vectors from origin.
            if name != 'dfu_boot_lpc1769.ld':
                asm += '.section .data.RAMAHB,"aw"\n.global ahb_data\nahb_data:\n.long 0x12345678\n'
            (tmp / 'vectors.s').write_text(asm)
            subprocess.run(['as', '--32', 'vectors.s', '-o', 'vectors.o'], cwd=tmp, check=True)
            subprocess.run(['ld', '-m', 'elf_i386', '-L', str(scripts),
                            '-L', str(root / 'ARM/ldscript'), '-T', 'image.ld',
                            '--gc-sections', '-u', 'ahb_data' if name != 'dfu_boot_lpc1769.ld' else 'ResetEntry',
                            'vectors.o', '-o', 'image.elf'], cwd=tmp, check=True)
            subprocess.run(['objcopy', '-O', 'binary', 'image.elf', 'image.bin'], cwd=tmp, check=True)
            words = struct.unpack('<8I', (tmp / 'image.bin').read_bytes()[:32])
            assert sum(words) & 0xffffffff == 0, (name, padding, words)
            assert all(word & 1 for word in words[1:7])
            # Keep the IAP ROM workspace out of all LPC17xx application stacks.
            assert words[0] == 0x10007fe0
            origin = 0xd000 if name == layouts[1] else 0
            assert origin + padding <= words[1] < origin + padding + 64
            if name != 'dfu_boot_lpc1769.ld':
                symbols = subprocess.check_output(['nm', 'image.elf'], cwd=tmp, text=True)
                address = int(next(line.split()[0] for line in symbols.splitlines()
                                   if line.split()[-1] == 'ahb_data'), 16)
                assert address == 0x2007c000, hex(address)
        print(name + ': checksum and image origin passed at two handler placements')

# Run the real LPC17xx SystemInit with and without the optional AHB section.
for opt in ['-O0', '-O2']:
    for ahb, vector_base in [(False, 0), (True, 0), (False, 0xd000), (True, 0xd000)]:
        with tempfile.TemporaryDirectory() as tmp:
            exe = str(Path(tmp) / 'ahb17_test')
            extra = ['-DTEST_AHB_DATA', '-Wl,--defsym=__data_ahb_size__=16'] if ahb else []
            extra += [f'-DTEST_VECTOR_BASE={vector_base}', f'-Wl,--defsym=__Vectors={vector_base}',
                      '-include', str(root / 'tests/lpc/startup/system17_shim.h')]
            subprocess.run(['gcc', '-std=gnu17', opt, '-Wall', '-Wextra',
                            '-Wno-sign-compare', '-Wno-unused-but-set-variable',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all', '-no-pie',
                            '-I' + str(root / 'tests/lpc/uart/shim'),
                            '-I' + str(root / 'include'),
                            '-I' + str(root / 'ARM/NXP/LPC17xx/Include'),
                            *extra, str(root / 'tests/lpc/startup/ahb17_test.c'),
                            str(root / 'ARM/NXP/LPC17xx/src/system_LPC17xx.c'),
                            '-o', exe], check=True)
            subprocess.run([exe], check=True)
