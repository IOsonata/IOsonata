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
source, count = re.subn(r'SYSCON->USB1CLKDIV = ([^;]+);',
                        r'TestUsb1Divider(\1);', source)
assert count == 2
assert source.count('((void (*)(uint32_t))LPCUSB_FRO_ROM_ENTRY)(frequency);') == 1
source = source.replace('((void (*)(uint32_t))LPCUSB_FRO_ROM_ENTRY)(frequency);',
                        'TestFroSetup(frequency);')
source = ('#include <stdint.h>\nvoid TestUsbDivider(uint32_t);\nvoid TestUsb1Divider(uint32_t);\n'
          'void TestFroSetup(uint32_t);\n' + source)
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

# Run both production layers with W1C and endpoint-skip hardware semantics.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        paths = []
        for index, code in enumerate([source,
                (root / 'ARM/NXP/LPC546xx/src/usb_lpc546xx.c').read_text(),
                (root / 'ARM/NXP/src/usb_ep0_lpc_fs.c').read_text(),
                (root / 'ARM/NXP/src/usb_data_lpc_fs.c').read_text()]):
            for reg, function in [('DEVCMDSTAT', 'TestDeviceWrite'),
                                  ('INTSTAT', 'TestInterruptWrite'),
                                  ('INTSETSTAT', 'TestInterruptSet'),
                                  ('EPSKIP', 'TestSkipWrite')]:
                code = re.sub(r'(?:USB0|USBHSD|pEp0->pReg|pData->pEp0->pReg|p?[Dd]ev->Ep0\.pReg)->' + reg +
                              r'\s*=\s*([^;]+);',
                              lambda m: function + '(' + m[1] + ');', code)
            path = tmp / ('usb' + str(index) + '.c')
            path.write_text(code)
            paths.append(str(path))
        for fixture in ['ep0546_test', 'data_test', 'lifecycle546_test', 'wakeup546_test', 'hs546_test']:
            exe = tmp / fixture
            subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all', '-no-pie',
                            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                            '-DLPC54628_SERIES', '-Wl,--section-start=.usb_ram=0x40100000',
                            '-include', str(root / 'tests/lpc/usb/ep0_model.h'),
                            *['-I' + str(root / p) for p in includes],
                            *paths, str(root / ('tests/lpc/usb/' + fixture + '.c')),
                            '-o', str(exe)], check=True)
            subprocess.run([str(exe)], check=True, timeout=5)

        # Real generic core, AppEvt queue, CDC, HID and vendor bulk classes.
        sources = [(path, 'gcc') for path in paths] + [
            (str(root / 'tests/lpc/usb/integration546_model.c'), 'gcc'),
            (str(root / 'src/cfifo.c'), 'gcc'),
        ] + [(str(root / path), 'g++') for path in [
            'src/device_intrf.cpp', 'src/usb/usb.cpp', 'src/usb/usb_intrf.cpp',
            'src/usb/usbd_cdc.cpp', 'src/usb/usbd_cdc_desc.cpp',
            'src/usb/usbd_hid.cpp', 'src/usb/usbd_bulk.cpp', 'src/usb/usb_int.cpp',
            'src/usb/usbd_epalloc.cpp', 'src/app_evt_handler.cpp', 'src/app_run.cpp',
            'tests/lpc/usb/cdc_test.cpp',
        ]]
        objects = []
        for index, (path, compiler) in enumerate(sources):
            obj = str(tmp / ('class_' + str(index) + '.o'))
            objects.append(obj)
            subprocess.run([compiler, '-std=' + ('gnu11' if compiler == 'gcc' else 'gnu++23'),
                            opt, '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            '-ffunction-sections', '-fdata-sections', '-DLPC54628_SERIES',
                            '-include', str(root / 'tests/lpc/usb/ep0_model.h'),
                            *['-I' + str(root / p) for p in includes],
                            '-c', path, '-o', obj], check=True)
        exe = str(tmp / 'cdc546')
        subprocess.run(['g++', '-no-pie', '-fsanitize=undefined', '-Wl,--gc-sections',
                        '-Wl,--section-start=.usb_ram=0x40100000', *objects, '-o', exe], check=True)
        for args in [[], ['composite'], ['composite', 'bulk']]:
            subprocess.run([exe, *args], check=True, timeout=5)

        obj = str(tmp / 'integration_test.o')
        subprocess.run(['g++', '-std=gnu++23', opt, '-fsanitize=undefined',
                        '-fno-sanitize-recover=all', '-ffunction-sections', '-fdata-sections',
                        '-DLPC54628_SERIES', *['-I' + str(root / p) for p in includes],
                        '-c', str(root / 'tests/lpc/usb/integration_test.cpp'), '-o', obj], check=True)
        exe = str(tmp / 'integration546')
        subprocess.run(['g++', '-no-pie', '-fsanitize=undefined', '-Wl,--gc-sections',
                        '-Wl,--section-start=.usb_ram=0x40100000', *objects[:9], obj,
                        '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

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
