#!/usr/bin/env python3
"""Check LPC11Uxx USB headers, hardware, EP0, lifecycle and SRAM placement."""
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
                        '-I' + str(root / 'ARM/NXP/include'),
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
                            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                            '-I' + str(root / 'tests/lpc/usb/shim'),
                            '-I' + str(root / 'include'),
                            '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                            '-I' + str(root / 'ARM/NXP/include'),
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

# Give the four side-effect registers their hardware write semantics.
# Instrument only MMIO assignments in a temporary copy; keep vendor types,
# reads, endpoint memory, control flow and callback code unchanged.
for opt in ['-O0', '-O2']:
    with tempfile.TemporaryDirectory() as tmp:
        tmp = Path(tmp)
        source = (root / 'ARM/NXP/LPC11xx/src/usb_lpc11uxx.c').read_text()
        for reg, function in [('DEVCMDSTAT', 'TestDeviceWrite'),
                              ('INTSTAT', 'TestInterruptWrite'),
                              ('INTSETSTAT', 'TestInterruptSet'),
                              ('EPSKIP', 'TestSkipWrite')]:
            source, count = re.subn(r'LPC_USB->' + reg + r'\s*=\s*([^;]+);',
                                    lambda m: function + '(' + m[1] + ');', source)
            assert count > 0, reg
        (tmp / 'usb.c').write_text(source)
        for fixture in ['ep0_test', 'lifecycle_test', 'data_test', 'wakeup_test']:
            exe = str(tmp / fixture)
            subprocess.run(['gcc', '-std=gnu11', opt, '-Wall', '-Wextra', '-Werror',
                            '-fsanitize=undefined', '-fno-sanitize-recover=all', '-no-pie',
                            '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
                            '-Wl,--section-start=.usb_ram=0x20004000',
                            '-include', str(root / 'tests/lpc/usb/ep0_model.h'),
                            '-I' + str(root / 'tests/lpc/usb/shim'),
                            '-I' + str(root / 'include'),
                            '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                            '-I' + str(root / 'ARM/NXP/include'),
                            str(root / ('tests/lpc/usb/' + fixture + '.c')), str(tmp / 'usb.c'),
                            '-o', exe], check=True)
            subprocess.run([exe], check=True, timeout=5)

        # Link the real generic USB core, interface, FIFO and DeviceIntrf with
        # the C controller. Only hardware/ROM and the application queue are
        # mocked. Host remote-wakeup permission is exercised through Chapter 9.
        sources = [
            (root / 'tests/lpc/usb/integration_model.c', 'gcc'),
            (tmp / 'usb.c', 'gcc'),
            (root / 'src/cfifo.c', 'gcc'),
            (root / 'src/device_intrf.cpp', 'g++'),
            (root / 'src/usb/usb.cpp', 'g++'),
            (root / 'src/usb/usb_intrf.cpp', 'g++'),
            (root / 'tests/lpc/usb/integration_test.cpp', 'g++'),
        ]
        objects = []
        for index, (path, compiler) in enumerate(sources):
            obj = str(tmp / ('integration_' + str(index) + '.o'))
            objects.append(obj)
            standard = 'gnu11' if compiler == 'gcc' else 'gnu++23'
            args = [compiler, '-std=' + standard, opt,
                    '-fsanitize=undefined', '-fno-sanitize-recover=all',
                    '-ffunction-sections', '-fdata-sections',
                    '-I' + str(root / 'tests/lpc/usb/shim'),
                    '-I' + str(root / 'include'),
                    '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                    '-I' + str(root / 'ARM/NXP/include')]
            if path == tmp / 'usb.c':
                args += ['-include', str(root / 'tests/lpc/usb/ep0_model.h')]
            subprocess.run([*args, '-c', str(path), '-o', obj], check=True)
        exe = str(tmp / 'integration_test')
        subprocess.run(['g++', '-no-pie', '-fsanitize=undefined',
                        '-Wl,--gc-sections', '-Wl,--section-start=.usb_ram=0x20004000',
                        *objects, '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

        # Keep the real CDC class, descriptors and application queue for
        # enumeration, control requests, notifications, loopback and reconnect.
        # The transport-only fixture above overrides descriptors and the queue.
        cdc_objects = objects[:-1]
        for index, path in enumerate(['src/usb/usbd_cdc.cpp',
                                      'src/usb/usbd_cdc_desc.cpp',
                                      'src/usb/usbd_hid.cpp',
                                      'src/usb/usbd_bulk.cpp',
                                      'src/usb/usb_int.cpp',
                                      'src/usb/usbd_epalloc.cpp',
                                      'src/app_evt_handler.cpp',
                                      'src/app_run.cpp',
                                      'tests/lpc/usb/cdc_test.cpp']):
            obj = str(tmp / ('cdc_' + str(index) + '.o'))
            subprocess.run(['g++', '-std=gnu++23', opt,
                            '-fsanitize=undefined', '-fno-sanitize-recover=all',
                            '-ffunction-sections', '-fdata-sections',
                            '-I' + str(root / 'tests/lpc/usb/shim'),
                            '-I' + str(root / 'include'),
                            '-I' + str(root / 'ARM/NXP/LPC11xx/include'),
                            '-I' + str(root / 'ARM/NXP/include'),
                            '-c', str(root / path), '-o', obj], check=True)
            cdc_objects.append(obj)
        exe = str(tmp / 'cdc_test')
        subprocess.run(['g++', '-no-pie', '-fsanitize=undefined',
                        '-Wl,--gc-sections', '-Wl,--section-start=.usb_ram=0x20004000',
                        *cdc_objects, '-o', exe], check=True)
        subprocess.run([exe], check=True, timeout=5)

        subprocess.run([exe, "composite"], check=True, timeout=5)

        subprocess.run([exe, "composite", "bulk"], check=True, timeout=5)
