#!/usr/bin/env python3
"""Compile/link the existing SAM4L TimerDemo for every virtual timer.

Uses the real vendor headers, startup, vectors and linker script. This is a
compile and link check. It does not run IOcomposer or test hardware.
"""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--toolchain-prefix', default='arm-none-eabi-')
p.add_argument('--build-dir', type=Path, default=root / 'tests/sam4l/build')
p.add_argument('--mcu', choices=['SAM4LC8C', 'SAM4LS2C', 'SAM4LS4C', 'SAM4LS8C'],
               default='SAM4LC8C')
a = p.parse_args()
family = 'SAM4LSxC' if a.mcu.startswith('SAM4LS') else 'SAM4LCxC'
linker = 'gcc_sam4lx' + a.mcu[-2] + '.ld'
project = root / 'ARM/Microchip/SAM4L' / family / 'exemples/TimerDemo/ioc'
for link in ET.parse(project / '.project').findall('.//link'):
    uri = link.findtext('locationURI')
    if not uri.startswith('PARENT-'):
        continue
    prefix, path = uri.split('-PROJECT_LOC/')
    parent = project
    for _ in range(int(prefix.split('-')[1])):
        parent = parent.parent
    assert (parent / path).is_file(), path
for config in ET.parse(project / '.cproject').findall('.//cconfiguration'):
    mode = config.find('.//configuration').get('name')
    for option in config.findall('.//option'):
        if option.get('superClass', '').endswith('cpp.linker.scriptfile'):
            for value in option:
                assert (project / mode / value.get('value').strip('"')).resolve().is_file()

files = ['src/CppRuntimeOverload.cpp', 'ARM/src/ResetEntry.c',
         'ARM/Microchip/SAM4L/src/vectors_sam4l.c',
         'ARM/Microchip/SAM4L/src/system_sam4l.c',
         'ARM/Microchip/SAM4L/src/iopincfg_sam4l.c',
         'ARM/Microchip/SAM4L/src/timer_sam4l.cpp',
         'ARM/Microchip/SAM4L/src/timer_sam4l_ast.cpp',
         'ARM/Microchip/SAM4L/src/timer_sam4l_tc.cpp',
         'src/coredev/timer.cpp', 'src/coredev/uart.cpp',
         'ARM/Microchip/SAM4L/src/uart_sam4l.cpp', 'src/uart_retarget.c',
         'src/stddev.c', 'src/cfifo.c', 'src/device_intrf.cpp']
includes = ['-I' + str(root / s) for s in ['include', 'ARM/include',
            'ARM/Microchip/SAM4L/include', 'ARM/CMSIS/Core/Include']]
def run(cmd):
    subprocess.run(cmd, check=True)
for mode, opt in [('Debug', '-O0'), ('Release', '-Os')]:
    out = a.build_dir.resolve() / a.mcu / mode
    out.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt, '-g',
             '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START',
             '-D__' + a.mcu + '__', '-DDEBUG' if mode == 'Debug' else '-DNDEBUG']
    def compile(path, extra=()):
        obj = out / (path.name + '.o')
        cpp = path.suffix == '.cpp'
        lang = ['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        run([a.toolchain_prefix + ('g++' if cpp else 'gcc'), *flags, *lang,
             *includes, *extra, '-c', str(path), '-o', str(obj)])
        return str(obj)
    objs = [compile(root / path) for path in files]
    lib = out / ('libIOsonata_' + family + '.a')
    run([a.toolchain_prefix + 'ar', 'rcs', str(lib), *objs])
    for dev in range(7):
        app = compile(root / 'exemples/timer/timer_demo.cpp',
                      ['-I' + str(project.parent / 'src'), '-DTIMER_DEMO_DEVNO=' + str(dev)])
        elf = out / ('TimerDemo-' + str(dev) + '.elf')
        run([a.toolchain_prefix + 'g++', *flags, '--specs=nano.specs', '--specs=nosys.specs',
             '-Wl,--gc-sections', '-L' + str(root / 'ARM/ldscript'),
             '-T' + str(root / 'ARM/Microchip/SAM4L/ldscript' / linker),
             app, str(lib), '-o', str(elf)])
        symbols = subprocess.check_output([a.toolchain_prefix + 'nm', str(elf)], text=True)
        for handler in ['AST_ALARM_Handler', 'AST_OVF_Handler', 'TC00_Handler',
                        'TC01_Handler', 'TC02_Handler', 'TC10_Handler', 'TC11_Handler', 'TC12_Handler']:
            assert ' T ' + handler + '\n' in symbols, handler + ' must override weak default vector'
    if family == 'SAM4LSxC':
        blinky = compile(root / 'exemples/misc/blinky.c',
                         ['-I' + str(project.parents[1] / 'Blinky/src')])
        run([a.toolchain_prefix + 'g++', *flags, '--specs=nano.specs', '--specs=nosys.specs',
             '-Wl,--gc-sections', '-L' + str(root / 'ARM/ldscript'),
             '-T' + str(root / 'ARM/Microchip/SAM4L/ldscript' / linker),
             blinky, str(lib), '-o', str(out / 'Blinky.elf')])
        for source in ['i2c_sam4l.cpp', 'spi_sam4l.cpp', 'usb_ctrlr_sam4l.cpp',
                       'usb_ctrlr_sam4l_iso.cpp']:
            compile(root / 'ARM/Microchip/SAM4L/src' / source)
    print(a.mcu + ' ' + mode + ': all seven TimerDemo devices compiled and linked', flush=True)
