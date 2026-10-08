#!/usr/bin/env python3
"""Compile/link the existing SAM4LCxC TimerDemo for every virtual timer.

Uses the real vendor headers, startup, vectors and linker script. This is a
source-closure check, not an IOcomposer invocation or hardware validation.
"""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--toolchain-prefix', default='arm-none-eabi-')
p.add_argument('--build-dir', type=Path, default=root / 'tests/sam4l/build')
a = p.parse_args()
project = root / 'ARM/Microchip/SAM4L/SAM4LCxC/exemples/TimerDemo/ioc'
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
         'src/coredev/timer.cpp']
includes = ['-I' + str(root / s) for s in ['include', 'ARM/include',
            'ARM/Microchip/SAM4L/include', 'ARM/CMSIS/Core/Include']]
def run(cmd):
    subprocess.run(cmd, check=True)
for mode, opt in [('Debug', '-O0'), ('Release', '-Os')]:
    out = a.build_dir.resolve() / mode
    out.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt, '-g',
             '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START',
             '-D__SAM4LC8C__', '-DDEBUG' if mode == 'Debug' else '-DNDEBUG']
    def compile(path, extra=()):
        obj = out / (path.name + '.o')
        cpp = path.suffix == '.cpp'
        lang = ['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
        run([a.toolchain_prefix + ('g++' if cpp else 'gcc'), *flags, *lang,
             *includes, *extra, '-c', str(path), '-o', str(obj)])
        return str(obj)
    objs = [compile(root / path) for path in files]
    lib = out / 'libIOsonata_SAM4LCxC.a'
    run([a.toolchain_prefix + 'ar', 'rcs', str(lib), *objs])
    for dev in range(7):
        app = compile(root / 'exemples/timer/timer_demo.cpp',
                      ['-I' + str(project.parent / 'src'), '-DTIMER_DEMO_DEVNO=' + str(dev)])
        elf = out / ('TimerDemo-' + str(dev) + '.elf')
        run([a.toolchain_prefix + 'g++', *flags, '--specs=nano.specs', '--specs=nosys.specs',
             '-Wl,--gc-sections', '-L' + str(root / 'ARM/ldscript'),
             '-T' + str(root / 'ARM/Microchip/SAM4L/ldscript/gcc_sam4lx8.ld'),
             app, str(lib), '-o', str(elf)])
        symbols = subprocess.check_output([a.toolchain_prefix + 'nm', str(elf)], text=True)
        for handler in ['AST_ALARM_Handler', 'AST_OVF_Handler', 'TC00_Handler',
                        'TC01_Handler', 'TC02_Handler', 'TC10_Handler', 'TC11_Handler', 'TC12_Handler']:
            assert ' T ' + handler + '\n' in symbols, handler + ' must override weak default vector'
    print(mode + ': all seven TimerDemo devices compiled and linked', flush=True)
