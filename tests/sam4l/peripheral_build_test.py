#!/usr/bin/env python3
"""Compile and link SAM4LS8C peripheral examples with Arm GNU.

Uses the IOcomposer example source links and application defines.
This checks selected library sources, not the full IOcomposer library.
"""
import argparse
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
p = argparse.ArgumentParser(description=__doc__)
p.add_argument('--toolchain-prefix', default='arm-none-eabi-')
p.add_argument('--taktos', type=Path)
p.add_argument('--build-dir', type=Path, default=root / 'tests/sam4l/build/peripherals')
a = p.parse_args()
cc = a.toolchain_prefix
takt = a.taktos.resolve() if a.taktos else None
examples = root / 'ARM/Microchip/SAM4L/SAM4LSxC/exemples'
common = [
    'src/CppRuntimeOverload.cpp', 'ARM/src/ResetEntry.c',
    'ARM/Microchip/SAM4L/src/vectors_sam4l.c',
    'ARM/Microchip/SAM4L/src/system_sam4l.c',
    'ARM/Microchip/SAM4L/src/iopincfg_sam4l.c',
    'src/cfifo.c', 'src/device_intrf.cpp', 'src/device.cpp',
    'src/coredev/uart.cpp', 'ARM/Microchip/SAM4L/src/uart_sam4l.cpp',
    'src/uart_retarget.c', 'src/stddev.c', 'src/prbs.c', 'src/syslog.cpp',
    'src/coredev/i2c.cpp', 'ARM/Microchip/SAM4L/src/i2c_sam4l.cpp',
    'src/coredev/spi.cpp', 'src/coredev/spi_soft.cpp',
    'ARM/Microchip/SAM4L/src/spi_sam4l.cpp',
    'src/coredev/timer.cpp', 'ARM/Microchip/SAM4L/src/timer_sam4l.cpp',
    'ARM/Microchip/SAM4L/src/timer_sam4l_ast.cpp',
    'ARM/Microchip/SAM4L/src/timer_sam4l_tc.cpp',
    'src/app_evt_handler.cpp', 'src/app_run.cpp',
    'ARM/Microchip/SAM4L/src/usb_ctrlr_sam4l.cpp',
    'ARM/Microchip/SAM4L/src/usb_ctrlr_sam4l_iso.cpp',
    'src/usb/usb.cpp', 'src/usb/usb_intrf.cpp', 'src/usb/usbd_epalloc.cpp',
    'src/usb/usbd_cdc.cpp', 'src/usb/usbd_cdc_desc.cpp', 'src/usb/usbd_hid.cpp',
    'src/usb/usb_int.cpp', 'src/usb/usb_iso.cpp',
    'src/usb/usbd_bulk.cpp', 'src/usb/usbd_msc.cpp', 'src/storage/diskio_impl.cpp',
]
inc = ['-I' + str(root / f) for f in [
    'include', 'ARM/include', 'ARM/Microchip/SAM4L/include', 'ARM/CMSIS/Core/Include']]

def run(cmd):
    subprocess.run(cmd, check=True)

def source_links(project):
    result = []
    for link in ET.parse(project / '.project').findall('.//link'):
        uri = link.findtext('locationURI')
        if uri.startswith('virtual:'):
            continue
        prefix, name = uri.split('-PROJECT_LOC/', 1)
        parent = project
        for _ in range(int(prefix.split('-')[1])):
            parent = parent.parent
        path = parent / name
        assert path.is_file(), path
        if path.suffix in ('.c', '.cpp', '.S'):
            result.append(path)
    return result

for mode, opt in [('Debug', '-O0'), ('Release', '-Os')]:
    out = a.build_dir.resolve() / mode
    out.mkdir(parents=True, exist_ok=True)
    flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt, '-g',
             '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START',
             '-D__SAM4LS8C__', '-DDEBUG' if mode == 'Debug' else '-DNDEBUG']
    def compile(path, dest, extra=()):
        obj = dest / (path.name + '.o')
        lang = ['-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if path.suffix == '.cpp' else ['-std=gnu17'] if path.suffix == '.c' else ['-x', 'assembler-with-cpp']
        run([cc + ('g++' if path.suffix == '.cpp' else 'gcc'),
             *flags, *lang, *inc, *extra, '-c', str(path), '-o', str(obj)])
        return str(obj)
    lib = out / 'libIOsonata_SAM4LSxC.a'
    objects = [compile(root / f, out) for f in common]
    run([cc + 'ar', 'rcs', str(lib), *objects])
    taktlib = None
    if takt:
        td = out / 'taktos'
        td.mkdir(exist_ok=True)
        tf = ['-DTAKT_ARCH_CM4', '-mfloat-abi=softfp', '-mfpu=fpv4-sp-d16',
              '-I' + str(takt / 'include'), '-I' + str(takt / 'ARM/include')]
        ts = sorted((takt / 'src').glob('taktos*.cpp'))
        ts += [takt / 'ARM/src/TaktKernelCM.cpp', takt / 'ARM/cm4/PendSV_M4.S']
        to = [compile(f, td, tf) for f in ts]
        taktlib = out / 'libTaktOS_M4.a'
        run([cc + 'ar', 'rcs', str(taktlib), *to])
    count = 0
    for project in sorted(examples.glob('*/ioc')):
        name = project.parent.name
        if name in ('Blinky', 'TimerDemo', 'DfuBoot'):
            continue
        sources = source_links(project)
        if name.endswith('TaktOS') and not takt:
            print('SKIP ' + name + ': pass --taktos to build', flush=True)
            continue
        cfg = next(c for c in ET.parse(project / '.cproject').findall('.//cconfiguration')
                   if c.find('.//configuration').get('name') == mode)
        defs = set()
        scripts = set()
        for option in cfg.findall('.//option'):
            sup = option.get('superClass', '')
            if sup.endswith(('.c.compiler.defs', '.cpp.compiler.defs')):
                defs.update('-D' + v.get('value') for v in option)
            if sup.endswith(('.c.linker.scriptfile', '.cpp.linker.scriptfile')):
                scripts.update((project / mode / v.get('value').strip('"')).resolve() for v in option)
        assert len(scripts) == 1 and all(f.is_file() for f in scripts), scripts
        dest = out / name
        dest.mkdir(exist_ok=True)
        extra = ['-I' + str(project.parent / 'src'), *sorted(defs)]
        if name.endswith('TaktOS'):
            extra += ['-I' + str(takt / 'include'), '-I' + str(takt / 'ARM/include')]
        app = [compile(f, dest, extra) for f in sources]
        libraries = [str(taktlib)] if name.endswith('TaktOS') else []
        elf = dest / (name + '.elf')
        run([cc + 'g++', *flags, '--specs=nano.specs', '--specs=nosys.specs',
             '-Wl,--gc-sections', '-L' + str(root / 'ARM/ldscript'),
             '-T' + str(next(iter(scripts))), *app, '-Wl,--start-group',
             *libraries, str(lib), '-lc', '-lm', '-lgcc', '-Wl,--end-group', '-o', str(elf)])
        count += 1
        print(mode + ': ' + name + ' linked', flush=True)
    print(mode + ': ' + str(count) + ' peripheral examples linked', flush=True)
