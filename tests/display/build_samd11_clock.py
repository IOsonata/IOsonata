#!/usr/bin/env python3
"""Link D11 ClockDisplayDemo using a managed archive.

Build IOsonata_SAMD11D14AM first, or pass --library-dir with Debug/Release
subdirectories containing libIOsonata_SAMD11D14AM.a. No enlarged linker region.
"""
import argparse
import os
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
TARGET = ROOT / 'ARM/Microchip/SAMD/SAMD11'
PREFIX = os.environ.get('CROSS_COMPILE', 'arm-none-eabi-')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--library-dir', type=Path, default=TARGET / 'lib/ioc')
    args = parser.parse_args()
    project = TARGET / 'exemples/ClockDisplayDemo/ioc'
    xml = ET.parse(project / '.cproject')
    opt = xml.find(".//option[@name='Include paths (-I)']")
    includes = ['-I' + str((project / 'Release' / v.get('value').strip('"')).resolve()) for v in opt]
    flags = ['-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft', '-Os', '-fsigned-char',
             '-ffunction-sections', '-fdata-sections', '-D__SAMD11D14AM__', '-D__PROGRAM_START', *includes]
    out = Path(tempfile.gettempdir()) / 'samd-clock-validation'
    for mode in ('Debug', 'Release'):
        archive = args.library_dir.resolve() / mode / 'libIOsonata_SAMD11D14AM.a'
        target = out / mode
        target.mkdir(parents=True, exist_ok=True)
        obj = target / 'clock.o'
        elf = target / 'ClockDisplayDemo.elf'
        defines = ['-DNDEBUG'] if mode == 'Release' else ['-g3']
        subprocess.run([PREFIX + 'g++', *flags, '-flto', '-std=gnu++23', '-fno-rtti',
                        '-fno-exceptions', *defines, '-c', str(ROOT / 'exemples/display/clock_display_demo.cpp'),
                        '-o', str(obj)], check=True)
        tick = target / 'timer_systick.o'
        subprocess.run([PREFIX + 'g++', *flags, '-flto', '-std=gnu++23', '-fno-rtti',
                        '-fno-exceptions', *defines, '-c', str(ROOT / 'ARM/src/timer_systick.cpp'),
                        '-o', str(tick)], check=True)
        subprocess.run([PREFIX + 'g++', *flags, '-flto', '--specs=nano.specs',
                        '-T', str(TARGET / 'ldscript/gcc_samd11d14a.ld'),
                        '-L' + str(ROOT / 'ARM/ldscript'), '-Wl,--gc-sections',
                        '-Wl,-Map,' + str(elf.with_suffix('.map')), str(obj), str(tick), str(archive),
                        '-Wl,--start-group', '-lgcc', '-lc', '-lm', '-Wl,--end-group',
                        '-o', str(elf)], check=True)
        symbols = subprocess.check_output([PREFIX + 'nm', '-C', str(elf)], text=True)
        assert all(n not in symbols for n in ('SamdUart', 'Semihost', 'SysLog', 'SamdRtcSetup',
                   'SamdTcSetup', 'SamdTccSetup', 'SamdSpimDma', 'SamdSpimInt'))
        assert any(tag + ' SysTick_Handler' in symbols for tag in ('T', 't'))
        assert 's_SysTickData' in symbols and '__Vectors' in symbols
        subprocess.run([PREFIX + 'size', str(elf)], check=True)
        subprocess.run([PREFIX + 'objcopy', '-O', 'ihex', str(elf), str(elf.with_suffix('.hex'))], check=True)
        print(mode, 'clock: PASS', flush=True)


if __name__ == '__main__':
    main()
