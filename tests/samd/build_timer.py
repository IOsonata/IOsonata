#!/usr/bin/env python3
"""Link D11 TimerDemo and check timer-family elimination using a managed archive.

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
    project = TARGET / 'exemples/TimerDemo/ioc'
    xml = ET.parse(project / '.cproject')
    opt = xml.find(".//option[@name='Include paths (-I)']")
    includes = ['-I' + str((project / 'Release' / v.get('value').strip('"')).resolve()) for v in opt]
    flags = ['-mcpu=cortex-m0plus', '-mthumb', '-mfloat-abi=soft', '-Os', '-fsigned-char',
             '-ffunction-sections', '-fdata-sections', '-D__SAMD11D14AM__', *includes]
    out = Path(tempfile.gettempdir()) / 'samd-timer-validation'
    for mode in ('Debug', 'Release'):
        archive = args.library_dir.resolve() / mode / 'libIOsonata_SAMD11D14AM.a'
        if not archive.is_file():
            parser.error('Managed archive missing: ' + str(archive))
        target = out / mode
        target.mkdir(parents=True, exist_ok=True)
        # The real demo retains its logs: semihosting in Debug, UART in Release.
        cases = [('demo', None, True)]
        if mode == 'Release':
            cases += [('dev' + str(dev), dev, True) for dev in (0, 1, 2, 3)]
            cases += [('runtime-lto', -1, True), ('runtime-native', -1, False)]
        for name, dev, lto in cases:
            demo = dev is None
            obj = target / (name + '.o')
            elf = target / (name + '.elf')
            source = ROOT / ('exemples/timer/timer_demo.cpp' if demo else 'tests/samd/timer_link.c')
            language = ['-std=gnu++23', '-fno-rtti', '-fno-exceptions'] if demo else ['-std=gnu17']
            options = ['-flto'] if lto else ['-fno-lto']
            defines = ['-DNDEBUG'] if mode == 'Release' else ['-g3']
            if not demo:
                defines += ['-DTIMER_LINK_DEVNO=' + str(dev)]
            subprocess.run([PREFIX + ('g++' if demo else 'gcc'), *flags, *language, *options,
                            *defines, '-c', str(source), '-o', str(obj)], check=True)
            # newlib can introduce _sbrk after LTO; retain the existing startup hook.
            retain = ['-Wl,--undefined=_sbrk'] if demo else []
            subprocess.run([PREFIX + 'g++', *flags, *options, '--specs=nano.specs',
                            *retain, '-T', str(TARGET / 'ldscript/gcc_samd11d14a.ld'),
                            '-L' + str(ROOT / 'ARM/ldscript'), '-Wl,--gc-sections',
                            '-Wl,-Map,' + str(elf.with_suffix('.map')), str(obj), str(archive),
                            '-Wl,--start-group', '-lgcc', '-lc', '-lm', '-Wl,--end-group',
                            '-o', str(elf)], check=True)
            symbols = subprocess.check_output([PREFIX + 'nm', '-C', str(elf)], text=True)
            selected = 0 if demo else dev
            if selected >= 0:
                absent = ('SamdTcSetup', 'SamdTccSetup', 'SamdCounterTimerIrq') if selected == 0 else (
                    ('SamdRtcSetup', 'SamdRtcWait', 'SamdRtcTimerIrq', 'SamdTccSetup') if selected < 3
                    else ('SamdRtcSetup', 'SamdRtcWait', 'SamdRtcTimerIrq', 'SamdTcSetup'))
                assert all(n not in symbols for n in absent), (name, 'Unused timer linked')
            else:
                assert all(n in symbols for n in ('SamdRtcSetup', 'SamdTcSetup', 'SamdTccSetup'))
            if demo:
                assert 'SamdUartDmaService' not in symbols and 'SamdUartPoll(' not in symbols
            print(mode, name, 'PASS', flush=True)
            subprocess.run([PREFIX + 'size', str(elf)], check=True)
    print('D11 timer link checks: PASS')


if __name__ == '__main__':
    main()
