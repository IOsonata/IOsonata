#!/usr/bin/env python3
"""Check runtime-selected C SPI modes in the D11 archive, with and without LTO.

Run tests/display/build_samd11_console.py first to build the archive.
"""
from pathlib import Path
import importlib.util
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
spec = importlib.util.spec_from_file_location('console', ROOT / 'tests/display/build_samd11_console.py')
build = importlib.util.module_from_spec(spec)
spec.loader.exec_module(build)
archive = (Path(tempfile.gettempdir()) / 'iosonata-oled1-build/LCDConsoleDisplayDemo'
           / 'display/Release/libIOsonata_SAMD11D14AM.a')
with tempfile.TemporaryDirectory(prefix='samd-spi-link-') as directory:
    out = Path(directory)
    obj = out / 'spi.o'
    flags = ['-mcpu=cortex-m0plus', '-mthumb', '-Os', '-ffunction-sections', '-fdata-sections']
    subprocess.run([build.PREFIX + 'gcc', *flags, '-flto', '-ffat-lto-objects',
                    '-std=gnu17', '-D__SAMD11D14AM__',
                    *['-I' + str(p) for p in build.INCLUDES], '-c',
                    str(ROOT / 'tests/samd/spi_link.c'), '-o', str(obj)], check=True)
    for lto in ('-flto', '-fno-lto'):
        elf = out / ('spi' + lto + '.elf')
        subprocess.run([build.PREFIX + 'g++', *flags, lto, '--specs=nano.specs',
                        '-T', str(build.TARGET / 'ldscript/gcc_samd11d14a.ld'),
                        '-L' + str(ROOT / 'ARM/ldscript'), '-Wl,--gc-sections',
                        str(obj), str(archive), '-Wl,--start-group', '-lgcc',
                        '-lc', '-lm', '-Wl,--end-group', '-o', str(elf)], check=True)
        symbols = subprocess.check_output([build.PREFIX + 'nm', str(elf)], text=True)
        for name in ('__Vectors', 'SamdSpimPolling', 'SamdSpimInt', 'SamdSpisArm',
                     'SamdSpimDmaWait', 'SamdDmacOpen'):
            assert name in symbols, 'Runtime-selected SPI mode missing: ' + name
        print('SAMD11 C SPI all modes ' + lto + ': link PASS', flush=True)
        subprocess.run([build.PREFIX + 'size', str(elf)], check=True)
