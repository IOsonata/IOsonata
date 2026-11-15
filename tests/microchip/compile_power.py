#!/usr/bin/env python3
"""Compile SAM4E/SAM4L power and clock sources with real Arm CMSIS headers.

This checks target compilation, not linking or hardware behavior.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
TARGETS = (
    ('SAM4E', ('SAM4E8C', 'SAM4E8CB', 'SAM4E8E',
               'SAM4E16C', 'SAM4E16CB', 'SAM4E16E'),
     ('system_sam4e.c', 'system_power_sam4e.cpp', 'system_clock_sam4e.cpp')),
    ('SAM4L', tuple('SAM4L' + family + size + package
                   for family in 'CS' for size in '248' for package in 'ABC'),
     ('system_sam4l.c', 'system_power_sam4l.cpp')),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain-prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='microchip-power-arm-') as tmp:
        out = Path(tmp)
        for family, parts, sources in TARGETS:
            base = ROOT / 'ARM/Microchip' / family
            includes = [ROOT / 'include', ROOT / 'ARM/include',
                        ROOT / 'ARM/CMSIS/Core/Include', base / 'include']
            for part in parts:
                flags = ['-mcpu=cortex-m4', '-mthumb', '-Os',
                         '-D__PROGRAM_START', '-D__' + part + '__']
                flags += ['-I' + str(path) for path in includes]
                files = [base / 'src' / source for source in sources]
                if family == 'SAM4E':
                    header = base / 'include' / (part.lower() + '.h')
                    # Expand every typed peripheral address, including those
                    # not referenced by the power and clock implementations.
                    names = re.findall(r'^#define\s+(SAM4E_\w+)\s+\(\(\w+\s*\*\)',
                                       header.read_text(), re.MULTILINE)
                    probe = out / (part + '.c')
                    probe.write_text('#include "sam4e.h"\n' + ''.join(
                        '_Static_assert(sizeof(*' + name + ') > 0, "' + name + '");\n'
                        for name in names))
                    files.append(probe)
                for source in files:
                    cpp = source.suffix == '.cpp'
                    language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
                    subprocess.run([args.toolchain_prefix + ('g++' if cpp else 'gcc'),
                                    *flags, *language, '-c', str(source), '-o',
                                    str(out / (part + '-' + source.name + '.o'))], check=True)
                print(part + ': power/clock target compilation PASS', flush=True)


if __name__ == '__main__':
    main()
