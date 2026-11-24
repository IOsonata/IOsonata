#!/usr/bin/env python3
"""Compile SAM4E/SAM4L power, clock and sleep-aware drivers with Arm headers.

This checks target compilation, not linking or hardware behavior.
"""
import argparse
import os
from pathlib import Path
import re
import shutil
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
XPACK_ROOTS = (
    Path('/opt/xPacks'),
    Path.home() / '.local' / 'xPacks',
    Path('C:/Program Files/xPacks'),
)
TARGETS = (
    ('SAM4E', ('SAM4E8C', 'SAM4E8CB', 'SAM4E8E',
               'SAM4E16C', 'SAM4E16CB', 'SAM4E16E'),
     ('system_sam4e.c', 'system_power_sam4e.cpp', 'system_clock_sam4e.cpp', 'timer_sam4e.cpp')),
    ('SAM4L', tuple('SAM4L' + family + size + package
                   for family in 'CS' for size in '248' for package in 'ABC'),
     ('system_sam4l.c', 'system_power_sam4l.cpp',
      'uart_sam4l.cpp', 'i2c_sam4l.cpp', 'spi_sam4l.cpp',
      'timer_sam4l.cpp', 'timer_sam4l_ast.cpp', 'timer_sam4l_tc.cpp')),
)


def toolchain_available(prefix):
    return all(shutil.which(prefix + compiler) is not None
               for compiler in ('gcc', 'g++'))


def resolve_toolchain(override=None):
    if override is not None:
        if toolchain_available(override):
            return override
        raise SystemExit('Arm GCC/G++ not found for --toolchain-prefix ' + override)

    if toolchain_available('arm-none-eabi-'):
        return 'arm-none-eabi-'

    for name in ('ARM_GCC_HOME', 'ARM_GCC_BIN', 'ARM_GCC_ROOT'):
        folder = os.environ.get(name)
        if not folder:
            continue
        directory = Path(folder).expanduser()
        if name == 'ARM_GCC_ROOT':
            directory /= 'bin'
        prefix = str(directory / 'arm-none-eabi-')
        if toolchain_available(prefix):
            return prefix

    makefile = ROOT / 'makefile_path.mk'
    if makefile.is_file():
        for line in makefile.read_text().splitlines():
            match = re.match(r'^\s*ARM_GCC_BIN\s*[:?+]?=\s*(\S+)', line)
            if match is not None and '$' not in match.group(1):
                prefix = str(Path(match.group(1)).expanduser() / 'arm-none-eabi-')
                if toolchain_available(prefix):
                    return prefix

    candidates = []
    for root in XPACK_ROOTS:
        if root.is_dir():
            candidates.extend(root.glob('*arm-none-eabi*/bin'))
    candidates.sort(key=lambda path: tuple(int(n) for n in
                    re.findall(r'\d+', path.parent.name)), reverse=True)
    for directory in candidates:
        prefix = str(directory / 'arm-none-eabi-')
        if toolchain_available(prefix):
            return prefix

    raise SystemExit('GNU Arm GCC/G++ not found in PATH, ARM_GCC_HOME, '
                     'makefile_path.mk or installed xPacks. '
                     'Use --toolchain-prefix /path/to/bin/arm-none-eabi-')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--toolchain-prefix', default=None,
                        help='Full compiler command prefix; detect xPack when omitted')
    args = parser.parse_args()
    prefix = resolve_toolchain(args.toolchain_prefix)
    print('Arm compiler prefix: ' + prefix, flush=True)
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
                    subprocess.run([prefix + ('g++' if cpp else 'gcc'),
                                    *flags, *language, '-c', str(source), '-o',
                                    str(out / (part + '-' + source.name + '.o'))], check=True)
                print(part + ': power/clock/peripheral target compilation PASS', flush=True)


if __name__ == '__main__':
    main()
