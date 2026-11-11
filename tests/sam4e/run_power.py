#!/usr/bin/env python3
"""Check SAM4E power/slow-clock code with host registers and vendor headers.

This is not an Arm firmware build or a hardware power-consumption test.
"""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
INCLUDES = ['-I' + str(ROOT / p) for p in
            ('tests/sam4e/power_shim', 'include', 'ARM/Microchip/SAM4E/include')]
SOURCES = ['ARM/Microchip/SAM4E/src/system_power_sam4e.cpp',
           'ARM/Microchip/SAM4E/src/system_clock_sam4e.cpp']

for source in SOURCES:
    subprocess.run(['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                    '-D__SAM4E16E__', *INCLUDES, '-fsyntax-only',
                    str(ROOT / source)], check=True)
subprocess.run(['gcc', '-std=c11', '-D__SAM4E16E__', *INCLUDES,
                '-fsyntax-only', str(ROOT / 'ARM/Microchip/SAM4E/src/system_sam4e.c')], check=True)
with tempfile.TemporaryDirectory(prefix='sam4e-power-') as directory:
    for opt in ('-O0', '-O2'):
        binary = Path(directory) / ('power' + opt)
        subprocess.run(['g++', '-std=c++17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *INCLUDES, str(ROOT / 'tests/sam4e/power_test.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
with tempfile.TemporaryDirectory(prefix='sam4e-clock-') as directory:
    for opt in ('-O0', '-O2'):
        binary = Path(directory) / ('clock' + opt)
        subprocess.run(['g++', '-std=c++17', opt, '-Wall', '-Wextra', '-Werror',
                        '-fsanitize=undefined', '-fno-sanitize-recover=all',
                        *INCLUDES, str(ROOT / 'tests/sam4e/clock_test.cpp'),
                        '-o', str(binary)], check=True)
        subprocess.run([str(binary)], check=True)
project = ROOT / 'ARM/Microchip/SAM4E/SAM4E16E/lib/ioc/.project'
links = ET.parse(project).findall('.//link')
for source in SOURCES:
    matches = [link for link in links if link.findtext('name') == 'src/' + Path(source).name]
    assert len(matches) == 1, source
    uri = matches[0].findtext('locationURI')
    prefix, relative = uri.split('-PROJECT_LOC/')
    base = project.parent
    for _ in range(int(prefix.split('-')[1])):
        base = base.parent
    assert (base / relative).resolve() == (ROOT / source).resolve(), source
print('SAM4E vendor-header syntax and Eclipse source links PASS')
