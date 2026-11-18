#!/usr/bin/env python3
"""Host register tests and device-header syntax checks, not an Arm build."""
from pathlib import Path
import subprocess
import tempfile
import xml.etree.ElementTree as ET

root = Path(__file__).resolve().parents[2]
includes = ['-I' + str(root / path) for path in
            ('tests/sam4l/power_shim', 'include', 'ARM/Microchip/SAM4L/include')]
source = root / 'ARM/Microchip/SAM4L/src/system_power_sam4l.cpp'
with tempfile.TemporaryDirectory(prefix='sam4l-power-') as tmp:
    for family in ('C', 'S'):
        for package in ('A', 'B', 'C'):
            for size in ('2', '4', '8'):
                define = '-D__SAM4L' + family + size + package + '__'
                flags = ['g++', '-std=c++17', '-Wall', '-Wextra', '-Werror',
                         define, *includes]
                subprocess.run([*flags, '-fsyntax-only', str(source)], check=True)
                if size != '8':
                    continue
                for opt in ('-O0', '-O2'):
                    binary = Path(tmp) / (family + package + opt)
                    subprocess.run([*flags, opt, '-fsanitize=undefined',
                                    '-fno-sanitize-recover=all',
                                    str(root / 'tests/sam4l/power_test.cpp'),
                                    '-o', str(binary)], check=True)
                    subprocess.run([str(binary)], check=True)
# PICOUART receives PB00 directly; enable GPIO input monitoring
# before checking the RX level, then restore the USART pin on resume.
uart_source = (root / 'ARM/Microchip/SAM4L/src/uart_sam4l.cpp').read_text()
uart_prep = uart_source.split('bool Sam4lUartPrepareSleep(void)', 1)[1].split(
    'bool Sam4lUartSuspendSleep(void)', 1)[0]
sequence = (
    'gpio->GPIO_GPERS = 1U;',
    'if ((gpio->GPIO_PVR & 1U) == 0U)',
    'dev->pReg->US_CR = US_CR_RXDIS | US_CR_TXDIS;',
    'SAM4L_PICOUART->PICOUART_CFG = PICOUART_CFG_SOURCE(2);',
    'SAM4L_PICOUART->PICOUART_CR = PICOUART_CR_EN;',
    'Sam4lPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK & ~dev->PbaMask);',
    'Sam4lPmWrite(&SAM4L_PM->PM_AWEN, awen | wake);',
)
positions = [uart_prep.index(step) for step in sequence]
assert positions == sorted(positions), 'SAM4L PICOUART GPIO input handover order'
assert 'if (!s_Sam4lUartSleep.bRxWasGpio)' in uart_source
assert 'gpio->GPIO_GPERC = 1U;' in uart_source
print('SAM4L PICOUART GPIO input handover source order PASS')

for project in (root / 'ARM/Microchip/SAM4L').glob('*/lib/ioc/.project'):
    matches = [link for link in ET.parse(project).findall('.//link')
               if link.findtext('name') == 'src/system_power_sam4l.cpp']
    assert len(matches) == 1, project
    prefix, path = matches[0].findtext('locationURI').split('-PROJECT_LOC/')
    parent = project.parent
    for _ in range(int(prefix.split('-')[1])):
        parent = parent.parent
    assert (parent / path).resolve() == source.resolve(), project
print('SAM4L: 18 device-header syntax checks and both Eclipse source links PASS')
