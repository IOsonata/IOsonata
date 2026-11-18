#!/usr/bin/env python3
"""Host register tests and device-header syntax checks, not an Arm build."""
from pathlib import Path
import subprocess
import sys
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
# PICOUART shares the physical RXD0 pin directly. Do not switch PB00
# to GPIO or gate USART0 before the low-power receiver is armed.
uart_source = (root / 'ARM/Microchip/SAM4L/src/uart_sam4l.cpp').read_text()
uart_prep = uart_source.split('bool Sam4lUartPrepareSleep(void)', 1)[1].split(
    'bool Sam4lUartSuspendSleep(void)', 1)[0]
uart_arm = uart_prep.split('s_Sam4lUartSleep.bActive = true;', 1)[1]
sequence = (
    'dev->pReg->US_IDR = 0xFFFFFFFFU;',
    'SAM4L_PICOUART->PICOUART_CFG = PICOUART_CFG_SOURCE(2);',
    'SAM4L_PICOUART->PICOUART_CR = PICOUART_CR_EN;',
    'if ((picoStatus & PICOUART_SR_EN) == 0U)',
    'if ((picoStatus & PICOUART_SR_DRDY) != 0U)',
    'dev->pReg->US_CR = US_CR_RXDIS | US_CR_TXDIS;',
    'Sam4lPmWrite(&SAM4L_PM->PM_PBAMASK, SAM4L_PM->PM_PBAMASK & ~dev->PbaMask);',
    'Sam4lPmWrite(&SAM4L_PM->PM_AWEN, awen | wake);',
)
positions = [uart_arm.index(step) for step in sequence]
assert positions == sorted(positions), 'SAM4L PICOUART direct RX handover order'
assert 'gpio->GPIO_GPERS' not in uart_prep
assert 'gpio->GPIO_PVR' not in uart_prep
assert 'gpio->GPIO_STERS' not in uart_prep
assert uart_arm.count('Sam4lUartResumeSleep();') == 1
assert 'SAM4L_PICO_ENABLE_TIMEOUT' in uart_source
assert 'rejected |= SAM4L_PICO_STALE_DATA;' not in uart_prep
assert 'SAM4L_PICO_REJECTED | SAM4L_PICO_ARMED_DATA' not in uart_prep
assert 's_Sam4lUartSleep.Diagnostic |= SAM4L_PICO_ARMED_DATA;' in uart_prep
uart_resume = uart_source.split('void Sam4lUartResumeSleep(void)\n{', 1)[1].split(
    'void UARTSetCtrlLineState(', 1)[0]
assert 'SAM4L_PICO_PREPARED | SAM4L_PICO_PM_WAKE' in uart_resume
assert 'SAM4L_PICO_PM_CAUSE | SAM4L_PICO_DATA_READY' in uart_resume
print('SAM4L PICOUART direct RX handover and retained-status handling PASS')

# Wake test changes host baud only while the normal USART is active.
# Both sides establish a 1 Mbaud handshake before the MCU can sleep.
demo_source = (root / 'exemples/misc/power_off_demo.cpp').read_text()
host_source = (root / 'Python/sam4l_uart_wake.py').read_text()
setup_prompt = demo_source.index('UART wake setup: send R at 1000000 baud')
switch_prompt = demo_source.index('UART sleep preparation: switch host to 9600 baud')
host_guard = demo_source.index('msDelay(750);', switch_prompt)
sleep_entry = demo_source.index('SystemSleep();', host_guard)
assert setup_prompt < switch_prompt < host_guard < sleep_entry
assert "if (rxByte != 'R')" in demo_source[setup_prompt:switch_prompt]
host_setup = host_source.index('if b"UART wake setup: send R" in line:')
host_switch = host_source.index('port.baudrate = 9600', host_setup)
host_settle = host_source.index('time.sleep(args.wake_delay)', host_switch)
host_send = host_source.index('send_byte(port, 2)', host_settle)
assert host_setup < host_switch < host_settle < host_send
assert 'parser.add_argument("--wake-delay", type=float, default=1.25' in host_source
assert 'if not 1.25 <= args.wake_delay < args.total_timeout:' in host_source
assert 'Old PowerOffDemo firmware.' in host_source
print('SAM4L UART host-rate handshake precedes PICOUART activation PASS')

# Exercise host acceptance with a simulated serial port, without hardware.
subprocess.run([sys.executable,
                str(root / 'tests/sam4l/uart_wake_host_test.py')], check=True)

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
