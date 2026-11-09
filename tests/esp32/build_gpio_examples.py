#!/usr/bin/env python3
"""Cross-build generic LED and pulse-train examples; no hardware execution."""
import argparse
from pathlib import Path
import subprocess
import re
import xml.etree.ElementTree as ET
from urllib.parse import unquote
from build_startup import ROOT, BASE, run

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--tool-prefix', default='riscv-none-elf-')
parser.add_argument('--output', type=Path, required=True)
args = parser.parse_args()
tool = args.tool_prefix
for chip in ('ESP32C3', 'ESP32C6'):
    names = {'ResetEntry.c', 'iatomic.c', 'timer.cpp', 'timer_esp32.cpp', 'led_gpio.cpp', 'led_ws2812.cpp', 'spi.cpp', 'spi_esp32.cpp', 'pulse_train.c',
             'esp32xx_irq.c', 'CppRuntimeOverload.cpp', 'uart.cpp',
             'uart_esp32.cpp', 'system_esp32_uart_clock.c', 'iopincfg_esp32.c',
             'device.cpp', 'device_intrf.cpp', 'cfifo.c', 'uart_retarget.c', 'stddev.c'}
    if chip in ('ESP32C3', 'ESP32C6'):
        names.update({'system_' + chip.lower() + '.c',
                      'Vectors_' + chip.lower() + '.c'})
    else:
        names.update({'system_esp32.c', 'Vectors_esp32.c'})
    names.add('system_esp32_system.c' if chip in ('ESP32C2', 'ESP32C3')
              else 'system_esp32_pcr.c')
    project = BASE / chip / 'lib/ioc'
    selected = []
    for link in ET.parse(project / '.project').findall('.//linkedResources/link'):
        uri = unquote(link.findtext('locationURI'))
        match = re.fullmatch(r'PARENT-(\d+)-PROJECT_LOC/(.+)', uri)
        if match and Path(match[2]).name in names:
            source = project.parents[int(match[1]) - 1] / match[2]
            assert source.is_file(), source
            selected.append(source)
    assert {p.name for p in selected} == names
    for config, opt in (('Debug', '-O0'), ('Release', '-Os')):
        directory = args.output.resolve() / chip / config
        directory.mkdir(parents=True, exist_ok=True)
        flags = ['-march=rv32imc_zicsr', '-mabi=ilp32', opt,
                 '-ffunction-sections', '-fdata-sections', '-D' + chip]
        flags += ['-I' + str(ROOT / p) for p in
                  ('include', 'RISCV/include', 'RISCV/Espressif/include')]
        objects = []
        for source in selected:
            cpp = source.suffix == '.cpp'
            obj = directory / (source.stem + '.o')
            run(tool + ('g++' if cpp else 'gcc'), *flags,
                *(['-std=gnu++20', '-fno-rtti', '-fno-exceptions'] if cpp else ['-std=gnu11']),
                '-c', source, '-o', obj)
            objects.append(obj)
        library = directory / 'libgpio.a'
        run(tool + 'ar', 'rcs', library, *objects)
        for name, filename in (('LedDemo', 'led_demo.cpp'), ('PulseTrain', 'pulse_train_test.c')):
            app = BASE / chip / 'exemples' / name
            links = ET.parse(app / 'ioc/.project').findall('.//linkedResources/link')
            uri = 'PARENT-7-PROJECT_LOC/exemples/misc/' + filename
            assert any(x.findtext('locationURI') == uri for x in links)
            obj = directory / (name + '.o')
            elf = obj.with_suffix('.elf')
            cpp = filename.endswith('.cpp')
            run(tool + ('g++' if cpp else 'gcc'), *flags,
                *(['-std=gnu++20', '-fno-rtti', '-fno-exceptions'] if cpp else ['-std=gnu11']),
                '-I' + str(app / 'src'), '-c', ROOT / 'exemples/misc' / filename, '-o', obj)
            run(tool + 'g++', *flags, obj, library, '-L' + str(ROOT / 'RISCV/ldscript'),
                '-Wl,--undefined=ResetEntry', '-Wl,--gc-sections', '--specs=nano.specs',
                '--specs=nosys.specs', '-T', BASE / 'ldscript' / ('gcc_' + chip.lower() + '.ld'), '-o', elf)
            symbols = subprocess.check_output([tool + 'nm', '-C', str(elf)], text=True)
            assert ('LedWs2812::' if cpp else 'PulseTrain') in symbols
            assert ('SPIInit' in symbols) == cpp
            for unwanted in ('Pwm', 'PWM', 'LedPwm'):
                assert unwanted not in symbols, unwanted
            assert ('UARTInit' in symbols) == cpp
            run(tool + 'objcopy', '-O', 'binary', elf, elf.with_suffix('.bin'))
            assert elf.with_suffix('.bin').read_bytes()[:8] == bytes.fromhex('1d04dbae1d04dbae')
            print(chip, config, name, 'PASS', flush=True)

# Existing boards keep PWM by default; GPIO-only boards can opt out.
for mode in ('default-pwm', 'gpio-only'):
    directory = args.output.resolve() / mode
    directory.mkdir(parents=True, exist_ok=True)
    board = ''.join(f'#define LED{i}_PORT 0\n#define LED{i}_PIN {i + 3}\n#define LED{i}_ACTIVE LED_LOGIC_HIGH\n' for i in (1, 2, 3))
    if mode == 'gpio-only':
        board += '#define LED_DEMO_PWM_ENABLE 0\n'
    (directory / 'board.h').write_text(board)
    obj = directory / 'led_demo.o'
    run(tool + 'g++', *flags, '-std=gnu++20', '-fno-rtti', '-fno-exceptions',
        '-I' + str(directory), '-c', ROOT / 'exemples/misc/led_demo.cpp', '-o', obj)
    symbols = subprocess.check_output([tool + 'nm', '-C', str(obj)], text=True)
    assert ('PWMInit' in symbols) == (mode == 'default-pwm')
    assert 'SPIInit' not in symbols
    print('Generic LED', mode, 'compile PASS', flush=True)
