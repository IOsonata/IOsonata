#!/usr/bin/env python3
"""Build LPC UART, SPI, I2C and LPC546xx peripheral examples in Debug and Release.

Copyright (c) 2026, I-SYST inc. MIT License.
Uses the timer suite's verified startup/runtime source lists and the managed
project board headers. No hardware is exercised. The LPC54628 EthernetDemo is
built when lwIP is in external/lwip beside IOsonata, else it is skipped.
"""
from pathlib import Path
import argparse
import re
import runpy
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]
LWIP = (ROOT.parent / 'external/lwip').resolve()
TARGETS = {
    '11': ('LPC11xx', 'LPC11U35', 'link.py', 'cortex-m0', 'LPC11XX_SERIES', 'gcc_lpc11u35.ld'),
    '17': ('LPC17xx', 'LPC1769', 'link17.py', 'cortex-m3', 'LPC1769', 'gcc_lpc1767_9.ld'),
    '546': ('LPC546xx', 'LPC54628', 'link546.py', 'cortex-m4', 'LPC54628_SERIES', 'gcc_lpc54628.ld'),
}
APPLICATIONS = {
    'UartLoopback': 'exemples/uart/uart_loopback.cpp',
    'SpiLoopback': 'exemples/spi/spi_loopback.cpp',
    'I2CScan': 'exemples/i2c/i2c_scan.cpp',
}
# Examples of one target only
TARGET_APPLICATIONS = {
    '546': {'I2CMasterSlave': 'exemples/i2c/i2c_master_slave.cpp',
            'SPIMasterSlave': 'exemples/spi/spi_master_slave.cpp',
            'WdtDemo': 'exemples/misc/wdt_demo.cpp',
            'RtcDemo': 'exemples/misc/rtc_demo.cpp',
            'AdcDemo': 'exemples/converters/adc_demo.cpp',
            'PwmDemo': 'exemples/pwm/pwm_demo.cpp',
            'NvmDemo': 'exemples/storage/nvm_demo.cpp',
            'SpifiFlashDemo': 'exemples/storage/nvm_demo.cpp',
            'SdramTest': 'exemples/misc/ext_ram_test.cpp',
            'SdCardTest': 'exemples/storage/sdcard_test.cpp',
            'AudioToneDemo': 'exemples/audio/codec_tone.cpp',
            'AudioLineLoopback': 'exemples/audio/codec_line_loopback.cpp',
            'LcdDemo': 'exemples/display/fb_display_demo.cpp'},
}
# Symbols the image must define
SYMBOLS = {
    'UartLoopback': ['UARTInit'],
    'SpiLoopback': ['SPIInit'],
    'I2CScan': ['I2CInit'],
    'I2CMasterSlave': ['I2CInit', 'DMA0_IRQHandler', 'FLEXCOMM2_IRQHandler', 'FLEXCOMM8_IRQHandler'],
    'SPIMasterSlave': ['SPIInit', 'DMA0_IRQHandler', 'FLEXCOMM3_IRQHandler', 'FLEXCOMM9_IRQHandler'],
    'WdtDemo': ['WdtInit', 'WdtStart', 'WDT_BOD_IRQHandler'],
    'RtcDemo': ['RTC_IRQHandler', '_ZN11RtcLpc546xx4InitERK12__Rtc_ConfigP11DeviceIntrf'],
    'AdcDemo': ['_ZN11AdcLpc546xx4InitERK12__ADC_ConfigP5TimerP11DeviceIntrf'],
    'PwmDemo': ['PWMInit', 'PWMSetDutyCycle'],
    'NvmDemo': ['NvmMcuErase', '_Z9NvmMcuCfgR9__Nvm_Cfg', 'IAPCopyRamToFlash'],
    'SpifiFlashDemo': ['Lpc546xxSpifiInit', 'QuadSPISendCmd', 'QuadSPISetMemSize'],
    'SdramTest': ['SdramInit'],
    'SdCardTest': ['SdmmcInit', '_ZN6SDCard4InitEP11DeviceIntrfP19__DiskIO_Cache_Desci',
                   '_ZN6SDCard8ReadSectEmPhm', '_ZN6SDCard9WriteSectEmPKhm'],
    'AudioToneDemo': ['I2SInit', 'Lpc546xxAudioPllStart', 'FLEXCOMM6_IRQHandler', 'DMA0_IRQHandler',
                      '_ZN6Wm89044InitERK15__Wm8904_ConfigP11DeviceIntrf'],
    'AudioLineLoopback': ['I2SInit', 'Lpc546xxAudioPllStart', 'FLEXCOMM6_IRQHandler', 'FLEXCOMM7_IRQHandler',
                          'DMA0_IRQHandler', '_ZN6Wm89044InitERK15__Wm8904_ConfigP11DeviceIntrf'],
    'LcdDemo': ['_ZN11LcdLpc546xx4InitERK13__Display_CfgP11DeviceIntrf', 'SdramInit', 'I2CInit',
                '_ZN10TouchGt9114InitERK14__Touch_ConfigP11DeviceIntrf',
                '_ZN11TouchFt54064InitERK14__Touch_ConfigP11DeviceIntrf'],
    'EthernetDemo': ['EthernetInit', 'EthernetPollLink', 'ETHERNET_IRQHandler', 'IpInit'],
}


def build(target, prefix, directory):
    family, mcu, script, cpu, define, linker = TARGETS[target]
    timer = runpy.run_path(str(ROOT / 'tests/lpc/timer' / script))
    sources = timer['SOURCES'] + ['src/coredev/i2c.cpp', 'src/coredev/spi.cpp']
    if target == '546':
        sources += ['ARM/NXP/LPC546xx/src/i2c_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/spi_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/wdt_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/rtc_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/adc_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/pwm_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/nvm_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/spifi_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/emc_lpc546xx.cpp',
                    'ARM/NXP/LPC546xx/src/sdmmc_lpc546xx.cpp',
                    'src/storage/sdcard_impl.cpp',
                    'src/crc.c',
                    'ARM/NXP/LPC546xx/src/i2s_lpc546xx.cpp',
                    'src/audio/wm8904.cpp',
                    'ARM/NXP/LPC546xx/src/lcd_lpc546xx.cpp',
                    'src/display/display.cpp',
                    'src/display/fonts/syst5x7.c',
                    'src/display/touch.cpp',
                    'src/display/touch_ft5406.cpp',
                    'src/display/touch_gt911.cpp',
                    'src/storage/diskio_impl.cpp',
                    'ARM/NXP/src/lpc_iap.c',
                    'src/storage/nvm.cpp',
                    'src/storage/nvm_region.cpp',
                    'src/miscdev/rtc.cpp',
                    'src/device.cpp',
                    'src/syslog.cpp']
    else:
        sources += ['ARM/NXP/src/i2c_lpc_master.cpp', 'ARM/NXP/src/spi_lpcxx.c']
        sources += (['ARM/NXP/LPC11xx/src/i2c_lpc11uxx.cpp', 'ARM/NXP/LPC11xx/src/spi_lpc11uxx.c']
                    if target == '11' else
                    ['ARM/NXP/LPC17xx/src/i2c_lpc17xx.c', 'ARM/NXP/LPC17xx/src/spi_lpc17xx.c'])
    project = ROOT / 'ARM/NXP' / family / mcu
    linked = timer['project_sources'](project / 'lib/ioc/.project')
    applications = {**APPLICATIONS, **TARGET_APPLICATIONS.get(target, {})}
    net_includes = []
    if target == '546':
        if LWIP.is_dir():
            sources += ['ARM/NXP/LPC546xx/src/ethernet_lpc546xx.cpp', 'src/net/ip_lwip.cpp',
                        'src/app_evt_handler.cpp', 'src/app_run.cpp', 'src/app_wait.cpp']
            sources += sorted(str(path) for path in linked if LWIP in path.parents)
            net_includes = ['-I' + str(ROOT / 'include/net/lwip'), '-I' + str(LWIP / 'src/include'),
                            '-DIOSONATA_LWIP_POOL_SECTION=.ram2']
            applications['EthernetDemo'] = 'exemples/net/ethernet_demo.cpp'
        else:
            print(f'{mcu} EthernetDemo: SKIP, no {LWIP}', flush=True)
    for source in sources:
        assert (ROOT / source).resolve() in linked, source
    for name, application in applications.items():
        example = project / 'exemples' / name
        app_links = timer['project_sources'](example / 'ioc/.project')
        assert ROOT / application in app_links
        assert example / 'src/board.h' in app_links
        for config in ET.parse(example / 'ioc/.cproject').findall('.//configuration'):
            text = ET.tostring(config, encoding='unicode')
            debug = config.get('name') == 'Debug'
            assert '--specs=' + ('rdimon' if debug else 'nosys') + '.specs' in text
            assert ('--specs=rdimon.specs' in text) == debug
    for mode in ['Debug', 'Release']:
        output = directory / target / mode
        output.mkdir(parents=True)
        flags = ['-mcpu=' + cpu, '-mthumb', '-fsigned-char', '-D' + define,
                 '-ffunction-sections', '-fdata-sections']
        if target == '546':
            flags += ['-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
        flags += ['-O0', '-g3', '-DDEBUG'] if mode == 'Debug' else ['-Os', '-DNDEBUG']
        includes = ['-I' + str(ROOT / path) for path in timer['INCLUDES']] + net_includes
        objects = []
        for index, source in enumerate(sources):
            cpp = source.endswith('.cpp')
            language = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
            obj = output / f'{index}.o'
            subprocess.run([prefix + ('g++' if cpp else 'gcc'), *flags, *includes, *language,
                            '-c', str(ROOT / source), '-o', str(obj)], check=True)
            objects.append(str(obj))
        archive = output / 'libIOsonata.a'
        subprocess.run([prefix + 'ar', 'rcs', str(archive), *objects], check=True)
        for name, application in applications.items():
            obj = output / (name + '.o')
            subprocess.run([prefix + 'g++', *flags, *includes,
                            '-I' + str(project / 'exemples' / name / 'src'),
                            '-std=gnu++23', '-fno-exceptions', '-fno-rtti',
                            '-c', str(ROOT / application), '-o', str(obj)], check=True)
            image = output / (name + '.elf')
            subprocess.run([prefix + 'g++', *flags, '--specs=nano.specs',
                            '--specs=' + ('rdimon' if mode == 'Debug' else 'nosys') + '.specs',
                            '-L' + str(ROOT / 'ARM/ldscript'),
                            '-T' + str(ROOT / 'ARM/NXP' / family / 'ldscript' / linker),
                            '-Wl,--gc-sections', str(obj), str(archive), '-o', str(image)], check=True)
            binary = image.with_suffix('.bin')
            subprocess.run([prefix + 'objcopy', '-O', 'binary', str(image), str(binary)], check=True)
            assert sum(struct.unpack_from('<8I', binary.read_bytes())) & 0xffffffff == 0
            symbols = subprocess.check_output([prefix + 'nm', str(image)], text=True)
            assert ('initialise_monitor_handles' in symbols) == (mode == 'Debug')
            for symbol in SYMBOLS[name]:
                assert re.search(r'^\w+ T ' + symbol + '$', symbols, re.M), symbol
            print(f'{mcu} {name} {mode}: vector checksum, driver link and semihosting PASS', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target', choices=TARGETS)
    parser.add_argument('--prefix', default='arm-none-eabi-')
    args = parser.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        for target in [args.target] if args.target else TARGETS:
            build(target, args.prefix, Path(tmp))


if __name__ == '__main__':
    main()
