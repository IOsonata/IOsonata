#!/usr/bin/env python3
"""Build the H753 library and bare-metal examples; emulate register tests.

Requires GNU Arm GCC, Python unicorn and pyelftools. Uses the real ST CMSIS
header and driver instructions. Ready flags are modeled, not physical timing.
Run from any directory: python3 tests/stm32h7/run.py
ARM_CPPFLAGS and ARM_LDFLAGS can describe a relocated compiler installation.
"""
import os
import fnmatch
import sys
from pathlib import Path
import shlex
import shutil
import struct
import subprocess
import tempfile
import xml.etree.ElementTree as ET

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

ROOT = Path(__file__).resolve().parents[2]
PORT = ROOT / 'ARM/ST/STM32H7xx'
PROJECT = PORT / 'STM32H753'
LWIP = Path(os.environ.get('LWIP_ROOT', ROOT.parent / 'external/lwip'))
FUSION = Path(os.environ.get('FUSION_ROOT', ROOT.parent / 'external/Fusion'))
EXAMPLES = {
    'I2CMasterSlave': 'exemples/i2c/i2c_master_slave.cpp',
    'SPIMasterSlave': 'exemples/spi/spi_master_slave.cpp',
    'CryptoTest': 'exemples/crypto/crypto_engine_test.cpp',
    'Blinky': 'exemples/misc/blinky.c',
    'TimerDemo': 'exemples/timer/timer_demo.cpp',
    'UartPrbsTxTest': 'exemples/uart/uart_prbs_tx.cpp',
    'UartLoopback': 'exemples/uart/uart_loopback.cpp',
    'UsbCdcLoopback': 'exemples/usb/usb_cdc_loopback.cpp',
    'UsbCdcPrbsTx': 'exemples/usb/usb_cdc_prbs_tx.cpp',
    'UsbDualCdcStress': 'exemples/usb/usb_dual_cdc_stress.cpp',
    'UsbCustomBulkLoopback': 'exemples/usb/usb_custom_bulk_loopback.cpp',
    'UsbHidLoopback': 'exemples/usb/usb_hid_loopback.cpp',
    'UsbIntLoopback': 'exemples/usb/usb_int_loopback.cpp',
    'UsbMscRamDisk': 'exemples/usb/usb_msc_ramdisk.cpp',
    'EthernetDemo': 'exemples/net/ethernet_demo.cpp',
    'EthernetUdpDemo': 'exemples/net/ethernet_demo.cpp',
    'EthernetTcpDemo': 'exemples/net/ethernet_demo.cpp',
}


def run(args):
    p = subprocess.run(list(map(str, args)), text=True, stdout=subprocess.PIPE,
                       stderr=subprocess.STDOUT, cwd=ROOT)
    if p.returncode:
        raise RuntimeError(' '.join(map(str, args)) + '\n' + p.stdout)
    return p.stdout


def emulate(path):
    u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    for address, size in ((0x08000000, 0x200000), (0x20000000, 0x20000),
                          (0x24000000, 0x80000), (0x40000000, 0x20000000),
                          (0xe0000000, 0x100000)):
        u.mem_map(address, size)
    with path.open('rb') as f:
        elf = ELFFile(f)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD':
                u.mem_write(segment['p_vaddr'], segment.data())
        symbols = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

    def read(address):
        return struct.unpack('<I', u.mem_read(address, 4))[0]

    def write(address, value):
        u.mem_write(address, struct.pack('<I', value))

    def ready(uc, access, address, size, value, data):
        if address == 0x58024400:  # RCC CR, HSI and PLL1 ready
            value = read(address)
            value = (value | 4) if value & 1 else value & ~4
            value = (value | (1 << 13)) if value & (1 << 12) and not read(symbols["UsbClockFailure"]) else value & ~(1 << 13)
            value = (value | (1 << 25)) if value & (1 << 24) else value & ~(1 << 25)
            write(address, value)
        elif address == 0x58024410:  # RCC CFGR clock switch acknowledgement
            value = read(address)
            write(address, (value & ~0x38) | ((value & 7) << 3))
        elif address in (0x58024818, 0x58024804):  # PWR VOS and ACTVOS ready
            write(address, read(address) | (1 << 13))

    u.hook_add(UC_HOOK_MEM_READ, ready)
    write(0x5802480c, 6)  # PWR CR3, supply configuration unlocked, LDO enabled

    def call(name):
        u.reg_write(UC_ARM_REG_SP, 0x2001fff0)
        u.reg_write(UC_ARM_REG_LR, 0x080ff001)
        u.emu_start(symbols[name] | 1, 0x080ff000, count=40000000)
        assert u.reg_read(UC_ARM_REG_PC) == 0x080ff000, name + ' did not return'
        return u.reg_read(UC_ARM_REG_R0)

    call('SystemInit')
    result = call('main')
    assert result == 0, f'register_test.cpp check failed on line {result}'


def main():
    for tool in ('gcc', 'g++', 'ar', 'size', 'nm'):
        if not shutil.which('arm-none-eabi-' + tool):
            raise SystemExit('Missing arm-none-eabi-' + tool)
    links = ET.parse(PROJECT / 'lib/ioc/.project').findall('.//linkedResources/link')
    # Linked paths can start at ARM, ST, the family, or the repository root.
    candidates = []
    project_dir = PROJECT / 'lib/ioc'
    for link in links:
        name = link.findtext('name', '')
        loc = link.findtext('locationURI', '')
        if not name.startswith('src/') or Path(name).suffix not in ('.c', '.cpp'):
            continue
        prefix, relative = loc.split('/', 1)
        assert prefix.startswith('PARENT-'), loc
        levels = int(prefix.split('-')[1])
        source = project_dir.parents[levels - 1] / relative
        if relative.startswith('external/lwip/'):
            source = LWIP / relative.removeprefix('external/lwip/')
        candidates.append((name, source))
    assert candidates and all(p.is_file() for _, p in candidates)
    includes = [ROOT / p for p in ('ARM/ST/include', 'ARM/include', 'ARM/CMSIS/Core/Include', 'include')]
    includes.append(PORT / 'include')
    base = ['-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx', '-D__PROGRAM_START',
            '-ffunction-sections', '-fdata-sections', '-g']
    base += ['-I' + str(p) for p in includes]
    base += shlex.split(os.environ.get('ARM_CPPFLAGS', ''))
    ldflags = shlex.split(os.environ.get('ARM_LDFLAGS', ''))
    with tempfile.TemporaryDirectory(prefix='stm32h753-') as tmp:
        tmp = Path(tmp)
        for label, opt, fpu in (('Debug', '-O0', []), ('Release', '-Os', []),
                                ('Debug_FPU', '-O0', ['-mfloat-abi=hard', '-mfpu=fpv5-d16']),
                                ('Release_FPU', '-Os', ['-mfloat-abi=hard', '-mfpu=fpv5-d16'])):
            out = tmp / label
            out.mkdir()
            flags = [*base, opt, *fpu, '-DDEBUG' if label.startswith('Debug') else '-DNDEBUG']
            project = ET.parse(PROJECT / 'lib/ioc/.cproject')
            config = next(c for c in project.findall('.//cconfiguration')
                          if c.find('storageModule').get('name') == label)
            exclusions = [pattern for entry in config.findall('.//sourceEntries/entry')
                          for pattern in entry.get('excluding', '').split('|') if pattern]
            sources = [source for name, source in candidates
                       if not any(fnmatch.fnmatchcase(name, pattern) or
                                  name.startswith(pattern.rstrip('/') + '/')
                                  for pattern in exclusions)]
            language_flags = {}
            # Use the project's actual build-directory-relative include paths.
            # Both C and C++ need the IOsonata profile and external lwIP headers.
            for language in ('c', 'cpp'):
                option = next(o for o in config.findall('.//option')
                              if o.get('superClass', '').endswith('.' + language + '.compiler.include.paths'))
                paths = [(PROJECT / 'lib/ioc' / label / v.get('value').strip('"')).resolve()
                         for v in option.findall('listOptionValue')]
                assert (ROOT / 'include/net/lwip').resolve() in paths
                assert (ROOT.parent / 'external/lwip/src/include').resolve() in paths
                language_flags[language] = ['-I' + str(p) for p in paths]
                language_flags[language] += ['-I' + str(LWIP / 'src/include'), '-I' + str(FUSION)]
                for option in config.findall('.//option'):
                    if option.get('superClass', '').endswith('.' + language + '.compiler.defs'):
                        language_flags[language] += ['-D' + v.get('value')
                                                     for v in option.findall('listOptionValue')]

            def compile_source(source, obj, extra=()):
                cpp = source.suffix == '.cpp'
                lang = ['-std=gnu++23', '-fno-exceptions', '-fno-rtti'] if cpp else ['-std=gnu17']
                run(['arm-none-eabi-g++' if cpp else 'arm-none-eabi-gcc', *flags, *lang, *language_flags['cpp' if cpp else 'c'],
                     *extra, '-c', source, '-o', obj])

            objects = []
            for source in sources:
                obj = out / (str(len(objects)) + '_' + source.stem + '.o')
                compile_source(source, obj)
                objects.append(obj)
            archive = out / 'libIOsonata_STM32H753.a'
            run(['arm-none-eabi-ar', 'rcs', archive, *objects])

            def link(obj, elf, usb=False):
                specs = (['-specs=nano.specs', '-specs=' +
                          ('rdimon.specs' if label.startswith('Debug') else 'nosys.specs')]
                         if usb else [])
                run(['arm-none-eabi-g++', '-mcpu=cortex-m7', '-mthumb', *fpu, *ldflags, *specs,
                     '-L' + str(ROOT / 'ARM/ldscript'), '-T' + str(PORT / 'ldscript/gcc_stm32h753xx.ld'),
                     '-Wl,--gc-sections', obj, archive, *([] if usb else ['-lnosys']), '-o', elf])

            for name, rel in EXAMPLES.items():
                obj = out / (name + '.o')
                compile_source(ROOT / rel, obj, ['-I' + str(PROJECT / 'exemples' / name / 'src')])
                link(obj, out / (name + '.elf'), usb=name.startswith('Usb') or name in ('CryptoTest', 'I2CMasterSlave', 'SPIMasterSlave'))
                with (out / (name + '.elf')).open('rb') as firmware:
                    elf = ELFFile(firmware)
                    symbols = {s.name: s for s in elf.get_section_by_name('.symtab').iter_symbols()}
                    dma = elf.get_section_by_name('.eth_dma')
                    if name.startswith('Ethernet'):
                        assert dma and dma['sh_type'] == 'SHT_NOBITS'
                        assert dma['sh_addr'] % 32 == 0
                        assert 0x24000000 <= dma['sh_addr'] < dma['sh_addr'] + dma['sh_size'] <= 0x24080000
                        handler = symbols['ETH_IRQHandler']
                        assert handler['st_info']['bind'] == 'STB_GLOBAL'
                        if name != 'EthernetDemo':
                            assert symbols['SockIntrfInit']['st_info']['bind'] == 'STB_GLOBAL'
                        for segment in elf.iter_segments():
                            offset = 0x08000000 + (16 + 61) * 4 - segment['p_vaddr']
                            if segment['p_type'] == 'PT_LOAD' and 0 <= offset < segment['p_filesz']:
                                assert struct.unpack_from('<I', segment.data(), offset)[0] == handler['st_value']
                                break
                        else:
                            raise AssertionError('Ethernet vector missing')
                        if name == 'EthernetDemo':
                            print(label + ' EthernetDemo: ' + run(['arm-none-eabi-size', out / (name + '.elf')]).strip(), flush=True)
                    else:
                        assert not dma or dma['sh_size'] == 0
                        assert 'IpInit' not in symbols
                if name.startswith('Usb'):
                    with (out / (name + '.elf')).open('rb') as firmware:
                        elf = ELFFile(firmware)
                        symbols = {s.name: s for s in elf.get_section_by_name('.symtab').iter_symbols()}
                        handler = symbols['OTG_FS_IRQHandler']
                        assert handler['st_info']['bind'] == 'STB_GLOBAL'
                        for segment in elf.iter_segments():
                            offset = 0x08000000 + (16 + 101) * 4 - segment['p_vaddr']
                            if segment['p_type'] == 'PT_LOAD' and 0 <= offset < segment['p_filesz']:
                                assert struct.unpack_from('<I', segment.data(), offset)[0] == handler['st_value']
                                break
                        else:
                            raise AssertionError('USB vector missing')
            run([sys.executable, ROOT / 'tests/stm32h7/check_link.py', out])
            obj = out / 'register_test.o'
            compile_source(ROOT / 'tests/stm32h7/register_test.cpp', obj)
            link(obj, out / 'register_test.elf')
            emulate(out / 'register_test.elf')
            print(label + ': library, all examples and register tests PASS', flush=True)
    print('Hardware timing, electrical behavior and data transfers remain untested.')


if __name__ == '__main__':
    main()
