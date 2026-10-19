#!/usr/bin/env python3
"""Check production WBA I2S, DeviceIntrf and CFifo using Cortex-M33 emulation.

SAI/RCC registers and DMA transfers are modeled; this does not measure audio
clock timing or replace hardware testing. Install requirements-pdm.txt.
"""
import os
import unicorn
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

from unicorn import (Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS,
                     UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_INVALID)
from unicorn.arm_const import (UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_LR,
                               UC_ARM_REG_R2, UC_ARM_REG_R3, UC_CPU_ARM_CORTEX_M33)
from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
SAI, RCC, CONTROL = 0x40015400, 0x46020C00, 0x500FF000


def run(cmd):
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode:
        raise RuntimeError(result.stdout + result.stderr)


class Model:
    def __init__(self, uc):
        self.uc = uc
        self.fault = 0
        self.checked = 0
        self.done = False
        self.hsi = 0
        self.cr1 = [0, 0]
        self.sr = [0, 0]
        self.rx = []

    def read(self, uc, access, addr, size, value, _):
        if addr == RCC:
            uc.mem_write(addr, struct.pack('<I', self.hsi | (0 if self.fault == 1 else 1 << 10)))
        for block in range(2):
            base = SAI + 4 + 0x20 * block
            if addr == base:
                uc.mem_write(addr, struct.pack('<I', self.cr1[block]))
            if addr == base + 0x14:
                status = self.sr[block]
                status |= ((1 << 16) | 8) if block == 1 and self.rx else (8 if block == 0 else 0)
                uc.mem_write(addr, struct.pack('<I', status))
            if addr == SAI + 0x40:
                uc.mem_write(addr, struct.pack('<I', self.rx.pop(0) if self.rx else 0))
                break

    def write(self, uc, access, addr, size, value, _):
        if addr == CONTROL:
            self.fault = value
            if value == 2:
                self.sr[1] |= 1
        elif addr == CONTROL + 4:
            assert not value & 0x80000000, f'I2S firmware check {value & 0x7fffffff} failed'
            self.checked = value
        elif addr == CONTROL + 8:
            self.done = bool(value)
            uc.emu_stop()
        elif addr == CONTROL + 20:
            self.rx.append(value)
        elif addr == SAI + 0x20:
            uc.mem_write(CONTROL + 28, struct.pack('<I', value))
        elif addr == RCC:
            self.hsi = value & ~(1 << 10)
        elif addr == RCC + 0x7C and value & (1 << 21):
            self.cr1 = [0, 0]
            self.sr = [0, 0]
            self.rx = []
            uc.mem_write(SAI, bytes(0x4C))
        for block in range(2):
            base = SAI + 4 + 0x20 * block
            if addr == base:
                if self.fault == 3 and self.cr1[block] & (1 << 16):
                    value |= 1 << 16
                self.cr1[block] = value
            elif addr == base + 0x18:
                self.sr[block] &= ~value


def emulate(path):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
    for address, size in ((0x08000000, 0x200000), (0x20000000, 0x80000),
                          (0x40000000, 0x20000), (0x46020000, 0x10000),
                          (0x50000000, 0x100000), (0xE000E000, 0x2000)):
        uc.mem_map(address, size)
    with path.open('rb') as file:
        elf = ELFFile(file)
        for segment in elf.iter_segments():
            if segment['p_type'] == 'PT_LOAD':
                uc.mem_write(segment['p_paddr'], segment.data())
        start = elf.header['e_entry']
    model = Model(uc)
    uc.hook_add(UC_HOOK_MEM_READ, model.read)
    uc.hook_add(UC_HOOK_MEM_WRITE, model.write)
    def invalid(uc, access, addr, size, value, _):
        raise RuntimeError(f'unmapped I2S access 0x{addr:x}, '
                           f'PC=0x{uc.reg_read(UC_ARM_REG_PC):x}, '
                           f'LR=0x{uc.reg_read(UC_ARM_REG_LR):x}')
    uc.hook_add(UC_HOOK_MEM_INVALID, invalid)
    uc.reg_write(UC_ARM_REG_SP, 0x2007FFF0)
    try:
        uc.emu_start(start | 1, 0, count=40_000_000)
    except Exception as error:
        raise RuntimeError(f'after check {model.checked}, '
                           f'PC=0x{uc.reg_read(UC_ARM_REG_PC):x}, '
                           f'LR=0x{uc.reg_read(UC_ARM_REG_LR):x}, '
                           f'R2=0x{uc.reg_read(UC_ARM_REG_R2):x}, '
                           f'R3=0x{uc.reg_read(UC_ARM_REG_R3):x}: {error}') from error
    assert model.done and model.checked >= 80, f'incomplete: {model.checked} checks'
    return model.checked


def main():
    if unicorn.__version__ == '2.1.4':
        raise SystemExit('NOT RUN: install tests/stm32wba/requirements-pdm.txt; '
                         'use the emulator version pinned in requirements-pdm.txt')
    cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
    cc = os.environ.get('ARM_CC', 'arm-none-eabi-gcc')
    if not shutil.which(cxx) or not shutil.which(cc):
        raise SystemExit('NOT RUN: arm-none-eabi compiler unavailable')
    with tempfile.TemporaryDirectory(prefix='wba-i2s-') as temp:
        temp = Path(temp)
        for opt in ('-O0', '-Os'):
            flags = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', opt,
                     '-DSTM32WBA65xx', '-ffunction-sections', '-fdata-sections',
                     '-fno-builtin', '-Wall', '-Wextra', '-Werror']
            flags += ['-I' + str(ROOT / directory) for directory in
                      ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
                       'ARM/ST/include', 'ARM/ST/STM32WBAxx/include')]
            objects = []
            for source in ('tests/stm32wba/i2s_fw.cpp', 'ARM/ST/STM32WBAxx/src/i2s_stm32wba.cpp',
                           'ARM/ST/STM32WBAxx/src/sai_stm32wba.cpp',
                           'src/device_intrf.cpp', 'src/cfifo.c'):
                obj = temp / (Path(source).stem + '.o')
                cpp = source.endswith('.cpp')
                language = ['-std=gnu++23', '-fno-rtti', '-fno-exceptions'] if cpp else ['-std=gnu17']
                run([cxx if cpp else cc, *flags, *language, '-c', str(ROOT / source), '-o', str(obj)])
                objects.append(str(obj))
            exe = temp / 'i2s.elf'
            run([cxx, *flags, '-nostdlib', '-Wl,--gc-sections',
                 '-Wl,--wrap=CFifoPeek,--wrap=CFifoResv,--wrap=CFifoPeekMultiple,--wrap=CFifoResvMultiple', '-T' + str(HERE / 'clock_fw.ld'),
                 *objects, '-lgcc', '-o', str(exe)])
            print(f'{opt}: I2S {emulate(exe)} checks PASS', flush=True)


if __name__ == '__main__':
    main()
