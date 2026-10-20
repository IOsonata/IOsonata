#!/usr/bin/env python3
"""Build the real WBA65 ADC4 port at -O0/-Os and run it in Unicorn.

Requires arm-none-eabi-g++, unicorn and pyelftools. Models W1C flags,
set-only CR commands, delayed calibration/ready/conversion and injected
failures. Not a simulation of analog voltages or electrical timing.
"""
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP
from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
ADC, RCC, CONTROL = 0x46021000, 0x46020C00, 0x500FF000


def run(cmd):
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode:
        raise RuntimeError(p.stdout + p.stderr)


class Model:
    def __init__(self, uc):
        self.uc = uc
        self.fault = 0
        self.reset()
        self.checked = 0
        self.done = False
        self.inputs = []

    def reset(self):
        self.reg = {0: 0, 4: 0, 8: 0, 12: 0, 16: 0, 20: 0, 40: 0, 64: 0}
        self.pending = None
        self.ticks = 0

    def ready(self):
        if not self.pending:
            return
        self.ticks -= 1
        if self.ticks:
            return
        p, self.pending = self.pending, None
        if p == 'cal':
            self.reg[8] &= ~(1 << 31)
        elif p == 'enable':
            self.reg[0] |= 1
        else:
            bits = 12 - 2 * ((self.reg[12] >> 2) & 3)
            self.reg[64] = 1 << (bits - 1)
            self.reg[0] |= 12 | (16 if self.fault == 2 else 0)
            self.reg[8] &= ~4

    def read(self, uc, access, addr, size, value, _):
        off = addr - ADC
        if off in self.reg:
            if off in (0, 8):
                self.ready()
            uc.mem_write(addr, struct.pack('<I', self.reg[off]))
            if off == 64:
                self.reg[0] &= ~4

    def write(self, uc, access, addr, size, value, _):
        if addr == CONTROL:
            self.fault = value
            return
        if addr == CONTROL + 4:
            assert not value & 0x80000000, f'firmware check {value & 0x7fffffff} failed'
            self.checked += 1
            return
        if addr == CONTROL + 8:
            assert value == self.checked
            self.done = True
            uc.emu_stop()
            return
        if addr == RCC + 0x6C and value & 32:
            self.reset()
            return
        if addr == ADC + 0x308:
            assert (value >> 18) & 15 == 7, 'ADC prescaler must be /16'
            return
        off = addr - ADC
        if off not in self.reg:
            return
        assert int.from_bytes(uc.mem_read(RCC + 0x94, 4), 'little') & 32, 'ADC clock disabled'
        if off == 0:
            self.reg[0] &= ~value  # W1C
        elif off == 8:
            if value & (1 << 28):
                self.reg[8] |= 1 << 28
                if self.fault != 3:
                    self.reg[0] |= 1 << 12
            if value & (1 << 31):
                assert not self.reg[8] & 1, 'calibration with ADEN set'
                assert self.reg[0] & (1 << 12), 'regulator not ready'
                self.reg[8] |= 1 << 31
                if self.fault != 4:
                    self.pending, self.ticks = 'cal', 3
            if value & 1:
                assert not self.reg[8] & (1 << 31), 'enable before calibration completed'
                self.reg[8] |= 1
                if self.fault != 5:
                    self.pending, self.ticks = 'enable', 3
            if value & 2:
                assert not self.reg[8] & 4, 'disable with conversion running'
                self.reg[8] &= ~3
                self.reg[0] &= ~1
            if value & 4:
                assert self.reg[8] & 1 and self.reg[0] & 1, 'conversion before ready'
                mask = self.reg[40]
                assert mask and mask & (mask - 1) == 0, 'one selected input required'
                self.inputs.append(mask.bit_length() - 1)
                self.reg[8] |= 4
                if self.fault not in (1, 6):
                    self.pending, self.ticks = 'convert', 3
            if value & 16:
                if self.fault == 6:
                    self.reg[8] |= 16
                else:
                    self.reg[8] &= ~(4 | 16)
                self.pending = None
        else:
            if off in (12, 16, 20, 40):
                assert not self.reg[8] & 4, 'configuration during conversion'
            self.reg[off] = value


def emulate(path):
    uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
    uc.mem_map(0x08000000, 0x200000)
    uc.mem_map(0x20000000, 0x80000)
    uc.mem_map(0x46020000, 0x10000)
    uc.mem_map(0x50000000, 0x100000)
    with path.open('rb') as f:
        elf = ELFFile(f)
        for seg in elf.iter_segments():
            if seg['p_type'] == 'PT_LOAD':
                uc.mem_write(seg['p_paddr'], seg.data())
        start = elf.header['e_entry']
    model = Model(uc)
    uc.hook_add(UC_HOOK_MEM_READ, model.read)
    uc.hook_add(UC_HOOK_MEM_WRITE, model.write)
    uc.reg_write(UC_ARM_REG_SP, 0x2007FFF0)
    uc.emu_start(start | 1, 0, count=40_000_000)
    assert model.done and model.checked >= 30, f'incomplete: {model.checked} checks'
    assert model.inputs and set(model.inputs) == {5, 6}, model.inputs
    return model.checked


def main():
    cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
    if not shutil.which(cxx):
        raise SystemExit('NOT RUN: arm-none-eabi-g++ unavailable')
    with tempfile.TemporaryDirectory(prefix='wba-adc-') as td:
        td = Path(td)
        for opt in ('-O0', '-Os'):
            flags = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', opt,
                     '-std=gnu++23', '-DSTM32WBA65xx', '-ffunction-sections', '-fdata-sections',
                     '-fno-rtti', '-fno-exceptions', '-fno-builtin', '-Wall', '-Wextra',
                     '-Wno-overloaded-virtual']
            flags += ['-I' + str(ROOT / d) for d in ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
                      'ARM/ST/include', 'ARM/ST/STM32WBAxx/include')]
            objects = []
            for src in ('tests/stm32wba/adc_fw.cpp', 'ARM/ST/STM32WBAxx/src/adc_stm32wba.cpp',
                        'src/device.cpp', 'src/device_intrf.cpp'):
                obj = td / (Path(src).stem + '.o')
                run([cxx, *flags, '-c', str(ROOT / src), '-o', str(obj)])
                objects.append(str(obj))
            exe = td / 'adc.elf'
            run([cxx, *flags, '-nostdlib', '-Wl,--gc-sections', '-T' + str(HERE / 'clock_fw.ld'),
                 *objects, '-lgcc', '-o', str(exe)])
            print(f'{opt}: ADC4 {emulate(exe)} checks PASS', flush=True)


if __name__ == '__main__':
    main()
