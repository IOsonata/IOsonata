#!/usr/bin/env python3
"""ARM register-model test of Nvm and both production data-memory ports.
Requires arm-none-eabi-g++, Python unicorn and pyelftools. No hardware claim.
"""
import os
from pathlib import Path
import struct
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

ROOT = Path(__file__).resolve().parents[3]
TC = os.environ.get('ARM_PREFIX', 'arm-none-eabi-')

class Model:
    def __init__(self, elf, ra):
        self.ra = ra
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        self.uc.mem_map(0, 0x100000)
        self.uc.mem_map(0x08000000, 0x100000)
        self.uc.mem_map(0x20000000, 0x20000)
        self.uc.mem_map(0x40000000, 0x800000)
        self.uc.mem_map(0xE0000000, 0x100000)
        with open(elf, 'rb') as f:
            e = ELFFile(f)
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    self.uc.mem_write(seg['p_vaddr'], seg.data())
            syms = e.get_section_by_name('.symtab')
            self.symbols = {s.name: s['st_value'] for s in syms.iter_symbols()}
        count = 15 if ra else 3
        self.regs = struct.unpack('<' + 'I' * count, self.uc.mem_read(self.symbols['g_Regs'], count * 4))
        self.base = 0x40100000 if ra else 0x08080000
        self.size = 8192 if ra else 16384
        self.uc.mem_write(self.base, bytes([0xFF if ra else 0]) * self.size)
        self.state = dict.fromkeys(self.regs, 0)
        self.state[self.regs[0]] = 0 if ra else 7
        self.key = 0
        self.mode_seq = []
        self.fault = None
        self.programs = self.erases = self.resets = 0
        self.uc.hook_add(UC_HOOK_MEM_WRITE, self.write)
        self.uc.hook_add(UC_HOOK_MEM_READ, self.read)

    def read(self, uc, access, addr, size, value, user):
        if addr in self.state:
            uc.mem_write(addr, int(self.state[addr]).to_bytes(size, 'little'))

    def write(self, uc, access, addr, size, value, user):
        if not self.ra:
            pecr, key, sr = self.regs
            if addr == sr:
                self.state[sr] &= ~value
            elif addr == key:
                if value == 0x89ABCDEF: self.key = 1
                elif value == 0x02030405 and self.key == 1:
                    self.state[pecr] &= ~1
                    self.key = 0
                else: raise AssertionError('bad EEPROM unlock keys')
            elif addr == pecr:
                assert not self.state[sr] & 1, 'PECR written while busy'
                self.state[pecr] = value
            elif self.base <= addr < self.base + self.size:
                assert size == 1 and not self.state[pecr] & 1
                assert not self.state[pecr] & ((1 << 3) | (1 << 9) | (1 << 10))
                self.programs += 1
                self.state[sr] = 1 if self.fault == 'timeout' else (1 << 8 if self.fault == 'error' else 2)
            return
        if addr not in self.state: return
        entry, mode, key, fisr, cmd, ready, err, blank, reset, sh, sl, eh, el, data, enable = self.regs
        if addr == entry:
            assert value in (0xAA80, 0xAA00)
            self.state[entry] = value & 0xFF
        elif addr == key:
            assert value == 0xA5
            self.mode_seq = []
        elif addr == mode:
            self.mode_seq.append(value)
            if len(self.mode_seq) == 3:
                a, b, c = self.mode_seq
                assert a == c and b == a ^ 0xFF
                self.state[mode] = value
        elif addr == reset:
            if value:
                self.resets += 1
                self.state[ready] = self.state[cmd] = self.state[err] = self.state[blank] = 0
        elif addr == cmd:
            self.state[cmd] = value
            if value == 0:
                self.state[ready] = 0
                return
            assert self.state[entry] == 0x80 and self.state[mode] in (0x10, 0x50)
            assert self.state[fisr] & 31 == 23
            start = (self.state[sh] << 16) | self.state[sl]
            end = (self.state[eh] << 16) | self.state[el]
            assert 0xFE000000 <= start <= end < 0xFE002000
            p = self.base + start - 0xFE000000
            n = end - start + 1
            if value == 0x83:
                self.state[blank] = 8 if any(b != 255 for b in uc.mem_read(p, n)) else 0
            elif value == 0x84:
                assert n == 1024 and start % 1024 == 0
                self.erases += 1
                uc.mem_write(p, b'\xff' * n)
            elif value == 0x81:
                assert n == 1 and self.state[blank] == 0
                self.programs += 1
                if self.fault == 'error': self.state[err] = 2
                elif self.fault != 'timeout': uc.mem_write(p, bytes([self.state[data] & 255]))
            else: raise AssertionError('unexpected FACI command')
            self.state[ready] = 0 if value == 0x81 and self.fault == 'timeout' else 0x40
        else:
            self.state[addr] = value

    def run(self, case):
        self.uc.reg_write(UC_ARM_REG_SP, 0x2001F000)
        self.uc.reg_write(UC_ARM_REG_LR, 0x000F0001)
        self.uc.reg_write(UC_ARM_REG_R0, case)
        self.uc.emu_start(self.symbols['RunTest'] | 1, 0x000F0000, count=60000000)
        assert self.uc.reg_read(UC_ARM_REG_PC) == 0x000F0000, 'test did not return'
        assert self.uc.reg_read(UC_ARM_REG_R0) == 0, 'checks.cpp line ' + str(self.uc.reg_read(UC_ARM_REG_R0))


def build(out, ra, opt):
    target = 'ARM/Renesas/RA4M1' if ra else 'ARM/ST/STM32L1xx'
    cpu = ['-mcpu=cortex-m4', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard'] if ra else ['-mcpu=cortex-m3', '-mfloat-abi=soft']
    flags = cpu + ['-mthumb', opt, '-std=gnu++23', '-fno-exceptions', '-fno-rtti', '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START', '-DRA4M1' if ra else '-DSTM32L152xE']
    flags += ['-I' + str(ROOT / p) for p in ['include', 'ARM/include', 'ARM/CMSIS/Core/Include', target + '/include', 'ARM/ST/include']]
    sources = ['tests/storage/data_nvm/checks.cpp', target + '/src/' + ('nvm_ra4m1.cpp' if ra else 'nvm_stm32l1xx.cpp'), 'src/storage/nvm.cpp', 'src/device.cpp', 'src/device_intrf.cpp', 'src/coredev/spi.cpp', 'src/CppRuntimeOverload.cpp', 'tests/storage/data_nvm/stubs.cpp']
    objs = []
    for i, src in enumerate(sources):
        obj = out / (str(i) + '.o')
        subprocess.run([TC + 'g++'] + flags + ['-c', str(ROOT / src), '-o', str(obj)], check=True)
        objs.append(str(obj))
    script = out / 'test.ld'
    script.write_text('MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 512K\n RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K }\nSECTIONS { .text : { *(.text*) *(.rodata*) } > FLASH\n .data : { *(.data*) } > RAM\n .bss : { *(.bss*) *(COMMON) } > RAM\n /DISCARD/ : { *(.ARM.exidx*) } }\n')
    elf = out / 'test.elf'
    subprocess.run([TC + 'g++'] + cpu + ['-mthumb', '--specs=nano.specs', '--specs=nosys.specs', '-nostartfiles', '-Wl,--gc-sections,-e,RunTest,-u,g_Regs', '-T' + str(script)] + objs + ['-o', str(elf)], check=True)
    return elf

with tempfile.TemporaryDirectory(prefix='data-nvm-') as td:
    for ra in (False, True):
        for opt in ('-O0', '-Os'):
            elf = build(Path(td), ra, opt)
            m = Model(elf, ra)
            m.run(0)
            assert m.programs > 0
            if ra: assert m.erases == 4 and m.state[m.regs[0]] == 0
            else: assert m.state[m.regs[0]] & 1
            for fault in ('error', 'timeout'):
                m = Model(elf, ra)
                m.fault = fault
                m.run(1)
                if ra:
                    assert m.resets > 0 and m.state[m.regs[0]] == 0
                m.fault = None
                if not ra: m.state[m.regs[2]] = 0 # hardware finishes after timeout
                m.run(2)
            print(('RA4M1' if ra else 'L152') + ' ' + opt + ': PASS', flush=True)
