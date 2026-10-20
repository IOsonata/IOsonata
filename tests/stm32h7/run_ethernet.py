#!/usr/bin/env python3
# MIT License
#
# Copyright (c) 2026, I-SYST inc., all rights reserved
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

"""Execute the production H753 Ethernet driver using real CMSIS addresses."""
import os
from pathlib import Path
import struct
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_PC

ROOT = Path(__file__).resolve().parents[2]


def build(out, opt):
    flags = ['-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx', '-std=gnu++23', opt,
             '-g', '-ffunction-sections', '-fdata-sections', '-fno-exceptions', '-fno-rtti']
    flags += ['-I' + str(ROOT / p) for p in ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
              'ARM/ST/STM32H7xx/include')]
    objects = []
    for src in ('ARM/ST/STM32H7xx/src/ethernet_stm32h7.cpp', 'src/device_intrf.cpp',
                'tests/stm32h7/ethernet_fw.cpp'):
        obj = out / (Path(src).stem + '.o')
        subprocess.run(['arm-none-eabi-g++', *flags, '-Wall', '-Werror', '-c', str(ROOT / src),
                        '-o', str(obj)], check=True)
        objects.append(str(obj))
    script = out / 'test.ld'
    script.write_text('MEMORY { FLASH (rx) : ORIGIN = 0x08000000, LENGTH = 2M\n'
                      'RAM (rwx) : ORIGIN = 0x20000000, LENGTH = 128K\n'
                      'RAM2 (rwx) : ORIGIN = 0x24000000, LENGTH = 512K }\n'
                      'SECTIONS { .text : { *(.text*) *(.rodata*) } > FLASH\n'
                      '.data : { *(.data*) } > RAM\n .bss : { *(.bss*) *(COMMON) } > RAM\n'
                      '.eth_dma (NOLOAD) : ALIGN(32) { *(.eth_dma) } > RAM2\n'
                      '/DISCARD/ : { *(.ARM.exidx*) } }\n')
    elf = out / 'test.elf'
    subprocess.run(['arm-none-eabi-g++', '-mcpu=cortex-m7', '-mthumb', '-nostartfiles',
                    '-specs=nano.specs', '-specs=nosys.specs', '-T' + str(script),
                    '-Wl,--gc-sections,-e,RunEthernetTest,-u,g_EthRegs', *objects, '-o', str(elf)], check=True)
    return elf


class Model:
    def __init__(self, path, cache=False, fault=None):
        self.u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for addr, size in ((0x08000000, 0x200000), (0x20000000, 0x20000),
                           (0x24000000, 0x80000), (0x40000000, 0x100000),
                           (0x58000000, 0x100000), (0xe0000000, 0x100000)):
            self.u.mem_map(addr, size)
        with path.open('rb') as f:
            e = ELFFile(f)
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    self.u.mem_write(seg['p_vaddr'], seg.data())
            self.syms = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
        self.regs = struct.unpack('<14I', self.u.mem_read(self.syms['g_EthRegs'], 56))
        self.values = {}
        self.phy = {0: 0x1000, 1: 0x24, 2: 7, 3: 0xC130, 31: 6 << 2}
        self.fault = fault
        self.cleans = self.invalidates = 0
        self.write32(0xe000ed14, (1 << 16) if cache else 0)
        self.u.hook_add(UC_HOOK_MEM_WRITE, self.write)
        self.u.hook_add(UC_HOOK_MEM_READ, self.read)

    def read32(self, addr):
        return struct.unpack('<I', self.u.mem_read(addr, 4))[0]

    def write32(self, addr, value):
        self.u.mem_write(addr, struct.pack('<I', value))

    def read(self, uc, access, addr, size, value, ctx):
        if addr in self.values:
            uc.mem_write(addr, self.values[addr].to_bytes(size, 'little'))

    def write(self, uc, access, addr, size, value, ctx):
        reset, mdio, data, status, mtl, *_ = self.regs
        if addr == reset:
            self.values[addr] = value if self.fault == 'reset' else value & ~1
        elif addr == mdio:
            self.values[addr] = value if self.fault == 'mdio' else value & ~1
            reg = (value >> 16) & 31
            if value & 0xC == 0xC:
                v = self.phy.get(reg, 0)
                if self.fault == 'identity' and reg == 3:
                    v = 0
                self.values[data] = v
            elif value & 0xC == 4:
                v = self.read32(data)
                self.phy[reg] = v if self.fault == 'phy-reset' else v & ~0x8000
        elif addr == status:
            self.values[addr] = self.values.get(addr, 0) & ~value
        elif addr == mtl:
            self.values[addr] = value & ~1
        elif addr in self.regs[12:]:
            assert value & 31 == 0, 'unaligned cache line'
            assert 0x24000000 <= value < 0x24080000, 'DMA cache operation outside AXI SRAM'
            if addr == self.regs[12]:
                self.cleans += 1
            else:
                self.invalidates += 1

    def run(self, case):
        self.u.reg_write(UC_ARM_REG_SP, 0x2001f000)
        self.u.reg_write(UC_ARM_REG_LR, 0x080ff001)
        self.u.reg_write(UC_ARM_REG_R0, case)
        self.u.emu_start(self.syms['RunEthernetTest'] | 1, 0x080ff000, count=50000000)
        assert self.u.reg_read(UC_ARM_REG_PC) == 0x080ff000, 'driver did not return'
        assert self.u.reg_read(UC_ARM_REG_R0) == 0, 'firmware check line ' + str(self.u.reg_read(UC_ARM_REG_R0))

    def rx(self, index, valid=True):
        base = self.read32(self.regs[6])
        desc = base + index * 32
        buf = self.read32(desc)
        self.u.mem_write(buf, bytes(i ^ 0x5A for i in range(64)))
        # DMA write-back overwrites address words. The driver must restore them.
        self.write32(desc, 0xDEADBEEF)
        self.write32(desc + 12, (0x30000000 if valid else 0x20008000) | 64)
        return desc, buf


def main():
    with tempfile.TemporaryDirectory(prefix='h7-ethernet-') as td:
        for opt in ('-O0', '-Os'):
            elf = build(Path(td), opt)
            for cache in (False, True):
                m = Model(elf, cache)
                m.run(0)
                m.run(10)
                tx = m.read32(m.regs[5])
                rx = m.read32(m.regs[6])
                assert tx % 32 == 0 and rx % 32 == 0
                assert 0x24000000 <= tx < 0x24080000 and 0x24000000 <= rx < 0x24080000
                assert m.read32(m.regs[7]) >> 18 & 7 == 4
                assert m.read32(m.regs[8]) >> 1 & 0x3fff == 1536
                m.run(1)
                for i in range(4):
                    assert m.read32(tx + i * 32 + 12) & 0x80000000
                m.write32(tx + 12, 0)
                m.run(2)
                desc, buf = m.rx(0)
                m.run(3)
                assert m.read32(desc) == buf and m.read32(desc + 12) & 0x80000000
                m.rx(1, valid=False)
                m.rx(2)
                m.run(3)
                m.rx(3)
                m.run(5)
                m.run(6)
                m.run(8)
                m.run(9)
                m.values[m.regs[3]] = 0x41
                m.run(11)
                m.phy[1] = 0
                m.run(12)
                m.phy[1] = 0x24
                m.phy[31] = 1 << 2
                m.run(13)
                m.phy[31] = 6 << 2
                m.run(10)
                m.run(7)
                m.run(14)
                if cache:
                    assert m.cleans > 0 and m.invalidates > 0
                print(opt + ' cache=' + str(cache) + ': descriptor/PHY/lifecycle PASS', flush=True)
            for fault in ('reset', 'mdio', 'identity', 'phy-reset'):
                m = Model(elf, fault=fault)
                m.run(100)
                print(opt + ' ' + fault + ': bounded initialization failure PASS', flush=True)
    print('H753 Ethernet register model: PASS; hardware not tested.')


if __name__ == '__main__':
    main()
