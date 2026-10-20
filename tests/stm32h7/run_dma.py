#!/usr/bin/env python3
"""Run the H753 DMA manager's Arm instructions against modeled registers.
Requires GNU Arm GCC, unicorn and pyelftools. No hardware throughput claim.
"""
import os
from pathlib import Path
import shlex
import struct
import subprocess
import tempfile
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

ROOT = Path(__file__).resolve().parents[2]

def main():
    with tempfile.TemporaryDirectory() as tmp:
        elfpath = Path(tmp) / 'dma.elf'
        inc = ['ARM/ST/include', 'ARM/ST/STM32H7xx/include', 'ARM/CMSIS/Core/Include']
        cmd = ['arm-none-eabi-g++', '-mcpu=cortex-m7', '-mthumb', '-DSTM32H753xx',
               '-std=gnu++17', '-O2', '-g', '-nostdlib', '-fno-exceptions', '-fno-rtti',
               '-Wl,-Ttext=0x08000000,-Tdata=0x20000000,-e,Stm32DmaChClaim']
        cmd += ['-I' + str(ROOT / p) for p in inc]
        cmd += shlex.split(os.environ.get('ARM_CPPFLAGS', ''))
        subprocess.run(cmd + [str(ROOT / 'ARM/ST/src/dma_stm32.cpp'), '-o', str(elfpath)], check=True)
        u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for a, n in [(0x08000000, 0x10000), (0x20000000, 0x20000),
                     (0x24000000, 0x80000), (0x40020000, 0x1000),
                     (0x58024000, 0x1000), (0xe000e000, 0x2000)]:
            u.mem_map(a, n)
        with elfpath.open('rb') as f:
            elf = ELFFile(f)
            sym = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}
            for seg in elf.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    u.mem_write(seg['p_vaddr'], seg.data())
        def rd(a): return struct.unpack('<I', u.mem_read(a, 4))[0]
        def wr(a, v): u.mem_write(a, struct.pack('<I', v))
        def call(name, *args):
            sp = 0x2001ff00
            for i, value in enumerate(args):
                if i < 4: u.reg_write(UC_ARM_REG_R0 + i, value)
                else: wr(sp + 4 * (i - 4), value)
            u.reg_write(UC_ARM_REG_SP, sp)
            u.reg_write(UC_ARM_REG_LR, 0x0800f001)
            u.emu_start(sym[name] | 1, 0x0800f000, count=4000000)
            assert u.reg_read(UC_ARM_REG_PC) == 0x0800f000, name + ' did not return'
            return u.reg_read(UC_ARM_REG_R0)
        C = lambda name, *a: call('Stm32DmaCh' + name, *a)
        stuck_addr = [None]
        def stuck(uc, access, addr, size, val, data):
            if addr == stuck_addr[0]: wr(addr, rd(addr) | 1)
        u.hook_add(UC_HOOK_MEM_READ, stuck, begin=0x40020000, end=0x400207ff)
        # Every stream, direction and width, both low/high flag registers.
        for ch in range(16):
            ctrl = 0x40020000 + (ch // 8) * 0x400
            stream = ctrl + 0x10 + (ch % 8) * 0x18
            shift = [0, 6, 16, 22][ch % 4]
            status = ctrl + (4 if ch & 4 else 0)
            clear = ctrl + (12 if ch & 4 else 8)
            assert C('Claim', ch, 0, ch + 1, 6, 1) == 1
            assert C('Claim', ch, 0, 99, 6, 1) == 0
            assert rd(0x580244d8) & (1 << (ch // 8)) # RCC AHB1ENR
            for width in range(3):
                for tx in (0, 1):
                    C('Setup', ch, 45, tx, 0x40004828, width, 1)
                    assert rd(0x40020800 + ch * 4) == 45
                    cr = rd(stream)
                    assert (cr >> 6) & 3 == tx
                    assert (cr >> 11) & 3 == width and (cr >> 13) & 3 == width
                    assert cr & 0x16 == 0x16 # TCIE, TEIE, DMEIE
                    assert rd(stream + 20) == 0 # direct mode
                    C('Start', ch, 0x24000000, 32 << width)
                    assert rd(stream) & 1 and rd(stream + 4) == 32
                    assert rd(stream + 12) == 0x24000000
                    wr(stream + 4, 7)
                    assert C('Remain', ch) == 7 << width
                    assert C('StopRemain', ch) == 7 << width
                    assert rd(stream) & 1 == 0
                    assert rd(clear) == 0x3d << shift
                    for flag, expected in [(0x20, 1), (1, 2), (4, 2), (8, 2)]:
                        wr(status, flag << shift)
                        assert C('Flags', ch) == expected
                        C('Ack', ch)
                        assert rd(clear) == 0x3d << shift
                        wr(status, 0)
                        assert C('Flags', ch) == expected
                        C('Clear', ch)
                        assert C('Flags', ch) == 0
            C('MemInc', ch, 0)
            assert rd(stream) & (1 << 10) == 0
            C('MemInc', ch, 1)
            # Invalid requests never enable a transfer.
            for addr, count in [(0x20000000, 32), (0x2001fff0, 32),
                                (0x24000001, 32), (0x24000000, 3),
                                (0x24000000, 0), (0x24000000, 65536 * 4)]:
                C('Start', ch, addr, count)
                assert rd(stream) & 1 == 0 and C('Flags', ch) & 2
            C('Setup', ch, 128, 1, 0x40004828, 0, 1)
            C('Start', ch, 0x24000000, 32)
            assert rd(stream) & 1 == 0 and C('Flags', ch) & 2
            C('Setup', ch, 45, 1, 0x40004828, 0, 0)
            assert rd(stream) & 0x16 == 0
            # Vector with no callback must clear only its own flag group.
            wr(status, 0x20 << shift)
            call('DMA%d_Stream%d_IRQHandler' % (1 + ch // 8, ch % 8))
            assert rd(clear) == 0x3d << shift
            wr(status, 0)
            C('Free', ch)
            assert rd(stream) == 0 and rd(0x40020800 + ch * 4) == 0
            assert C('Claim', ch, 0, 99, 6, 0) == 1
            C('Free', ch)
        assert C('Claim', 16, 0, 0, 0, 0) == 0
        # Hardware that will not acknowledge EN=0 must retain ownership and
        # must not have its addresses/count overwritten by Setup or Start.
        ch, cr = 0, 0x40020010
        assert C('Claim', ch, 0, 1, 6, 0) == 1
        C('Setup', ch, 45, 1, 0x40004828, 0, 0)
        C('Start', ch, 0x24000000, 32)
        stuck_addr[0] = cr
        C('Start', ch, 0x24000100, 64)
        assert rd(cr + 12) == 0x24000000 and rd(cr + 4) == 32, (hex(rd(cr+12)), rd(cr+4), hex(rd(cr)))
        assert C('Flags', ch) & 2
        C('Free', ch)
        assert C('Claim', ch, 0, 2, 6, 0) == 0
        stuck_addr[0] = None
        C('Free', ch)
        assert C('Claim', ch, 0, 2, 6, 0) == 1
        print('PASS: H753 DMA Arm instructions, 16 streams, routing, widths, flags, vectors, ownership and disable timeout')

if __name__ == '__main__': main()
