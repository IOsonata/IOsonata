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
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
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
        subprocess.run(cmd + [str(ROOT / 'ARM/ST/src/dma_stm32.cpp'), '-Wl,--section-start=.bus_dma=0x38000000', '-lc', '-lgcc', '-o', str(elfpath)], check=True)
        u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for a, n in [(0x08000000, 0x10000), (0x20000000, 0x20000),
                     (0x24000000, 0x80000), (0x38000000, 0x10000), (0x58025000, 0x1000), (0x40020000, 0x1000),
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
        for ch in range(16, 24):
            reg = 0x58025408 + (ch-16)*20
            assert C('Claim', ch, 0, ch+1, 6, 1) == 1
            assert C('Claim', ch, 0, 99, 6, 1) == 0
            for w in range(3):
                for tx in (0,1):
                    C('Setup', ch, 13, tx, 0x58001c24, w, 1)
                    assert rd(0x58025800+(ch-16)*4) == 13
                    assert (rd(reg)>>8)&3 == w and (rd(reg)>>10)&3 == w
                    assert bool(rd(reg)&16) == bool(tx)
                    C('Start',ch,0x38000000,32<<w)
                    assert rd(reg)&1 and rd(reg+4)==32
                    wr(reg+4,7)
                    assert C('StopRemain',ch)==7<<w
                    assert not rd(reg)&1
                    for bad in (0x20000000,0x24000000):
                        C('Start',ch,bad,32<<w)
                        assert C('Flags',ch)&2 and not rd(reg)&1
            C('Free',ch)
        assert C('Claim', 24, 0, 0, 0, 0) == 0
        # W1C behavior for buffered completion/reload tests.
        def clear_flags(uc, access, a, size, value, ctx):
            for base in (0x40020000,0x40020400):
                if a in (base+8,base+12): wr(a-8,rd(a-8)&~value)
            if a==0x58025404: wr(0x58025400,rd(0x58025400)&~value)
        u.hook_add(UC_HOOK_MEM_WRITE, clear_flags)
        # Execute cache-maintenance branches as well. This checks instruction
        # paths and buffer bounds, not physical cache coherency.
        wr(0xe000ed14, 1 << 16)
        # Unaligned DTCM TX/RX, reload, partial abort, fixed-address dummy/sink.
        for ch in (0,7,8,15,16,23):
            reg=0x58025408+(ch-16)*20 if ch>=16 else 0x40020010+(ch//8)*0x400+(ch%8)*24
            status=0x58025400 if ch>=16 else 0x40020000+(ch//8)*0x400+(4 if ch&4 else 0)
            shift=(ch-16)*4 if ch>=16 else (0,6,16,22)[ch%4]
            tc=(2 if ch>=16 else 32)<<shift
            assert C('Claim',ch,0,101,6,1)==1
            for tx in (0,1):
                C('Setup',ch,13 if ch>=16 else 33,tx,0x58001c24 if ch>=16 else 0x40005424,0,1)
                source=bytes((k*7)&255 for k in range(1200))
                u.mem_write(0x20010000,b'X'+source+b'Y')
                C('BufferStart',ch,0x20010001,1200)
                stage=rd(reg+12)
                assert 0x38000000<=stage<0x38010000
                assert rd(reg+4)==512
                if tx: assert bytes(u.mem_read(stage,512))==source[:512]
                else: u.mem_write(stage,b'A'*512)
                wr(reg+4,0); wr(reg,rd(reg)&~1); wr(status,tc)
                assert C('BufferFlags',ch)==0
                assert C('BufferRemain',ch)==688
                if tx: assert bytes(u.mem_read(stage,512))==source[512:1024]
                else: u.mem_write(stage,b'B'*100)
                wr(reg+4,412)
                assert C('BufferStopRemain',ch)==588
                if not tx:
                    assert bytes(u.mem_read(0x20010001,612))==b'A'*512+b'B'*100
                    assert bytes(u.mem_read(0x20010265,588))==source[612:]
                assert bytes(u.mem_read(0x20010000,1))==b'X'
                assert bytes(u.mem_read(0x200104b1,1))==b'Y'
                C('MemInc',ch,0)
                C('BufferStart',ch,0x20010001,32)
                assert not rd(reg)&(128 if ch>=16 else 1024)
                C('BufferStopRemain',ch)
                C('MemInc',ch,1)
                C('BufferStart',ch,0x20010001,31)
                wr(reg+4,1000) # Impossible/stale count must never underflow a copy.
                assert C('BufferStopRemain',ch)==31
                assert C('BufferFlags',ch)&2
                assert bytes(u.mem_read(0x20010000,1))==b'X'
                assert bytes(u.mem_read(0x200104b1,1))==b'Y'
            C('Free',ch)
        # Allocation never steals a UART/bus owner's channel, including BDMA.
        for ch in range(24): assert C('Claim',ch,0,202,6,0)==1
        assert C('Alloc',0,0,303,6,0)==0xffffffff
        assert C('Alloc',1,0,303,6,0)==0xffffffff
        for ch in (5,21): C('Free',ch)
        assert C('Alloc',0,0,303,6,0)==5
        assert C('Alloc',1,0,303,6,0)==21
        for ch in range(24): C('Free',ch)
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
        print('PASS: H753 DMA Arm instructions, 16 streams + 8 BDMA channels, routing, widths, staging/reload/partial stop, guards, ownership and disable timeout')

if __name__ == '__main__': main()
