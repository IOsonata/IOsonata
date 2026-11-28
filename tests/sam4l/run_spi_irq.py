#!/usr/bin/env python3
"""Build and run the SAM4L SPI master transfer check.

The production spi_sam4l.cpp and device_intrf.cpp are built for the SAM4L and
run in an Arm emulator (unicorn) with a SPI and PDCA register model. The model
does nothing by itself: a transfer is synchronous and waits, and each time the
firmware has run for a while without returning, the model takes its next
steps (a PDCA beat, else the SPI shifting the word written to TDR), until an
interrupt is pending or a step sets an error flag, then runs the handler of an
NVIC enabled line with an enabled status flag set, when PRIMASK is clear, and
again while the line stays pending, as the NVIC does with a level interrupt.

Usage: run_spi_irq.py [--cxx arm-none-eabi-g++]
Requires an Arm GNU C++ compiler and the Python packages unicorn and
pyelftools. This is a register level check, not hardware validation: bus
timing is not modelled.
"""
import argparse
import subprocess
import tempfile
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import (UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
                               UC_ARM_REG_PRIMASK)

ROOT = Path(__file__).resolve().parents[2]
QUANTUM = 4000
BURST = 64
RET = 0xF0000

# SPI status
RDRF, TDRE, MODF, OVRES, TXEMPTY = 1 << 0, 1 << 1, 1 << 2, 1 << 3, 1 << 9
SR_READ_CLEAR = MODF | OVRES | (1 << 8) | (1 << 10)
# PDCA status
RCZ, TRC, TERR = 1 << 0, 1 << 1, 1 << 2


def build(cxx, opt, out):
    flags = [cxx, '-std=gnu++20', '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt, '-g',
             '-fno-inline-functions-called-once', '-fno-exceptions', '-fno-rtti',
             '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START', '-D__SAM4LS8C__',
             '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
             '-I' + str(ROOT / 'ARM/Microchip/SAM4L/include'), '-I' + str(ROOT / 'ARM/CMSIS/Core/Include')]
    sources = [ROOT / 'tests/sam4l/spi_irq_fixture.cpp', ROOT / 'src/device_intrf.cpp']
    args = flags + [str(s) for s in sources] + [
        '-nostartfiles', '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,init']
    args += ['-Wl,-u,' + s for s in ('isr', 'tx', 'rx', 'read', 'busy', 'events', 'layout')]
    subprocess.run(args + ['-o', str(out)], check=True)


class Page:
    """4 KB of peripheral address space: register models, the rest kept as written."""

    def __init__(self):
        self.mem = bytearray(0x1000)
        self.regs = []

    def add(self, offset, size, model):
        self.regs.append((offset, offset + size, model))

    def find(self, off):
        for lo, hi, m in self.regs:
            if lo <= off < hi:
                return m, off - lo
        return None, off

    def read(self, uc, off, size, data):
        m, o = self.find(off)
        if m is not None:
            return m.read(o, size) & ((1 << (8 * size)) - 1)
        return int.from_bytes(self.mem[off:off + size], 'little')

    def write(self, uc, off, size, value, data):
        m, o = self.find(off)
        if m is not None:
            m.write(o, size, value)
        else:
            self.mem[off:off + size] = (value & ((1 << (8 * size)) - 1)).to_bytes(size, 'little')


class Spi:
    """SPI master: a step shifts the word written to TDR and receives the next
    value of 0x40, 0x41 and so on (with 0x80, 0x81 and so on in the high byte
    of a 16-bit word). ovf_word ends that word with OVRES, its data lost. An
    SR read clears MODF, OVRES, NSSR and UNDES."""

    def __init__(self, L):
        self.CR, self.RDR, self.TDR, self.SR, self.IER, self.IDR, self.IMR, self.CSR = L
        self.reset()
        self.ovf_word = None

    def reset(self):
        self.sr, self.imr, self.rdr, self.csr0 = 0, 0, 0, 0
        self.on = False
        self.tdr = None
        self.sent, self.rx_n = [], 0

    def bits(self):
        return 8 + ((self.csr0 >> 4) & 0xF)

    def read(self, off, size):
        if off == self.SR:
            v = self.sr | ((1 << 16) if self.on else 0)
            self.sr &= ~SR_READ_CLEAR
            return v
        if off == self.RDR:
            self.sr &= ~RDRF
            return self.rdr
        if off == self.IMR:
            return self.imr
        if off == self.CSR:
            return self.csr0
        return 0

    def write(self, off, size, v):
        if off == self.CR:
            if v & (1 << 7):            # SWRST
                self.sr, self.imr, self.tdr, self.on = 0, 0, None, False
            if v & 2:                   # SPIDIS
                self.on = False
                self.sr &= ~(TDRE | TXEMPTY)
            if v & 1:                   # SPIEN
                self.on = True
                if self.tdr is None:
                    self.sr |= TDRE | TXEMPTY
        elif off == self.TDR:
            assert self.on and self.tdr is None, 'TDR written while not empty'
            self.tdr = v & ((1 << self.bits()) - 1)
            self.sr &= ~(TDRE | TXEMPTY)
        elif off == self.IER:
            self.imr |= v
        elif off == self.IDR:
            self.imr &= ~v
        elif off == self.CSR:
            self.csr0 = v

    def step(self):
        if self.tdr is None:
            return None
        self.sent.append(self.tdr)
        self.tdr = None
        n = self.rx_n
        self.rx_n += 1
        v = ((0x40 + n) & 0xFF) | (((0x80 + n) & 0xFF) << 8)
        if len(self.sent) - 1 == self.ovf_word or self.sr & RDRF:
            self.sr |= OVRES
        else:
            self.rdr = v & ((1 << self.bits()) - 1)
            self.sr |= RDRF
        self.sr |= TDRE | TXEMPTY
        return 'spi'

    def irq(self):
        return bool(self.sr & self.imr)


class Pdca:
    """One PDCA channel: a beat per step when its SPI flag is set, the memory
    address and count updated, TRC when the count reaches 0 (no reload)."""

    def __init__(self, c, L, rx):
        self.c = c
        self.MAR, self.TCR, self.CR, self.MR, self.IER, self.IDR, self.IMR, self.ISR = L
        self.rx = rx
        self.mar = self.tcr = self.mr = self.imr = 0
        self.isr = TRC | RCZ
        self.on = False
        self.beats = 0

    def read(self, off, size):
        if off == self.MAR:
            return self.mar
        if off == self.TCR:
            return self.tcr
        if off == self.MR:
            return self.mr
        if off == self.IMR:
            return self.imr
        if off == self.ISR:
            return self.isr
        return 0

    def write(self, off, size, v):
        if off == self.MAR:
            self.mar = v
        elif off == self.TCR:
            self.tcr = v & 0xFFFF
            self.isr = (self.isr & ~TRC) | (TRC if self.tcr == 0 else 0)
        elif off == self.MR:
            self.mr = v
        elif off == self.CR:
            if v & 2:
                self.on = False
            if v & 1:
                self.on = True
            if v & (1 << 8):
                self.isr &= ~TERR
        elif off == self.IER:
            self.imr |= v
        elif off == self.IDR:
            self.imr &= ~v

    def step(self):
        spi = self.c.spi
        if not self.on or self.tcr == 0:
            return None
        size = 1 << (self.mr & 3)
        if self.rx:
            if not spi.sr & RDRF:
                return None
            v = spi.read(spi.RDR, 4)
            self.c.u.mem_write(self.mar, (v & ((1 << (8 * size)) - 1)).to_bytes(size, 'little'))
        else:
            if not spi.sr & TDRE:
                return None
            v = int.from_bytes(self.c.u.mem_read(self.mar, size), 'little')
            spi.write(spi.TDR, 4, v)
        self.mar += size
        self.tcr -= 1
        self.beats += 1
        if self.tcr == 0:
            self.isr |= TRC
        return 'beat'

    def irq(self):
        return bool(self.isr & self.imr)


class Check:
    def __init__(self, elf):
        u = self.u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0x40000000, 0x8000),
                           (0x40009000, 0x99000), (0x400A3000, 0x5D000),
                           (0xE0000000, 0xE000), (0xE000F000, 0xF1000)]:
            u.mem_map(base, size)
        with open(elf, 'rb') as f:
            e = ELFFile(f)
            self.sym = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    u.mem_write(seg['p_vaddr'], seg.data())
        L = [self.call('layout', i) & 0xFFFFFFFF for i in range(23)]
        self.BUF = L[1]
        assert L[0] == 0x40008000, hex(L[0])
        self.SPI_IRQ, self.RX_IRQ, self.TX_IRQ = L[20:23]
        self.spi = Spi(L[2:10])
        self.prx, self.ptx = Pdca(self, L[12:20], True), Pdca(self, L[12:20], False)
        self.pages = {}
        for base in (0x40008000, 0x400A2000, 0xE000E000):
            p = self.pages[base] = Page()
            u.mmio_map(base, 0x1000, p.read, None, p.write, None)
        self.pages[0x40008000].add(0, 0x100, self.spi)
        self.pages[0x400A2000].add(L[10] - 0x400A2000, 0x40, self.prx)
        self.pages[0x400A2000].add(L[11] - 0x400A2000, 0x40, self.ptx)
        self.pages[0xE000E000].add(0x100, 0x100, self)
        self.iser = 0
        self.blocked = set()

    # ---- NVIC enable and clear registers
    def read(self, off, size):
        if off < 0x80:
            return (self.iser >> (8 * off)) & 0xFFFFFFFF
        return 0

    def write(self, off, size, v):
        if off < 0x80:
            self.iser |= v << (8 * off)
        elif off < 0x100:
            self.iser &= ~(v << (8 * (off - 0x80)))

    # ---- emulator
    def setup(self, args, sp):
        for i, a in enumerate(args):
            self.u.reg_write(UC_ARM_REG_R0 + i, a)
        self.u.reg_write(UC_ARM_REG_SP, sp)
        self.u.reg_write(UC_ARM_REG_LR, RET | 1)

    def call(self, name, *args, sp=0x200FF000):
        self.setup(args, sp)
        self.u.emu_start(self.sym[name] | 1, RET, count=50000000)
        assert self.u.reg_read(UC_ARM_REG_PC) == RET, (name, 'did not return')
        r = self.u.reg_read(UC_ARM_REG_R0)
        return r - (1 << 32) if r & 0x80000000 else r

    def irq_pending(self):
        """IRQ line to take, None when none is pending or PRIMASK is set.
        The lines in blocked are held off."""
        if self.u.reg_read(UC_ARM_REG_PRIMASK) & 1:
            return None
        for irq, m in ((self.RX_IRQ, self.prx), (self.TX_IRQ, self.ptx), (self.SPI_IRQ, self.spi)):
            if irq not in self.blocked and self.iser & (1 << irq) and m.irq():
                return irq
        return None

    def step(self):
        for m in (self.prx, self.ptx, self.spi):
            r = m.step()
            if r is not None:
                return r
        return None

    def burst(self):
        """Steps until nothing is left, an interrupt is pending or a step
        sets OVRES. Returns the steps taken."""
        n = 0
        while n < BURST and self.irq_pending() is None:
            ovf = self.spi.sr & OVRES
            if self.step() is None:
                break
            n += 1
            if self.spi.sr & OVRES != ovf:
                break
        return n

    def run(self, name, *args, idle=False):
        """Call name and serve it each time it waits. Returns what it
        returned, the interrupts run and the waits with nothing to serve."""
        u = self.u
        self.setup(args, 0x200FF000)
        pc = self.sym[name] | 1
        steps = irqs = waits = 0
        while True:
            u.emu_start(pc, RET, count=QUANTUM)
            stopped = u.reg_read(UC_ARM_REG_PC)
            if stopped == RET:
                break
            ctx = u.context_save()
            if self.irq_pending() is None:
                n = self.burst()
                if n == 0:
                    assert idle, (name, args, 'waits with nothing to serve', steps, irqs)
                    waits += 1
                steps += n
            irq = self.irq_pending()
            entries = 0
            while irq is not None:
                self.call('isr', irq, sp=0x200F0000)
                irqs += 1
                entries += 1
                assert entries < 50, (name, args, 'IRQ', irq, 'pending after its handler')
                irq = self.irq_pending()
            u.context_restore(ctx)
            pc = stopped | 1
            assert steps + irqs + waits < 20000, (name, args, 'does not end')
        r = u.reg_read(UC_ARM_REG_R0)
        return (r - (1 << 32) if r & 0x80000000 else r), irqs, waits

    def init(self, dma, irq, wide=False):
        self.spi.ovf_word = None
        assert self.call('init', dma, irq, wide) == 1
        self.spi.sent, self.spi.rx_n = [], 0

    def data(self, n):
        return list(self.u.mem_read(self.BUF, n))

    def fill(self, values):
        self.u.mem_write(self.BUF, bytes(values))


def rx_bytes(n, wide=False, first=0):
    out = []
    for k in range(first, first + (n // 2 if wide else n)):
        out.append((0x40 + k) & 0xFF)
        if wide:
            out.append((0x80 + k) & 0xFF)
    return out


def check_mode(c, dma, irq):
    mode = 'dma=%d irq=%d' % (dma, irq)
    s = c.spi

    def done(what, irqs, waits):
        assert (irqs > 0) == bool(irq) and waits == 0, (mode, what, 'interrupts', irqs, waits)
        assert c.call('busy') == 0, (mode, what, 'busy')
        # The handlers mask the interrupts they took
        assert s.imr == 0 and c.prx.imr == 0 and c.ptx.imr == 0, (mode, what, 'interrupt left enabled', hex(s.imr))

    c.init(dma, irq)
    for n in (1, 3, 64, 65, 300):
        s.sent, s.rx_n = [], 0
        data = [(i * 5 + 3) & 0xFF for i in range(n)]
        c.fill(data)
        r, irqs, waits = c.run('tx', n)
        assert r == n and s.sent == data, (mode, 'tx', n, r)
        done('tx %d' % n, irqs, waits)
    for n in (1, 2, 64, 200):
        s.sent, s.rx_n = [], 0
        c.fill([0] * n)
        r, irqs, waits = c.run('rx', n)
        assert r == n and c.data(n) == rx_bytes(n) and s.sent == [0xFF] * n, (mode, 'rx', n, r)
        done('rx %d' % n, irqs, waits)
    # Command then read with the chip select kept
    s.sent, s.rx_n = [], 0
    r, irqs, waits = c.run('read', 4)
    assert r == 4 and s.sent == [0x9F, 0x03] + [0xFF] * 4 and c.data(4) == rx_bytes(4, first=2), (mode, 'read', r)
    done('read', irqs, waits)
    if dma:
        # Receive overflow on byte 70, in the second PDCA chunk: the bytes
        # received, the wait ends on the error
        s.sent, s.rx_n, s.ovf_word = [], 0, 70
        r, irqs, waits = c.run('rx', 100)
        s.ovf_word = None
        assert r == 70 and waits == 0 and c.call('busy') == 0, (mode, 'overflow', r, waits)
    if irq:
        # A flag whose interrupt is held off does not end the wait. With PDCA
        # the channels move the words and the wait ends on its time limit.
        s.sent, s.rx_n = [], 0
        c.blocked = {c.SPI_IRQ, c.RX_IRQ, c.TX_IRQ}
        c.fill([1, 2, 3, 4])
        r, irqs, waits = c.run('tx', 4, idle=True)
        c.blocked = set()
        assert r == (4 if dma else 0) and irqs == 0 and waits > 0, (mode, 'held off', r, irqs, waits)
        c.init(dma, irq)
        c.fill([5, 6])
        r, irqs, waits = c.run('tx', 2)
        assert r == 2 and s.sent == [5, 6], (mode, 'after the time limit', r, s.sent)
    # 16-bit words: little endian in the buffer
    c.init(dma, irq, True)
    c.fill([0x34, 0x12, 0x78, 0x56, 0xBC, 0x9A])
    r, irqs, waits = c.run('tx', 6)
    assert r == 6 and s.sent == [0x1234, 0x5678, 0x9ABC], (mode, 'tx 16-bit', r, [hex(x) for x in s.sent])
    s.sent, s.rx_n = [], 0
    c.fill([0] * 6)
    r, irqs, waits = c.run('rx', 6)
    assert r == 6 and c.data(6) == rx_bytes(6, True) and s.sent == [0xFFFF] * 3, (mode, 'rx 16-bit', r, c.data(6))
    done('16-bit', irqs, waits)
    assert c.call('events') == 0, (mode, 'master reported an event')


def main(elf, label):
    for dma in (0, 1):
        for irq in (0, 1):
            check_mode(Check(elf), dma, irq)
    print(label, 'SAM4L SPI master transfers: PASS')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--cxx', default='arm-none-eabi-g++')
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        for opt in ('-O1', '-Os'):
            elf = Path(tmp) / 'spi_irq.elf'
            build(a.cxx, opt, elf)
            main(elf, opt)
