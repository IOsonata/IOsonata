#!/usr/bin/env python3
"""Build and run the SAM D21 SERCOM I2C and SPI master transfer check.

The production i2c_samd.cpp, spi_samd.cpp, dmac_samd.c and device_intrf.cpp
are built for the SAM D21 and run in an Arm emulator (unicorn) with a SERCOM
I2C master, SERCOM SPI master and DMAC register model. The model does nothing
by itself: a transfer is synchronous and waits, and each time the firmware has
run for a while without returning, the model takes its next steps (a DMAC beat
on a set trigger, else a SERCOM step), up to BURST of them, until an
interrupt is pending or a step sets an error flag, then runs the interrupt
handler of an NVIC enabled line with an enabled flag set, when PRIMASK is
clear. The hardware is faster than the firmware loop, as at the real clock
rates, so that the time limits of the waits hold.

Usage: run_master_irq.py [--cxx arm-none-eabi-g++]
Requires an Arm GNU C/C++ compiler and the Python packages unicorn and
pyelftools. This is a register level check, not hardware validation: bus
timing is not modelled.
"""
import argparse
import collections
import struct
import subprocess
import tempfile
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import (UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0,
                               UC_ARM_REG_PRIMASK, UC_CPU_ARM_CORTEX_M0)

ROOT = Path(__file__).resolve().parents[2]
QUANTUM = 4000
BURST = 64
RET = 0xF0000

# SERCOM flags and status
MB, SB, I2C_ERROR = 0x01, 0x02, 0x80
DRE, TXC, RXC, SPI_ERROR = 0x01, 0x02, 0x04, 0x80
BUSERR, RXNACK, BUFOVF = 0x01, 0x04, 0x04
SMEN, ACKACT = 1 << 8, 1 << 18
# IRQ lines of the SAM D21
DMAC_IRQ, SPI_IRQ, I2C_IRQ = 6, 9, 11


def build(cxx, opt, out):
    gcc = cxx[:-3] + 'gcc' if cxx.endswith('g++') else cxx
    common = ['-mcpu=cortex-m0plus', '-mthumb', opt, '-g', '-fno-inline-functions-called-once',
              '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START', '-D__SAMD21J18A__',
              '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
              '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/Microchip/SAMD/include'),
              '-I' + str(ROOT / 'ARM/Microchip/SAMD/SAMD21/include')]
    cpp = ['tests/samd/master_irq_fixture.cpp', 'ARM/Microchip/SAMD/src/i2c_samd.cpp',
           'ARM/Microchip/SAMD/src/spi_samd.cpp', 'ARM/Microchip/SAMD/src/shared_intrf_samd.cpp',
           'src/coredev/shared_intrf.cpp', 'src/device_intrf.cpp']
    c = ['ARM/Microchip/SAMD/src/dmac_samd.c', 'ARM/src/iatomic.c']
    objs = []
    for i, src in enumerate(cpp + c):
        obj = out.parent / ('%d.o' % i)
        tool = [cxx, '-std=gnu++20', '-fno-exceptions', '-fno-rtti'] if src in cpp else [gcc, '-std=gnu17']
        subprocess.run(tool + common + ['-c', str(ROOT / src), '-o', str(obj)], check=True)
        objs.append(str(obj))
    exports = ('isr', 'i2c_tx', 'i2c_rx', 'i2c_read_reg', 'i2c_write_data', 'i2c_busy', 'spi_init',
               'spi_tx', 'spi_rx', 'spi_read', 'spi_busy', 'events', 'layout')
    args = [cxx, '-mcpu=cortex-m0plus', '-mthumb', '-nostartfiles'] + objs + [
        '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,i2c_init']
    args += ['-Wl,-u,' + s for s in exports]
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


class I2cm:
    """SERCOM in I2C master mode, smart mode, 7-bit addresses. A step does
    the next bus action: the address, a byte sent, a byte received or the
    STOP. A slave answers every address; nack_addr refuses the next address,
    nack_byte the byte of that index, buserr_byte ends that byte with a bus
    error (ERROR set, MB not set)."""

    def __init__(self):
        self.reset()
        self.nack_addr = False
        self.nack_byte = None
        self.buserr_byte = None

    def reset(self):
        self.ctrla = self.ctrlb = self.inten = self.flag = self.status = 0
        self.bus = 0
        self.data = 0
        self.reading = False
        self.todo = collections.deque()
        self.sent, self.addrs = [], []
        self.rx_n = 0

    def read(self, off, size):
        if off == 0x00:
            return self.ctrla
        if off == 0x04:
            return self.ctrlb
        if off in (0x14, 0x16):
            return self.inten
        if off == 0x18:
            return self.flag
        if off == 0x1A:
            return self.status | (self.bus << 4)
        if off == 0x1C:
            return 0
        if off == 0x28:
            d = self.data
            if self.reading and self.flag & SB and self.ctrlb & SMEN:
                self.flag &= ~SB
                if not self.ctrlb & ACKACT:
                    self.todo.append(self.byte_in)
            return d
        return 0

    def write(self, off, size, v):
        if off == 0x00:
            if v & 1:
                self.reset()
                return
            if v & 2 and not self.ctrla & 2:
                self.bus = 0
            self.ctrla = v
        elif off == 0x04:
            self.ctrlb = v & ~(3 << 16)
            if (v >> 16) & 3 == 3:
                assert self.bus == 2, 'STOP without the bus'
                self.flag &= ~(MB | SB)
                self.reading = False
                self.todo.append(self.stop)
        elif off == 0x14:
            self.inten &= ~v
        elif off == 0x16:
            self.inten |= v
        elif off == 0x18:
            self.flag &= ~(v & (MB | SB | I2C_ERROR))
        elif off == 0x1A:
            self.status &= ~(v & 0x7C7)
            if (v >> 4) & 3 == 1:
                self.bus = 1
        elif off == 0x24:
            assert self.bus in (1, 2), ('address with bus state', self.bus)
            self.flag &= ~(MB | SB)
            self.status &= ~RXNACK
            self.addrs.append(v & 0xFF)
            self.todo.append(lambda: self.addr_done(v & 1))
        elif off == 0x28:
            assert self.bus == 2 and not self.reading, 'data write without the bus'
            self.flag &= ~MB
            self.sent.append(v & 0xFF)
            k = len(self.sent) - 1
            self.todo.append(lambda: self.byte_out(k))

    def addr_done(self, rd):
        self.bus = 2
        if self.nack_addr:
            self.nack_addr = False
            self.status |= RXNACK
            self.flag |= MB
        elif rd:
            self.reading = True
            self.byte_in()
        else:
            self.flag |= MB
        return 'address'

    def byte_out(self, k):
        if k == self.buserr_byte:
            self.status |= BUSERR
            self.flag |= I2C_ERROR
            return 'bus error'
        if k == self.nack_byte:
            self.status |= RXNACK
        else:
            self.status &= ~RXNACK
        self.flag |= MB
        return 'tx'

    def byte_in(self):
        self.data = (0x40 + self.rx_n) & 0xFF
        self.rx_n += 1
        self.flag |= SB
        return 'rx'

    def stop(self):
        self.bus = 1
        return 'stop'

    def step(self):
        return self.todo.popleft()() if self.todo else None

    def settle(self):
        """The bus actions left after a transfer returned, its STOP."""
        while self.todo:
            self.step()

    def trigger(self, tx):
        return bool(self.flag & (MB if tx else SB))

    def irq(self):
        return bool(self.flag & self.inten)


class Spim:
    """SERCOM in SPI master mode. A step shifts the byte written to DATA and
    receives 0x40, 0x41 and so on. Two received bytes are kept, ovf_byte
    drops the byte of that index with the overflow flags."""

    def __init__(self):
        self.reset()
        self.ovf_byte = None

    def reset(self):
        self.ctrla = self.ctrlb = self.inten = self.flag = self.status = 0
        self.tx = None
        self.rx = collections.deque()
        self.sent = []
        self.rx_n = 0

    def read(self, off, size):
        if off == 0x00:
            return self.ctrla
        if off == 0x04:
            return self.ctrlb
        if off in (0x14, 0x16):
            return self.inten
        if off == 0x18:
            return self.flag
        if off == 0x1A:
            return self.status
        if off == 0x1C:
            return 0
        if off == 0x28:
            if not self.rx:
                return 0
            d = self.rx.popleft()
            if not self.rx:
                self.flag &= ~RXC
            return d
        return 0

    def write(self, off, size, v):
        if off == 0x00:
            if v & 1:
                self.reset()
                return
            if v & 2 and not self.ctrla & 2:
                self.flag |= DRE
            self.ctrla = v
        elif off == 0x04:
            self.ctrlb = v
        elif off == 0x14:
            self.inten &= ~v
        elif off == 0x16:
            self.inten |= v
        elif off == 0x18:
            self.flag &= ~(v & (TXC | SPI_ERROR))
        elif off == 0x1A:
            self.status &= ~(v & BUFOVF)
        elif off == 0x28:
            assert self.tx is None, 'DATA written over a byte not sent'
            self.tx = v & 0xFF
            self.flag &= ~(DRE | TXC)

    def step(self):
        if self.tx is None:
            return None
        self.sent.append(self.tx)
        self.tx = None
        v = (0x40 + self.rx_n) & 0xFF
        self.rx_n += 1
        if len(self.sent) - 1 == self.ovf_byte or len(self.rx) >= 2:
            self.status |= BUFOVF
            self.flag |= SPI_ERROR
        else:
            self.rx.append(v)
            self.flag |= RXC
        self.flag |= DRE | TXC
        return 'spi'

    def trigger(self, tx):
        return bool(self.flag & (DRE if tx else RXC))

    def irq(self):
        return bool(self.flag & self.inten)


class Dmac:
    """DMAC: byte beats, one per trigger, a block per descriptor, the
    channel disabled at its end with TCMPL, the count left written back at
    WRBADDR at the end and at a disable. Triggers: SERCOM n RX 1 + 2n, TX
    2 + 2n."""

    def __init__(self, check):
        self.c = check
        self.reset()

    def reset(self):
        self.ctrl = self.base = self.wrb = self.chid = 0
        self.ch = [dict(on=False, ctrlb=0, inten=0, flag=0, cnt=0, src=0, dst=0, si=False, di=False)
                   for _ in range(12)]
        self.blocks = 0

    def read(self, off, size):
        ch = self.ch[self.chid]
        if off == 0x00:
            return self.ctrl
        if off == 0x20:
            for i, c in enumerate(self.ch):
                if c['flag'] & c['inten']:
                    return i | (c['flag'] << 8)
            return 0
        if off == 0x34:
            return self.base
        if off == 0x38:
            return self.wrb
        if off == 0x3F:
            return self.chid
        if off == 0x40:
            return 2 if ch['on'] else 0
        if off == 0x44:
            return ch['ctrlb']
        if off in (0x4C, 0x4D):
            return ch['inten']
        if off == 0x4E:
            return ch['flag']
        return 0

    def write(self, off, size, v):
        ch = self.ch[self.chid]
        if off == 0x00:
            if v & 1:
                self.reset()
            else:
                self.ctrl = v
        elif off == 0x20:
            self.ch[v & 0xF]['flag'] &= ~((v >> 8) & 7)
        elif off == 0x34:
            self.base = v
        elif off == 0x38:
            self.wrb = v
        elif off == 0x3F:
            self.chid = v & 0xF
        elif off == 0x40:
            if v & 2 and not ch['on']:
                self.fetch(ch, self.chid)
            elif not v & 2 and ch['on']:
                ch['on'] = False
                self.writeback(ch, self.chid)
        elif off == 0x44:
            assert not ch['on'], 'CHCTRLB written with the channel enabled'
            ch['ctrlb'] = v
        elif off == 0x4C:
            ch['inten'] &= ~v
        elif off == 0x4D:
            ch['inten'] |= v
        elif off == 0x4E:
            ch['flag'] &= ~v

    def fetch(self, ch, n):
        btctrl, cnt, src, dst = struct.unpack('<HHII', self.c.u.mem_read(self.base + 16 * n, 12))
        assert btctrl & 1 and cnt, ('descriptor', hex(btctrl), cnt)
        ch['si'], ch['di'] = bool(btctrl & (1 << 10)), bool(btctrl & (1 << 11))
        ch['src'] = src - cnt if ch['si'] else src
        ch['dst'] = dst - cnt if ch['di'] else dst
        ch['cnt'] = cnt
        ch['on'] = True

    def writeback(self, ch, n):
        self.c.u.mem_write(self.wrb + 16 * n + 2, struct.pack('<H', ch['cnt']))

    def step(self):
        if not self.ctrl & 2:
            return None
        for n, ch in enumerate(self.ch):
            if not ch['on']:
                continue
            trig = (ch['ctrlb'] >> 8) & 0x3F
            model = self.c.sercom.get((trig - 1) // 2)
            if model is None or not model.trigger(trig % 2 == 0):
                continue
            v = self.c.load(ch['src'])
            self.c.store(ch['dst'], v)
            ch['src'] += ch['si']
            ch['dst'] += ch['di']
            ch['cnt'] -= 1
            if ch['cnt'] == 0:
                ch['on'] = False
                ch['flag'] |= 2         # TCMPL
                self.writeback(ch, n)
                self.blocks += 1
            return 'beat'
        return None

    def irq(self):
        return any(c['flag'] & c['inten'] for c in self.ch)


class Check:
    def __init__(self, elf):
        u = self.u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        u.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M0)
        for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0x40000000, 0x100000),
                           (0x41000000, 0x4000), (0x41005000, 0xFB000), (0x42002000, 0xFE000),
                           (0x60000000, 0x1000), (0xE0000000, 0xE000), (0xE000F000, 0xF1000)]:
            u.mem_map(base, size)
        with open(elf, 'rb') as f:
            e = ELFFile(f)
            self.sym = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    u.mem_write(seg['p_vaddr'], seg.data())
        self.i2c, self.spi, self.dmac = I2cm(), Spim(), Dmac(self)
        self.sercom = {0: self.spi, 2: self.i2c}
        self.pages = {}
        for base in (0x41004000, 0x42000000, 0x42001000, 0xE000E000):
            p = self.pages[base] = Page()
            u.mmio_map(base, 0x1000, p.read, None, p.write, None)
        self.pages[0x41004000].add(0x800, 0x50, self.dmac)
        self.pages[0x42000000].add(0x800, 0x40, self.spi)
        self.pages[0x42001000].add(0x000, 0x40, self.i2c)
        self.pages[0xE000E000].add(0x100, 0x200, self)
        self.iser = 0
        self.blocked = set()
        L = [self.call('layout', i) & 0xFFFFFFFF for i in range(15)]
        self.BUF = L[0]
        assert L[1:4] == [0x42001000, 0x42000800, 0x41004800], [hex(x) for x in L[1:4]]
        assert L[4:15] == [0x14, 0x16, 0x18, 0x1A, 0x24, 0x28, 0x28, 0x20, 0x3F, 0x40, 0x4E], L[4:15]

    # ---- NVIC enable and clear registers
    def read(self, off, size):
        return self.iser if off in (0x00, 0x80) else 0

    def write(self, off, size, v):
        if off == 0x00:
            self.iser |= v
        elif off == 0x80:
            self.iser &= ~v

    # ---- DMAC access to memory and registers
    def load(self, a):
        base = a & ~0xFFF
        if base in self.pages:
            return self.pages[base].read(self.u, a - base, 1, None)
        return self.u.mem_read(a, 1)[0]

    def store(self, a, v):
        base = a & ~0xFFF
        if base in self.pages:
            self.pages[base].write(self.u, a - base, 1, v, None)
        else:
            self.u.mem_write(a, bytes([v & 0xFF]))

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
        for irq, m in ((DMAC_IRQ, self.dmac), (SPI_IRQ, self.spi), (I2C_IRQ, self.i2c)):
            if irq not in self.blocked and self.iser & (1 << irq) and m.irq():
                return irq
        return None

    def step(self):
        r = self.dmac.step()
        for m in (self.i2c, self.spi):
            if r is None:
                r = m.step()
        return r

    def burst(self):
        """Steps until nothing is left, an interrupt is pending or an error
        flag is set. Returns the steps taken."""
        n = 0
        while n < BURST and self.irq_pending() is None:
            errors = (self.i2c.flag & I2C_ERROR) | (self.spi.flag & SPI_ERROR)
            if self.step() is None:
                break
            n += 1
            if (self.i2c.flag & I2C_ERROR) | (self.spi.flag & SPI_ERROR) != errors:
                break
        return n

    def run(self, name, *args, idle=False):
        """Call name and serve it each time it waits. Returns what it
        returned, the steps served, the interrupts run and the waits with
        nothing to serve."""
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
            if irq is not None:
                self.call('isr', irq, sp=0x200F0000)
                irqs += 1
            u.context_restore(ctx)
            pc = stopped | 1
            assert steps + irqs + waits < 20000, (name, args, 'does not end')
        r = u.reg_read(UC_ARM_REG_R0)
        return (r - (1 << 32) if r & 0x80000000 else r), steps, irqs, waits

    def data(self, n):
        return list(self.u.mem_read(self.BUF, n))

    def fill(self, values):
        self.u.mem_write(self.BUF, bytes(values))


def rx_values(n, first=0):
    return [(0x40 + first + i) & 0xFF for i in range(n)]


def check_i2c(c, dma, irq):
    mode = 'I2C dma=%d irq=%d' % (dma, irq)
    m = c.i2c

    def init():
        m.reset()
        m.nack_addr, m.nack_byte, m.buserr_byte = False, None, None
        assert c.call('i2c_init', dma, irq) == 1, mode
        m.sent, m.addrs, m.rx_n = [], [], 0

    def tx(data, expect, idle=False):
        m.sent, m.addrs = [], []
        c.fill(data)
        r, steps, irqs, waits = c.run('i2c_tx', len(data), idle=idle)
        m.settle()
        assert r == expect, (mode, 'tx', len(data), r, expect)
        assert c.call('i2c_busy') == 0, (mode, 'busy after tx')
        return irqs, waits

    def rx(n, first):
        c.fill([0] * n)
        m.addrs = []
        r, steps, irqs, waits = c.run('i2c_rx', n)
        m.settle()
        assert r == n and c.data(n) == rx_values(n, first), (mode, 'rx', n, r, c.data(n))
        assert m.addrs == [0x45] and m.bus == 1, (mode, 'rx address and STOP', m.addrs, m.bus)
        return irqs

    init()
    for n in (1, 3, 40):
        data = [(i * 7 + 1) & 0xFF for i in range(n)]
        irqs, waits = tx(data, n)
        assert m.sent == data and m.addrs == [0x44] and m.bus == 1, (mode, 'tx', m.sent, m.addrs, m.bus)
        assert (irqs > 0) == bool(irq) and waits == 0, (mode, 'tx interrupts', irqs)
        # The handler masks the interrupt it took
        assert dma or m.inten == 0, (mode, 'interrupt left enabled', hex(m.inten))
    for n in (1, 2, 33):
        m.rx_n = 0
        irqs = rx(n, 0)
        assert (irqs > 0) == bool(irq), (mode, 'rx interrupts', irqs)
    # Register read: the command, a repeated START, the read
    m.sent, m.addrs, m.rx_n = [], [], 0
    r, steps, irqs, waits = c.run('i2c_read_reg', 5)
    m.settle()
    assert r == 5 and m.sent == [0xA7, 0x5C] and c.data(5) == rx_values(5), (mode, 'read', r, m.sent)
    assert m.addrs == [0x44, 0x45] and m.bus == 1, (mode, 'read addresses', m.addrs)
    assert c.call('i2c_busy') == 0
    # Combined write from the buffer DeviceIntrfWrite() builds
    m.sent = []
    c.fill(range(0x50, 0x56))
    r, steps, irqs, waits = c.run('i2c_write_data', 6)
    m.settle()
    assert r == 5 and m.sent == list(range(0x50, 0x56)), (mode, 'write', r, m.sent)
    # Address refused: nothing moved, the bus is used again after
    m.nack_addr = True
    tx([1, 2, 3, 4], 0)
    assert m.bus == 1, (mode, 'STOP after a refused address')
    tx([9], 1)
    # Last byte refused: one byte less
    m.nack_byte = 3
    tx([1, 2, 3, 4], 3)
    # Byte 1 refused: the transmit stops there without DMA, the DMA channel
    # sends all
    m.nack_byte = 1
    tx([1, 2, 3, 4], 4 if dma else 1)
    m.nack_byte = None
    if dma:
        # Bus error on byte 2 of a DMA block: the byte on the bus is not
        # counted, the wait ends on the error
        m.buserr_byte = 2
        irqs, waits = tx([1, 2, 3, 4, 5, 6], 2)
        assert waits == 0, (mode, 'bus error wait')
        m.buserr_byte = None
        tx([7, 8], 2)
    if irq:
        # A flag whose interrupt is held off does not end the wait: the
        # transfer ends on its time limit
        c.blocked = {I2C_IRQ, DMAC_IRQ}
        irqs, waits = tx([1, 2, 3, 4], 0, idle=True)
        c.blocked = set()
        assert irqs == 0 and waits > 0, (mode, 'held off', irqs, waits)
        tx([5, 6], 2)
        if dma:
            # The end of the DMA block held off: the bytes are sent, the wait
            # of the block ends on its time limit
            c.blocked = {DMAC_IRQ}
            irqs, waits = tx([1, 2, 3, 4], 4, idle=True)
            c.blocked = set()
            assert waits > 0, (mode, 'block end held off', irqs, waits)
            tx([5, 6], 2)
    else:
        assert m.inten == 0, (mode, 'interrupt enabled in polling mode', hex(m.inten))
    assert c.call('events') == 0, (mode, 'master reported an event')


def check_spi(c, dma, irq):
    mode = 'SPI dma=%d irq=%d' % (dma, irq)
    m = c.spi
    m.reset()
    m.ovf_byte = None
    assert c.call('spi_init', dma, irq) == 1, mode

    for n in (1, 3, 40, 300):
        m.sent, m.rx_n = [], 0
        data = [(i * 5 + 3) & 0xFF for i in range(n)]
        c.fill(data)
        r, steps, irqs, waits = c.run('spi_tx', n)
        assert r == n and m.sent == data, (mode, 'tx', n, r)
        assert (irqs > 0) == bool(irq) and waits == 0, (mode, 'tx interrupts', irqs, waits)
        assert dma or m.inten == 0, (mode, 'interrupt left enabled', hex(m.inten))
        assert c.call('spi_busy') == 0
    for n in (1, 2, 33, 300):
        m.sent, m.rx_n = [], 0
        c.fill([0] * n)
        r, steps, irqs, waits = c.run('spi_rx', n)
        assert r == n and c.data(n) == rx_values(n) and m.sent == [0xFF] * n, (mode, 'rx', n, r)
        assert (irqs > 0) == bool(irq) and waits == 0, (mode, 'rx interrupts', irqs, waits)
    # Command then read with the chip select kept
    m.sent, m.rx_n = [], 0
    r, steps, irqs, waits = c.run('spi_read', 4)
    assert r == 4 and m.sent == [0x9F, 0x03] + [0xFF] * 4 and c.data(4) == rx_values(4, 2), (mode, 'read', r)
    if dma:
        # Receive overflow on byte 5: the bytes received, the wait ends on
        # the error
        m.sent, m.rx_n, m.ovf_byte = [], 0, 5
        r, steps, irqs, waits = c.run('spi_rx', 20)
        m.ovf_byte = None
        assert r == 5 and waits == 0, (mode, 'overflow', r, waits)
        assert c.call('spi_busy') == 0
    if irq:
        # A flag whose interrupt is held off does not end the wait
        m.sent, m.rx_n = [], 0
        c.blocked = {SPI_IRQ, DMAC_IRQ}
        r, steps, irqs, waits = c.run('spi_tx', 4, idle=True)
        c.blocked = set()
        # The DMA channels move the bytes, the wait ends on its time limit
        assert r == (4 if dma else 0) and irqs == 0 and waits > 0, (mode, 'held off', r, irqs, waits)
        m.sent, m.rx_n = [], 0
        m.rx.clear()
        m.flag &= ~RXC
        c.fill([5, 6])
        r, steps, irqs, waits = c.run('spi_tx', 2)
        assert r == 2 and m.sent == [5, 6], (mode, 'after the time limit', r, m.sent)
    else:
        assert m.inten == 0, (mode, 'interrupt enabled in polling mode', hex(m.inten))
    assert c.call('events') == 0, (mode, 'master reported an event')


def main(elf, label):
    for dma in (0, 1):
        for irq in (0, 1):
            c = Check(elf)
            check_i2c(c, dma, irq)
            check_spi(c, dma, irq)
    print(label, 'SAM D21 I2C and SPI master transfers: PASS')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--cxx', default='arm-none-eabi-g++')
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        for opt in ('-O1', '-Os'):
            elf = Path(tmp) / 'master_irq.elf'
            build(a.cxx, opt, elf)
            main(elf, opt)
