#!/usr/bin/env python3
"""Build and run the SAM4L TWIM master transfer check.

The production i2c_sam4l.cpp and device_intrf.cpp are built for the SAM4L and
run in an Arm emulator (unicorn) with a TWIM and PDCA register model. The model
does nothing by itself: a transfer is synchronous and waits, and each time the
firmware has run for a while without returning, the model makes the TWIM or
the PDCA take its next step, then runs the TWIM interrupt handler when an
enabled status flag is set, which is what the NVIC does.

Usage: run_i2c_irq.py [--cxx arm-none-eabi-g++]
Requires an Arm GNU C++ compiler and the Python packages unicorn and pyelftools.
This is a register level check, not hardware validation: bus timing is not
modelled.
"""
import argparse
import struct
import subprocess
import tempfile
from pathlib import Path
from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_WRITE, UC_HOOK_MEM_READ
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

ROOT = Path(__file__).resolve().parents[2]

# TWIM status and command bits
RXRDY, TXRDY, CCOMP, IDLE, ANAK = 1 << 0, 1 << 1, 1 << 3, 1 << 4, 1 << 8
SR_CLEAR = CCOMP | ANAK | (1 << 9) | (1 << 10) | (1 << 11) | (1 << 12) | (1 << 13) | (1 << 14)
C_READ, C_TENBIT, C_REPSAME, C_START, C_STOP, C_VALID, C_ACKLAST = \
    1 << 0, 1 << 11, 1 << 12, 1 << 13, 1 << 14, 1 << 15, 1 << 25
QUANTUM = 20000


def build(cxx, opt, out):
    flags = [cxx, '-std=gnu++20', '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt, '-g',
             '-fno-inline-functions-called-once', '-fno-exceptions', '-fno-rtti',
             '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START', '-D__SAM4LS8C__',
             '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
             '-I' + str(ROOT / 'ARM/Microchip/SAM4L/include'), '-I' + str(ROOT / 'ARM/CMSIS/Core/Include')]
    sources = [ROOT / 'tests/sam4l/i2c_irq_fixture.cpp', ROOT / 'src/device_intrf.cpp']
    args = flags + [str(s) for s in sources] + [
        '-nostartfiles', '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,init']
    args += ['-Wl,-u,' + s for s in ('isr', 'tx', 'rx', 'read_reg', 'write_data', 'busy', 'events', 'layout')]
    subprocess.run(args + ['-o', str(out)], check=True)


class Check:
    def __init__(self, elf):
        u = self.u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0x40000000, 0x100000),
                           (0xE0000000, 0x100000)]:
            u.mem_map(base, size)
        with open(elf, 'rb') as f:
            e = ELFFile(f)
            self.sym = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    u.mem_write(seg['p_vaddr'], seg.data())
        L = [self.call('layout', i) & 0xFFFFFFFF for i in range(18)]
        self.T, self.BUF = L[0], L[1]
        (self.CR, self.CMDR, self.NCMDR, self.RHR, self.THR, self.SR,
         self.IER, self.IDR, self.IMR, self.SCR) = L[2:12]
        self.PDCA = {'rx': L[12], 'tx': L[13]}
        self.MAR, self.TCR, self.PCR, self.PISR = L[14:18]
        self.reset_model()
        u.hook_add(UC_HOOK_MEM_WRITE, self.write)
        u.hook_add(UC_HOOK_MEM_READ, self.read)

    # ---- emulator
    def get(self, addr):
        return struct.unpack('<I', self.u.mem_read(addr, 4))[0]

    def put(self, addr, value):
        self.u.mem_write(addr, struct.pack('<I', value & 0xFFFFFFFF))

    def setup(self, name, args, sp):
        for i, a in enumerate(args):
            self.u.reg_write(UC_ARM_REG_R0 + i, a)
        self.u.reg_write(UC_ARM_REG_SP, sp)
        self.u.reg_write(UC_ARM_REG_LR, 0xF0001)

    def call(self, name, *args, sp=0x200FF000):
        self.setup(name, args, sp)
        self.u.emu_start(self.sym[name] | 1, 0xF0000, count=50000000)
        assert self.u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return')
        r = self.u.reg_read(UC_ARM_REG_R0)
        return r - (1 << 32) if r & 0x80000000 else r

    # ---- TWIM and PDCA register model
    def reset_model(self):
        self.sr, self.imr, self.cmd, self.ncmd, self.done = IDLE, 0, 0, 0, 0
        self.thr_full = self.stop = False
        self.pdca = {'rx': False, 'tx': False}
        self.sent, self.cmds, self.rx_n = [], [], 0
        self.anak = False
        self.blocked = False

    def sync(self):
        self.put(self.T + self.SR, self.sr)
        self.put(self.T + self.IMR, self.imr)

    def write(self, uc, access, addr, size, value, data):
        off = addr - self.T
        if 0 <= off < 0x48:
            if off == self.CR:
                if value & (1 << 7):        # SWRST
                    self.sr, self.cmd, self.ncmd, self.done = IDLE, 0, 0, 0
                    self.thr_full = self.stop = False
                if value & (1 << 8):        # STOP
                    self.stop = True
            elif off == self.CMDR:
                self.cmd, self.done, self.thr_full = value, 0, False
                if value & C_VALID:
                    self.cmds.append(value)
                    self.sr &= ~IDLE
            elif off == self.NCMDR:
                self.ncmd = value
                if value & C_VALID:
                    self.cmds.append(value)
            elif off == self.THR:
                self.sent.append(value & 0xFF)
                self.thr_full = True
                self.sr &= ~TXRDY
            elif off == self.IER:
                self.imr |= value
            elif off == self.IDR:
                self.imr &= ~value
            elif off == self.SCR:
                self.sr &= ~(value & SR_CLEAR)
            self.sync()
            return
        for k, ch in self.PDCA.items():
            if addr == ch + self.PCR:
                if value & 2:
                    self.pdca[k] = False
                if value & 1:
                    self.pdca[k] = True

    def read(self, uc, access, addr, size, value, data):
        if addr == self.T + self.RHR:
            self.sr &= ~RXRDY
            self.sync()

    def complete(self):
        self.sr |= CCOMP
        if self.cmd & C_STOP:
            self.sr |= IDLE
        if self.ncmd & C_VALID:
            self.cmd, self.ncmd, self.done = self.ncmd, 0, 0
        else:
            self.cmd = 0

    def step(self):
        """The TWIM, or the PDCA, takes its next step. None: nothing to do."""
        r = self._step()
        self.sync()
        return r

    def _step(self):
        if self.stop:
            self.stop = False
            self.sr |= IDLE
            return 'stop'
        if not self.cmd & C_VALID:
            return None
        nb = (self.cmd >> 16) & 0xFF
        if self.anak and self.done == 0:
            self.anak = False
            self.sr |= ANAK
            self.cmd = 0
            return 'anak'
        if self.cmd & C_READ:
            if self.done >= nb or self.sr & RXRDY:
                return None
            v = (0x40 + self.rx_n) & 0xFF
            self.rx_n += 1
            ch = self.PDCA['rx']
            if self.pdca['rx'] and self.get(ch + self.TCR):
                self.u.mem_write(self.get(ch + self.MAR), bytes([v]))
                self.put(ch + self.MAR, self.get(ch + self.MAR) + 1)
                self.put(ch + self.TCR, self.get(ch + self.TCR) - 1)
            else:
                self.put(self.T + self.RHR, v)
                self.sr |= RXRDY
            self.done += 1
            if self.done == nb:
                self.complete()
            return 'rx'
        if nb == 0:
            self.complete()
            return 'address'
        ch = self.PDCA['tx']
        if not self.thr_full and self.pdca['tx'] and self.get(ch + self.TCR):
            mar = self.get(ch + self.MAR)
            self.sent.append(self.u.mem_read(mar, 1)[0])
            self.put(ch + self.MAR, mar + 1)
            self.put(ch + self.TCR, self.get(ch + self.TCR) - 1)
            self.thr_full = True
        if self.thr_full:
            self.thr_full = False
            self.done += 1
            if self.done == nb:
                self.complete()
            else:
                self.sr |= TXRDY
            return 'tx'
        if not self.sr & TXRDY and self.done < nb:
            self.sr |= TXRDY
            return 'txrdy'
        return None

    def irq_pending(self):
        return not self.blocked and (self.sr & self.imr) != 0

    def run(self, name, *args, idle=False):
        """Call name and serve it each time it waits. Returns what it
        returned, the steps served and the interrupts run."""
        u = self.u
        self.setup(name, args, 0x200FF000)
        pc = self.sym[name] | 1
        steps = irqs = 0
        while True:
            u.emu_start(pc, 0xF0000, count=QUANTUM)
            stopped = u.reg_read(UC_ARM_REG_PC)
            if stopped == 0xF0000:
                break
            ctx = u.context_save()
            if not self.irq_pending():
                r = self.step()
                assert r is not None or idle, (name, args, 'waits with nothing to serve', steps, irqs)
                steps += 1
            if self.irq_pending():
                self.call('isr', sp=0x200F0000)
                irqs += 1
            u.context_restore(ctx)
            pc = stopped | 1
            assert steps + irqs < 3000, (name, args, 'does not end')
        r = u.reg_read(UC_ARM_REG_R0)
        return (r - (1 << 32) if r & 0x80000000 else r), steps, irqs

    def init(self, dma, irq, tenbit=False):
        self.reset_model()
        assert self.call('init', dma, irq, tenbit) == 1
        self.reset_model()

    def data(self, n):
        return list(self.u.mem_read(self.BUF, n))

    def fill(self, values):
        self.u.mem_write(self.BUF, bytes(values))


def nbytes(c):
    return (c >> 16) & 0xFF


def chunks(n):
    out = []
    while n:
        out.append(min(n, 255))
        n -= out[-1]
    return out


def check_commands(cmds, length, read, tenbit=False):
    """One command per chunk of at most 255 bytes, START on the first, STOP on
    the last, a read acknowledges the last byte of all but the last chunk."""
    parts = chunks(length)
    if tenbit:
        assert not cmds[0] & C_READ and nbytes(cmds[0]) == 0 and cmds[0] & C_TENBIT, hex(cmds[0])
        cmds = cmds[1:]
    assert len(cmds) == len(parts), ([hex(c) for c in cmds], parts)
    for i, (c, n) in enumerate(zip(cmds, parts)):
        last = i == len(parts) - 1
        assert nbytes(c) == n and bool(c & C_READ) == read, (i, hex(c))
        assert bool(c & C_START) == (i == 0) and bool(c & C_STOP) == last, (i, hex(c))
        if read:
            assert bool(c & C_ACKLAST) == (not last), (i, hex(c))
        if tenbit and i == 0:
            assert c & C_REPSAME, hex(c)


def main(c, label):
    rx_values = lambda n, first=0: [(0x40 + first + i) & 0xFF for i in range(n)]
    for dma in (0, 1):
        mode = 'interrupt' + (' and PDCA' if dma else '')
        # Transmit and receive from the caller's buffer, command after command
        for n in (1, 3, 255, 300, 600):
            c.init(dma, 1)
            data = [(i * 7 + 1) & 0xFF for i in range(n)]
            c.fill(data)
            r, steps, irqs = c.run('tx', n)
            assert r == n and c.sent == data, (mode, 'tx', n, r)
            check_commands(c.cmds, n, False)
            assert irqs > 0 and c.imr == 0 and c.call('busy') == 0, (mode, 'tx end', n, irqs, hex(c.imr))
        for n, tenbit in ((1, False), (4, False), (300, False), (4, True), (300, True)):
            c.init(dma, 1, tenbit)
            c.fill([0] * n)
            r, steps, irqs = c.run('rx', n)
            assert r == n and c.data(n) == rx_values(n), (mode, 'rx', n, tenbit, r)
            check_commands(c.cmds, n, True, tenbit)
            assert irqs > 0 and c.imr == 0 and c.call('busy') == 0, (mode, 'rx end', n)
        # Register read: the write command and the read queued in NCMDR
        c.init(dma, 1)
        r, steps, irqs = c.run('read_reg', 5)
        assert r == 5 and c.sent == [0xA7, 0x5C] and c.data(5) == rx_values(5), (mode, 'read', r, c.sent)
        assert [(bool(x & C_READ), nbytes(x)) for x in c.cmds] == [(False, 2), (True, 5)], [hex(x) for x in c.cmds]
        assert c.call('busy') == 0
        # Combined write: sent from the buffer DeviceIntrfWrite() builds
        c.init(dma, 1)
        c.fill(range(0x50, 0x56))
        r, steps, irqs = c.run('write_data', 6)
        assert r == 5 and c.sent == list(range(0x50, 0x56)), (mode, 'write', r, c.sent)
        # Address not acknowledged: nothing moved, the bus is used again after
        c.init(dma, 1)
        c.anak = True
        r, steps, irqs = c.run('tx', 4)
        assert r == 0 and c.call('busy') == 0, (mode, 'nack', r)
        c.fill([9])
        r, steps, irqs = c.run('tx', 1)
        assert r == 1 and c.sent == [9], (mode, 'after nack', r, c.sent)
        # A flag whose interrupt is held off does not end the wait: the
        # transfer ends on its time limit and the TWIM is reset.
        c.init(dma, 1)
        c.blocked = True
        r, steps, irqs = c.run('tx', 4, idle=True)
        c.blocked = False
        assert r == 0 and irqs == 0 and c.call('busy') == 0, (mode, 'held off', r)
        c.reset_model()
        c.fill([5, 6])
        r, steps, irqs = c.run('tx', 2)
        assert r == 2 and c.sent == [5, 6], (mode, 'after the time limit', r, c.sent)
        assert c.call('events') == 0, (mode, 'master reported an event')

        # Polled: the same transfers, no interrupt enabled
        c.init(dma, 0)
        c.fill([1, 2, 3])
        r, steps, irqs = c.run('tx', 3)
        assert r == 3 and c.sent == [1, 2, 3] and irqs == 0 and c.imr == 0, ('polled tx', dma, r)
        r, steps, irqs = c.run('rx', 3)
        assert r == 3 and c.data(3) == rx_values(3) and irqs == 0, ('polled rx', dma, r)
    print(label, 'SAM4L I2C master transfers: PASS')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--cxx', default='arm-none-eabi-g++')
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        for opt in ('-O1', '-Os'):
            elf = Path(tmp) / 'i2c_irq.elf'
            build(a.cxx, opt, elf)
            main(Check(elf), opt)
