#!/usr/bin/env python3
"""Build and run the SAM4L TWIS slave transfer check.

The production i2c_sam4l.cpp is built for the SAM4L and run in an Arm
emulator (unicorn) with a TWIS and PDCA register model that plays the bus
master. The model takes one bus step at a time and runs the TWIS or the PDCA
receive interrupt handler when an enabled flag is set, which is what the NVIC
does. The slave event handler is the register memory of the I2CMasterSlave
example.

The transmit holding register (THR) keeps its byte across transfers, and a
PDCA transmit channel loads THR as soon as it is empty, while the byte before
is still on the bus. A byte the master refuses is then left in THR.

Checked, with the slave in interrupt and in DMA mode: a register write, reads
of 1, 2 and 8 bytes at the same offset, reads without a command that continue
after the last one, the data on the bus, the count of each completion, and
no byte left in THR after a read.

Usage: run_i2c_slave.py [--cxx arm-none-eabi-g++]
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
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC, UC_ARM_REG_R0

ROOT = Path(__file__).resolve().parents[2]

# TWIS status bits
RXRDY, TXRDY, TCOMP, TRA, NAK, SAM, STO, REP, BTF = \
    1 << 0, 1 << 1, 1 << 3, 1 << 5, 1 << 8, 1 << 16, 1 << 21, 1 << 22, 1 << 23
SR_CLEAR = 0x00FFFFC8          # the flags SCR clears
CR_ACK, CR_SWRST = 1 << 12, 1 << 7
# PDCA
P_TEN, P_TDIS, P_RCZ, P_TRC = 1 << 0, 1 << 1, 1 << 0, 1 << 1
SLAVE_ADDR = 0x22


def build(cxx, opt, out):
    flags = [cxx, '-std=gnu++20', '-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt, '-g',
             '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti',
             '-ffunction-sections', '-fdata-sections', '-D__PROGRAM_START', '-D__SAM4LS8C__',
             '-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
             '-I' + str(ROOT / 'ARM/Microchip/SAM4L/include'), '-I' + str(ROOT / 'ARM/CMSIS/Core/Include')]
    sources = [ROOT / 'tests/sam4l/i2c_slave_fixture.cpp']
    args = flags + [str(s) for s in sources] + [
        '-nostartfiles', '-Wl,-Ttext=0x10000,-Tdata=0x20000000,--gc-sections,-e,init']
    args += ['-Wl,-u,' + s for s in ('twis_isr', 'pdca_isr', 'layout')]
    subprocess.run(args + ['-o', str(out)], check=True)


class Check:
    def __init__(self, elf):
        u = self.u = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for base, size in [(0, 0x100000), (0x20000000, 0x100000), (0xE0000000, 0x100000)]:
            u.mem_map(base, size)
        with open(elf, 'rb') as f:
            e = ELFFile(f)
            self.sym = {s.name: s['st_value'] for s in e.get_section_by_name('.symtab').iter_symbols()}
            for seg in e.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    u.mem_write(seg['p_vaddr'], seg.data())
        # The layout is read before the peripherals are mapped: plain memory
        # first, then the TWIS and PDCA pages as register callbacks. A memory
        # hook that writes memory inside a Thumb IT block makes unicorn run
        # the instructions after the block as conditional ones; register
        # callbacks do not.
        u.mem_map(0x40000000, 0x100000)
        L = [self.call('layout', i) & 0xFFFFFFFF for i in range(24)]
        u.mem_unmap(0x40000000, 0x100000)
        self.T, self.MEM = L[0], L[1]
        (self.CR, self.RHR, self.THR, self.SR, self.IER, self.IDR, self.IMR, self.SCR) = L[2:10]
        self.CH = {'rx': L[10], 'tx': L[11]}
        (self.MAR, self.PSR, self.TCR, self.PCR, self.PIER, self.PIDR, self.PIMR, self.PISR) = L[12:20]
        self.DONE, self.NDONE = L[20], L[21]
        self.PID = {'rx': L[22], 'tx': L[23]}
        twis_page = self.T & ~0xFFF
        pdca_page = self.CH['rx'] & ~0xFFF
        assert self.CH['tx'] & ~0xFFF == pdca_page
        for base, size in ((0x40000000, twis_page - 0x40000000), (twis_page + 0x1000, pdca_page - twis_page - 0x1000),
                           (pdca_page + 0x1000, 0x40100000 - pdca_page - 0x1000)):
            u.mem_map(base, size)
        u.mmio_map(twis_page, 0x1000, self.twis_rd, None, self.twis_wr, None)
        u.mmio_map(pdca_page, 0x1000, self.pdca_rd, None, self.pdca_wr, None)
        self.twis_page, self.pdca_page = twis_page, pdca_page
        self.reset_model()

    # ---- emulator
    def get(self, addr):
        return struct.unpack('<I', self.u.mem_read(addr, 4))[0]

    def call(self, name, *args, sp=0x200FF000):
        for i, a in enumerate(args):
            self.u.reg_write(UC_ARM_REG_R0 + i, a)
        self.u.reg_write(UC_ARM_REG_SP, sp)
        self.u.reg_write(UC_ARM_REG_LR, 0xF0001)
        self.u.emu_start(self.sym[name] | 1, 0xF0000, count=5000000)
        assert self.u.reg_read(UC_ARM_REG_PC) == 0xF0000, (name, 'did not return')
        return self.u.reg_read(UC_ARM_REG_R0)

    # ---- TWIS and PDCA register model
    def reset_model(self):
        self.sr, self.imr, self.cr, self.rhr = TXRDY, 0, 0, 0
        self.thr = None
        self.en = {'rx': False, 'tx': False}
        self.ch = {k: {'mar': 0, 'psr': 0, 'tcr': 0, 'imr': 0} for k in ('rx', 'tx')}
        self.script, self.xfer, self.pos = [], None, 0
        self.bus = []                   # bytes the master received, per read

    def status(self):
        return (self.sr & ~TXRDY) | (TXRDY if self.thr is None else 0)

    def twis_rd(self, uc, offset, size, user):
        off = self.twis_page + offset - self.T
        if off == self.SR:
            return self.status()
        if off == self.IMR:
            return self.imr
        if off == self.CR:
            return self.cr
        if off == self.RHR:
            self.sr &= ~RXRDY
            return self.rhr
        return 0

    def twis_wr(self, uc, offset, size, value, user):
        off = self.twis_page + offset - self.T
        if off == self.CR:
            if value & CR_SWRST:
                self.sr, self.thr, self.imr = TXRDY, None, 0
            self.cr = value & ~CR_SWRST
        elif off == self.THR:
            self.thr = value & 0xFF
        elif off == self.IER:
            self.imr |= value
        elif off == self.IDR:
            self.imr &= ~value
        elif off == self.SCR:
            self.sr &= ~(value & SR_CLEAR)

    def channel(self, offset):
        addr = self.pdca_page + offset
        for k, base in self.CH.items():
            if base <= addr < base + 0x40:
                return k, addr - base
        return None, None

    def pdca_rd(self, uc, offset, size, user):
        k, r = self.channel(offset)
        if k is None:
            return 0
        c = self.ch[k]
        if r == self.MAR:
            return c['mar']
        if r == self.PSR:
            return c['psr']
        if r == self.TCR:
            return c['tcr']
        if r == self.PIMR:
            return c['imr']
        if r == self.PISR:
            return (P_TRC | P_RCZ) if c['tcr'] == 0 else P_RCZ
        return 0

    def pdca_wr(self, uc, offset, size, value, user):
        k, r = self.channel(offset)
        if k is None:
            return
        c = self.ch[k]
        if r == self.MAR:
            c['mar'] = value
        elif r == self.PSR:
            c['psr'] = value
        elif r == self.TCR:
            c['tcr'] = value & 0xFFFF
        elif r == self.PIER:
            c['imr'] |= value
        elif r == self.PIDR:
            c['imr'] &= ~value
        elif r == self.PCR:
            if value & P_TDIS:
                self.en[k] = False
            if value & P_TEN:
                self.en[k] = True
            self.pdca_tx()

    def active(self, k):
        c = self.ch[k]
        return self.en[k] and c['psr'] == self.PID[k] and c['tcr'] > 0

    def pdca_tx(self):
        """The transmit channel loads THR whenever it is empty."""
        if self.thr is None and self.active('tx'):
            c = self.ch['tx']
            self.thr = self.u.mem_read(c['mar'], 1)[0]
            c['mar'] += 1
            c['tcr'] -= 1

    def step(self):
        """The bus master takes its next step. None: it waits for the slave."""
        if self.xfer is None:
            if not self.script:
                return 'end'
            self.xfer, self.pos = self.script.pop(0), 0
            kind, _, start = self.xfer
            self.sr = (self.sr & ~TRA) | SAM | (TRA if kind == 'R' else 0) | (REP if start == 'Sr' else 0)
            if kind == 'R':
                self.bus.append([])
            return 'address'
        kind, arg, _ = self.xfer
        if self.sr & SAM:
            return None                 # address match stretch
        if kind == 'W':
            if self.pos < len(arg):
                if self.sr & RXRDY:
                    return None         # RHR full, the slave stretches
                v = arg[self.pos]
                if self.active('rx'):
                    c = self.ch['rx']
                    self.u.mem_write(c['mar'], bytes([v]))
                    c['mar'] += 1
                    c['tcr'] -= 1
                else:
                    self.rhr = v
                    self.sr |= RXRDY
                self.sr |= BTF
                self.pos += 1
                return 'rx'
        else:
            if self.pos < arg:
                self.pdca_tx()
                if self.thr is None:
                    return None         # THR empty, the slave stretches
                self.bus[-1].append(self.thr)
                self.thr = None
                self.pdca_tx()          # the next byte is loaded while this one is on the bus
                self.pos += 1
                self.sr |= BTF | (NAK if self.pos == arg else 0)
                return 'tx'
        # STOP, or a repeated START when the next transfer begins with Sr
        if not (self.script and self.script[0][2] == 'Sr'):
            self.sr |= STO | TCOMP
        self.xfer = None
        return 'stop'

    def pending(self):
        if self.status() & self.imr:
            return 'twis_isr'
        c = self.ch['rx']
        if ((P_TRC | P_RCZ) if c['tcr'] == 0 else P_RCZ) & c['imr']:
            return 'pdca_isr'
        return None

    def run(self, script):
        """Play the bus transfers, serving the interrupts in between."""
        self.script = list(script)
        self.bus = []
        steps = 0
        while True:
            isr = self.pending()
            while isr:
                self.call(isr, sp=0x200F0000)
                steps += 1
                assert steps < 20000, 'interrupt does not end'
                isr = self.pending()
            r = self.step()
            if r == 'end':
                break
            assert r is not None or self.pending(), ('bus stretched with no interrupt pending', self.xfer, self.pos)
            steps += 1
            assert steps < 20000, 'transfers do not end'
        return self.bus

    def init(self, dma):
        self.reset_model()
        assert self.call('init', dma) == 1

    def done(self):
        n = self.get(self.NDONE)
        return [struct.unpack('<i', self.u.mem_read(self.DONE + 4 * i, 4))[0] for i in range(n)]


def reg_read(offset, n):
    return [('W', [offset], 'S'), ('R', n, 'Sr')]


def main(c, label):
    wr = list(range(0xA0, 0xA8))
    for dma in (0, 1):
        mode = 'DMA' if dma else 'interrupt'
        c.init(dma)
        script = [('W', [4] + wr, 'S')]
        for n in (1, 2, 8):
            script += reg_read(4, n)
        script += [('R', 4, 'S'), ('R', 1, 'S')]
        bus = c.run(script)
        want = [wr[:1], wr[:2], wr, [12, 13, 14, 15], [16]]
        assert bus == want, (mode, 'bus data', [[hex(b) for b in x] for x in bus])
        assert c.done() == [9, 1, 2, 8, 4, 1], (mode, 'completion counts', c.done())
        assert c.thr is None, (mode, 'byte left in THR after the reads', hex(c.thr))
        # Boundary reads, then a read after each
        c.init(dma)
        script, want, counts = [], [], []
        for n in (15, 16, 17, 31, 32, 33, 63, 64, 65):
            script += reg_read(32, n) + [('R', 1, 'S')]
            want += [list(range(32, 32 + n)), [32 + n]]
            counts += [n, 1]
        assert c.run(script) == want, (mode, 'boundary reads')
        assert c.done() == counts, (mode, 'boundary completion counts', c.done())
    print(label, 'SAM4L I2C slave transfers: PASS')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--cxx', default='arm-none-eabi-g++')
    a = ap.parse_args()
    with tempfile.TemporaryDirectory() as tmp:
        for opt in ('-O1', '-Os'):
            elf = Path(tmp) / 'i2c_slave.elf'
            build(a.cxx, opt, elf)
            main(Check(elf), opt)
