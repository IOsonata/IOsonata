#!/usr/bin/env python3
"""Run the real Cortex-M0+ SPI/I2C drivers through their public interfaces.

The model implements documented RE01 register side effects, independently
checks clock equations and checks the RIIC dummy/WAIT/ACK/STOP sequence.
It models immediate transfers and injected stretches/faults, not analog
waveforms, DMA or interrupt timing. Needs unicorn and pyelftools.
"""
import struct
import sys

from elftools.elf.elffile import ELFFile
from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_PROT_ALL
from unicorn import UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2
from unicorn.arm_const import UC_ARM_REG_R3, UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_PC

SPI_BASE = (0x40072000, 0x40072100)
IIC_BASE = (0x40053000, 0x40053100)
PORT = 0x40040000
PFS = 0x40040800
MSTPB = 0x40047000
SCKDIVCR = 0x4001E020
STOP = 0x17FF00


class SpiModel:
    def __init__(self, emu, no):
        self.e, self.no, self.base = emu, no, SPI_BASE[no]
        self.pending = False
        self.error = 0
        self.frames = []
        self.rx = []
        self.fault_at = 0
        self.fault = 'error'
        self.force_status = None

    def read(self, off, size):
        if off == 3:
            status = 0x20 | self.error
            if self.pending:
                status |= 2 if self.error or self.fault == 'stall' and self.fault_at == len(self.frames) else 0x80
            self.e.put(self.base + 3, self.force_status if self.force_status is not None else status, 1)
        elif off == 4 and self.pending and not self.error:
            frame = self.frames[-1]
            assert size == frame['size'], 'SPI receive access width'
            value = self.rx.pop(0) if self.rx else frame['tx'] ^ 0x5A5A
            self.e.put(self.base + 4, value & ((1 << frame['bits']) - 1), size)
            frame['drained'] = True
            self.pending = False

    def write(self, off, size, value):
        if off == 0 and not value & 0x40:
            self.pending = False
            self.error = 0
        elif off == 3:
            self.error &= value
        elif off == 4:
            assert self.e.get(self.base) & 0x49 == 0x49, 'SPI enabled software-CS master'
            assert not self.pending, 'SPI receive was not drained before the next frame'
            cmd = self.e.get(self.base + 0x10, 2)
            spb = (cmd >> 8) & 15
            bits = 8 if spb == 4 else spb + 1
            assert 8 <= bits <= 16
            assert size == (1 if bits == 8 else 2), 'SPI transmit access width'
            assert bool(self.e.get(self.base + 0xB) & 0x40) == (bits == 8)
            self.frames.append(dict(tx=value, bits=bits, size=size,
                                    cs=self.e.outputs[self.no] & 0x18, drained=False))
            self.pending = True
            if self.fault_at == len(self.frames) and self.fault == 'error':
                self.error = 1


class IicModel:
    def __init__(self, emu, no):
        self.e, self.no, self.base = emu, no, IIC_BASE[no]
        self.status = 0
        self.control = 0
        self.stage = 'idle'
        self.events = []
        self.rx = bytes(range(0x30, 0x40))
        self.index = 0
        self.nack_next = False
        self.stop_pending = False
        self.reset = False
        self.mode = 1
        self.writes = 0
        self.fault_at = -1  # 0 = address, positive = transmitted data byte.
        self.fault = 'nack'
        self.rx_fault_at = -1
        self.stop_stall = False
        self.external_busy = False
        self.stretch_reads = 0

    def fail(self):
        if self.fault == 'nack':
            self.status = 0xD4
        elif self.fault == 'al':
            self.status = 2
            self.control = 0x80
            self.external_busy = True
            self.stage = 'other'
        else:
            self.status = 4

    def stop(self):
        if not self.stop_stall:
            self.status = 8
            self.control = 0
            self.stage = 'idle'
            self.stop_pending = False

    def read(self, off, size):
        assert size == 1
        if off == 1:
            self.e.put(self.base + off, self.control | (0x80 if self.external_busy else 0), 1)
        elif off == 9:
            status = self.status
            if self.stage == 'read' and self.index == self.rx_fault_at:
                self.fail()
                status = self.status
            if self.stretch_reads and status & 0x20:
                self.stretch_reads -= 1
                status &= ~0x20
            self.e.put(self.base + off, status, 1)
        elif off == 0x13:
            assert self.status & 0x20, 'ICDRR read without RDRF'
            if self.stage == 'dummy':
                assert bool(self.mode & 0x40) == (len(self.rx) <= 2), 'RIIC initial WAIT'
                assert bool(self.mode & 8) == (len(self.rx) == 1), 'RIIC one-byte NACK'
                self.e.put(self.base + off, 0xFF, 1)
                self.events.append(('dummy',))
                self.stage = 'read'
                self.nack_next = bool(self.mode & 8)
                self.index = 0
                self.status = 0x20
            else:
                assert self.stage == 'read'
                remaining = len(self.rx) - self.index
                assert remaining > 0, 'RIIC read beyond requested bytes'
                if remaining <= 3:
                    assert self.mode & 0x40, 'RIIC final three bytes must WAIT'
                if remaining <= 2:
                    assert self.mode & 8, 'RIIC final-byte NACK must be prepared early'
                assert self.nack_next == (remaining == 1), 'RIIC ACK only intermediate bytes'
                if remaining == 1:
                    assert self.stop_pending, 'RIIC STOP before final ICDRR read'
                self.e.put(self.base + off, self.rx[self.index], 1)
                self.events.append(('rx', self.rx[self.index], self.nack_next))
                self.nack_next = bool(self.mode & 8)
                self.index += 1
                if self.stop_pending:
                    self.stop()
                else:
                    self.status = 0x20

    def write(self, off, size, value):
        assert size == 1
        if off == 0:
            if value & 0x40 and not self.reset:
                self.events.append(('reset',))
                self.status = 0
                self.control = 0x80 if self.external_busy else 0
                self.stage = 'idle'
                self.stop_pending = False
            self.reset = bool(value & 0x40)
        elif off == 1:
            if value & 2:
                assert not self.control & 0x80 and not self.external_busy, 'START on busy bus'
                self.events.append(('start',))
                self.stage = 'address'
                self.control = 0xE0
                self.status = 0x84
                self.writes = 0
            elif value & 4:
                assert self.control & 0x40 and not self.stop_pending, 'invalid repeated START'
                self.events.append(('restart',))
                self.stage = 'address'
                self.status = 0x84
                self.control = 0xE0
            elif value & 8:
                assert self.control & 0xC0 == 0xC0, 'STOP without bus ownership'
                self.events.append(('stop', self.stage, self.index))
                self.stop_pending = True
                if self.stage not in ('dummy', 'read') or not self.status & 0x20:
                    self.stop()
        elif off == 4:
            if not self.reset and not self.mode & 0x10:
                assert (value ^ self.mode) & 8 == 0, 'ACKBT changed while protected'
            self.mode = value
        elif off == 9:
            self.status &= value | 0xE0  # Transfer flags are hardware-owned.
        elif off == 0x12:
            assert self.status & 0x80, 'ICDRT write without TDRE'
            if self.stage == 'address':
                self.events.append(('address', value))
                if self.fault_at == 0:
                    self.fail()
                elif value & 1:
                    self.stage = 'dummy'
                    self.status = 0xA4
                    self.control = 0xC0
                else:
                    self.stage = 'write'
                    self.status = 0xC4
            else:
                assert self.stage == 'write'
                self.events.append(('tx', value))
                self.writes += 1
                if self.writes == self.fault_at:
                    self.fail()
                else:
                    self.status = 0xC4


class Emulator:
    def __init__(self, filename):
        self.uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        for addr, size in ((0, 0x180000), (0x20000000, 0x40000),
                           (0x40000000, 0x1000000), (0xE0000000, 0x100000)):
            self.uc.mem_map(addr, size, UC_PROT_ALL)
        self.sym = {}
        with open(filename, 'rb') as f:
            elf = ELFFile(f)
            for symbol in elf.get_section_by_name('.symtab').iter_symbols():
                self.sym[symbol.name] = symbol['st_value']
            # Initialize static RAM exactly from the linked image; there is no
            # clock startup or main loop in these direct public-interface calls.
            for seg in elf.iter_segments():
                if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                    self.uc.mem_write(seg['p_vaddr'], seg.data())
        self.outputs = [0xFFFF] * 9
        self.gpio = []
        self.held_sda = False
        self.held_scl = False
        self.release_after = 0
        self.pulses = 0
        self.spi = [SpiModel(self, n) for n in range(2)]
        self.iic = [IicModel(self, n) for n in range(2)]
        self.uc.hook_add(UC_HOOK_MEM_READ, self.read, begin=0x40000000, end=0x400FFFFF)
        self.uc.hook_add(UC_HOOK_MEM_WRITE, self.write, begin=0x40000000, end=0x400FFFFF)
        self.clocks(32000000, 32000000, 1)
        self.put(MSTPB, 0xFFFFFFFF, 4)
        self.put(self.sym['SystemMicroSecLoopCnt'], 1, 4)

    def get(self, addr, size=1):
        return int.from_bytes(self.uc.mem_read(addr, size), 'little')

    def put(self, addr, value, size=1):
        self.uc.mem_write(addr, value.to_bytes(size, 'little'))

    def clocks(self, iclk, source, pckb_div):
        self.put(self.sym['SystemCoreClock'], iclk, 4)
        self.put(self.sym['s_PeriphSrcFreq'], source, 4)
        self.put(SCKDIVCR, pckb_div << 8, 4)

    def read(self, uc, access, addr, size, value, data):
        for model in self.spi + self.iic:
            if model.base <= addr < model.base + 0x20:
                model.read(addr - model.base, size)
                return
        if PORT <= addr < PORT + 9 * 0x20 and (addr - PORT) % 0x20 == 6:
            no = (addr - PORT) // 0x20
            levels = self.outputs[no]
            if no == 2:
                if self.held_sda:
                    levels &= ~1
                if self.held_scl:
                    levels &= ~2
            self.put(addr, levels, size)

    def write(self, uc, access, addr, size, value, data):
        for model in self.spi + self.iic:
            if model.base <= addr < model.base + 0x20:
                model.write(addr - model.base, size, value)
                return
        if PORT <= addr < PORT + 9 * 0x20:
            no, off = divmod(addr - PORT, 0x20)
            if off in (8, 10):
                old = self.outputs[no]
                self.outputs[no] = old & ~value if off == 8 else old | value
                self.put(PORT + no * 0x20, self.outputs[no], 2)  # PODR alias.
                self.gpio.append((no, off == 10, value))
                if no == 2 and off == 10 and value & 2 and not old & 2:
                    self.pulses += 1
                    if self.release_after and self.pulses >= self.release_after:
                        self.held_sda = False
        if PFS <= addr < PFS + 9 * 16 * 4:
            no, pin = divmod((addr - PFS) // 4, 16)
            old = self.outputs[no]
            self.outputs[no] = old | (1 << pin) if value & 1 else old & ~(1 << pin)
            self.put(PORT + no * 0x20, self.outputs[no], 2)
            if no < 2 and pin in (3, 4) and value & 4:
                assert value & 1, 'CS must be preloaded high before enabling its output'
            if no in (2, 3) and value & 4:
                assert value & 0x40 and not value & 0x80, 'GPIO bus clear must stay N-channel open-drain'
                assert value & 1, 'bus clear must begin with released SDA/SCL'

    def call(self, name, *args):
        self.uc.reg_write(UC_ARM_REG_SP, 0x2003F000)
        self.uc.reg_write(UC_ARM_REG_LR, STOP | 1)
        for reg, arg in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2, UC_ARM_REG_R3), args):
            self.uc.reg_write(reg, arg & 0xFFFFFFFF)
        self.uc.emu_start(self.sym[name] | 1, STOP, count=30000000)
        assert self.uc.reg_read(UC_ARM_REG_PC) == STOP, name + ' exceeded the instruction budget'
        value = self.uc.reg_read(UC_ARM_REG_R0)
        return value if value < 0x80000000 else value - 0x100000000

    def op(self, bus, op, arg=0, length=0):
        return self.call('SerialCall', bus, op, arg, length)

    def init(self, bus, rate=100000, options=None):
        return self.call('SerialInit', bus, rate, (8 if bus < 2 else 0) if options is None else options)

    def buffers(self):
        self.uc.mem_write(self.sym['SerialTx'], bytes(range(130)))
        self.uc.mem_write(self.sym['SerialRx'], b'\xCC' * 130)

    def received(self, count):
        addr = self.sym['SerialRx']
        assert self.get(addr) == 0xCC and self.get(addr + count + 1) == 0xCC, 'receive buffer overrun'
        return bytes(self.uc.mem_read(addr + 1, count))


def test_spi(filename):
    e = Emulator(filename)
    assert e.init(0, 1000000) and not e.op(0, 24) and e.init(1, 1000000)
    assert not e.op(0, 23), 'duplicate SPI pins'
    assert e.get(MSTPB, 4) == 0xFFF3FFFF
    assert e.op(0, 19) and e.op(1, 19)
    assert not e.op(0, 18) and not e.op(1, 18), 'controller ownership'
    for bits in range(8, 17):
        for mode in range(4):
            assert e.init(0, 1000000, bits | ((mode & 1) << 5) | ((mode & 2) << 5) | 0x80)
            cmd = e.get(SPI_BASE[0] + 0x10, 2)
            assert cmd & 3 == mode and cmd & 0x1000
            m = e.spi[0]
            m.frames.clear()
            m.rx = [0x155, 0xABCD, 0x3AA, 0x5678, 0x9ABC, 0xDEF0]
            e.gpio.clear()
            e.buffers()
            size = 1 if bits == 8 else 2
            assert e.op(0, 2, 1, (size << 16) | (size * 3)) == size * 3
            assert [f['tx'] for f in m.frames] == ([1] if size == 1 else [0x201]) + [0xA5 if size == 1 else 0xA5A5] * 3
            assert all(f['drained'] and f['cs'] == 8 for f in m.frames)
            assert e.gpio == [(0, False, 16), (0, True, 16)], 'CS must span command and response'
            expected = b''.join((v & ((1 << bits) - 1)).to_bytes(size, 'little') for v in (0xABCD, 0x3AA, 0x5678))
            assert e.received(size * 3) == expected
            assert e.op(0, 22) == 0x155 & ((1 << bits) - 1)
            assert not e.op(0, 16)
    # Real divider equation, independent clocks and representability boundaries.
    for iclk in (2000000, 24000000, 32000000, 64000000):
        e.clocks(iclk, 64000000, 2)
        for rate in (1, (iclk + 4095) // 4096, 123456, 1000000, 40000000, 0xFFFFFFFF):
            actual = e.op(0, 10, rate)
            if not actual:
                assert rate * 4096 < iclk
                continue
            cmd = e.get(SPI_BASE[0] + 0x10, 2)
            divisor = 2 * (e.get(SPI_BASE[0] + 10) + 1) * (1 << ((cmd >> 2) & 3))
            assert actual == iclk // divisor and iclk <= rate * divisor
    assert not e.op(0, 10, 0)
    e.clocks(2000000, 2000000, 0)
    assert e.init(0, 100000, 16)
    e.buffers()
    e.spi[0].frames.clear()
    assert e.op(0, 2, 0, (1 << 16) | 4) == 0 and not e.spi[0].frames, 'odd prefix must not start response'
    assert not e.op(0, 4, 2) and not e.op(0, 16), 'invalid CS releases generic busy flag'
    assert e.op(0, 4, 0) and not e.op(0, 4, 0), 'generic busy ownership'
    assert not e.op(0, 10, 50000) and not e.init(0, 100000, 8)
    e.op(0, 6)
    assert not e.op(0, 16)
    assert e.init(0, 100000, 8)
    for fault in ('error', 'stall'):
        m = e.spi[0]
        m.frames.clear()
        m.fault, m.fault_at = fault, 2
        assert e.op(0, 0, 0, 4) == 1
        assert e.outputs[0] & 0x18 == 0x18 and not e.op(0, 16)
        m.fault_at = 0
        assert e.op(0, 0, 0, 4) == 4
    assert e.op(0, 12) == 2 and e.op(0, 13) == 1 and e.op(0, 13) == 0
    assert not e.op(0, 4) and not e.op(0, 16)
    assert e.op(0, 12) == 1
    assert e.init(0, 100000, 8 | 0x100)
    e.gpio.clear()
    assert e.op(0, 0, 99, 4) == 4 and not e.gpio, 'manual CS is application owned'
    assert e.op(0, 20), 'unsupported PHY must not silently change configuration'
    for options in (7, 17, 8 | 0x200, 8 | 0x400, 8 | 0x800, 8 | 0x1000):
        assert not e.init(0, 100000, options)
    assert e.op(0, 15) and e.op(1, 15)
    assert e.get(MSTPB, 4) == 0xFFFFFFFF
    assert e.init(0, 100000, 8) and e.op(0, 15)
    e.spi[0].frames.clear()
    assert e.call('SerialCppProbe', 0) == 4
    print('SPI: modes, widths, clocks, CS, errors, ownership and C++ dispatch passed', flush=True)


def test_i2c(filename):
    e = Emulator(filename)
    assert e.init(2) and not e.op(2, 24) and e.init(3)
    assert not e.op(2, 23, 2) and not e.op(2, 23, 4), 'I2C pull-down/invalid resistor'
    assert e.get(MSTPB, 4) == 0xFFFFFCFF
    assert e.op(2, 19) and e.op(3, 19) and not e.op(2, 18) and not e.op(3, 18)
    for no in range(2):
        bus, m = no + 2, e.iic[no]
        for length in (1, 2, 3, 4, 17):
            m.rx = bytes(range(0x40, 0x40 + length))
            m.events.clear()
            m.stretch_reads = 5
            e.buffers()
            assert e.op(bus, 2, 0x50, (2 << 16) | length) == length
            assert e.received(length) == m.rx
            assert m.events[:6] == [('start',), ('address', 0xA0), ('tx', 1), ('tx', 2), ('restart',), ('address', 0xA1)]
            assert sum(v[0] == 'stop' for v in m.events) == 1
            assert m.events[-2][0] == 'stop' and m.events[-1] == ('rx', m.rx[-1], True)
            assert m.stage == 'idle' and m.mode == 1 and not e.op(bus, 16)
        m.events.clear()
        e.buffers()
        assert e.op(bus, 3, 0x50, (2 << 16) | 3) == 3
        assert m.events[:7] == [('start',), ('address', 0xA0), ('tx', 1), ('tx', 2), ('tx', 16), ('tx', 17), ('tx', 18)]
        m.rx = b'\x12\x34'
        m.events.clear()
        assert e.op(bus, 1, 0x50, 2) == 2
        assert m.events[:3] == [('start',), ('address', 0xA1), ('dummy',)]
    for pclk in (2000000, 8000000, 16000000, 32000000):
        e.clocks(64000000, pclk * 2, 1)
        for rate in (1000, 10000, 50000, 100000, 123456, 400000):
            actual = e.op(2, 10, rate)
            if not actual:
                assert rate * 8960 < pclk
                continue
            base = IIC_BASE[0]
            cks = (e.get(base + 2) >> 4) & 7
            brl, brh = e.get(base + 16), e.get(base + 17)
            assert brl & 0xE0 == 0xE0 and brh & 0xE0 == 0xE0
            offset = 5 if cks == 0 else 4
            low = ((brl & 31) + offset) * (1 << cks)
            high = ((brh & 31) + offset) * (1 << cks)
            assert actual == pclk // (low + high) and pclk <= rate * (low + high)
            ln, hn = (4700, 4000) if rate <= 100000 else (1300, 600)
            assert low * 1000000000 >= ln * pclk and high * 1000000000 >= hn * pclk
    assert not e.op(2, 10, 0) and not e.op(2, 10, 400001)
    e.clocks(2000000, 2000000, 0)
    assert e.init(2)
    m = e.iic[0]
    for fault in ('nack', 'al', 'stall'):
        for at in (0, 2):
            m.fault, m.fault_at = fault, at
            m.events.clear()
            e.buffers()
            # A failed register prefix must not produce a read address.
            assert e.op(2, 2, 0x50, (3 << 16) | 4) == 0
            assert not any(v == ('address', 0xA1) for v in m.events)
            assert not e.op(2, 16)
            if fault == 'al':
                assert not any(v[0] == 'stop' for v in m.events), 'arbitration loss must not issue STOP'
            m.fault_at = -1
            m.external_busy = False
            m.control = 0
            assert e.op(2, 0, 0x50, 4) == 4
    m.fault, m.fault_at = 'nack', 3
    assert e.op(2, 0, 0x50, 5) == 2, 'count only acknowledged data bytes'
    m.fault_at = -1
    m.rx = b'\x31\x32\x33\x34'
    m.rx_fault_at = 2
    m.fault = 'stall'
    e.buffers()
    assert e.op(2, 1, 0x50, 4) == 2 and e.received(2) == m.rx[:2]
    m.rx_fault_at = -1
    assert e.op(2, 0, 0x50, 4) == 4
    m.stop_stall = True
    assert e.op(2, 0, 0x50, 4) == 4
    assert m.stage == 'idle', 'stalled STOP must reset the local engine'
    m.stop_stall = False
    assert e.op(2, 0, 0x50, 4) == 4
    m.external_busy = True
    m.events.clear()
    assert not e.op(2, 4, 0x50) and not e.op(2, 16) and not m.events
    m.external_busy = False
    assert not e.op(2, 4, 0x80) and not e.op(2, 16)
    assert e.op(2, 4, 0x50) and not e.op(2, 4, 0x50)
    assert not e.op(2, 10, 50000) and not e.init(2)
    e.op(2, 6)
    assert not e.op(2, 16)
    assert e.op(2, 12) == 2 and e.op(2, 13) == 1 and e.op(2, 13) == 0
    assert not e.op(2, 4, 0x50) and e.op(2, 12) == 1
    # Explicit bus clear honors open-drain and a slave holding SCL low.
    e.held_sda, e.release_after, e.pulses = True, 3, 0
    e.op(2, 14)
    assert not e.held_sda and e.pulses == 4  # Three clocks, then STOP setup.
    assert e.get(PFS + 2 * 64, 4) >> 24 == 15
    assert e.get(PFS + 2 * 64 + 4, 4) >> 24 == 15
    e.held_scl, e.pulses = True, 0
    e.op(2, 14)
    assert e.pulses == 0
    e.held_scl = False
    e.held_sda, e.release_after, e.pulses = True, 0, 0
    e.op(2, 14)
    assert e.held_sda and e.pulses == 10, 'bus clear must stop after nine recovery clocks'
    e.held_sda = False
    assert e.op(2, 13) == 0
    e.op(2, 14)
    assert not e.get(IIC_BASE[0]) & 0x80 and e.op(2, 17) == 0
    assert e.op(2, 12) == 1
    assert e.op(2, 0, 0x50, 4) == 4
    for options in (1, 2, 4, 8, 16):
        assert not e.init(2, options=options)
    assert not e.init(2, 400001)
    assert e.op(2, 15) and e.op(3, 15)
    assert e.get(MSTPB, 4) == 0xFFFFFFFF
    assert e.init(2) and e.op(2, 15)
    m.events.clear()
    m.rx = b'\x40\x41\x42\x43'
    assert e.call('SerialCppProbe', 2) == 4
    print('I2C: timing, repeated START, ACK/STOP, faults, bus clear and C++ dispatch passed', flush=True)


if __name__ == '__main__':
    test_spi(sys.argv[1])
    test_i2c(sys.argv[1])
