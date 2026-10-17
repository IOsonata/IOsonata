#!/usr/bin/env python3
"""STM32WBA65 SPI master driver test on an instruction set emulator.

Run: python3 tests/stm32wba/run_spi.py

Needs arm-none-eabi-g++, and the python packages unicorn and pyelftools.
Without them the test prints NOT RUN.

spi_fw.cpp is built for the Cortex-M33 with the generic STM32 SPI driver
ARM/ST/src/spi_stm32.cpp, the generic SPI and device interface
sources and the STM32 GPIO driver, and run in unicorn. The SPI1 and SPI3
registers are a model of the SPI version 2 master (CFG1, CFG2, TSIZE) of the
reference manual:

- CSTART with SPE set starts a transfer of TSIZE items; every TXDR write
  exchanges one item with a slave that returns (3 * n + 1) for item n
- TXP, RXP, EOT and OVR flags; the receive FIFO holds 8 bytes and an
  overrun is flagged when the driver lets more than that come in
- a received item can be read 20 status reads after it was sent, so the
  receive FIFO fills when the driver sends too far ahead of what it reads
- the GPIO BSRR writes of the chip select pins are logged with the transfers

The model flags what the reference manual does not allow: CR2, CFG1 or CFG2
written with SPE set, CSTART with SPE clear, TXDR written when no item is
left to send, RXDR read with no data, and a data register access whose width
does not match the data size.

Not hardware validation: no electrical test and no timing.

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
import collections
from pathlib import Path
import re
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
sys.path.insert(0, str(HERE))
from run_i2c import Gpdma
HDR = ROOT / 'ARM/ST/STM32WBAxx/include/stm32wba65xx.h'
FCPU = 16_000_000
RX_FIFO_BYTES = 8

_defs = {}
for line in HDR.read_text().splitlines():
    m = re.match(r'#define\s+(\w+)\s+\(?(.*?)\)?\s*(?:/\*.*)?$', line)
    if m and m.group(1) not in _defs:
        _defs[m.group(1)] = m.group(2)


def hdr(name):
    """Value of a numeric #define of the device header."""
    expr = _defs[name]
    expr = re.sub(r'\b(0x[0-9A-Fa-f]+|\d+)U?L?L?\b', r'\1', expr)
    expr = re.sub(r'\(\(?uint32_t\)?\s*', '(', expr)

    def sub(m):
        return str(hdr(m.group(0))) if m.group(0) in _defs else m.group(0)
    expr = re.sub(r'\b[A-Za-z_]\w*\b', sub, expr)
    return eval(expr)


def rcc_off(name):
    """Offset of an RCC register from its comment in the device header."""
    m = re.search(r'__IO uint32_t %s;\s*/\*!<.*?Address offset: (0x[0-9A-Fa-f]+)' % name, HDR.read_text())
    return int(m.group(1), 16)


def run(args, **kw):
    p = subprocess.run([str(a) for a in args], cwd=ROOT, text=True,
                       stdout=subprocess.PIPE, stderr=subprocess.STDOUT, **kw)
    if p.returncode != 0:
        raise SystemExit(f'FAIL ({p.returncode}): {" ".join(map(str, args))}\n{p.stdout}')
    return p.stdout


def build(tmp):
    cpu = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', '-Os', '-DSTM32WBA65xx',
           '-ffunction-sections', '-fdata-sections', '-g']
    inc = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
           '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/ST/include'),
           '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include')]
    objs = []
    cxx = ['arm-none-eabi-g++', *cpu, '-std=gnu++23', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', *inc]
    for src in (HERE / 'spi_fw.cpp', ROOT / 'ARM/ST/src/spi_stm32.cpp',
                ROOT / 'ARM/ST/STM32WBAxx/src/gpdma_stm32wba.cpp',
                ROOT / 'src/coredev/spi.cpp', ROOT / 'src/device_intrf.cpp',
                ROOT / 'ARM/ST/src/iopincfg_stm32.cpp'):
        obj = tmp / (src.stem + '.o')
        run([*cxx, '-c', src, '-o', obj])
        objs.append(obj)
    obj = tmp / 'system_stm32wbaxx.o'
    run(['arm-none-eabi-gcc', *cpu, '-std=gnu17', *inc, '-c',
         ROOT / 'ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c', '-o', obj])
    objs.append(obj)
    elf = tmp / 'spi_fw.elf'
    run(['arm-none-eabi-g++', *cpu, '-nostartfiles', '-Wl,--gc-sections', '-Wl,-T,' + str(HERE / 'spi_fw.ld'),
         '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
    return elf


class Log:
    def __init__(self):
        self.ev = []
        self.mark = -1

    def add(self, e):
        self.ev.append((self.mark, *e))

    def of(self, mark, names=('SPI1', 'GPIO')):
        return [e[1:] for e in self.ev if e[0] == mark and e[1] in names]


class Spi:
    """SPI version 2 master model."""

    SR_RXP = 1 << 0
    SR_TXP = 1 << 1
    SR_EOT = 1 << 3
    SR_UDR = 1 << 5
    SR_OVR = 1 << 6

    def __init__(self, name, log, violations, req_tx=0, req_rx=0):
        self.name = name
        self.req_tx = req_tx
        self.req_rx = req_rx
        self.gpdma = None
        self.in_dma = False
        self.script = None
        self.stxq = []
        self.sent_by_slave = []
        self.log = log
        self.viol = violations
        self.reg = collections.defaultdict(int)
        self.cfg1w = []
        self.cfg2w = []
        self.active = False
        self.tsize = 0
        self.sent = 0
        self.rxq = []
        self.flags = 0
        self.polls = 0

    def v(self, what):
        self.viol.append(f'{self.name}: {what} (mark {self.log.mark})')

    def spe(self):
        return bool(self.reg[0x00] & 1)

    def item_bytes(self):
        return 2 if (self.reg[0x08] & 0x1F) + 1 > 8 else 1

    def mask(self):
        return (1 << ((self.reg[0x08] & 0x1F) + 1)) - 1

    def slave(self):
        return not (self.reg[0x0C] & hdr('SPI_CFG2_MASTER'))

    def dma_service(self):
        """The GPDMA channels move the items of the TXP and RXP requests."""
        if self.gpdma is None or self.in_dma or not self.spe():
            return
        self.in_dma = True
        ib = self.item_bytes()
        cfg1 = self.reg[0x08]
        progress = True
        while progress:
            progress = False
            if cfg1 & hdr('SPI_CFG1_TXDMAEN') and self.gpdma.find(self.req_tx) is not None:
                if self.slave():
                    if len(self.stxq) * ib < RX_FIFO_BYTES:
                        self.stxq.append(self.gpdma.take(self.req_tx, ib))
                        progress = True
                elif self.active and self.sent < self.tsize and len(self.rxq) * ib < RX_FIFO_BYTES:
                    self.write_txdr(ib, self.gpdma.take(self.req_tx, ib))
                    progress = True
            if cfg1 & hdr('SPI_CFG1_RXDMAEN') and self.gpdma.find(self.req_rx) is not None:
                if self.rxq and self.rxq[0][1] <= self.polls:
                    self.gpdma.put(self.req_rx, self.rxq.pop(0)[0], ib)
                    progress = True
        self.in_dma = False

    def slave_script(self, frames):
        mask = self.mask()
        for fr in frames:
            while not self.spe():
                yield
            self.log.add((self.name, 'F', len(fr)))
            self.sent_by_slave.append([])
            for b in fr:
                for _ in range(5):
                    if not self.rxq:
                        break
                    yield
                if self.stxq:
                    t = self.stxq.pop(0)
                else:
                    t = self.reg[0x4C] & mask
                    self.flags |= self.SR_UDR
                self.sent_by_slave[-1].append(t)
                if len(self.rxq) * self.item_bytes() >= RX_FIFO_BYTES:
                    self.flags |= self.SR_OVR
                else:
                    self.rxq.append((b & mask, 0))
                self.dma_service()
                yield
            for _ in range(20):
                if not self.rxq:
                    break
                self.dma_service()
                yield
            self.flags |= self.SR_EOT
            self.log.add((self.name, 'E'))
            while self.flags & self.SR_EOT:
                yield

    def step(self):
        if self.script is not None:
            try:
                next(self.script)
            except StopIteration:
                self.script = None

    def rd_reg(self, off, size):
        if off == 0x14:
            self.polls += 1
            self.dma_service()
            v = self.flags
            if self.rxq and self.rxq[0][1] <= self.polls:
                v |= self.SR_RXP
            if self.slave():
                if self.spe() and len(self.stxq) * self.item_bytes() < RX_FIFO_BYTES:
                    v |= self.SR_TXP
            elif self.active:
                v |= self.SR_TXP
            return v
        if off == 0x30:
            if not self.rxq or self.rxq[0][1] > self.polls:
                self.v('RXDR read with no data')
                return 0
            if size != self.item_bytes():
                self.v(f'RXDR read of {size} bytes, data size needs {self.item_bytes()}')
            return self.rxq.pop(0)[0]
        return self.reg[off]

    def wr_reg(self, off, size, val):
        if off == 0x00:
            if (self.reg[0] & 1) and not (val & 1):
                self.stop()
            if val & (1 << 9):
                if not (val & 1):
                    self.v('CSTART with SPE clear')
                else:
                    self.start()
                val &= ~(1 << 9)
            self.reg[0] = val
            self.dma_service()
        elif off in (0x04, 0x08, 0x0C):
            if self.spe():
                self.v(f'register {off:#x} written with SPE set')
            self.reg[off] = val
            if off == 0x08:
                self.cfg1w.append(val)
            if off == 0x0C:
                self.cfg2w.append(val)
        elif off == 0x18:
            if val & (1 << 3):
                self.flags &= ~self.SR_EOT
            if val & (1 << 6):
                self.flags &= ~self.SR_OVR
            if val & (1 << 5):
                self.flags &= ~self.SR_UDR
        elif off == 0x20:
            self.write_txdr(size, val)
        else:
            self.reg[off] = val

    def start(self):
        self.tsize = self.reg[0x04] & 0xFFFF
        self.sent = 0
        self.active = True
        self.rxq = []
        self.flags &= ~self.SR_EOT
        self.log.add((self.name, 'T', self.tsize))
        if self.tsize == 0:
            self.v('transfer of 0 items')
        self.dma_service()

    def stop(self):
        if self.active and self.sent < self.tsize:
            self.log.add((self.name, 'ABORT', self.sent))
            self.v('SPE cleared before the end of the transfer')
        self.active = False
        self.rxq = []
        self.stxq = []
        self.flags = 0

    def write_txdr(self, size, val):
        if self.slave():
            if size != self.item_bytes():
                self.v(f'TXDR write of {size} bytes, data size needs {self.item_bytes()}')
            self.stxq.append(val)
            return
        if not self.active or self.sent >= self.tsize:
            self.v('TXDR written with no item left to send')
            return
        if size != self.item_bytes():
            self.v(f'TXDR write of {size} bytes, data size needs {self.item_bytes()}')
        rx = (3 * self.sent + 1) & self.mask()
        self.log.add((self.name, 'X', val, rx))
        self.rxq.append((rx, self.polls + 20))
        if len(self.rxq) * self.item_bytes() > RX_FIFO_BYTES:
            self.flags |= self.SR_OVR
            self.v('receive FIFO overrun')
        self.sent += 1
        if self.sent == self.tsize:
            self.flags |= self.SR_EOT
            self.active_done = True


class Gpio:
    """BSRR of GPIOA and GPIOB, logged for the chip select pins."""

    WATCH = {('A', 12), ('A', 11)}

    def __init__(self, log):
        self.log = log

    def bsrr(self, port, val):
        for pin in range(16):
            if (port, pin) not in self.WATCH:
                continue
            if val & (1 << pin):
                self.log.add(('GPIO', f'{port}{pin}', 1))
            elif val & (1 << (pin + 16)):
                self.log.add(('GPIO', f'{port}{pin}', 0))


def main():
    arm = shutil.which('arm-none-eabi-g++')
    try:
        from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
        from unicorn.arm_const import UC_CPU_ARM_CORTEX_M33
        from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_SP, UC_ARM_REG_PC
        from elftools.elf.elffile import ELFFile
    except ImportError as e:
        print(f'NOT RUN: {e}. pip install unicorn pyelftools')
        return
    if not arm:
        print('NOT RUN: arm-none-eabi-g++ not found')
        return

    with tempfile.TemporaryDirectory(prefix='wba-spi-') as tmp:
        elf_path = build(Path(tmp))
        elf = ELFFile(open(elf_path, 'rb'))
        syms = {s.name: s['st_value'] for s in elf.get_section_by_name('.symtab').iter_symbols()}

        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.ctl_set_cpu_model(UC_CPU_ARM_CORTEX_M33)
        uc.mem_map(0x08000000, 0x200000)
        uc.mem_map(0x20000000, 0x80000)
        uc.mem_map(0xE0000000, 0x100000)
        for seg in elf.iter_segments():
            if seg['p_type'] == 'PT_LOAD' and seg['p_filesz']:
                uc.mem_write(seg['p_paddr'], seg.data())

        log = Log()
        viol = []
        gpdma = Gpdma(viol)
        gpdma.uc = uc
        spi1 = Spi('SPI1', log, viol, 2, 1)
        spi3 = Spi('SPI3', log, viol, 4, 3)
        spi2 = Spi('SPI2', log, viol, 53, 52)
        for d in (spi1, spi2, spi3):
            d.gpdma = gpdma
        gpdma.notify = lambda: [d.dma_service() for d in (spi1, spi2, spi3)]
        gbase = hdr('GPDMA1_BASE_NS')
        gpio = Gpio(log)
        devs = {hdr('SPI1_BASE_NS'): spi1, hdr('SPI2_BASE_NS'): spi2, hdr('SPI3_BASE_NS'): spi3}
        gpio_base = {hdr('GPIOA_BASE_NS'): 'A', hdr('GPIOB_BASE_NS'): 'B'}
        regs = collections.defaultdict(int)
        rcc_log = []
        rcc = hdr('RCC_BASE_NS')

        def rd(uc, off, size, base):
            a = base + off
            d = devs.get(a & ~0xFF)
            if d is not None:
                return d.rd_reg(a & 0xFF, size)
            if gbase <= a < gbase + 0x1000:
                return gpdma.rd(a - gbase)
            return regs[a]

        def wr(uc, off, size, val, base):
            a = base + off
            d = devs.get(a & ~0xFF)
            if d is not None:
                d.wr_reg(a & 0xFF, size, val)
                return
            if gbase <= a < gbase + 0x1000:
                gpdma.wr(a - gbase, val)
                return
            g = gpio_base.get(a & ~0x3FF)
            if g is not None and (a & 0x3FF) == 0x18:
                gpio.bsrr(g, val)
            if rcc <= a < rcc + 0x400:
                rcc_log.append((a - rcc, val))
            regs[a] = val

        for b, s in ((0x40000000, 0x40000), (0x42020000, 0x10000), (0x46000000, 0x30000)):
            uc.mmio_map(b, s, rd, b, wr, b)

        frames = [bytes(0x10 + i for i in range(5)), bytes(0x20 + i for i in range(12)),
                  bytes(0x30 + i for i in range(25)), bytes([0x40])]
        dma_at = {}
        cfg1_at12 = []
        run1 = []

        def read_slave(uc):
            n = int.from_bytes(uc.mem_read(syms['g_SpiTestSlvCnt'], 4), 'little')
            ev = [int.from_bytes(uc.mem_read(syms['g_SpiTestSlvEvt'] + 4 * i, 4), 'little', signed=True)
                  for i in range(n * 2)]
            sn = [bytes(uc.mem_read(syms['g_SpiTestSlvSnap'] + 24 * i, 24)) for i in range(4)]
            return ev, sn

        def on_mark(uc, addr, size, user):
            log.mark = uc.reg_read(UC_ARM_REG_R0)
            dma_at[log.mark] = gpdma.moved
            if log.mark == 12:
                cfg1_at12.append(len(spi1.cfg1w))
            if log.mark == 16:
                run1.append(read_slave(uc))
            if log.mark in (15, 16):
                spi3.script = spi3.slave_script(frames)

        def on_step(uc, addr, size, user):
            spi3.step()

        state = {'done': False}

        def on_done(uc, addr, size, user):
            state['done'] = True
            uc.emu_stop()

        uc.hook_add(UC_HOOK_CODE, on_mark, begin=syms['SpiTestMark'] & ~1, end=syms['SpiTestMark'] & ~1)
        uc.hook_add(UC_HOOK_CODE, on_step, begin=syms['SpiTestSlaveStep'] & ~1, end=syms['SpiTestSlaveStep'] & ~1)
        uc.hook_add(UC_HOOK_CODE, on_done, begin=syms['SpiTestDoneHook'] & ~1, end=syms['SpiTestDoneHook'] & ~1)

        vec = bytes(uc.mem_read(0x08000000, 8))
        uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
        pc = int.from_bytes(vec[4:8], 'little')
        try:
            uc.emu_start(pc, 0xFFFFFFFF, count=400_000_000)
        except Exception as ex:
            raise SystemExit(f'FAIL emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex}')
        if not state['done']:
            raise SystemExit('FAIL firmware did not finish')

        res_addr = syms['g_SpiTestResult']
        n = int.from_bytes(uc.mem_read(res_addr + 4 * 63, 4), 'little', signed=True)
        res = [int.from_bytes(uc.mem_read(res_addr + 4 * i, 4), 'little', signed=True) for i in range(n)]
        rx = bytes(uc.mem_read(syms['g_SpiTestRx'], 512))
        rx2 = [bytes(uc.mem_read(syms['g_SpiTestRx2'] + 320 * i, 320)) for i in range(4)]
        slv_n = int.from_bytes(uc.mem_read(syms['g_SpiTestSlvCnt'], 4), 'little')
        slv = [int.from_bytes(uc.mem_read(syms['g_SpiTestSlvEvt'] + 4 * i, 4), 'little', signed=True)
               for i in range(slv_n * 2)]
        slv_snap = [bytes(uc.mem_read(syms['g_SpiTestSlvSnap'] + 24 * i, 24)) for i in range(4)]

    errors = []
    checks = 0

    def expect(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            errors.append(msg)

    P = lambda m, names=('SPI1', 'GPIO'): log.of(m, names)
    tx = bytes((i * 7 + 3) & 0xFF for i in range(512))

    def X(txv, n, mask=0xFF):
        return ('SPI1', 'X', txv, (3 * n + 1) & mask)

    def cs_frame(pin, tsize, items):
        return [('GPIO', pin, 0), ('SPI1', 'T', tsize), *items, ('GPIO', pin, 1)]

    # 0 init: chip selects high first, rate
    expect(res[0] == 1, 'SPIInit failed')
    expect(res[1] == 1000000, f'rate at 1 MHz request: {res[1]}')
    e0 = P(0)
    expect(e0[:2] == [('GPIO', 'A12', 1), ('GPIO', 'A11', 1)], f'chip selects not set high at init: {e0[:4]}')

    # 1 transmit 5 bytes
    expect(P(1) == cs_frame('A12', 5, [X(tx[i], i) for i in range(5)]), f'tx events {P(1)}')
    expect(res[2] == 5, f'tx result {res[2]}')

    # 2 receive 6 bytes with the dummy byte
    exp = [X(0xA5, i) for i in range(6)]
    expect(P(2) == cs_frame('A12', 6, exp), f'rx events {P(2)}')
    expect(res[3] == 6 and rx[:6] == bytes((3 * i + 1) & 0xFF for i in range(6)), f'rx result {res[3]} data {rx[:6].hex()}')

    # 3 command then read, one chip select frame
    exp = [('GPIO', 'A12', 0), ('SPI1', 'T', 1), X(0x9F, 0), ('SPI1', 'T', 4)] + \
          [X(0xA5, i) for i in range(4)] + [('GPIO', 'A12', 1)]
    expect(P(3) == exp, f'read events {P(3)}')
    expect(res[4] == 4 and rx[16:20] == bytes((3 * i + 1) & 0xFF for i in range(4)), f'read result {res[4]} data {rx[16:20].hex()}')

    # 4 command and data in one transfer
    data = [0x02, 0x10, tx[0], tx[1], tx[2]]
    expect(P(4) == cs_frame('A12', 5, [X(data[i], i) for i in range(5)]), f'write events {P(4)}')
    expect(res[5] == 3, f'write result {res[5]} (3 data bytes)')

    # 5 70000 bytes in two hardware transfers
    e5 = P(5)
    tsz = [x[2] for x in e5 if x[:2] == ('SPI1', 'T')]
    items = [x for x in e5 if x[:2] == ('SPI1', 'X')]
    expect(tsz == [65535, 4465], f'70000 byte transfer sizes {tsz}')
    expect(len(items) == 70000 and all(x[2] == (i & 0xFF) for i, x in enumerate(items)), '70000 byte data differ')
    expect(e5[0] == ('GPIO', 'A12', 0) and e5[-1] == ('GPIO', 'A12', 1) and
           sum(1 for x in e5 if x[0] == 'GPIO') == 2, 'chip select around the 70000 bytes')
    expect(res[6] == 70000, f'70000 byte result {res[6]}')

    # 6 second chip select, and one that does not exist
    expect(P(6) == cs_frame('A11', 2, [X(tx[0], 0), X(tx[1], 1)]), f'chip select 1 events {P(6)}')
    expect(res[7] == 2 and res[8] == 0, f'chip select results {res[7:9]}')

    # 7 rates
    expect(res[9] == 125000, f'100 kHz request gave {res[9]}')
    expect(res[10] == 8000000, f'50 MHz request gave {res[10]}')
    expect(res[11] == 1000000, f'rate 0 gave {res[11]}')
    mbr = [(v >> 28) & 7 for v in spi1.cfg1w]
    expect(mbr[:5] == [0, 3, 6, 0, 3], f'MBR values {mbr[:5]}')
    expect(cfg1_at12[0] == 9, f'CFG1 written {cfg1_at12[0]} times, 9 expected (not per transfer)')

    # 8 mode 3, LSB first, 12 bit data
    expect(res[12] == 1 and res[13] == 6 and res[14] == 4, f'12 bit results {res[12:15]}')
    it = [tx[0] | tx[1] << 8, tx[2] | tx[3] << 8, tx[4] | tx[5] << 8]
    e8 = P(8)
    expect(e8[:4] == [('GPIO', 'A12', 1), ('GPIO', 'A11', 1), ('GPIO', 'A12', 0), ('SPI1', 'T', 3)] or
           e8[:2] == [('GPIO', 'A12', 1), ('GPIO', 'A11', 1)], f'12 bit events {e8[:6]}')
    xs = [x for x in e8 if x[:2] == ('SPI1', 'X')]
    expect([x[2] for x in xs] == it + [0x5A5A, 0x5A5A], f'12 bit tx items {[hex(x[2]) for x in xs]}')
    expect(rx[32:36] == bytes([1, 0, 4, 0]), f'12 bit rx {rx[32:36].hex()}')
    cfg2 = spi1.cfg2w
    want = (hdr('SPI_CFG2_MASTER') | hdr('SPI_CFG2_SSM') | hdr('SPI_CFG2_AFCNTR') | hdr('SPI_CFG2_CPOL') |
            hdr('SPI_CFG2_CPHA') | hdr('SPI_CFG2_LSBFRST'))
    expect(want in cfg2, f'CFG2 for mode 3 LSB first: {[hex(v) for v in cfg2]}')
    expect(cfg2[0] == (hdr('SPI_CFG2_MASTER') | hdr('SPI_CFG2_SSM') | hdr('SPI_CFG2_AFCNTR')), f'CFG2 for mode 0: {cfg2[0]:#x}')
    expect(any((v & 0x1F) == 11 for v in spi1.cfg1w), 'DSIZE 11 not written')
    expect(any((v & 0x1F) == 7 for v in spi1.cfg1w), 'DSIZE 7 not written')

    # 9 chip select left to the application
    e9 = P(9)
    expect([x for x in e9 if x[0] == 'GPIO' and x[2] == 0] == [] , f'chip select driven with SPICSEL_MAN: {e9}')
    expect(('SPI1', 'T', 3) in e9 and res[15] == 1 and res[16] == 3, f'manual chip select results {res[15:17]}')

    # 10 slave, interrupt and DMA are accepted, the others are refused
    expect(res[17:24] == [1, 0, 1, 1, 0, 0, 0], f'configurations {res[17:24]}')

    # 11 SPI3
    expect(res[24] == 1 and res[25] == 8000000 and res[26] == 3 and res[27] == 3, f'SPI3 results {res[24:28]}')
    e3 = log.of(11, ('SPI3',))
    expect([x for x in e3 if x[1] == 'T'] == [('SPI3', 'T', 3), ('SPI3', 'T', 3)], f'SPI3 events {e3[:4]}')
    expect(not [x for x in P(11, ('GPIO',))], 'GPIO events on SPI3 without chip select')
    expect(rx[48:51] == bytes([1, 4, 7]), f'SPI3 rx {rx[48:51].hex()}')

    def seq(n):
        return bytes((3 * i + 1) & 0xFF for i in range(n))

    # 12 interrupt master
    expect(res[28:36] == [1, -1, 5, -1, 6, -1, 300, 300], f'interrupt results {res[28:36]}')
    e12 = P(12)
    k = e12.index(('GPIO', 'A12', 0))
    expect(e12[k:k + 8] == cs_frame('A12', 5, [X(tx[i], i) for i in range(5)]), f'interrupt tx events {e12[k:k + 8]}')
    expect(rx[64:70] == seq(6), f'interrupt rx data {rx[64:70].hex()}')
    expect(rx2[0][:300] == seq(300), 'interrupt rx 300 data')

    # 13 DMA master, polling wait
    expect(res[36:43] == [1, 5, 300, 300, 1, 6, 4], f'DMA polling results {res[36:43]}')
    expect(rx2[1][:300] == seq(300), 'DMA rx 300 data')
    e13 = P(13)
    xs = [x for x in e13 if x[:2] == ('SPI1', 'X')]
    it = [tx[0] | tx[1] << 8, tx[2] | tx[3] << 8, tx[4] | tx[5] << 8]
    expect([x[2] for x in xs[-5:]] == it + [0xA5A5, 0xA5A5], f'DMA 12 bit items {[hex(x[2]) for x in xs[-5:]]}')
    expect(rx[96:100] == bytes([1, 0, 4, 0]), f'DMA 12 bit rx {rx[96:100].hex()}')
    expect(dma_at[14] - dma_at[13] == 10 + 600 + 600 + 12 + 8, f'bytes moved by DMA {dma_at[14] - dma_at[13]}')

    # 14 DMA master, interrupt
    expect(res[43:48] == [1, -1, 129, -1, 129], f'DMA interrupt results {res[43:48]}')
    expect(rx2[2][:129] == seq(129), 'DMA interrupt rx data')
    expect(dma_at[15] - dma_at[14] == 129 * 4, f'bytes moved by DMA in interrupt mode {dma_at[15] - dma_at[14]}')

    # 15 and 16 slave, polling of the interrupt and DMA
    expect(res[48] == 1 and res[49] == 1, f'slave init results {res[48:50]}')
    exp_ev = [1, 0, 2, 5, 1, 0, 2, 12, 1, 0, 2, -1, 1, 0, 2, 1, 1, 0]
    for name, (ev, sn) in (('interrupt', run1[0]), ('DMA', (slv, slv_snap))):
        expect(ev == exp_ev, f'slave {name} events {ev}')
        expect(sn[0][:5] == frames[0] and sn[1][:12] == frames[1] and sn[2][:20] == frames[2][:20] and
               sn[3][0] == 0x40 and sn[3][1:20] == frames[2][1:20], f'slave {name} data {[x.hex() for x in sn]}')
    td = bytes(0xC0 + i for i in range(8))
    want = [td[:5], td + bytes([0x3C]) * 4, td + bytes([0x3C]) * 17, td[:1]]
    got = [bytes(x) for x in spi3.sent_by_slave]
    expect(got == want + want, f'slave transmit data {[g.hex() for g in got]}')
    expect(dma_at[17] - dma_at[16] == 38 + 5 * 8, f'bytes moved by the slave DMA {dma_at[17] - dma_at[16]}')

    # clocks
    def rcc_set(name, bit):
        return any(o == rcc_off(name) and v & bit for o, v in rcc_log)
    expect(rcc_set('APB2ENR', hdr('RCC_APB2ENR_SPI1EN')), 'SPI1 clock not enabled')
    expect(rcc_set('APB7ENR', hdr('RCC_APB7ENR_SPI3EN')), 'SPI3 clock not enabled')
    expect(rcc_set('APB2RSTR', hdr('RCC_APB2RSTR_SPI1RST')), 'SPI1 reset not pulsed')
    expect(rcc_set('APB7RSTR', hdr('RCC_APB7RSTR_SPI3RST')), 'SPI3 reset not pulsed')
    k1 = [v for o, v in rcc_log if o == rcc_off('CCIPR1')]
    expect(k1 and all(not v & hdr('RCC_CCIPR1_SPI1SEL_Msk') for v in k1), 'SPI1 kernel clock not PCLK2')
    k3 = [v for o, v in rcc_log if o == rcc_off('CCIPR3')]
    expect(k3 and all(not v & hdr('RCC_CCIPR3_SPI3SEL_Msk') for v in k3), 'SPI3 kernel clock not PCLK7')

    # internal slave select and CR1
    expect(spi1.reg[0x00] & hdr('SPI_CR1_SSI'), 'SSI not set')

    expect(not viol, f'model violations: {viol[:6]}')

    for m in errors:
        print('FAIL:', m)
    if errors:
        raise SystemExit(f'STM32WBA65 SPI: {len(errors)} of {checks} checks failed')
    print(f'STM32WBA65 SPI master on the emulator: {checks} checks, 0 failures')
    print('Hardware not tested.')


if __name__ == '__main__':
    main()
