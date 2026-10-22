#!/usr/bin/env python3
"""STM32WBA65 I2C driver test on an instruction set emulator.

Run: python3 tests/stm32wba/run_i2c.py

Needs arm-none-eabi-g++, and the python packages unicorn and pyelftools.
Without them the test prints NOT RUN.

i2c_fw.cpp is built for the Cortex-M33 with the generic STM32 I2C driver
ARM/ST/src/i2c_stm32.cpp, the GPDMA channel manager, the generic I2C and
device interface sources and the STM32 GPIO driver, and run in unicorn. The
I2C1 and I2C3 registers are a model of the I2C version 2 peripheral of the
reference manual:

- master: START with SADD, RD_WRN and NBYTES, the address phase against two
  slave devices (0x50, and 0x51 that refuses the third byte of a write)
- TXIS, RXNE, TCR (RELOAD), TC (software end), STOPF (AUTOEND) and NACKF
  with the automatic STOP after a NACK
- NBYTES counted down per byte, written again after TCR, the repeated START
  after TC
- slave: two own addresses, ADDR with DIR and ADDCODE, RXNE, TXIS, NACKF and
  STOPF, driven by a scripted bus master
- GPDMA1 channels with REQSEL, memory address and count, moving the bytes of
  the TXIS and RXNE requests

The runs: polling, interrupt (the vector is called from the firmware, the
model has no interrupt controller), DMA with polling, DMA with interrupt, and
the slave with the events the I2C examples expect.

The model records the bus transactions and flags what the reference manual
does not allow: TXDR written without TXIS, RXDR read without RXNE, START or
NBYTES written while a transfer is running and neither TC nor TCR is set,
TIMINGR written with PE set, a DMA channel enabled with no count. The TIMINGR
values written are checked against the minimum SCL low and high times and the
SCLDEL and SDADEL ranges.

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
HDR = ROOT / 'ARM/ST/STM32WBAxx/include/stm32wba65xx.h'
FCPU = 16_000_000

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
    cpu = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', '-Os', '-DSTM32WBA65xx', '-DIOSONATA_STM32_WBA',
           '-ffunction-sections', '-fdata-sections', '-g']
    inc = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
           '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/ST/include'),
           '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include')]
    objs = []
    cxx = ['arm-none-eabi-g++', *cpu, '-std=gnu++23', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', *inc]
    for src in (HERE / 'i2c_fw.cpp', ROOT / 'ARM/ST/src/i2c_stm32.cpp',
                ROOT / 'ARM/ST/STM32WBAxx/src/gpdma_stm32wba.cpp',
                ROOT / 'src/coredev/i2c.cpp', ROOT / 'src/device_intrf.cpp',
                ROOT / 'ARM/ST/src/iopincfg_stm32.cpp'):
        obj = tmp / (src.stem + '.o')
        run([*cxx, '-c', src, '-o', obj])
        objs.append(obj)
    obj = tmp / 'system_stm32wbaxx.o'
    run(['arm-none-eabi-gcc', *cpu, '-std=gnu17', *inc, '-c',
         ROOT / 'ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c', '-o', obj])
    objs.append(obj)
    elf = tmp / 'i2c_fw.elf'
    run(['arm-none-eabi-g++', *cpu, '-nostartfiles', '-Wl,--gc-sections', '-Wl,-T,' + str(HERE / 'i2c_fw.ld'),
         '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
    return elf


class Slave:
    def __init__(self, addr, refuse_at=None):
        self.addr = addr
        self.mem = bytearray((i * 5 + 1) & 0xFF for i in range(256))
        self.ptr = 0
        self.refuse_at = refuse_at
        self.nwr = 0

    def start_write(self):
        self.nwr = 0
        self.first = True

    def write(self, b):
        """Return True when the byte is acknowledged."""
        self.nwr += 1
        if self.refuse_at is not None and self.nwr == self.refuse_at:
            return False
        if self.first:
            self.ptr = b
            self.first = False
        else:
            self.mem[self.ptr & 0xFF] = b
            self.ptr += 1
        return True

    def read(self):
        b = self.mem[self.ptr & 0xFF]
        self.ptr += 1
        return b


class I2c:
    """I2C version 2 master model."""

    def __init__(self, name, base, log, violations, req_tx=0, req_rx=0):
        self.name = name
        self.req_tx = req_tx
        self.req_rx = req_rx
        self.gpdma = None
        self.in_dma = False
        self.base = base
        self.log = log
        self.viol = violations
        self.slaves = {}
        self.reg = collections.defaultdict(int)
        self.timings = []
        self.cr1w = []
        self.f = dict(txis=0, rxne=0, nackf=0, stopf=0, tc=0, tcr=0, busy=0, berr=0, arlo=0, ovr=0,
                      addr=0, dir=0)
        self.addcode = 0
        self.smode = False
        self.script = None
        self.stx = []
        self.got = []
        self.nbytes = 0
        self.reload = 0
        self.autoend = 0
        self.rd = 0
        self.slave = None
        self.txdr_full = False
        self.sadd = 0

    def v(self, what):
        self.viol.append(f'{self.name}: {what} (mark {self.log.mark})')

    # ---- register access ---------------------------------------------
    def isr(self):
        f = self.f
        v = 0
        if not self.txdr_full:
            v |= 1 << hdr('I2C_ISR_TXE_Pos')
        for k, n in (('txis', 'TXIS'), ('rxne', 'RXNE'), ('nackf', 'NACKF'), ('stopf', 'STOPF'),
                     ('tc', 'TC'), ('tcr', 'TCR'), ('berr', 'BERR'), ('arlo', 'ARLO'),
                     ('ovr', 'OVR'), ('busy', 'BUSY'), ('addr', 'ADDR'), ('dir', 'DIR')):
            if f[k]:
                v |= 1 << hdr(f'I2C_ISR_{n}_Pos')
        v |= (self.addcode & 0x7F) << hdr('I2C_ISR_ADDCODE_Pos')
        return v

    def rd_reg(self, off):
        if off == 0x18:
            return self.isr()
        if off == 0x24:
            if not self.f['rxne']:
                self.v('RXDR read without RXNE')
                return 0
            val = self.reg[0x24]
            self.f['rxne'] = 0
            if self.smode:
                return val
            self.log.add((self.name, 'R', val))
            self.nbytes -= 1
            self.set_nbytes(self.nbytes)
            self.after_byte()
            return val
        return self.reg[off]

    def wr_reg(self, off, val):
        if off == 0x00:
            if (self.reg[0] & 1) and not (val & 1):
                self.reset_state()
            self.reg[0] = val
            self.cr1w.append(val)
            self.dma_service()
        elif off == 0x04:
            self.write_cr2(val)
        elif off == 0x10:
            if self.reg[0] & 1:
                self.v('TIMINGR written with PE set')
            self.reg[0x10] = val
            self.timings.append((val, self.reg[0]))
        elif off == 0x18:
            if val & (1 << hdr('I2C_ISR_TXE_Pos')):
                self.txdr_full = False
                self.f['txis'] = self.f['txis']
        elif off == 0x1C:
            for k, n in (('nackf', 'NACKCF'), ('stopf', 'STOPCF'), ('berr', 'BERRCF'),
                         ('arlo', 'ARLOCF'), ('ovr', 'OVRCF'), ('addr', 'ADDRCF')):
                if val & (1 << hdr(f'I2C_ICR_{n}_Pos')):
                    self.f[k] = 0
        elif off == 0x28:
            self.write_txdr(val)
        else:
            self.reg[off] = val

    # ---- bus master played against the slave mode -----------------------
    def own(self, addr):
        oar1 = self.reg[0x08]
        oar2 = self.reg[0x0C]
        if (oar1 & (1 << 15)) and ((oar1 >> 1) & 0x7F) == addr:
            return True
        return bool(oar2 & (1 << 15)) and ((oar2 >> 1) & 0x7F) == addr

    def master_script(self, ops):
        self.smode = True
        for op in ops:
            kind, addr = op[0], op[1]
            if not self.own(addr):
                self.log.add((self.name, 'NA', addr))
                continue
            rd = kind == 'R'
            self.log.add((self.name, 'S', addr, 'R' if rd else 'W'))
            self.f['addr'] = 1
            self.f['dir'] = 1 if rd else 0
            self.addcode = addr
            while self.f['addr']:
                yield
            if rd:
                self.stx = []
                for _ in range(op[2] + 1):
                    self.f['txis'] = 1
                    self.dma_service()
                    while self.f['txis']:
                        yield
                # TXIS comes again as soon as the last byte is in the shift
                # register: that byte waits in TXDR and is never sent
                self.stx.pop()
                self.txdr_full = True
                self.got.append(bytes(self.stx))
                self.f['nackf'] = 1
                while self.f['nackf']:
                    yield
            else:
                for b in op[2]:
                    self.reg[0x24] = b
                    self.f['rxne'] = 1
                    self.dma_service()
                    while self.f['rxne']:
                        yield
            if op[-1] == 'P':
                self.f['stopf'] = 1
                while self.f['stopf']:
                    yield
                self.log.add((self.name, 'P'))

    def step(self):
        if self.script is not None:
            try:
                next(self.script)
            except StopIteration:
                self.script = None

    def reset_state(self):
        for k in self.f:
            self.f[k] = 0
        self.txdr_full = False
        self.nbytes = 0

    # ---- transfer ----------------------------------------------------
    def set_nbytes(self, n):
        c = self.reg[4]
        c = (c & ~(0xFF << 16)) | ((n & 0xFF) << 16)
        self.reg[4] = c

    def write_cr2(self, val):
        start = bool(val & (1 << 13))
        stop = bool(val & (1 << 14))
        nb = (val >> 16) & 0xFF
        reload = (val >> 24) & 1
        autoend = (val >> 25) & 1
        rd = (val >> 10) & 1
        sadd = (val >> 1) & 0x7F
        if not (self.reg[0] & 1):
            self.v('CR2 written with PE clear')
        if start:
            if self.f['busy'] and not self.f['tc']:
                self.v('START while a transfer runs and TC is not set')
            self.process_start(sadd, rd, nb, reload, autoend)
        elif stop:
            if not self.f['busy']:
                self.v('STOP with the bus idle')
            self.reg[4] = val & ~(1 << 14)
            self.end_transfer()
        else:
            if self.f['tcr']:
                # next chunk
                self.f['tcr'] = 0
                self.nbytes = nb
                self.reload = reload
                self.autoend = autoend
                self.reg[4] = val
                self.log.add((self.name, 'NB', nb))
                if nb == 0:
                    self.v('NBYTES 0 written after TCR')
                self.next_byte()
            elif self.f['busy']:
                if ((val ^ self.reg[4]) >> 16) & 0xFF:
                    self.v('NBYTES written while a transfer runs')
                self.reg[4] = val
            else:
                self.reg[4] = val

    def process_start(self, sadd, rd, nb, reload, autoend):
        restart = self.f['busy']
        self.f['tc'] = 0
        self.f['tcr'] = 0
        self.reg[4] = 0
        self.reg[4] = (sadd << 1) | (rd << 10) | (nb << 16) | (reload << 24) | (autoend << 25)
        self.rd = rd
        self.nbytes = nb
        self.reload = reload
        self.autoend = autoend
        self.sadd = sadd
        self.f['busy'] = 1
        self.txdr_full = False
        self.log.add((self.name, 'Sr' if restart else 'S', sadd, 'R' if rd else 'W', nb))
        self.slave = self.slaves.get(sadd)
        if self.slave is None:
            self.f['nackf'] = 1
            self.log.add((self.name, 'NACK'))
            self.auto_stop()
            return
        if not rd:
            self.slave.start_write()
        if nb == 0:
            if autoend:
                self.auto_stop()
            else:
                self.f['tc'] = 1
            return
        self.next_byte()

    def next_byte(self):
        if self.rd:
            self.reg[0x24] = self.slave.read()
            self.f['rxne'] = 1
        else:
            self.f['txis'] = 1
        self.dma_service()

    def dma_service(self):
        if self.gpdma is None or self.in_dma:
            return
        self.in_dma = True
        cr1 = self.reg[0]
        while True:
            if self.f['txis'] and (cr1 & hdr('I2C_CR1_TXDMAEN')):
                b = self.gpdma.take(self.req_tx)
                if b is None:
                    break
                self.write_txdr(b)
            elif self.f['rxne'] and (cr1 & hdr('I2C_CR1_RXDMAEN')) and self.gpdma.find(self.req_rx) is not None:
                self.gpdma.put(self.req_rx, self.rd_reg(0x24))
            else:
                break
        self.in_dma = False

    def write_txdr(self, val):
        if not self.f['txis']:
            self.v('TXDR written without TXIS')
            return
        self.f['txis'] = 0
        if self.smode:
            self.stx.append(val & 0xFF)
            return
        self.log.add((self.name, 'B', val & 0xFF))
        if not self.slave.write(val & 0xFF):
            self.f['nackf'] = 1
            self.log.add((self.name, 'NACK'))
            self.auto_stop()
            return
        self.nbytes -= 1
        self.set_nbytes(self.nbytes)
        self.after_byte()

    def after_byte(self):
        if self.nbytes > 0:
            self.next_byte()
        elif self.reload:
            self.f['tcr'] = 1
        elif self.autoend:
            self.auto_stop()
        else:
            self.f['tc'] = 1

    def auto_stop(self):
        self.f['stopf'] = 1
        self.end_transfer()

    def end_transfer(self):
        self.log.add((self.name, 'P'))
        self.f['busy'] = 0
        self.f['tc'] = 0
        self.f['tcr'] = 0
        self.f['txis'] = 0
        self.f['rxne'] = 0
        self.f['stopf'] = 1


class Gpdma:
    """GPDMA1 model: channels with a request, a memory address and a count.
    The peripheral model calls take and put for each request."""
    CH0 = 0x50
    STRIDE = 0x80

    def __init__(self, viol):
        self.viol = viol
        self.ch = [dict(ccr=0, ctr1=0, ctr2=0, cbr1=0, csar=0, cdar=0, csr=0, active=False) for _ in range(8)]
        self.uc = None
        self.notify = None
        self.moved = 0

    def idx(self, off):
        if off < self.CH0 or off >= self.CH0 + 8 * self.STRIDE:
            return None, None
        return divmod(off - self.CH0, self.STRIDE)

    def rd(self, off):
        n, r = self.idx(off)
        if n is None:
            return 0
        c = self.ch[n]
        if r == 0x10:
            return c['csr'] | (0 if c['active'] else 1)
        return {0x14: c['ccr'], 0x40: c['ctr1'], 0x44: c['ctr2'], 0x48: c['cbr1'],
                0x4C: c['csar'], 0x50: c['cdar']}.get(r, 0)

    def wr(self, off, val):
        n, r = self.idx(off)
        if n is None:
            return
        c = self.ch[n]
        if r == 0x0C:
            c['csr'] &= ~val
        elif r == 0x14:
            if val & hdr('DMA_CCR_RESET'):
                c['ccr'] = 0
                c['active'] = False
                return
            en = bool(val & hdr('DMA_CCR_EN')) and not (val & hdr('DMA_CCR_SUSP'))
            if en and not c['active'] and (c['cbr1'] & 0xFFFF) == 0:
                self.viol.append(f'GPDMA channel {n} enabled with a zero count')
            c['ccr'] = val
            c['active'] = en and (c['cbr1'] & 0xFFFF) > 0
            if c['active'] and self.notify:
                self.notify()
        elif r == 0x40:
            c['ctr1'] = val
        elif r == 0x44:
            c['ctr2'] = val
        elif r == 0x48:
            if c['active']:
                self.viol.append(f'GPDMA channel {n} count written while enabled')
            c['cbr1'] = val
        elif r == 0x4C:
            c['csar'] = val
        elif r == 0x50:
            c['cdar'] = val

    def find(self, req):
        for n, c in enumerate(self.ch):
            if c['active'] and (c['ctr2'] & 0x7F) == req:
                return n
        return None

    def done(self, n):
        c = self.ch[n]
        c['cbr1'] = 0
        c['active'] = False
        c['ccr'] &= ~hdr('DMA_CCR_EN')
        c['csr'] |= hdr('DMA_CSR_TCF')

    def take(self, req, n=1):
        """Next item of Count bytes of a memory to peripheral transfer, None if no channel."""
        ch = self.find(req)
        if ch is None:
            return None
        c = self.ch[ch]
        b = int.from_bytes(bytes(self.uc.mem_read(c['csar'], n)), 'little')
        if c['ctr1'] & hdr('DMA_CTR1_SINC'):
            c['csar'] += n
        c['cbr1'] -= n
        self.moved += n
        if (c['cbr1'] & 0xFFFF) == 0:
            self.done(ch)
        return b

    def put(self, req, v, n=1):
        ch = self.find(req)
        c = self.ch[ch]
        self.uc.mem_write(c['cdar'], int(v).to_bytes(n, 'little'))
        if c['ctr1'] & hdr('DMA_CTR1_DINC'):
            c['cdar'] += n
        c['cbr1'] -= n
        self.moved += n
        if (c['cbr1'] & 0xFFFF) == 0:
            self.done(ch)


class Log:
    def __init__(self):
        self.ev = []
        self.mark = -1

    def add(self, e):
        self.ev.append((self.mark, *e))

    def of(self, mark, name='I2C1'):
        return [e[2:] for e in self.ev if e[0] == mark and e[1] == name]


def timing_check(i2c_name, clk, timing, cr1, rate_min, rate_max, actual, errors):
    presc = (timing >> 28) & 0xF
    scldel = (timing >> 20) & 0xF
    sdadel = (timing >> 16) & 0xF
    sclh = (timing >> 8) & 0xFF
    scll = timing & 0xFF
    tclk = 1e9 / clk
    tp = tclk * (presc + 1)
    tlow = (scll + 1) * tp
    thigh = (sclh + 1) * tp
    tsync = 2 * (50 + 2 * tclk)
    est = 1e9 / (tsync + tlow + thigh)
    if actual <= 100000:
        lo, hi, tr, tsu, tf = 4700, 4000, 1000, 250, 300
    elif actual <= 400000:
        lo, hi, tr, tsu, tf = 1300, 600, 300, 100, 300
    else:
        lo, hi, tr, tsu, tf = 500, 260, 120, 50, 120
    if tlow < lo:
        errors.append(f'{i2c_name} TIMINGR {timing:#x}: low {tlow:.0f} ns < {lo}')
    if thigh < hi:
        errors.append(f'{i2c_name} TIMINGR {timing:#x}: high {thigh:.0f} ns < {hi}')
    if (scldel + 1) * tp < tr + tsu:
        errors.append(f'{i2c_name} TIMINGR {timing:#x}: SCLDEL too short')
    if sdadel * tp + 0 < tf - 50 - 3 * tclk:
        errors.append(f'{i2c_name} TIMINGR {timing:#x}: SDADEL too short')
    if not (rate_min <= est <= rate_max):
        errors.append(f'{i2c_name} TIMINGR {timing:#x}: rate {est:.0f} not in {rate_min}..{rate_max}')
    if abs(est - actual) > actual * 0.02:
        errors.append(f'{i2c_name} TIMINGR {timing:#x}: reported {actual}, computed {est:.0f}')


def main():
    arm = shutil.which('arm-none-eabi-g++')
    try:
        from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE, UC_HOOK_BLOCK
        from unicorn.arm_const import UC_CPU_ARM_CORTEX_M33
        from unicorn.arm_const import UC_ARM_REG_R0, UC_ARM_REG_SP, UC_ARM_REG_PC, UC_ARM_REG_LR, UC_ARM_REG_PRIMASK
        from elftools.elf.elffile import ELFFile
    except ImportError as e:
        print(f'NOT RUN: {e}. pip install unicorn pyelftools')
        return
    if not arm:
        print('NOT RUN: arm-none-eabi-g++ not found')
        return

    with tempfile.TemporaryDirectory(prefix='wba-i2c-') as tmp:
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
        i2c1 = I2c('I2C1', hdr('I2C1_BASE_NS'), log, viol, 6, 5)
        i2c3 = I2c('I2C3', hdr('I2C3_BASE_NS'), log, viol, 9, 8)
        i2c1.gpdma = i2c3.gpdma = gpdma
        gpdma.notify = lambda: (i2c1.dma_service(), i2c3.dma_service())
        gbase = hdr('GPDMA1_BASE_NS')
        for dev in (i2c1, i2c3):
            dev.slaves[0x50] = Slave(0x50)
        i2c1.slaves[0x51] = Slave(0x51, refuse_at=3)
        devs = {d.base: d for d in (i2c1, i2c3)}
        regs = collections.defaultdict(int)
        rcc_log = []
        rcc = hdr('RCC_BASE_NS')

        def rd(uc, off, size, base):
            a = base + off
            d = devs.get(a & ~0xFF)
            if d is not None:
                return d.rd_reg(a & 0xFF)
            if gbase <= a < gbase + 0x1000:
                return gpdma.rd(a - gbase)
            return regs[a]

        def wr(uc, off, size, val, base):
            a = base + off
            d = devs.get(a & ~0xFF)
            if d is not None:
                d.wr_reg(a & 0xFF, val)
                return
            if gbase <= a < gbase + 0x1000:
                gpdma.wr(a - gbase, val)
                return
            if rcc <= a < rcc + 0x400:
                rcc_log.append((a - rcc, val))
            regs[a] = val

        for b, s in ((0x40000000, 0x40000), (0x42020000, 0x10000), (0x46000000, 0x30000)):
            uc.mmio_map(b, s, rd, b, wr, b)

        dma_at = {}
        w50 = bytes(range(1, 4))
        w20 = bytes(range(0x40, 0x54))
        slave_ops = [
            ('W', 0x30, w50, 'P'),
            ('W', 0x30, bytes([4]), 'Sr'), ('R', 0x30, 4, 'P'),
            ('R', 0x31, 10, 'P'),
            ('W', 0x31, w20, 'P'),
            ('W', 0x55, b'\x01', 'P'),
        ]

        def on_mark(uc, addr, size, user):
            log.mark = uc.reg_read(UC_ARM_REG_R0)
            dma_at[log.mark] = gpdma.moved
            if log.mark in (14, 18):
                i2c3.script = i2c3.master_script(slave_ops)

        def on_step(uc, addr, size, user):
            i2c3.step()

        state = {'done': False}

        def on_done(uc, addr, size, user):
            state['done'] = True
            uc.emu_stop()

        uc.hook_add(UC_HOOK_CODE, on_mark, begin=syms['I2cTestMark'] & ~1, end=syms['I2cTestMark'] & ~1)
        uc.hook_add(UC_HOOK_CODE, on_step, begin=syms['I2cTestSlaveStep'] & ~1, end=syms['I2cTestSlaveStep'] & ~1)
        uc.hook_add(UC_HOOK_CODE, on_done, begin=syms['I2cTestDoneHook'] & ~1, end=syms['I2cTestDoneHook'] & ~1)

        active = False
        pending = False
        ticks = 0
        def dispatch(uc, address, size, ctx):
            nonlocal pending, ticks
            if active or not int.from_bytes(uc.mem_read(syms['g_I2cTestAutoIrq'], 4), 'little'):
                return
            ticks += 1
            if ticks % 16 or uc.reg_read(UC_ARM_REG_PRIMASK):
                return
            cr1, f = i2c1.reg[0], i2c1.f
            if ((f['txis'] and cr1 & 2) or (f['stopf'] and cr1 & 32) or
                (f['nackf'] and cr1 & 16) or ((f['tc'] or f['tcr']) and cr1 & 64)):
                pending = True
                uc.emu_stop()
        uc.hook_add(UC_HOOK_BLOCK, dispatch)
        vec = bytes(uc.mem_read(0x08000000, 8))
        uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
        pc = int.from_bytes(vec[4:8], 'little')
        try:
            for _ in range(1000):
                pending = False
                uc.emu_start(pc, 0xFFFFFFFF, count=40_000_000)
                if not pending:
                    break
                saved = uc.context_save()
                active = True
                uc.reg_write(UC_ARM_REG_LR, 0x081ff001)
                uc.emu_start(syms['I2C1_EV_IRQHandler'] | 1, 0x081ff000, count=100000)
                assert uc.reg_read(UC_ARM_REG_PC) == 0x081ff000
                active = False
                uc.context_restore(saved)
                pc = uc.reg_read(UC_ARM_REG_PC) | 1
        except Exception as ex:
            raise SystemExit(f'FAIL emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex}')
        if not state['done']:
            raise SystemExit('FAIL firmware did not finish')

        long_count = int.from_bytes(uc.mem_read(syms['g_I2cTestLongCount'], 4), 'little', signed=True)
        res_addr = syms['g_I2cTestResult']
        n = int.from_bytes(uc.mem_read(res_addr + 4 * 63, 4), 'little', signed=True)
        res = [int.from_bytes(uc.mem_read(res_addr + 4 * i, 4), 'little', signed=True) for i in range(n)]
        rx = bytes(uc.mem_read(syms['g_I2cTestRx'], 512))
        rx2 = [bytes(uc.mem_read(syms['g_I2cTestRx2'] + 320 * i, 320)) for i in range(3)]
        slv_n = int.from_bytes(uc.mem_read(syms['g_I2cTestSlvCnt'], 4), 'little')
        slv = [int.from_bytes(uc.mem_read(syms['g_I2cTestSlvEvt'] + 4 * i, 4), 'little') for i in range(slv_n * 2)]
        slv_rx = [bytes(uc.mem_read(syms['g_I2cTestSlvRx'] + 16 * i, 16)) for i in range(2)]
        slv_rx2 = [bytes(uc.mem_read(syms['g_I2cTestSlvRx2'] + 16 * i, 16)) for i in range(2)]
        slv_dma_init = int.from_bytes(uc.mem_read(syms['g_I2cTestSlvDmaInit'], 4), 'little')

    errors = []
    checks = 0

    def expect(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            errors.append(msg)

    P = lambda m: log.of(m)
    s50 = i2c1.slaves[0x50]
    s51 = i2c1.slaves[0x51]
    tx = bytes((i * 7 + 3) & 0xFF for i in range(512))

    # 0 init
    expect(res[0] == 1, 'I2CInit failed')
    expect(80000 <= res[1] <= 100000, f'rate at 100 kHz request: {res[1]}')

    # 1 probe
    expect(P(1) == [('S', 0x50, 'W', 0), ('P',), ('S', 0x23, 'W', 0), ('NACK',), ('P',)], f'probe events {P(1)}')
    expect(res[2:4] == [1, 0], f'probe results {res[2:4]}')

    # 2 write 4 bytes
    expect(P(2) == [('S', 0x50, 'W', 0), ('P',), ('S', 0x50, 'W', 4)] + [('B', b) for b in tx[:4]] + [('P',)],
           f'write events {P(2)}')
    expect(res[4] == 4, f'write result {res[4]}')

    # 3 write 300, reload
    e = P(3)
    expect(e[2:3] == [('S', 0x50, 'W', 255)] and ('NB', 45) in e and e.count(('NB', 45)) == 1 and e[-1] == ('P',),
           f'write 300 events {e[:6]}')
    expect([x[1] for x in e if x[0] == 'B'] == list(tx[:300]), 'write 300 bytes differ')
    expect(res[5] == 300, f'write 300 result {res[5]}')

    # 4 command and read with repeated start
    e = P(4)
    exp4 = [('S', 0x50, 'W', 0), ('P',), ('S', 0x50, 'W', 1), ('B', 2), ('Sr', 0x50, 'R', 5)]
    expect(e[:5] == exp4 and e[-1] == ('P',) and len(e) == 11, f'read events {e}')
    expect(res[6] == 5, f'read result {res[6]}')

    # 5 read 300
    e = P(5)
    expect(e[:5] == [('S', 0x50, 'W', 0), ('P',), ('S', 0x50, 'W', 1), ('B', 0), ('Sr', 0x50, 'R', 255)] and
           ('NB', 45) in e and e[-1] == ('P',), f'read 300 events {e[:6]}')
    expect(res[7] == 300, f'read 300 result {res[7]}')
    got5 = rx[8:308]
    reads5 = bytes(x[1] for x in e if x[0] == 'R')
    expect(got5 == reads5 and len(reads5) == 300, 'read 300 data differ from the bus')

    # 6 refused byte
    e = P(6)
    expect(e == [('S', 0x51, 'W', 0), ('P',), ('S', 0x51, 'W', 5), ('B', tx[0]), ('B', tx[1]), ('B', tx[2]),
                 ('NACK',), ('P',)], f'refused byte events {e}')
    expect(res[8] == 2, f'refused byte result {res[8]} (2 bytes accepted)')

    # 7 absent device then the bus works
    e = P(7)
    expect(e[:3] == [('S', 0x23, 'W', 0), ('NACK',), ('P',)] and e[3] == ('S', 0x50, 'W', 0), f'absent events {e}')
    expect(res[9] == 0 and res[10] == 2, f'absent results {res[9:11]}')

    # 8 plain read
    e = P(8)
    expect(e[0] == ('S', 0x50, 'R', 3) and e[-1] == ('P',) and len(e) == 5, f'plain read events {e}')
    expect(res[11] == 3 and rx[320:323] == bytes(x[1] for x in e if x[0] == 'R'), 'plain read data')

    # 9 rates
    expect(340000 <= res[12] <= 400000, f'400 kHz request gave {res[12]}')
    expect(800000 <= res[13] <= 1000000, f'1 MHz request gave {res[13]}')
    expect(80000 <= res[14] <= 100000, f'rate 0 gave {res[14]}')

    # 10 slave without an address is refused, interrupt master is accepted
    expect(res[15:17] == [0, 1], f'mode checks {res[15:17]}')

    # 11 I2C3
    expect(res[17] == 1 and 340000 <= res[18] <= 400000 and res[19] == 3, f'I2C3 results {res[17:20]}')
    e = log.of(11, 'I2C3')
    expect(e[-1] == ('P',) and ('S', 0x50, 'W', 3) in e, f'I2C3 events {e}')

    # 12 100 MHz bus clock
    expect(80000 <= res[20] <= 100000 and 340000 <= res[21] <= 400000 and 800000 <= res[22] <= 1000000 and res[23] == 2,
           f'100 MHz clock results {res[20:24]}')

    # 13 interrupt master
    b = len(res) - 29
    expect(res[b] == 1, 'interrupt master init failed')
    e = log.of(13)
    expect(res[b + 1] == -1 and res[b + 2] == 5, f'interrupt write results {res[b + 1:b + 3]}')
    expect(e[:5] == [('S', 0x50, 'W', 0), ('P',), ('S', 0x50, 'W', 5), ('B', tx[0]), ('B', tx[1])],
           f'interrupt write events {e[:6]}')
    expect(res[b + 3] == -1 and res[b + 4] == 4, f'interrupt read results {res[b + 3:b + 5]}')
    reads = [x for x in e if x[0] == 'R']
    expect(rx[400:404] == bytes(x[1] for x in reads[:4]), 'interrupt read data differ from the bus')
    expect(res[b + 5] == -1 and res[b + 6] == 2, f'interrupt refused byte results {res[b + 5:b + 7]}')
    expect(res[b + 7] == -1 and res[b + 8] == 300, f'interrupt read 300 results {res[b + 7:b + 9]}')
    expect(rx2[0][:300] == bytes(x[1] for x in reads[4:]) and len(reads) == 304,
           f'interrupt read 300 data, {len(reads)} bytes on the bus')
    expect(res[b + 9] == -1 and long_count == 100, f'interrupt write 100: return {res[b + 9]}, callback {long_count}')
    expect(res[b + 10] == 0, f'write to an absent device {res[b + 10]}')

    # 14 slave on I2C3
    expect(res[b + 11] == 1, 'slave init failed')
    sl = log.of(14, 'I2C3')
    exp_ev = [('S', 0x30, 'W'), ('P',), ('S', 0x30, 'W'), ('S', 0x30, 'R'), ('P',), ('S', 0x31, 'R'), ('P',),
              ('S', 0x31, 'W'), ('P',), ('NA', 0x55), ]
    expect(sl == exp_ev, f'slave bus events {sl}')
    pairs = list(zip(slv[0::2], slv[1::2]))
    exp_cb = [(2, 0), (3, 3),
              (2, 0), (1, 1), (3, 4),
              (1, 0), (3, 8),
              (2, 0), (3, 16)]
    expect(pairs[:9] == exp_cb, f'slave callback events {pairs[:9]}')
    expect(slv_rx[0][:3] == bytes([4, 2, 3]), f'slave write data {slv_rx[0][:3].hex()}')
    expect(slv_rx[1] == w20[:16], f'slave write data past the buffer {slv_rx[1].hex()}')
    got = i2c3.got
    exp_a = bytes(0xA0 + i for i in range(4))
    exp_b = bytes(0xB0 + i for i in range(8)) + b'\xff\xff'
    expect(got[:2] == [exp_a, exp_b], f'slave read data {[g.hex() for g in got[:2]]}')

    # 18 the same requests with the DMA channels
    expect(slv_dma_init == 1, 'slave DMA init failed')
    expect(log.of(18, 'I2C3') == exp_ev, f'slave DMA bus events {log.of(18, "I2C3")}')
    expect(pairs[9:] == exp_cb, f'slave DMA callback events {pairs[9:]}')
    expect(slv_rx2[0][:3] == bytes([4, 2, 3]), f'slave DMA write data {slv_rx2[0][:3].hex()}')
    expect(slv_rx2[1] == w20[:16], f'slave DMA write data past the buffer {slv_rx2[1].hex()}')
    expect(got[2:] == [exp_a, exp_b], f'slave DMA read data {[g.hex() for g in got[2:]]}')
    expect(gpdma.moved - dma_at[18] == 3 + 1 + 5 + 8 + 16, f'slave DMA bytes moved {gpdma.moved - dma_at[18]}')

    # 15 DMA master, polling wait
    d = b + 12
    expect(res[d:d + 8] == [1, 5, 300, 4, 300, 2, 0, 6], f'DMA polling results {res[d:d + 8]}')
    e = log.of(15)
    expect(bytes(x[1] for x in e if x[0] == 'B') == tx[:5] + tx[:300] + tx[:3] + bytes([7]), 'DMA write bytes on the bus')
    reads = [x[1] for x in e if x[0] == 'R']
    expect(bytes(reads[:4]) == rx[420:424] and bytes(reads[4:304]) == rx2[1][:300] and bytes(reads[304:]) == rx[430:436],
           'DMA read data differ from the bus')
    expect(dma_at[16] - dma_at[15] == 5 + 300 + 4 + 300 + 3 + 1 + 6, f'bytes moved by DMA {dma_at[16] - dma_at[15]}')

    # 16 DMA master, interrupt
    d += 8
    expect(res[d:d + 9] == [1, -1, 5, -1, 4, -1, 300, -1, 2], f'DMA interrupt results {res[d:d + 9]}')
    e = log.of(16)
    reads = [x[1] for x in e if x[0] == 'R']
    expect(bytes(reads[:4]) == rx[440:444] and bytes(reads[4:]) == rx2[2][:300], 'DMA interrupt read data differ from the bus')
    expect(dma_at[17] - dma_at[16] == 5 + 4 + 300 + 3, f'bytes moved by DMA in interrupt mode {dma_at[17] - dma_at[16]}')

    # TIMINGR values against the reference manual limits
    t1 = [t for t, _ in i2c1.timings]
    expect(len(t1) >= 4, f'TIMINGR writes {len(t1)}')
    if len(t1) >= 4:
        timing_check('I2C1', FCPU, t1[0], 0, 80000, 100000, res[1], errors)
        timing_check('I2C1', FCPU, t1[1], 0, 340000, 400000, res[12], errors)
        timing_check('I2C1', FCPU, t1[2], 0, 800000, 1000000, res[13], errors)
        timing_check('I2C1', FCPU, t1[3], 0, 80000, 100000, res[14], errors)
    if len(t1) >= 7:
        timing_check('I2C1', 100_000_000, t1[4], 0, 80000, 100000, res[20], errors)
        timing_check('I2C1', 100_000_000, t1[5], 0, 340000, 400000, res[21], errors)
        timing_check('I2C1', 100_000_000, t1[6], 0, 800000, 1000000, res[22], errors)
    else:
        expect(False, f'TIMINGR writes at 100 MHz: {len(t1)}')
    t3 = [t for t, _ in i2c3.timings]
    expect(len(t3) == 3, f'I2C3 TIMINGR writes {len(t3)}')
    if t3:
        timing_check('I2C3', FCPU, t3[0], 0, 340000, 400000, res[18], errors)
    # fast mode plus drive: on after the 1 MHz request, off after the 100 kHz one
    fmp = 1 << hdr('I2C_CR1_FMP_Pos')
    states = [bool(v & fmp) for v in i2c1.cr1w]
    expect(True in states, 'fast mode plus drive never set')
    expect(any(a and not b for a, b in zip(states, states[1:])), 'fast mode plus drive not cleared at 100 kHz')

    # clocks: enable bits in the RCC registers of each instance
    def rcc_set(name, bit):
        return any(o == rcc_off(name) and v & bit for o, v in rcc_log)
    expect(rcc_set('APB1ENR1', hdr('RCC_APB1ENR1_I2C1EN')), 'I2C1 clock not enabled')
    expect(rcc_set('APB7ENR', hdr('RCC_APB7ENR_I2C3EN')), 'I2C3 clock not enabled')
    expect(rcc_set('APB1RSTR1', hdr('RCC_APB1RSTR1_I2C1RST')), 'I2C1 reset not pulsed')
    expect(rcc_set('APB7RSTR', hdr('RCC_APB7RSTR_I2C3RST')), 'I2C3 reset not pulsed')
    kernel1 = [v for o, v in rcc_log if o == rcc_off('CCIPR1')]
    expect(kernel1 and all(not v & hdr('RCC_CCIPR1_I2C1SEL_Msk') for v in kernel1), 'I2C1 kernel clock not PCLK1')
    kernel3 = [v for o, v in rcc_log if o == rcc_off('CCIPR3')]
    expect(kernel3 and all(not v & hdr('RCC_CCIPR3_I2C3SEL_Msk') for v in kernel3), 'I2C3 kernel clock not PCLK7')

    # slave memory after the writes
    expect(s50.mem[3] == tx[1] and s50.mem[(3 + 298) & 0xFF] == tx[299], 'slave memory after the 300 byte write')
    expect(s51.mem[tx[0]] == tx[1] and s51.mem[(tx[0] + 1) & 0xFF] != tx[2], 'refusing slave memory')

    expect(not viol, f'model violations: {viol[:6]}')

    for m in errors:
        print('FAIL:', m)
    if errors:
        raise SystemExit(f'STM32WBA65 I2C: {len(errors)} of {checks} checks failed')
    print(f'STM32WBA65 I2C master on the emulator: {checks} checks, 0 failures')
    print('Hardware not tested.')


if __name__ == '__main__':
    main()
