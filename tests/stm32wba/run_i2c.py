#!/usr/bin/env python3
"""STM32WBA65 I2C master driver test on an instruction set emulator.

Run: python3 tests/stm32wba/run_i2c.py

Needs arm-none-eabi-g++, and the python packages unicorn and pyelftools.
Without them the test prints NOT RUN.

i2c_fw.cpp is built for the Cortex-M33 with the I2C master driver
ARM/ST/STM32WBAxx/src/i2c_stm32wba.cpp, the generic I2C and device interface
sources and the STM32 GPIO driver, and run in unicorn. The I2C1 and I2C3
registers are a model of the I2C version 2 master of the reference manual:

- START with SADD, RD_WRN and NBYTES, the address phase against two slave
  devices (0x50, and 0x51 that refuses the third byte of a write)
- TXIS, RXNE, TCR (RELOAD), TC (software end), STOPF (AUTOEND) and NACKF
  with the automatic STOP after a NACK
- NBYTES counted down per byte, written again after TCR, the repeated START
  after TC

The model records the bus transactions and flags what the reference manual
does not allow: TXDR written without TXIS, RXDR read without RXNE, START or
NBYTES written while a transfer is running and neither TC nor TCR is set,
TIMINGR written with PE set. The TIMINGR values written are checked against
the minimum SCL low and high times and the SCLDEL and SDADEL ranges.

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
    cpu = ['-mcpu=cortex-m33', '-mthumb', '-mfloat-abi=soft', '-Os', '-DSTM32WBA65xx',
           '-ffunction-sections', '-fdata-sections', '-g']
    inc = ['-I' + str(ROOT / 'include'), '-I' + str(ROOT / 'ARM/include'),
           '-I' + str(ROOT / 'ARM/CMSIS/Core/Include'), '-I' + str(ROOT / 'ARM/ST/include'),
           '-I' + str(ROOT / 'ARM/ST/STM32WBAxx/include')]
    objs = []
    cxx = ['arm-none-eabi-g++', *cpu, '-std=gnu++23', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', *inc]
    for src in (HERE / 'i2c_fw.cpp', ROOT / 'ARM/ST/STM32WBAxx/src/i2c_stm32wba.cpp',
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

    def __init__(self, name, base, log, violations):
        self.name = name
        self.base = base
        self.log = log
        self.viol = violations
        self.slaves = {}
        self.reg = collections.defaultdict(int)
        self.timings = []
        self.cr1w = []
        self.f = dict(txis=0, rxne=0, nackf=0, stopf=0, tc=0, tcr=0, busy=0, berr=0, arlo=0, ovr=0)
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
                     ('ovr', 'OVR'), ('busy', 'BUSY')):
            if f[k]:
                v |= 1 << hdr(f'I2C_ISR_{n}_Pos')
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
                         ('arlo', 'ARLOCF'), ('ovr', 'OVRCF')):
                if val & (1 << hdr(f'I2C_ICR_{n}_Pos')):
                    self.f[k] = 0
        elif off == 0x28:
            self.write_txdr(val)
        else:
            self.reg[off] = val

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

    def write_txdr(self, val):
        if not self.f['txis']:
            self.v('TXDR written without TXIS')
            return
        self.f['txis'] = 0
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
        i2c1 = I2c('I2C1', hdr('I2C1_BASE_NS'), log, viol)
        i2c3 = I2c('I2C3', hdr('I2C3_BASE_NS'), log, viol)
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
            return regs[a]

        def wr(uc, off, size, val, base):
            a = base + off
            d = devs.get(a & ~0xFF)
            if d is not None:
                d.wr_reg(a & 0xFF, val)
                return
            if rcc <= a < rcc + 0x400:
                rcc_log.append((a - rcc, val))
            regs[a] = val

        for b, s in ((0x40000000, 0x40000), (0x42020000, 0x10000), (0x46000000, 0x30000)):
            uc.mmio_map(b, s, rd, b, wr, b)

        def on_mark(uc, addr, size, user):
            log.mark = uc.reg_read(UC_ARM_REG_R0)

        state = {'done': False}

        def on_done(uc, addr, size, user):
            state['done'] = True
            uc.emu_stop()

        uc.hook_add(UC_HOOK_CODE, on_mark, begin=syms['I2cTestMark'] & ~1, end=syms['I2cTestMark'] & ~1)
        uc.hook_add(UC_HOOK_CODE, on_done, begin=syms['I2cTestDoneHook'] & ~1, end=syms['I2cTestDoneHook'] & ~1)

        vec = bytes(uc.mem_read(0x08000000, 8))
        uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
        pc = int.from_bytes(vec[4:8], 'little')
        try:
            uc.emu_start(pc, 0xFFFFFFFF, count=40_000_000)
        except Exception as ex:
            raise SystemExit(f'FAIL emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex}')
        if not state['done']:
            raise SystemExit('FAIL firmware did not finish')

        res_addr = syms['g_I2cTestResult']
        n = int.from_bytes(uc.mem_read(res_addr + 4 * 63, 4), 'little', signed=True)
        res = [int.from_bytes(uc.mem_read(res_addr + 4 * i, 4), 'little', signed=True) for i in range(n)]
        rx = bytes(uc.mem_read(syms['g_I2cTestRx'], 512))

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

    # 10 refused modes
    expect(res[15:17] == [0, 0], f'unsupported modes {res[15:17]}')

    # 11 I2C3
    expect(res[17] == 1 and 340000 <= res[18] <= 400000 and res[19] == 3, f'I2C3 results {res[17:20]}')
    e = log.of(11, 'I2C3')
    expect(e[-1] == ('P',) and ('S', 0x50, 'W', 3) in e, f'I2C3 events {e}')

    # 12 100 MHz bus clock
    expect(80000 <= res[20] <= 100000 and 340000 <= res[21] <= 400000 and 800000 <= res[22] <= 1000000 and res[23] == 2,
           f'100 MHz clock results {res[20:24]}')

    # TIMINGR values against the reference manual limits
    t1 = [t for t, _ in i2c1.timings]
    expect(len(t1) >= 4, f'TIMINGR writes {len(t1)}')
    if len(t1) >= 4:
        timing_check('I2C1', FCPU, t1[0], 0, 80000, 100000, res[1], errors)
        timing_check('I2C1', FCPU, t1[1], 0, 340000, 400000, res[12], errors)
        timing_check('I2C1', FCPU, t1[2], 0, 800000, 1000000, res[13], errors)
        timing_check('I2C1', FCPU, t1[3], 0, 80000, 100000, res[14], errors)
    if len(t1) >= 7:
        timing_check('I2C1', 100_000_000, t1[-3], 0, 80000, 100000, res[20], errors)
        timing_check('I2C1', 100_000_000, t1[-2], 0, 340000, 400000, res[21], errors)
        timing_check('I2C1', 100_000_000, t1[-1], 0, 800000, 1000000, res[22], errors)
    else:
        expect(False, f'TIMINGR writes at 100 MHz: {len(t1)}')
    t3 = [t for t, _ in i2c3.timings]
    expect(len(t3) == 1, f'I2C3 TIMINGR writes {len(t3)}')
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
