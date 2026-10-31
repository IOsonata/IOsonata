#!/usr/bin/env python3
"""STM32WBA65 core clock setup test on an instruction set emulator.

Run: python3 tests/stm32wba/run_clock.py

Needs arm-none-eabi-g++, and the python packages unicorn and pyelftools.
Without them the test prints NOT RUN.

clock_fw.cpp is built for the Cortex-M33 with
ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c and run in unicorn. The RCC, PWR
and flash access control registers are a model that follows the clock switch
sequence of the reference manual and of the ST HAL (HAL_RCC_OscConfig and
HAL_RCC_ClockConfig):

- HSION, HSEON and PLL1ON set the ready flags (HSE can be made to fail)
- PLL1 settings checked at PLL1ON: input 4 to 16 MHz, VCO 128 to 544 MHz, the
  input range bits, nothing written while PLL1 runs
- SW only accepted for a ready source, SWS follows; the PLL1R pre divider
  (RCLKPRE) must be set when PLL1R is selected and is released after
- flash wait states and voltage scaling range 1 checked at every clock
  change against the frequency, the HCLK frequency never above what the
  wait states and the scaling allow
- the order of the register writes is recorded and checked

Not hardware validation: no PLL lock time, no electrical behavior.

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
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
HDR = ROOT / 'ARM/ST/STM32WBAxx/include/stm32wba65xx.h'
HDRTXT = HDR.read_text()

_defs = {}
for line in HDRTXT.splitlines():
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


def reg_off(struct, field):
    """Offset of a register from its comment in the device header."""
    end = re.search(r'\}\s*%s;' % struct, HDRTXT).start()
    blk = HDRTXT[HDRTXT.rindex('typedef struct', 0, end):end]
    m = re.search(r'__IO uint32_t %s;[^\n]*?Address offset: (0x[0-9A-Fa-f]+)' % field, blk)
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
    obj = tmp / 'clock_fw.o'
    run(['arm-none-eabi-g++', *cpu, '-std=gnu++23', '-Wall', '-Werror', '-fno-exceptions', '-fno-rtti', *inc,
         '-c', HERE / 'clock_fw.cpp', '-o', obj])
    objs.append(obj)
    obj = tmp / 'system_stm32wbaxx.o'
    run(['arm-none-eabi-gcc', *cpu, '-std=gnu17', '-Wall', '-Werror', *inc, '-c',
         ROOT / 'ARM/ST/STM32WBAxx/src/system_stm32wbaxx.c', '-o', obj])
    objs.append(obj)
    elf = tmp / 'clock_fw.elf'
    run(['arm-none-eabi-g++', *cpu, '-nostartfiles', '-Wl,--gc-sections', '-Wl,-T,' + str(HERE / 'clock_fw.ld'),
         '--specs=nosys.specs', '--specs=nano.specs', *objs, '-o', elf])
    return elf


HSI = 16_000_000
HSE = 32_000_000


class Log:
    def __init__(self):
        self.ev = []
        self.mark = -1

    def add(self, e):
        self.ev.append((self.mark, *e))

    def of(self, mark):
        return [e[1:] for e in self.ev if e[0] == mark]


class Clocks:
    """RCC, PWR and flash registers of the clock setup."""

    def __init__(self, log, violations):
        self.log = log
        self.viol = violations
        self.hse_works = True
        self.reg = collections.defaultdict(int)
        self.cr = (1 << hdr('RCC_CR_HSION_Pos')) | (1 << hdr('RCC_CR_HSIRDY_Pos'))
        self.sw = 0
        self.vosr = 0
        self.vosrdy = 0
        self.actvosrdy = 0
        # Reset state: 16 MHz runs in voltage range 2 with one wait state
        self.acr = 1
        self.icache_cr = hdr('ICACHE_CR_WAYSEL')
        self.reg[('PWR', reg_off('PWR_TypeDef', 'CR2'))] = hdr('PWR_CR2_ICRAMPDS')
        self.pll_cfg = None
        self.pll_up = False
        self.max_freq = 0
        self.o = {n: reg_off('RCC_TypeDef', n) for n in ('CR', 'CFGR1', 'CFGR2', 'CFGR3', 'PLL1CFGR', 'PLL1DIVR')}
        self.po = {n: reg_off('PWR_TypeDef', n) for n in ('VOSR', 'SVMSR')}
        self.fo = reg_off('FLASH_TypeDef', 'ACR')

    def v(self, what):
        self.viol.append(f'{what} (mark {self.log.mark})')

    def bit(self, name):
        return 1 << hdr(name)

    # ---- frequency ---------------------------------------------------------
    def pll_out(self):
        cfg = self.reg[self.o['PLL1CFGR']]
        div = self.reg[self.o['PLL1DIVR']]
        src = HSE if (cfg & 3) == 3 else HSI
        if self.cr & self.bit('RCC_CR_HSEPRE_Pos'):
            src = src // 2 if (cfg & 3) == 3 else src
        m = ((cfg >> 8) & 7) + 1
        n = (div & 0x1FF) + 1
        r = ((div >> 24) & 0x7F) + 1
        return src // m * n // r

    def sysclk(self):
        if self.sw == 0:
            return HSI
        if self.sw == 2:
            return HSE >> ((self.cr >> hdr('RCC_CR_HSEPRE_Pos')) & 1)
        f = self.pll_out()
        if self.reg[self.o['PLL1CFGR']] & self.bit('RCC_PLL1CFGR_PLL1RCLKPRE_Pos'):
            f //= 2
        return f

    def hclk(self):
        h = (self.reg[self.o['CFGR2']] & 7)
        shift = {0: 0, 1: 0, 2: 0, 3: 0, 4: 1, 5: 2, 6: 3, 7: 4}[h]
        return self.sysclk() >> shift

    def allowed(self):
        """Highest HCLK the wait states and the voltage scaling allow."""
        ws = self.acr & 0xF
        if self.vosr & self.bit('PWR_VOSR_VOS_Pos') and self.actvosrdy:
            return {0: 32_000_000, 1: 64_000_000, 2: 96_000_000}.get(ws, 100_000_000)
        return {0: 8_000_000}.get(ws, 16_000_000)

    def check_speed(self, what):
        h = self.hclk()
        self.max_freq = max(self.max_freq, h)
        if h > self.allowed():
            self.v(f'{what}: HCLK {h} above {self.allowed()} allowed by wait states {self.acr & 0xF} and voltage scaling')

    # ---- register access ---------------------------------------------------
    def rd(self, off, base):
        if base == 'ICACHE':
            return self.icache_cr if off == 0 else 0
        if base == 'RCC':
            if off == self.o['CR']:
                return self.cr
            if off == self.o['CFGR1']:
                return (self.reg[off] & ~0xC) | (self.sw << hdr('RCC_CFGR1_SWS_Pos'))
            if off == self.o['PLL1CFGR']:
                v = self.reg[off]
                if not v & self.bit('RCC_PLL1CFGR_PLL1RCLKPRE_Pos') and self.pll_up:
                    v |= self.bit('RCC_PLL1CFGR_PLL1RCLKPRERDY_Pos')
                return v
        elif base == 'PWR':
            if off == self.po['VOSR']:
                return self.vosr | (self.bit('PWR_VOSR_VOSRDY_Pos') if self.vosrdy else 0)
            if off == self.po['SVMSR']:
                return (self.bit('PWR_SVMSR_ACTVOSRDY_Pos') if self.actvosrdy else 0) | \
                       (self.bit('PWR_SVMSR_ACTVOS_Pos') if self.vosr & self.bit('PWR_VOSR_VOS_Pos') else 0)
        elif base == 'FLASH':
            if off == self.fo:
                return self.acr
        return self.reg[(base, off)] if base != 'RCC' else self.reg[off]

    def wr(self, off, base, val):
        if base == 'ICACHE':
            if off != 0:
                self.v('unexpected ICACHE register write')
            if self.icache_cr & hdr('ICACHE_CR_EN'):
                self.v('reconfigured an already enabled cache')
            self.icache_cr = val
            self.log.add(('ICACHE', val))
        elif base == 'RCC':
            self.wr_rcc(off, val)
        elif base == 'PWR':
            if off == self.po['VOSR']:
                self.vosr = val & ~self.bit('PWR_VOSR_VOSRDY_Pos')
                self.vosrdy = self.log.mark != 6
                self.actvosrdy = self.log.mark != 7
                self.log.add(('VOS', (val >> hdr('PWR_VOSR_VOS_Pos')) & 1))
            else:
                self.reg[(base, off)] = val
        elif base == 'FLASH':
            if off == self.fo:
                self.acr = val
                self.log.add(('WS', val & 0xF))
                self.check_speed('flash latency')
            else:
                self.reg[(base, off)] = val

    def wr_rcc(self, off, val):
        o = self.o
        if off == o['CR']:
            old = self.cr
            on = lambda n: bool(val & self.bit(f'RCC_CR_{n}ON_Pos'))
            if on('HSE'):
                if not old & self.bit('RCC_CR_HSEON_Pos'):
                    self.log.add(('HSEON',))
                self.cr |= self.bit('RCC_CR_HSEON_Pos')
                if self.hse_works:
                    self.cr |= self.bit('RCC_CR_HSERDY_Pos')
            else:
                if self.sw == 2 or (self.sw == 3 and (self.reg[o['PLL1CFGR']] & 3) == 3):
                    self.v('HSE switched off while it is the system clock source')
                if old & self.bit('RCC_CR_HSEON_Pos'):
                    self.log.add(('HSEOFF',))
                self.cr &= ~(self.bit('RCC_CR_HSEON_Pos') | self.bit('RCC_CR_HSERDY_Pos'))
            if on('HSI'):
                self.cr |= self.bit('RCC_CR_HSION_Pos') | self.bit('RCC_CR_HSIRDY_Pos')
            if val & self.bit('RCC_CR_HSEPRE_Pos'):
                self.cr |= self.bit('RCC_CR_HSEPRE_Pos')
            else:
                self.cr &= ~self.bit('RCC_CR_HSEPRE_Pos')
            pllon = bool(val & self.bit('RCC_CR_PLL1ON_Pos'))
            if pllon and not self.pll_up:
                self.pll_start()
            elif not pllon and self.pll_up:
                if self.log.mark == 10:
                    return
                if self.sw == 3:
                    self.v('PLL1 switched off while it is the system clock')
                self.pll_up = False
                self.cr &= ~(self.bit('RCC_CR_PLL1ON_Pos') | self.bit('RCC_CR_PLL1RDY_Pos'))
                self.log.add(('PLLOFF',))
            if old & self.bit('RCC_CR_HSION_Pos') and not on('HSI') and self.sw == 0:
                self.v('HSI switched off while it is the system clock')
        elif off == o['CFGR1']:
            sw = val & 3
            if self.log.mark == 9 and sw == 0:
                self.reg[off] = val
                return
            if sw != self.sw:
                if sw == 3 and not self.pll_up:
                    self.v('SW to PLL1R with PLL1 not ready')
                if sw == 3 and not self.reg[o['PLL1CFGR']] & self.bit('RCC_PLL1CFGR_PLL1RCLKPRE_Pos'):
                    self.v('SW to PLL1R without the PLL1R pre divider set')
                if sw == 2 and not self.cr & self.bit('RCC_CR_HSERDY_Pos'):
                    self.v('SW to HSE with HSE not ready')
                if sw == 1:
                    self.v('SW reserved value')
                self.sw = sw
                self.log.add(('SW', sw))
                self.check_speed('clock switch')
            self.reg[off] = val
        elif off == o['PLL1CFGR']:
            old = self.reg[off]
            self.reg[off] = val
            if self.pll_up and (val ^ old) & ~(self.bit('RCC_PLL1CFGR_PLL1RCLKPRE_Pos') |
                                               self.bit('RCC_PLL1CFGR_PLL1RCLKPRESTEP_Pos') |
                                               self.bit('RCC_PLL1CFGR_PLL1RCLKPRERDY_Pos')):
                self.v('PLL1CFGR configuration changed while PLL1 runs')
            if (val ^ old) & self.bit('RCC_PLL1CFGR_PLL1RCLKPRE_Pos'):
                self.log.add(('PRE', 1 if val & self.bit('RCC_PLL1CFGR_PLL1RCLKPRE_Pos') else 0))
                self.check_speed('PLL1R pre divider')
        elif off == o['PLL1DIVR']:
            if self.pll_up:
                self.v('PLL1DIVR written while PLL1 runs')
            self.reg[off] = val
        else:
            self.reg[off] = val
            if off in (o['CFGR2'], o['CFGR3']):
                self.log.add(('CFG', off, val))
                self.check_speed('bus divider')

    def pll_start(self):
        cfg = self.reg[self.o['PLL1CFGR']]
        div = self.reg[self.o['PLL1DIVR']]
        src = HSE if (cfg & 3) == 3 else HSI if (cfg & 3) == 2 else None
        m = ((cfg >> 8) & 7) + 1
        n = (div & 0x1FF) + 1
        r = ((div >> 24) & 0x7F) + 1
        if src is None:
            self.v('PLL1 started with no source selected')
            return
        if (cfg & 3) == 3 and not self.cr & self.bit('RCC_CR_HSERDY_Pos'):
            self.v('PLL1 started from HSE with HSE not ready')
            return
        vin = src // m
        vco = vin * n
        if not 4_000_000 <= vin <= 16_000_000:
            self.v(f'PLL1 input {vin} out of range')
        if not 128_000_000 <= vco <= 544_000_000:
            self.v(f'PLL1 VCO {vco} out of range')
        rge = (cfg >> 2) & 3
        if rge != (3 if vin > 8_000_000 else 0):
            self.v(f'PLL1 input range bits {rge} for input {vin}')
        if not cfg & self.bit('RCC_PLL1CFGR_PLL1REN_Pos'):
            self.v('PLL1R output not enabled')
        self.pll_up = True
        self.cr |= self.bit('RCC_CR_PLL1ON_Pos') | self.bit('RCC_CR_PLL1RDY_Pos')
        self.log.add(('PLL', 'HSE' if src == HSE else 'HSI', m, n, r, vin, vco))


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

    with tempfile.TemporaryDirectory(prefix='wba-clock-') as tmp:
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
        clk = Clocks(log, viol)
        rcc = hdr('RCC_BASE_NS')
        pwr = hdr('PWR_BASE_NS')
        fla = hdr('FLASH_R_BASE_NS')
        plain = collections.defaultdict(int)

        def where(a):
            for name, base, size in (('RCC', rcc, 0x400), ('PWR', pwr, 0x400), ('FLASH', fla, 0x400),
                                     ('ICACHE', hdr('ICACHE_BASE_NS'), 0x400)):
                if base <= a < base + size:
                    return name, a - base
            return None, 0

        def rd(uc, off, size, base):
            a = base + off
            name, o = where(a)
            if name:
                return clk.rd(o, name)
            return plain[a]

        def wr(uc, off, size, val, base):
            a = base + off
            name, o = where(a)
            if name:
                clk.wr(o, name, val)
                return
            plain[a] = val

        for b, s in ((0x40000000, 0x40000), (0x42020000, 0x10000), (0x46000000, 0x30000)):
            uc.mmio_map(b, s, rd, b, wr, b)

        def on_mark(uc, addr, size, user):
            log.mark = uc.reg_read(UC_ARM_REG_R0)
            clk.hse_works = log.mark != 4
            if log.mark == 1:
                # A boot loader/application's enabled two-way mode is retained.
                clk.icache_cr |= hdr('ICACHE_CR_WAYSEL')

        state = {'done': False}

        def on_done(uc, addr, size, user):
            state['done'] = True
            uc.emu_stop()

        uc.hook_add(UC_HOOK_CODE, on_mark, begin=syms['ClkTestMark'] & ~1, end=syms['ClkTestMark'] & ~1)
        uc.hook_add(UC_HOOK_CODE, on_done, begin=syms['ClkTestDoneHook'] & ~1, end=syms['ClkTestDoneHook'] & ~1)

        vec = bytes(uc.mem_read(0x08000000, 8))
        uc.reg_write(UC_ARM_REG_SP, int.from_bytes(vec[0:4], 'little'))
        pc = int.from_bytes(vec[4:8], 'little')
        try:
            uc.emu_start(pc, 0xFFFFFFFF, count=50_000_000)
        except Exception as ex:
            raise SystemExit(f'FAIL emulator stopped at {uc.reg_read(UC_ARM_REG_PC):#x}: {ex}')
        if not state['done']:
            raise SystemExit('FAIL firmware did not finish')

        res_addr = syms['g_ClkTestResult']
        n = int.from_bytes(uc.mem_read(res_addr + 4 * 63, 4), 'little', signed=True)
        res = [int.from_bytes(uc.mem_read(res_addr + 4 * i, 4), 'little', signed=True) for i in range(n)]

    errors = []
    checks = 0

    def expect(cond, msg):
        nonlocal checks
        checks += 1
        if not cond:
            errors.append(msg)

    P = log.of
    f100 = 100_000_000

    # 0 default: HSI16 into PLL1, 100 MHz, buses undivided
    expect(res[0:4] == [f100] * 4, f'default clocks {res[0:4]}')
    expect(res[4] == 0, f'default oscillator type {res[4]}')
    e = P(0)
    pll = [x for x in e if x[0] == 'PLL']
    expect(len(pll) == 1 and pll[0][1] == 'HSI' and pll[0][4] * 0 == 0, f'PLL setup from HSI {pll}')
    if pll:
        _, s, m, nn, r, vin, vco = pll[0]
        expect(vin // 1 * nn // r == f100 and vco == 200_000_000, f'PLL1 settings {pll[0]}')
    kinds = [x[0] for x in e]
    expect(kinds.index('VOS') < kinds.index('WS') < kinds.index('PLL') < kinds.index('SW'),
           f'order of the steps {kinds}')
    expect(('VOS', 1) in e and ('WS', 3) in e, 'range 1 and 3 wait states for 100 MHz')
    expect(('SW', 3) in e, 'no switch to PLL1R')
    pre = [x[1] for x in e if x[0] == 'PRE']
    expect(pre and pre[-1] == 0 and 1 in pre, f'PLL1R pre divider sequence {pre}')
    expect(kinds.index('SW') < len(kinds) - 1 and e[-1] == ('PRE', 0), 'pre divider not released after the switch')

    expect([x[1] for x in P(0) if x[0] == 'ICACHE'] == [0, hdr('ICACHE_CR_EN')],
           'cold start must select direct-mapped mode before enabling ICACHE')
    expect(clk.reg[('PWR', reg_off('PWR_TypeDef', 'CR2'))] & hdr('PWR_CR2_ICRAMPDS') == 0,
           'ICACHE RAM retention not enabled')
    expect(clk.icache_cr == hdr('ICACHE_CR_EN') | hdr('ICACHE_CR_WAYSEL'),
           'enabled cache mode was not preserved across clock changes')
    expect(not any(x[0] == 'ICACHE' for mark in range(1, 11) for x in P(mark)),
           'clock changes or fallback reconfigured an enabled cache')

    # 1 crystal: HSE into PLL1
    expect(res[5] == 1, 'SystemCoreClockSelect(XTAL, 32 MHz) failed')
    expect(res[6:10] == [f100] * 4, f'HSE clocks {res[6:10]}')
    expect(res[10] == 1, f'oscillator type after select {res[10]}')
    e = P(1)
    pll = [x for x in e if x[0] == 'PLL']
    expect(len(pll) == 1 and pll[0][1] == 'HSE' and pll[0][2] * 0 == 0, f'PLL setup from HSE {pll}')
    if pll:
        _, s, m, nn, r, vin, vco = pll[0]
        expect(HSE // m * nn // r == f100 and vco == 200_000_000 and (m, nn, r) == (4, 25, 2), f'PLL1 HSE settings {pll[0]}')
    kinds = [x[0] for x in e]
    expect(kinds.index('HSEON') < kinds.index('PLL'), 'HSE started before PLL1')
    expect(('SW', 0) in e and kinds.index(('SW')) < kinds.index('PLLOFF'), 'back to HSI16 before PLL1 is turned off')

    # 2 refused frequency
    expect(res[11] == 0 and res[12] == f100, f'24 MHz crystal: {res[11:13]}')
    expect(not P(2), f'registers touched by a refused select: {P(2)[:3]}')

    # 3 back to RC
    expect(res[13] == 1 and res[14:18] == [f100] * 4, f'RC select {res[13:18]}')
    k3 = [x[0] for x in P(3)]
    expect('HSEOFF' in k3 and 'HSEON' not in k3, f'HSE not stopped for the RC select: {k3}')

    # 4 crystal that does not start
    expect(res[18] == 1 and res[19:23] == [f100] * 4, f'failed crystal clocks {res[18:23]}')
    expect(res[23] == 0 and res[24] == HSI, f'failed crystal recorded as type {res[23]} freq {res[24]}')
    pll = [x for x in P(4) if x[0] == 'PLL']
    expect(len(pll) == 1 and pll[0][1] == 'HSI', f'fallback PLL source {pll}')

    # 5 bus prescalers
    expect(res[25:29] == [50_000_000, 25_000_000, f100 >> 4, 0], f'prescaler results {res[25:29]}')
    expect(res[29:33] == [f100, 50_000_000, 25_000_000, f100 >> 4], f'clocks after the prescalers {res[29:33]}')

    expect(res[33:37] == [HSI] * 4, f'VOSRDY failure clocks {res[33:37]}')
    expect(res[37:41] == [HSI] * 4, f'ACTVOSRDY failure clocks {res[37:41]}')
    for mark in (6, 7):
        expect(not any(e[0] == 'PLL' for e in P(mark)),
               f'PLL started without voltage acknowledgement at mark {mark}')
    expect(res[41:45] == [f100] * 4, f'voltage recovery clocks {res[41:45]}')
    expect(res[45:49] == [f100] * 4, f'HSI switch failure clocks {res[45:49]}')
    expect(not any(e[0] == 'PLLOFF' for e in P(9)),
           'PLL stopped after the switch to HSI failed')
    expect(res[49:53] == [HSI] * 4, f'PLL stop failure clocks {res[49:53]}')

    # SystemCoreClockUpdate decodes the registers: the value matches the model
    expect(clk.sysclk() == HSI, f'model sysclk {clk.sysclk()}')
    expect(clk.max_freq <= f100, f'peak HCLK {clk.max_freq}')

    expect(not viol, f'model violations: {viol[:6]}')

    for m in errors:
        print('FAIL:', m)
    if errors:
        raise SystemExit(f'STM32WBA65 clock: {len(errors)} of {checks} checks failed')
    print(f'STM32WBA65 core clock on the emulator: {checks} checks, 0 failures')
    print('Hardware not tested.')


if __name__ == '__main__':
    main()
