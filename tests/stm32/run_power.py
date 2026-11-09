#!/usr/bin/env python3
"""Build the STM32 power functions with power_fw.cpp and run them in Unicorn.

Run: python3 tests/stm32/run_power.py

POWER_PARTS=STM32L476xx,STM32H753xx runs only these parts.

The power functions of system_core_clock.h, ARM/ST/src/system_power_stm32.cpp,
are built at -O0 and -Os for the parts of PARTS: F0, F4, L1, L4, L4+, G4 and
H7. The emulator calls each function directly against a model of the RCC
reset flags, PWR, RTC backup registers and alarm flag, GPIO pull resistors
and SCB SCR. The register addresses and bits come from the CMSIS header
through power_fw.cpp; the wake up pins and the RAM kept in power off come
from the reference manuals and datasheets in the tables below. The model
applies:

- reset flags cleared by RMVF, held cleared while RMVF is set (H7)
- PWR Standby and wake up flags cleared by CSBF and CWUF (F0, F4, L1), SCR
  (L4, G4: CSBF, CWUF1 to CWUF5, WUFI not cleared), WKUPCR and CPUCR CSSF
  (H7); EWUP of PWR CSR writable, SBF and WUF read only
- backup register and backup SRAM regulator writes only with DBP set; BRRDY
  set with BREN
- PWR CR4 read back after its write before the low power mode (L4 errata)

Checks: reset reason of a power on, the reset pin, a software reset, the
IWDG, a reset instead of a low power mode, a Standby exit by a pin, by a
timer and by the reset pin, the cached value and the flags cleared; the
retention registers; the wake up pins accepted and refused; the registers
written by SystemPowerOff up to WFI, Standby or Shutdown and the RAM kept;
SystemRamRetain ranges. Requires arm-none-eabi-g++, unicorn and pyelftools.
Hardware not tested by this script.

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
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE
from unicorn import arm_const
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR, UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2
from elftools.elf.elffile import ELFFile
from elftools.elf.sections import SymbolTableSection

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent

PERIPH = 0x40000000
PERIPH_SIZE = 0x20000000
SCS = 0xE000E000
RET = 0x081FF000

A, B, C, D, E, F, G, H, I = range(9)

# Part define, family folder, CPU, WKUP pins of the datasheet (WKUP1 first),
# wake up on low level
PARTS = [
	('STM32F030x8', 'STM32F0xx', 'M0', [(A, 0), (C, 13)], False),
	('STM32F401xC', 'STM32F4xx', 'M4', [(A, 0)], False),
	('STM32L152xE', 'STM32L1xx', 'M3', [(A, 0), (C, 13), (E, 6)], False),
	('STM32L432xx', 'STM32L4xx', 'M4', [(A, 0), (C, 13), (E, 6), (A, 2), (C, 5)], True),
	('STM32L476xx', 'STM32L4xx', 'M4', [(A, 0), (C, 13), (E, 6), (A, 2), (C, 5)], True),
	('STM32L496xx', 'STM32L4xx', 'M4', [(A, 0), (C, 13), (E, 6), (A, 2), (C, 5)], True),
	('STM32L4S9xx', 'STM32L4xx', 'M4', [(A, 0), (C, 13), (E, 6), (A, 2), (C, 5)], True),
	('STM32G474xx', 'STM32G4xx', 'M4', [(A, 0), (C, 13), (E, 6), (A, 2), (C, 5)], True),
	('STM32H753xx', 'STM32H7xx', 'M7', [(A, 0), (A, 2), (I, 8), (C, 13), (I, 11), (C, 1)], True),
]
GCC_CPU = {'M0': 'cortex-m0', 'M3': 'cortex-m3', 'M4': 'cortex-m4', 'M7': 'cortex-m7'}

POWERON, PIN, WDT, SOFT, WAKE_PIN, WAKE_TIMER = 1, 2, 4, 8, 0x100, 0x1000
DISABLE, LOW, HIGH, TOGGLE = 0, 1, 2, 3


def run(cmd):
	p = subprocess.run(cmd, capture_output=True, text=True)
	if p.returncode:
		raise RuntimeError(' '.join(cmd) + '\n' + p.stdout + p.stderr)


class Fail(Exception):
	pass


class Model:
	def __init__(self, t):
		self.t = t
		self.fam = t[0]
		self.rst, self.rmvf = t[1], t[2]
		self.dbp_reg, self.dbp = t[25], t[26]
		self.bkp, self.nbkp = t[10], t[11]
		self.mem = {}
		self.log = []
		if self.fam == 1:
			self.cr, self.csr, self.csbf, self.cwuf, self.pdds, self.sbf, self.wuf, self.ewup = t[30:38]
		elif self.fam == 2:
			(self.cr1, self.cr3, self.cr4, self.sr1, self.scr, self.pucra, self.lpms, self.standby,
			 self.shutdown, self.rrs, self.apc, self.sbf, self.wufi, self.csbf, self.cwuf) = t[30:45]
			self.cr4_read = True
		else:
			(self.cr2, self.cpucr, self.wkupcr, self.wkupfr, self.wkupepr, self.bren, self.brrdy,
			 self.pdds, self.cssf, self.sbf) = t[30:40]

	def get(self, a):
		return self.mem.get(a, 0)

	def set(self, a, v):
		self.mem[a] = v & 0xFFFFFFFF

	def read(self, uc, offset, size, base):
		a = base + offset
		v = self.get(a & ~3)
		if self.fam == 1 and a == self.cr:
			v &= ~(self.csbf | self.cwuf)
		elif self.fam == 2:
			if a == self.scr:
				v = 0
			elif a == self.cr4:
				self.cr4_read = True
		elif self.fam == 3:
			if a == self.wkupcr:
				v = 0
			elif a == self.cpucr:
				v &= ~self.cssf
			elif a == self.cr2:
				v = (v & ~self.brrdy) | (self.brrdy if v & self.bren else 0)
		return (v >> (8 * (a & 3))) & ((1 << (8 * size)) - 1)

	def write(self, uc, offset, size, value, base):
		a = base + offset
		if size != 4:
			old = self.get(a & ~3)
			sh = 8 * (a & 3)
			mask = ((1 << (8 * size)) - 1) << sh
			value = (old & ~mask) | ((value << sh) & mask)
			a &= ~3
		self.log.append((a, value))
		if a == self.rst:
			self.set(a, self.rmvf if value & self.rmvf else value)
			return
		if self.bkp and self.bkp <= a < self.bkp + 4 * self.nbkp:
			if not self.get(self.dbp_reg) & self.dbp:
				raise Fail(f'backup register 0x{a:x} written without DBP')
		if self.fam == 1:
			if a == self.cr:
				csr = self.get(self.csr)
				if value & self.csbf:
					csr &= ~self.sbf
				if value & self.cwuf:
					csr &= ~self.wuf
				self.set(self.csr, csr)
				value &= ~(self.csbf | self.cwuf)
			elif a == self.csr:
				value = (self.get(a) & (self.sbf | self.wuf)) | (value & ~(self.sbf | self.wuf))
		elif self.fam == 2:
			if a == self.scr:
				sr1 = self.get(self.sr1)
				if value & self.csbf:
					sr1 &= ~self.sbf
				sr1 &= ~(value & self.cwuf)
				self.set(self.sr1, sr1)
				return
			if a == self.cr4:
				self.cr4_read = False
			if a == self.cr1 and (value & self.lpms) != (self.get(a) & self.lpms) and not self.cr4_read:
				raise Fail('low power mode set before PWR_CR4 was read back')
		else:
			if a == self.wkupcr:
				self.set(self.wkupfr, self.get(self.wkupfr) & ~value)
				return
			if a == self.cpucr:
				if value & self.cssf:
					value &= ~self.sbf
				value &= ~self.cssf
			if a == self.cr2 and not self.get(self.dbp_reg) & self.dbp:
				raise Fail('PWR_CR2 written without DBP')
		self.set(a, value)


class Emu:
	def __init__(self, path, cpu):
		self.uc = uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
		uc.ctl_set_cpu_model(getattr(arm_const, 'UC_CPU_ARM_CORTEX_' + cpu))
		uc.mem_map(0x08000000, 0x200000)
		uc.mem_map(0x20000000, 0x80000)
		self.sym = {}
		with path.open('rb') as f:
			elf = ELFFile(f)
			for seg in elf.iter_segments():
				if seg['p_type'] == 'PT_LOAD':
					uc.mem_write(seg['p_vaddr'], seg.data())
			for sec in elf.iter_sections():
				if isinstance(sec, SymbolTableSection):
					for s in sec.iter_symbols():
						if s.name:
							self.sym[s.name] = (s['st_value'], s['st_size'])
		ta = self.sym['g_PwrTestAddr'][0]
		n = self.sym['g_PwrTestAddr'][1] // 4
		self.t = list(struct.unpack(f'<{n}I', bytes(uc.mem_read(ta, 4 * n))))
		self.m = Model(self.t)
		uc.mmio_map(PERIPH, PERIPH_SIZE, self.m.read, PERIPH, self.m.write, PERIPH)
		uc.mem_map(0xE0000000, SCS - 0xE0000000)
		uc.mmio_map(SCS, 0x1000, self.m.read, SCS, self.m.write, SCS)
		uc.mem_map(SCS + 0x1000, 0xE0100000 - SCS - 0x1000)
		self.wfi = False
		off, sz = self.sym['SystemPowerOff']
		uc.hook_add(UC_HOOK_CODE, self.code, begin=off & ~1, end=(off & ~1) + sz)

	def code(self, uc, address, size, _):
		if size == 2 and bytes(uc.mem_read(address, 2)) == b'\x30\xbf':
			self.wfi = True
			uc.emu_stop()

	def static(self, name):
		for s, (v, sz) in self.sym.items():
			if s.endswith(name) and s.startswith('_ZL'):
				return v, sz
		raise KeyError(name)

	def poke(self, name, value):
		v, sz = self.static(name)
		self.uc.mem_write(v, value.to_bytes(sz, 'little'))

	def call(self, fn, *args):
		uc = self.uc
		self.m.log.clear()
		self.wfi = False
		for r, v in zip((UC_ARM_REG_R0, UC_ARM_REG_R1, UC_ARM_REG_R2), args):
			uc.reg_write(r, v & 0xFFFFFFFF)
		uc.reg_write(UC_ARM_REG_LR, RET | 1)
		uc.reg_write(UC_ARM_REG_SP, 0x2007FF00)
		uc.emu_start(self.sym[fn][0] | 1, RET, count=2_000_000)
		return uc.reg_read(UC_ARM_REG_R0)


def check(what, got, exp):
	if got != exp:
		raise Fail(f'{what}: {got}, expected {exp}')


def test_reset(e):
	t, m = e.t, e.m
	pinf, pwrf, sft, iwdg, lpwr = t[3], t[4], t[5], t[6], t[7]
	rtc_isr, alraf = t[8], t[9]

	def standby(sb, pin, timer):
		if m.fam == 1:
			m.set(m.csr, (m.sbf if sb else 0) | (m.wuf if pin or timer else 0))
			if rtc_isr:
				m.set(rtc_isr, alraf if timer else 0)
		elif m.fam == 2:
			m.set(m.sr1, (m.sbf if sb else 0) | (2 if pin else 0) | (m.wufi if timer else 0))
		else:
			m.set(m.cpucr, m.sbf if sb else 0)
			m.set(m.wkupfr, 8 if pin else 0)
			m.set(rtc_isr, alraf if timer else 0)

	def reason(rst, sb=False, pin=False, timer=False):
		e.poke('s_bStPwrResetRead', 0)
		m.set(m.rst, rst)
		standby(sb, pin, timer)
		r = e.call('SystemResetReason')
		check('reset flags cleared, RMVF clear', m.get(m.rst), 0)
		if m.fam == 1:
			check('SBF and WUF cleared', m.get(m.csr) & (m.sbf | m.wuf), 0)
		elif m.fam == 2:
			check('SBF and WUF cleared', m.get(m.sr1) & (m.sbf | 0x1F), 0)
		else:
			check('SBF and WKUPF cleared', (m.get(m.cpucr) & m.sbf, m.get(m.wkupfr)), (0, 0))
		return r

	cases = [
		('power on', reason(pwrf | pinf), POWERON),
		('reset pin', reason(pinf), PIN),
		('software', reason(sft | pinf), SOFT),
		('IWDG', reason(iwdg | pinf), WDT),
		('low power mode reset', reason(lpwr | pinf), SOFT),
		('Standby exit by a pin', reason(pwrf | pinf, True, True), WAKE_PIN),
		('Standby exit by the reset pin', reason(pinf, True), PIN),
		('no flag', reason(0), POWERON),
	]
	if m.fam != 1 or rtc_isr:
		cases.append(('Standby exit by a timer', reason(pwrf | pinf, True, False, True), WAKE_TIMER))
	for what, got, exp in cases:
		check(f'reset reason, {what}', hex(got), hex(exp))
	m.set(m.rst, sft)
	got = e.call('SystemResetReason')
	check('second call cached', (hex(got), m.get(m.rst), m.log), (hex(cases[-1][2]), sft, []))
	return len(cases) + 1


def test_retreg(e):
	t, m = e.t, e.m
	n = t[11]
	check('retention registers', e.call('SystemRetRegCount'), n)
	for i in range(n):
		check(f'write {i}', e.call('SystemRetRegWrite', i, 0xA5000000 + i), 1)
		check(f'register {i}', m.get(t[10] + 4 * i), 0xA5000000 + i)
		check(f'read {i}', e.call('SystemRetRegRead', i), 0xA5000000 + i)
	check(f'write {n} refused', (e.call('SystemRetRegWrite', n, 1), m.log), (0, []))
	check('read -1', e.call('SystemRetRegRead', -1), 0)
	return 3 * n + 3


def test_wakepin(e, pins, low):
	n = 0
	for p, b in pins:
		check(f'P{chr(65 + p)}{b} high', e.call('SystemWakePin', p, b, HIGH), 1)
		check(f'P{chr(65 + p)}{b} low', e.call('SystemWakePin', p, b, LOW), 1 if low else 0)
		check(f'P{chr(65 + p)}{b} toggle refused', e.call('SystemWakePin', p, b, TOGGLE), 0)
		check(f'P{chr(65 + p)}{b} disable', e.call('SystemWakePin', p, b, DISABLE), 1)
		n += 4
	for p, b in [(B, 0), (A, 1), (C, 12), (A, 16)]:
		if (p, b) not in pins:
			check(f'P{chr(65 + p)}{b} refused', e.call('SystemWakePin', p, b, HIGH), 0)
			n += 1
	return n


def gpio(e, port):
	return e.t[13 + port]


def set_pull(e, port, pin, pull):
	m = e.m
	a = gpio(e, port) + 0x0C
	m.set(a, (m.get(a) & ~(3 << (2 * pin))) | (pull << (2 * pin)))


def test_poweroff(e, pins, low):
	t, m = e.t, e.m
	scr = t[12]
	n = 0
	for p, b in pins:
		e.call('SystemWakePin', p, b, DISABLE)

	def off():
		m.set(scr, 0)
		e.call('SystemPowerOff')
		check('WFI reached', e.wfi, True)
		check('SLEEPDEEP', m.get(scr) & 4, 4)

	if m.fam == 1:
		e.call('SystemWakePin', *pins[0], HIGH)
		e.call('SystemWakePin', *pins[-1], HIGH)
		m.set(m.csr, m.sbf | m.wuf)
		off()
		en = 1 | (1 << (len(pins) - 1))
		check('EWUP', (m.get(m.csr) // m.ewup) & 7, en)
		check('PDDS, SBF and WUF cleared', (m.get(m.cr) & m.pdds, m.get(m.csr) & (m.sbf | m.wuf)), (m.pdds, 0))
		ewup_at = next(i for i, (a, v) in enumerate(m.log) if a == m.csr)
		cwuf_at = next(i for i, (a, v) in enumerate(m.log) if a == m.cr and v & m.cwuf)
		check('WUF cleared after EWUP', ewup_at < cwuf_at, True)
		n += 6
	elif m.fam == 2:
		set_pull(e, A, 0, 1)
		set_pull(e, C, 13, 2)
		set_pull(e, A, 2, 0)
		e.call('SystemWakePin', A, 0, LOW)
		e.call('SystemWakePin', C, 13, HIGH)
		e.call('SystemWakePin', A, 2, HIGH)
		e.call('SystemWakePin', A, 2, DISABLE)
		m.set(m.sr1, m.sbf | 0x1F)
		m.set(m.cr3, 0x10)
		m.set(m.pucra + 16, 1 << 13)		# PUCRC, a pull up left from before
		off()
		check('EWUP1 and EWUP2', m.get(m.cr3) & 0x1F, 3)
		check('WP1', m.get(m.cr4) & 0x1F, 1)
		check('PUCRA PA0, PDCRC and PUCRC PC13', (m.get(m.pucra) & 1, m.get(m.pucra + 20) >> 13 & 1,
			  m.get(m.pucra + 16) >> 13 & 1), (1, 1, 0))
		check('APC', m.get(m.cr3) & m.apc, m.apc)
		check('SBF and WUF cleared', m.get(m.sr1) & (m.sbf | 0x1F), 0)
		check('Shutdown, no SRAM2', (m.get(m.cr1) & m.lpms, m.get(m.cr3) & m.rrs), (m.shutdown, 0))
		clr = next(i for i, (a, v) in enumerate(m.log) if a == m.cr3 and v & 0x1F == 0)
		wp = next(i for i, (a, v) in enumerate(m.log) if a == m.cr4)
		en = next(i for i, (a, v) in enumerate(m.log) if a == m.cr3 and v & 0x1F == 3)
		check('EWUP cleared, CR4, EWUP set', clr < wp < en, True)
		e.call('SystemRamRetain', t[22], 16, 1)
		off()
		check('Standby, SRAM2 kept', (m.get(m.cr1) & m.lpms, m.get(m.cr3) & m.rrs), (m.standby, m.rrs))
		e.call('SystemRamRetain', t[22], 16, 0)
		off()
		check('SRAM2 dropped, Shutdown', (m.get(m.cr1) & m.lpms, m.get(m.cr3) & m.rrs), (m.shutdown, 0))
		n += 13
	else:
		set_pull(e, A, 0, 1)
		set_pull(e, C, 13, 2)
		e.call('SystemWakePin', A, 0, LOW)
		e.call('SystemWakePin', C, 13, HIGH)
		m.set(m.wkupfr, 0x3F)
		m.set(m.cpucr, m.sbf)
		m.set(t[25], 0)
		off()
		epr = 1 | 8 | (1 << 8) | (1 << 16) | (2 << 22)
		check('WKUPEPR', hex(m.get(m.wkupepr)), hex(epr))
		check('WKUPF cleared', m.get(m.wkupfr), 0)
		check('PDDS D1 D2 D3, SBF cleared', (m.get(m.cpucr) & m.pdds, m.get(m.cpucr) & m.sbf), (m.pdds, 0))
		check('no backup SRAM', m.get(m.cr2) & m.bren, 0)
		e.call('SystemRamRetain', t[22] + 0xFF0, 16, 1)
		off()
		check('backup SRAM kept', m.get(m.cr2) & m.bren, m.bren)
		n += 9
	return n


def test_ram(e):
	t = e.t
	base, size, alias = t[22], t[23], t[24]
	if base == 0:
		for a in (0x20000000, 0x10000000, 0x38800000):
			check(f'0x{a:x} refused', e.call('SystemRamRetain', a, 16, 1), 0)
		return 3
	cases = [
		('first byte', base, 1, 1), ('whole', base, size, 1), ('last byte', base + size - 1, 1, 1),
		('past the end', base + size - 8, 16, 0), ('after', base + size, 4, 0), ('before', base - 4, 8, 0),
		('size 0', base + 16, 0, 1), ('SRAM1', 0x20000000, 16, 0),
	]
	if alias:
		cases += [('alias', alias, size, 1), ('alias past the end', alias + size - 4, 8, 0)]
	else:
		# Where an alias would follow SRAM1 (L47x), CCM SRAM (G4), after the backup SRAM (H7)
		cases += [('no alias', 0x20018000 if base == 0x10000000 else base + size, 16, 0)]
	for what, a, s, exp in cases:
		check(f'RAM {what}', e.call('SystemRamRetain', a, s, 1), exp)
		e.call('SystemRamRetain', a, s, 0)
	return len(cases)


def main():
	cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
	if not shutil.which(cxx):
		raise SystemExit('NOT RUN: arm-none-eabi-g++ unavailable')
	sel = os.environ.get('POWER_PARTS')
	with tempfile.TemporaryDirectory(prefix='stm32-power-') as td:
		td = Path(td)
		for part, family, cpu, pins, low in PARTS:
			if sel and part not in sel.split(','):
				continue
			for opt in ('-O0', '-Os'):
				flags = [f'-mcpu={GCC_CPU[cpu]}', '-mthumb', '-mfloat-abi=soft', opt,
						 '-std=gnu++23', '-D' + part, '-ffunction-sections', '-fdata-sections',
						 '-fno-rtti', '-fno-exceptions', '-fno-builtin', '-Wall', '-Wextra', '-Werror',
						 '-Wno-missing-field-initializers']
				flags += ['-I' + str(ROOT / d) for d in ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
						  'ARM/ST/include', 'ARM/ST/' + family + '/include')]
				objects = []
				for src in ('tests/stm32/power_fw.cpp', 'ARM/ST/src/system_power_stm32.cpp'):
					obj = td / (Path(src).stem + '.o')
					run([cxx, *flags, '-c', str(ROOT / src), '-o', str(obj)])
					objects.append(str(obj))
				exe = td / 'power.elf'
				run([cxx, *flags, '-nostdlib', '-Wl,--gc-sections', '-Wl,-u,g_PwrTestAddr', '-T' + str(HERE / 'power_fw.ld'),
					 *objects, '-lgcc', '-o', str(exe)])
				n = 0
				try:
					for test in (test_reset, test_retreg, test_ram):
						n += test(Emu(exe, cpu))
					n += test_wakepin(Emu(exe, cpu), pins, low)
					n += test_poweroff(Emu(exe, cpu), pins, low)
				except (Fail, UcError) as ex:
					raise SystemExit(f'FAIL {part} {opt}: {ex}')
				print(f'{part} {opt}: {n} checks PASS', flush=True)
	print('Register model only. Hardware not tested by this script.')


if __name__ == '__main__':
	main()
