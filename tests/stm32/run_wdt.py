#!/usr/bin/env python3
"""Build the STM32 watchdog driver with wdt_fw.cpp and run it in Unicorn.

Run: python3 tests/stm32/run_wdt.py

WDT_PARTS=STM32L476xx,STM32G474xx runs only these parts, WDT_SRC builds
another copy of the driver.

The driver, ARM/ST/src/wdt_stm32.cpp, is built at -O0 and -Os for the parts
of PARTS: F0, F4, L1, L4, L4+, G4, H7 and WBA. Each scenario of wdt_fw.cpp
runs in its own emulator, against a model of the IWDG in time: usDelay of the
firmware (delay_shim/idelay.h) moves the time. The model applies the rules
of the reference manuals:

- key register: 0xCCCC starts the IWDG (counter from the reload value after
  the LSI start), 0x5555 opens the PR, RLR, WINR and EWCR registers, any
  other key closes them, 0xAAAA reloads the counter
- PR, RLR, WINR and EWCR written only while open and while their update
  flag (PVU, RVU, WVU, EWU) is clear; the update completes 5 counter clock
  periods later, only while the IWDG runs; a reload takes the reload value
  in effect
- the counter clock is the LSI of the datasheets (40 kHz F0, 37 kHz L1,
  32 kHz other families), divided by 128 with the LSI predivider, then by
  the prescaler; the reset comes reload + 1 ticks after the reload
- a reload while the counter is above the window value resets the MCU; a
  window register write reloads the counter
- WBA: SR ONF while the IWDG runs; EWIF when the counter reaches EWIT - 1,
  cleared by EWIC
- no window register on the F4 and L1, no EWCR outside the WBA

The firmware arms each expected reset with its time and cause; the model
checks them. Requires arm-none-eabi-g++, unicorn and pyelftools. Not a
simulation of the LSI dispersion; hardware not tested by this script.

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
from fractions import Fraction
import math
import os
from pathlib import Path
import shutil
import struct
import subprocess
import tempfile

from unicorn import Uc, UcError, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS
from unicorn import arm_const
from unicorn.arm_const import UC_ARM_REG_SP
from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent

CONTROL = 0x5FFF0000
PERIPH = 0x40000000
PERIPH_SIZE = 0x20000000
NVIC = 0xE000E000

# Part define, family folder, CPU, typical LSI frequency of its datasheet
PARTS = [
	('STM32F030x8', 'STM32F0xx', 'M0', 40000), ('STM32F401xC', 'STM32F4xx', 'M4', 32000),
	('STM32L152xE', 'STM32L1xx', 'M3', 37000), ('STM32L412xx', 'STM32L4xx', 'M4', 32000),
	('STM32L476xx', 'STM32L4xx', 'M4', 32000), ('STM32L4S9xx', 'STM32L4xx', 'M4', 32000),
	('STM32G474xx', 'STM32G4xx', 'M4', 32000), ('STM32H753xx', 'STM32H7xx', 'M7', 32000),
	('STM32WBA65xx', 'STM32WBAxx', 'M33', 32000),
]
GCC_CPU = {'M0': 'cortex-m0', 'M3': 'cortex-m3', 'M4': 'cortex-m4', 'M7': 'cortex-m7', 'M33': 'cortex-m33'}
SCENARIOS = {0: 'basic', 1: 'window', 2: 'hardware start', 3: 'boot loader start', 4: 'early wakeup', 5: 'predivider',
			 6: 'hardware start + boot loader'}

KR, PR, RLR, SR, WINR, EWCR = 0x00, 0x04, 0x08, 0x0C, 0x10, 0x14
UPDATE = {PR: 0, RLR: 1, WINR: 2, EWCR: 3}		# PVU, RVU, WVU, EWU bit
SR_ONF, SR_EWIF = 1 << 8, 1 << 14
EWCR_EWIC, EWCR_EWIE = 1 << 14, 1 << 15
LSI_START_US = 100
UPDATE_PERIODS = 5


def run(cmd):
	p = subprocess.run(cmd, capture_output=True, text=True)
	if p.returncode:
		raise RuntimeError(' '.join(cmd) + '\n' + p.stdout + p.stderr)


class Model:
	def __init__(self, uc, lsi, scenario):
		self.uc = uc
		self.lsi_base = lsi
		self.scenario = scenario
		self.ctl = [0] * 32
		self.mem = {}
		self.base = None
		self.window = self.ewi = self.onf = False
		self.pr_max = 6
		self.checked = 0
		self.done = False
		self.reset_cause = None
		self.expect = None
		self.events = []
		self.keys = 0
		self.t = Fraction(0)
		self.running = False
		self.access = False
		self.reg = {PR: 0, RLR: 0xFFF, WINR: 0xFFF, EWCR: 0}
		self.eff = dict(self.reg)
		self.pending = {}				# register: (completion time, value)
		self.r_ref = Fraction(4096)		# ticks left before the reset at t_ref
		self.t_ref = Fraction(0)
		self.last_reload = 0
		self.ewif = False
		self.ewi_armed = False
		self.ewi_time = None

	# LSI after the predivider, counter tick in usec
	def lsi(self):
		addr, bit = self.ctl[13], self.ctl[14]
		if addr and self.plain_read(addr, 4) & bit:
			return self.lsi_base // 128
		return self.lsi_base

	def div(self, pr):
		return 4 << min(pr, self.pr_max)

	def tick(self):
		return Fraction(self.div(self.eff[PR]) * 1000000, self.lsi())

	def left(self, t):
		if t <= self.t_ref:
			return self.r_ref
		return self.r_ref - (t - self.t_ref) / self.tick()

	def counter(self, t):
		return math.ceil(self.left(t)) - 1

	def reload(self, t):
		self.r_ref = Fraction(self.eff[RLR] + 1)
		self.t_ref = t
		self.ewi_armed = True
		self.last_reload = int(t)

	def start(self, t):
		self.running = True
		self.reload(t)
		self.t_ref = t + LSI_START_US

	# Next event of the IWDG: update done, early wakeup, reset
	def next_event(self):
		ev = []
		for reg, (when, _) in self.pending.items():
			if self.running:
				ev.append((when, 'update', reg))
		if self.running:
			ewit = self.eff[EWCR] & 0xFFF
			if self.ewi and self.ewi_armed and ewit > 1:
				ev.append((self.t_ref + max(self.r_ref - ewit, 0) * self.tick(), 'ewi', None))
			ev.append((self.t_ref + self.r_ref * self.tick(), 'reset', 1))
		return min(ev, key=lambda e: e[0]) if ev else None

	def advance(self, t_new):
		while not self.done:
			ev = self.next_event()
			if ev is None or ev[0] > t_new:
				break
			when, kind, arg = ev
			if kind == 'update':
				_, value = self.pending.pop(arg)
				if arg == PR:
					self.r_ref, self.t_ref = self.left(when), max(when, self.t_ref)
				self.eff[arg] = value
				if arg == WINR:
					self.reload(when)
			elif kind == 'ewi':
				self.ewif = True
				self.ewi_armed = False
				self.ewi_time = when
			else:
				self.do_reset(when, arg)
		if not self.done:
			self.t = t_new

	def do_reset(self, when, cause):
		e = self.expect
		assert e is not None, f'unexpected reset (cause {cause}) at {float(when):.1f} us'
		assert e[0] <= when <= e[1], f'reset at {float(when):.1f} us, expected {e[0]} to {e[1]}'
		assert cause == e[2], f'reset cause {cause}, expected {e[2]}'
		if self.ewi and self.eff[EWCR] & EWCR_EWIE:
			# One event, after the early wakeup flag, before the reset
			assert len(self.events) == 1 and self.ewi_time is not None and self.ewi_time <= self.events[0] <= when, \
				f'event {self.events}, flag {self.ewi_time}, reset {float(when)}'
			assert when - self.events[0] <= 2 * self.tick(), 'event more than 2 ticks before the reset'
		self.reset_cause = cause
		self.done = True
		self.uc.emu_stop()

	def update_time(self, reg):
		return self.t + UPDATE_PERIODS * self.tick()

	def sr(self):
		v = 0
		for reg in self.pending:
			v |= 1 << UPDATE[reg]
		if self.onf and self.running:
			v |= SR_ONF
		if self.ewif:
			v |= SR_EWIF
		return v

	# Registers of the other peripherals: memory
	def plain_read(self, addr, size):
		return (self.mem.get(addr & ~3, 0) >> (8 * (addr & 3))) & ((1 << (8 * size)) - 1)

	def plain_write(self, addr, size, value):
		sh = 8 * (addr & 3)
		mask = ((1 << (8 * size)) - 1) << sh
		self.mem[addr & ~3] = (self.mem.get(addr & ~3, 0) & ~mask) | ((value << sh) & mask)

	# MMIO callbacks of the peripheral space and of the NVIC
	# After the reset or the end the emulation stops at the end of the
	# translated block: the accesses until then are ignored
	def read(self, uc, offset, size, region):
		addr = region + offset
		if self.done:
			return 0
		if CONTROL <= addr < CONTROL + 0x80:
			n = (addr - CONTROL) // 4
			val = {0: self.scenario, 9: self.lsi(), 10: int(self.t),
				   11: (1 if self.running else 0) | (2 if self.ewif and self.eff[EWCR] & EWCR_EWIE else 0),
				   16: self.div(self.eff[PR]), 17: self.eff[RLR], 18: self.eff[WINR], 19: self.keys,
				   20: self.last_reload}.get(n, self.ctl[n])
			return val & 0xFFFFFFFF
		if self.base is None or not self.base <= addr < self.base + 0x400:
			return self.plain_read(addr, size)
		assert size == 4, f'IWDG access of {size} bytes'
		off = addr - self.base
		self.check_offset(off)
		if off == SR:
			val = self.sr()
		elif off == KR:
			val = 0
		else:
			val = self.reg[off]
		return val

	def check_offset(self, off):
		assert off in (KR, PR, RLR, SR, WINR, EWCR), f'IWDG offset 0x{off:x}'
		assert off != WINR or self.window, 'window register on a part without it'
		assert off != EWCR or self.ewi, 'EWCR on a part without it'

	def write(self, uc, offset, size, value, region):
		addr = region + offset
		value &= 0xFFFFFFFF
		if self.done:
			return
		if CONTROL <= addr < CONTROL + 0x80:
			self.control(uc, (addr - CONTROL) // 4, value)
			return
		if NVIC + 0x180 <= addr < NVIC + 0x1A0:
			# ICER clears the ISER bits
			self.plain_write(addr - 0x80, 4, self.plain_read(addr - 0x80, 4) & ~value)
			return
		if self.base is None or not self.base <= addr < self.base + 0x400:
			self.plain_write(addr, size, value)
			return
		assert size == 4, f'IWDG access of {size} bytes'
		off = addr - self.base
		self.check_offset(off)
		if off == KR:
			self.keys += 1
			self.access = value == 0x5555
			if value == 0xCCCC and not self.running:
				self.start(self.t)
			elif value == 0xAAAA and self.running:
				if self.counter(self.t) > self.eff[WINR]:
					self.do_reset(self.t, 2)
					return
				self.reload(self.t)
			return
		if off == SR:
			return
		if off == EWCR:
			if value & EWCR_EWIC:
				self.ewif = False
			value &= ~EWCR_EWIC
			if value == self.reg[EWCR] and EWCR not in self.pending:
				return
		assert self.access, f'IWDG register 0x{off:x} written while closed'
		assert off not in self.pending, f'IWDG register 0x{off:x} written during its update'
		mask = {PR: (1 << (4 if self.pr_max > 6 else 3)) - 1, RLR: 0xFFF, WINR: 0xFFF, EWCR: 0xFFF | EWCR_EWIE}[off]
		self.reg[off] = value & mask
		self.pending[off] = (self.update_time(off), value & mask)

	def control(self, uc, n, value):
		self.ctl[n] = value
		if n == 1:
			assert not value & 0x80000000, f'firmware check {value & 0x7fffffff} failed'
			self.checked += 1
		elif n == 2:
			assert value == self.checked
			assert self.expect is None, f'reset expected between {self.expect[0]} and {self.expect[1]} us'
			self.done = True
			uc.emu_stop()
		elif n == 3:
			self.advance(self.t + value)
		elif n == 5:
			self.expect = (self.ctl[4], value, self.ctl[6])
		elif n == 7:
			f = self.ctl[8]
			self.base = value
			self.window, self.ewi, self.onf = bool(f & 1), bool(f & 2), bool(f & 4)
			self.pr_max = (f >> 8) & 15
		elif n == 12:
			if value == 1:
				# Hardware watchdog, from the reset
				self.start(Fraction(0))
			elif value == 2:
				# Boot loader: started, reload 100 and the prescaler value past
				# the largest divider written (7 on 3 bits, 9 on 4 bits), updates
				# in progress
				pr = 7 if self.pr_max == 6 else 9
				if not self.running:
					self.start(self.t)
				self.reg[PR], self.reg[RLR] = pr, 100
				self.pending[PR] = (self.update_time(PR), pr)
				self.pending[RLR] = (self.update_time(RLR), 100)
		elif n == 15:
			self.events.append(self.t)


def emulate(path, cpu, lsi, scenario):
	uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
	uc.ctl_set_cpu_model(getattr(arm_const, 'UC_CPU_ARM_CORTEX_' + cpu))
	uc.mem_map(0x08000000, 0x200000)
	uc.mem_map(0x20000000, 0x80000)
	model = Model(uc, lsi, scenario)
	# MMIO for the peripherals and the NVIC: memory hooks inside IT blocks
	# skip instructions in this Unicorn version
	uc.mmio_map(PERIPH, PERIPH_SIZE, model.read, PERIPH, model.write, PERIPH)
	uc.mem_map(0xE0000000, NVIC - 0xE0000000)
	uc.mmio_map(NVIC, 0x1000, model.read, NVIC, model.write, NVIC)
	uc.mem_map(NVIC + 0x1000, 0xE0100000 - NVIC - 0x1000)
	with path.open('rb') as f:
		elf = ELFFile(f)
		for seg in elf.iter_segments():
			if seg['p_type'] == 'PT_LOAD':
				uc.mem_write(seg['p_paddr'], seg.data())
		start = elf.header['e_entry']
	uc.reg_write(UC_ARM_REG_SP, 0x2007FFF0)
	uc.emu_start(start | 1, 0, count=50_000_000)
	assert model.done, f'incomplete: {model.checked} checks'
	return model


def scenario(exe, part, opt, cpu, lsi, sc):
	try:
		return emulate(exe, cpu, lsi, sc)
	except (AssertionError, UcError) as e:
		raise SystemExit(f'FAIL {part} {opt} {SCENARIOS[sc]}: {e}')


def main():
	cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
	if not shutil.which(cxx):
		raise SystemExit('NOT RUN: arm-none-eabi-g++ unavailable')
	with tempfile.TemporaryDirectory(prefix='stm32-wdt-') as td:
		td = Path(td)
		sel = os.environ.get('WDT_PARTS')
		for part, family, cpu, lsi in PARTS:
			if sel and part not in sel.split(','):
				continue
			for opt in ('-O0', '-Os'):
				flags = [f'-mcpu={GCC_CPU[cpu]}', '-mthumb', '-mfloat-abi=soft', opt,
						 '-std=gnu++23', '-D' + part, '-ffunction-sections', '-fdata-sections',
						 '-fno-rtti', '-fno-exceptions', '-fno-builtin', '-Wall', '-Wextra', '-Werror',
						 '-Wno-missing-field-initializers', '-I' + str(HERE / 'delay_shim')]
				flags += ['-I' + str(ROOT / d) for d in ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
						  'ARM/ST/include', 'ARM/ST/' + family + '/include')]
				objects = []
				for src in ('tests/stm32/wdt_fw.cpp', os.environ.get('WDT_SRC', 'ARM/ST/src/wdt_stm32.cpp')):
					obj = td / (Path(src).stem + '.o')
					run([cxx, *flags, '-c', str(ROOT / src), '-o', str(obj)])
					objects.append(str(obj))
				exe = td / 'wdt.elf'
				run([cxx, *flags, '-nostdlib', '-Wl,--gc-sections', '-T' + str(HERE / 'wdt_fw.ld'),
					 *objects, '-lgcc', '-o', str(exe)])
				first = scenario(exe, part, opt, cpu, lsi, 0)
				results = [f'basic {first.checked}']
				todo = [2, 3]
				if first.window:
					todo.append(1)
				if not first.onf:
					todo.append(6)
				if first.ewi:
					todo.append(4)
				if first.ctl[13]:
					todo.append(5)
				for sc in sorted(todo):
					m = scenario(exe, part, opt, cpu, lsi, sc)
					assert m.checked >= 2, f'{SCENARIOS[sc]}: {m.checked} checks'
					results.append(f'{SCENARIOS[sc]} {m.checked}')
				print(f'{part} {opt}: ' + ', '.join(results) + ' PASS', flush=True)
	print('IWDG model only. Hardware not tested by this script.')


if __name__ == '__main__':
	main()
