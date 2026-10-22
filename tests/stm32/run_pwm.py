#!/usr/bin/env python3
"""Build the STM32 PWM driver with pwm_fw.cpp and run it in Unicorn.

Run: python3 tests/stm32/run_pwm.py

PWM_PARTS=STM32L476xx,STM32G474xx runs only these parts, PWM_SRC builds
another copy of the driver.

The driver, ARM/ST/src/pwm_stm32.cpp, is built with the TIM driver
(timer_tim_stm32.cpp, its TIM table and Stm32TimInit) at -O0 and -Os for the
parts of PARTS: F0, F4, L1, L4, L4+, G4, H7 and WBA. It runs against a model
of the TIM registers that applies the rules of the reference manuals:

- TIM written only with its RCC clock on, reset by its RCC reset bit
- 16 bit prescaler; 16 bit period, counter and compare registers except on
  the 32 bit TIM
- no compare register or CCMR2 written above the channels of the TIM, no
  BDTR on a TIM without break function, no center aligned mode on a TIM
  without it, CMS not changed while the counter runs (RM0351 TIMx_CR1)
- the pin level of a channel: off without CCxE, or MOE on the TIM with
  break function; PWM mode 1 active while the counter is below the compare
  value, counting up to ARR in edge mode, up then down in center mode;
  CCxP inverts; forced inactive and forced active modes
- GPIO BSRR applied to ODR

The firmware reads the period and the time at high level of an output from
the model and checks them against the frequency and duty cycle it asked for.
The model uses the preload registers: the output after the next update
event. Requires arm-none-eabi-g++, unicorn and pyelftools. Not a simulation
of electrical timing; hardware not tested by this script.

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

from unicorn import Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE
from unicorn import arm_const
from unicorn.arm_const import UC_ARM_REG_SP
from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent

CONTROL = 0x5FFF0000
PERIPH = 0x40000000
PERIPH_SIZE = 0x20000000

# Part define, family folder, CPU
PARTS = [
	('STM32F030x8', 'STM32F0xx', 'M0'), ('STM32F072xB', 'STM32F0xx', 'M0'),
	('STM32F401xC', 'STM32F4xx', 'M4'), ('STM32F446xx', 'STM32F4xx', 'M4'),
	('STM32L152xE', 'STM32L1xx', 'M3'),
	('STM32L432xx', 'STM32L4xx', 'M4'), ('STM32L476xx', 'STM32L4xx', 'M4'), ('STM32L4S9xx', 'STM32L4xx', 'M4'),
	('STM32G474xx', 'STM32G4xx', 'M4'), ('STM32G431xx', 'STM32G4xx', 'M4'),
	('STM32H753xx', 'STM32H7xx', 'M7'),
	('STM32WBA65xx', 'STM32WBAxx', 'M33'),
]
GCC_CPU = {'M0': 'cortex-m0', 'M3': 'cortex-m3', 'M4': 'cortex-m4', 'M7': 'cortex-m7', 'M33': 'cortex-m33'}

CR1, SR, EGR, CCMR1, CCMR2, CCER, CNT, PSC, ARR, CCR1, BDTR = 0x00, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x34, 0x44
CR1_CEN, CR1_DIR, CR1_CMS = 1, 1 << 4, 3 << 5
BDTR_MOE = 1 << 15
NOT_DRIVEN, FROZEN = 0xFFFFFFFF, 0xFFFFFFFE


def run(cmd):
	p = subprocess.run(cmd, capture_output=True, text=True)
	if p.returncode:
		raise RuntimeError(' '.join(cmd) + '\n' + p.stdout + p.stderr)


class Model:
	def __init__(self, uc):
		self.uc = uc
		self.ctl = [0] * 16
		self.tims = {}
		self.gpio = set()
		self.ug = 0
		self.checked = 0
		self.done = False

	def word(self, addr):
		return struct.unpack('<I', self.uc.mem_read(addr, 4))[0]

	def tim_at(self, addr):
		for base, t in self.tims.items():
			if base <= addr < base + 0x400:
				return base, t
		return None, None

	def clock_on(self, t):
		return self.word(t['en']) & t['mask'] != 0

	def reset(self, t):
		t['reg'] = {}

	# Output of a channel: period and clocks at high level
	def wave(self, base, ch):
		t = self.tims[base]
		r = t['reg']
		if not self.clock_on(t):
			return 0, NOT_DRIVEN
		psc = r.get(PSC, 0) + 1
		arr = r.get(ARR, 0)
		cr1 = r.get(CR1, 0)
		center = cr1 & CR1_CMS
		if center:
			period = 2 * arr * psc
		else:
			assert not cr1 & CR1_DIR, 'edge mode counting down'
			period = (arr + 1) * psc
		if ch >= t['nch']:
			return period, NOT_DRIVEN
		ccmr = r.get(CCMR1 if ch < 2 else CCMR2, 0)
		sh = (ch & 1) * 8
		ocm = (ccmr >> (sh + 4)) & 7
		if t['ocm3']:
			ocm |= ((ccmr >> (16 + sh)) & 1) << 3
		ccer = r.get(CCER, 0) >> (4 * ch)
		if not ccer & 1 or (t['brk'] and not r.get(BDTR, 0) & BDTR_MOE):
			return period, NOT_DRIVEN
		assert (ccmr >> sh) & 3 == 0, f'channel {ch} not an output'
		ccr = r.get(CCR1 + 4 * ch, 0)
		if ocm == 4:
			active = 0
		elif ocm == 5:
			active = period
		elif ocm == 6:
			if not cr1 & CR1_CEN:
				return period, FROZEN
			if center:
				active = (min(ccr, arr) + max(0, min(ccr - 1, arr))) * psc
			else:
				active = min(ccr, arr + 1) * psc
		else:
			raise AssertionError(f'output compare mode {ocm} on channel {ch}')
		return period, period - active if ccer & 2 else active

	def read(self, uc, access, addr, size, value, _):
		if CONTROL <= addr < CONTROL + 0x40:
			n = (addr - CONTROL) // 4
			if n == 8:
				val = self.ug
			elif n in (13, 14):
				val = self.wave(self.ctl[11], self.ctl[12])[n - 13]
			else:
				val = self.ctl[n]
			uc.mem_write(addr, struct.pack('<I', val))
			return
		base, t = self.tim_at(addr)
		if t is None:
			return
		assert size == 4, f'TIM access of {size} bytes'
		off = addr - base
		val = t['reg'].get(off, 0) if self.clock_on(t) and off != EGR else 0
		uc.mem_write(addr, struct.pack('<I', val))

	def write(self, uc, access, addr, size, value, _):
		value &= 0xFFFFFFFF
		if CONTROL <= addr < CONTROL + 0x40:
			n = (addr - CONTROL) // 4
			self.ctl[n] = value
			if n == 1:
				assert not value & 0x80000000, f'firmware check {value & 0x7fffffff} failed'
				self.checked += 1
			elif n == 2:
				assert value == self.checked
				self.done = True
				uc.emu_stop()
			elif n == 3:
				c = self.ctl
				self.tims[value] = {'rst': c[4], 'en': c[9], 'mask': c[5], 'b32': bool(c[6] & 1),
									'brk': bool(c[6] & 2), 'cms': bool(c[6] & 4), 'ocm3': bool(c[6] & 8),
									'nch': (c[6] >> 8) & 0xFF,
									'reg': {}}
			elif n == 7:
				self.gpio.add(value)
			return
		for t in self.tims.values():
			if addr == t['rst'] and value & t['mask']:
				self.reset(t)
		if addr - 0x18 in self.gpio:
			odr = self.word(addr - 4)
			odr = (odr & ~(value >> 16)) | (value & 0xFFFF)
			uc.mem_write(addr - 4, struct.pack('<I', odr & 0xFFFF))
			return
		base, t = self.tim_at(addr)
		if t is None:
			return
		assert size == 4, f'TIM access of {size} bytes'
		assert self.clock_on(t), f'TIM 0x{base:08x} written with its clock off'
		off = addr - base
		r = t['reg']
		bits = 0xFFFFFFFF if t['b32'] else 0xFFFF
		if off == CR1:
			cms = value & CR1_CMS
			assert t['cms'] or not cms, 'center aligned mode on a TIM without it'
			assert not (r.get(CR1, 0) & CR1_CEN and cms != r.get(CR1, 0) & CR1_CMS), 'CMS changed while counting'
			r[CR1] = value
		elif off == SR:
			r[SR] = r.get(SR, 0) & value
		elif off == EGR:
			if value & 1:
				self.ug += 1
		elif off == CCMR2:
			assert t['nch'] > 2, 'CCMR2 written on a TIM without channel 3 and 4'
			r[off] = value
		elif CCR1 <= off <= CCR1 + 12:
			assert (off - CCR1) // 4 < t['nch'], f'CCR{(off - CCR1) // 4 + 1} written on a TIM of {t["nch"]} channels'
			r[off] = value & bits
		elif off in (CNT, ARR):
			r[off] = value & bits
		elif off == PSC:
			r[off] = value & 0xFFFF
		elif off == BDTR:
			assert t['brk'], 'BDTR written on a TIM without break function'
			r[off] = value
		else:
			r[off] = value


def emulate(path, cpu):
	uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
	uc.ctl_set_cpu_model(getattr(arm_const, 'UC_CPU_ARM_CORTEX_' + cpu))
	uc.mem_map(0x08000000, 0x200000)
	uc.mem_map(0x20000000, 0x80000)
	uc.mem_map(PERIPH, PERIPH_SIZE)
	uc.mem_map(0xE0000000, 0x100000)
	with path.open('rb') as f:
		elf = ELFFile(f)
		for seg in elf.iter_segments():
			if seg['p_type'] == 'PT_LOAD':
				uc.mem_write(seg['p_paddr'], seg.data())
		start = elf.header['e_entry']
	model = Model(uc)
	uc.hook_add(UC_HOOK_MEM_READ, model.read, begin=PERIPH, end=PERIPH + PERIPH_SIZE - 1)
	uc.hook_add(UC_HOOK_MEM_WRITE, model.write, begin=PERIPH, end=PERIPH + PERIPH_SIZE - 1)
	uc.reg_write(UC_ARM_REG_SP, 0x2007FFF0)
	uc.emu_start(start | 1, 0, count=20_000_000)
	assert model.done and model.checked >= 60, f'incomplete: {model.checked} checks'
	return model.checked, len(model.tims)


def main():
	cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
	if not shutil.which(cxx):
		raise SystemExit('NOT RUN: arm-none-eabi-g++ unavailable')
	with tempfile.TemporaryDirectory(prefix='stm32-pwm-') as td:
		td = Path(td)
		sel = os.environ.get('PWM_PARTS')
		for part, family, cpu in PARTS:
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
				for src in ('tests/stm32/pwm_fw.cpp', os.environ.get('PWM_SRC', 'ARM/ST/src/pwm_stm32.cpp'),
							'ARM/ST/src/timer_tim_stm32.cpp'):
					obj = td / (Path(src).stem + '.o')
					# The TIM driver has unused parameters on its callbacks
					extra = ['-Wno-unused-parameter'] if 'timer_' in src else []
					run([cxx, *flags, *extra, '-c', str(ROOT / src), '-o', str(obj)])
					objects.append(str(obj))
				exe = td / 'pwm.elf'
				run([cxx, *flags, '-nostdlib', '-Wl,--gc-sections', '-T' + str(HERE / 'pwm_fw.ld'),
					 *objects, '-lgcc', '-o', str(exe)])
				checks, tims = emulate(exe, cpu)
				print(f'{part} {opt}: {tims} TIM, {checks} checks PASS', flush=True)
	print('Register model only. Hardware not tested by this script.')


if __name__ == '__main__':
	main()
