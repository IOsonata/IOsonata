#!/usr/bin/env python3
"""Build the STM32L4 ADC1 driver with adc_fw.cpp and run it in Unicorn.

Run: python3 tests/stm32l4/run_adc.py

The driver, ARM/ST/STM32L4xx/src/adc_stm32l4.cpp, is built at -O0 and -Os for
the parts of PARTS and run against a model of the ADC1 registers. usDelay of
the firmware (adc_shim/idelay.h) moves the time of the model, so the model
checks the waits of RM0351 and the datasheets:

- 20 us of regulator start before the calibration or the enable
- ADEN ignored for 4 ADC clocks after the calibration (ES0250 2.8.4)
- temperature sensor 120 us after TSEN and 15 us after the enable,
  reference 12 us after VREFEN and after the enable
- sampling time of the internal inputs: 4 us, 5 us, 12 us
- a conversion started more than 1 ms after the previous one or after the
  calibration gives a wrong result (ES0250 2.8.11)
- an input pin reads 0 V while its GPIOx_ASCR switch is open, on the parts
  that have the register
- conversions do not wait for the data register without AUTDLY, a result not
  read in time is an overrun
- synchronous clock, CKMODE written while the ADC is disabled
- no LDM, STM, LDRD or STRD on the ADC registers (ES0250 2.8.3, 2.8.15)

It also models the set only CR commands, W1C flags and the hardware
oversampler, and injects faults. Requires arm-none-eabi-g++, unicorn and
pyelftools. Not a simulation of analog voltages or electrical timing.

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
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_PC
from elftools.elf.elffile import ELFFile

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent

ADC = 0x50040000
CCR = ADC + 0x308
RCC = 0x40021000
GPIO = 0x48000000
CONTROL = 0x500FF000

# Part define and whether it has GPIOx_ASCR
PARTS = [('STM32L476xx', True), ('STM32L4R5xx', False), ('STM32L412xx', False)]

CR_ADEN, CR_ADDIS, CR_ADSTART, CR_JADSTART, CR_ADSTP, CR_JADSTP = 1, 2, 4, 8, 16, 32
CR_ADVREGEN, CR_DEEPPWD, CR_ADCALDIF, CR_ADCAL = 1 << 28, 1 << 29, 1 << 30, 1 << 31
ISR_ADRDY, ISR_EOC, ISR_EOS, ISR_OVR = 1, 4, 8, 16
CFGR_AUTDLY, CFGR_JQDIS = 1 << 14, 1 << 31
CCR_VREFEN, CCR_TSEN, CCR_VBATEN = 1 << 22, 1 << 23, 1 << 24

SMP_CLK = [2.5, 6.5, 12.5, 24.5, 47.5, 92.5, 247.5, 640.5]
SAR_CLK = {12: 12.5, 10: 10.5, 8: 8.5, 6: 6.5}

# Input voltages, VREF+ 3.3 V, 0.1 V per input number for the others
VREF = 3.3
VOLT = {0: 1.212, 1: 0.5, 5: 1.5, 14: 2.0, 17: 0.8, 18: 1.0}
MIN_SAMPLE = {0: 4.0, 17: 5.0, 18: 12.0}
PIN = {1: (2, 0), 2: (2, 1), 3: (2, 2), 4: (2, 3), 5: (0, 0), 6: (0, 1), 7: (0, 2), 8: (0, 3),
	   9: (0, 4), 10: (0, 5), 11: (0, 6), 12: (0, 7), 13: (2, 4), 14: (2, 5), 15: (1, 0), 16: (1, 1)}

ADC_OFFSETS = (0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x30, 0x34, 0x38, 0x3C, 0x40, 0xB0, 0xB4)


def run(cmd):
	p = subprocess.run(cmd, capture_output=True, text=True)
	if p.returncode:
		raise RuntimeError(' '.join(cmd) + '\n' + p.stdout + p.stderr)


class Model:
	def __init__(self, uc, ascr):
		self.uc = uc
		self.ascr = ascr
		self.fault = 0
		self.core = 80000000
		self.checked = 0
		self.done = False
		self.t = 0.0
		self.scans = []
		self.reset()

	def reset(self):
		self.reg = {off: 0 for off in ADC_OFFSETS}
		self.reg[0x08] = CR_DEEPPWD
		self.reg[0x0C] = CFGR_JQDIS
		self.ccr = 0
		self.t_regen = None
		self.t_cal = None		# end of the last calibration
		self.t_ready = None		# ADC ready
		self.t_vrefen = None
		self.t_tsen = None
		self.t_last = None		# end of the last conversion or calibration
		self.events = {}
		self.scan = None

	# Clocks
	def hclk(self):
		hpre = (struct.unpack('<I', self.uc.mem_read(RCC + 0x08, 4))[0] >> 4) & 15
		shift = [0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9][hpre]
		return self.core >> shift

	def fadc(self):
		ck = (self.ccr >> 16) & 3
		assert ck, 'asynchronous ADC clock selected'
		return self.hclk() / (1, 2, 4)[ck - 1]

	def clk_us(self, n):
		return n * 1e6 / self.fadc()

	# Conversions
	def resolution(self):
		return 12 - 2 * ((self.reg[0x0C] >> 3) & 3)

	def ratio(self):
		cfgr2 = self.reg[0x10]
		if not cfgr2 & 1:
			return 1, 0
		return 2 << ((cfgr2 >> 2) & 7), (cfgr2 >> 5) & 15

	def smp(self, inp):
		reg = self.reg[0x14] if inp < 10 else self.reg[0x18]
		return SMP_CLK[(reg >> (3 * (inp % 10))) & 7]

	def duration(self, inp):
		ratio, _ = self.ratio()
		return self.clk_us((self.smp(inp) + SAR_CLK[self.resolution()]) * ratio)

	def value(self, inp, start):
		res = self.resolution()
		full = (1 << res) - 1
		volt = VOLT.get(inp, 0.1 * inp)
		if inp in PIN:
			port, pin = PIN[inp]
			if self.ascr:
				ascr = struct.unpack('<I', self.uc.mem_read(GPIO + port * 0x400 + 0x2C, 4))[0]
				if not ascr >> pin & 1:
					volt = 0.0
		else:
			bit = {0: CCR_VREFEN, 17: CCR_TSEN, 18: CCR_VBATEN}[inp]
			if not self.ccr & bit or self.clk_us(self.smp(inp)) < MIN_SAMPLE[inp] - 1e-9:
				volt = 0.0
			elif inp == 17 and (start - self.t_tsen < 120 - 1e-9 or start - self.t_ready < 15 - 1e-9):
				volt = 0.0
			elif inp == 0 and start - max(self.t_vrefen, self.t_ready) < 12 - 1e-9:
				volt = 0.0
		raw = min(full, int(volt / VREF * full + 0.5))
		ratio, shift = self.ratio()
		raw = (raw * ratio) >> shift
		if self.t_last is None or start - self.t_last > 1000:
			raw ^= 0x0F0  # ES0250 2.8.11
		return raw & 0xFFFF

	def start_conversion(self):
		s = self.scan
		inp = s['inputs'][s['rank']]
		s['start'] = self.t
		s['end'] = None if self.fault in (1, 3) else self.t + self.duration(inp)

	def step(self):
		# Apply the events due at the current time, oldest first
		while True:
			due = [(t, name) for name, t in self.events.items() if t is not None and t <= self.t + 1e-9]
			s = self.scan
			if s and s['end'] is not None and s['end'] <= self.t + 1e-9 and not s['wait']:
				due.append((s['end'], 'conv'))
			if not due:
				return
			t, name = min(due)
			if name == 'conv':
				self.complete(t)
				continue
			del self.events[name]
			if name == 'cal':
				self.reg[0x08] &= ~CR_ADCAL
				self.t_cal = self.t_last = t
			elif name == 'ready':
				self.reg[0x00] |= ISR_ADRDY
				self.t_ready = t
			elif name == 'disable':
				self.reg[0x08] &= ~(CR_ADEN | CR_ADDIS)
				self.reg[0x00] &= ~ISR_ADRDY
				self.t_ready = None
			elif name == 'stop':
				self.reg[0x08] &= ~(CR_ADSTART | CR_ADSTP)
				self.scan = None

	def complete(self, t):
		s = self.scan
		inp = s['inputs'][s['rank']]
		val = self.value(inp, s['start'])
		self.t_last = t
		if self.reg[0x00] & ISR_EOC:
			self.reg[0x00] |= ISR_OVR	# OVRMOD 0 keeps the data not read
		else:
			self.reg[0x40] = val
			self.reg[0x00] |= ISR_EOC
		if self.fault == 2:
			self.reg[0x00] |= ISR_OVR
		s['rank'] += 1
		if s['rank'] == len(s['inputs']):
			self.reg[0x00] |= ISR_EOS
			self.reg[0x08] &= ~CR_ADSTART
			self.scan = None
		elif self.reg[0x0C] & CFGR_AUTDLY:
			s['wait'] = True
			s['end'] = None
		else:
			saved = self.t
			self.t = t
			self.start_conversion()
			self.t = saved

	def data_read(self):
		self.reg[0x00] &= ~ISR_EOC
		s = self.scan
		if s and s['wait']:
			s['wait'] = False
			self.start_conversion()

	# Memory hooks
	def check_insn(self, uc):
		pc = uc.reg_read(UC_ARM_REG_PC)
		hw = struct.unpack('<H', uc.mem_read(pc, 2))[0]
		assert (hw & 0xF000) != 0xC000 and (hw & 0xFE00) != 0xE800, \
			f'multiple or dual load/store on an ADC register at 0x{pc:08x}'

	def clock_on(self, uc):
		enr = struct.unpack('<I', uc.mem_read(RCC + 0x4C, 4))[0]
		assert enr & (1 << 13), 'ADC clock disabled'

	def read(self, uc, access, addr, size, value, _):
		if ADC <= addr < ADC + 0x400:
			self.check_insn(uc)
			self.clock_on(uc)
			assert size == 4, 'ADC access not 32 bits'
			self.step()
			off = addr - ADC
			if addr == CCR:
				uc.mem_write(addr, struct.pack('<I', self.ccr))
			elif off in self.reg:
				uc.mem_write(addr, struct.pack('<I', self.reg[off] & 0xFFFFFFFF))
				if off == 0x40:
					self.data_read()

	def write(self, uc, access, addr, size, value, _):
		value &= 0xFFFFFFFF
		if CONTROL <= addr < CONTROL + 0x20:
			n = (addr - CONTROL) // 4
			if n == 0:
				self.fault = value
			elif n == 1:
				assert not value & 0x80000000, f'firmware check {value & 0x7fffffff} failed'
				self.checked += 1
			elif n == 2:
				assert value == self.checked
				self.done = True
				uc.emu_stop()
			elif n == 3:
				self.t += value
				self.step()
			elif n == 4:
				self.core = value
			return
		if addr == RCC + 0x2C and value & (1 << 13):
			self.reset()
			return
		if not ADC <= addr < ADC + 0x400:
			return
		self.check_insn(uc)
		self.clock_on(uc)
		assert size == 4, 'ADC access not 32 bits'
		self.step()
		cr = self.reg[0x08]
		if addr == CCR:
			changed = self.ccr ^ value
			if changed & (0xF << 18 | 3 << 16):
				assert not cr & CR_ADEN, 'ADC clock changed while enabled'
			if changed & (CCR_VREFEN | CCR_TSEN | CCR_VBATEN):
				assert not cr & CR_ADSTART, 'internal input changed during a conversion'
			if value & ~self.ccr & CCR_VREFEN:
				self.t_vrefen = self.t
			if value & ~self.ccr & CCR_TSEN:
				self.t_tsen = self.t
			self.ccr = value
			return
		off = addr - ADC
		if off == 0x00:
			self.reg[0x00] &= ~value
		elif off == 0x08:
			self.write_cr(value)
		elif off in (0x0C, 0x10, 0x14, 0x18, 0x30, 0x34, 0x38, 0x3C):
			assert not cr & CR_ADSTART, f'register 0x{off:02x} written during a conversion'
			self.reg[off] = value
		elif off == 0xB0:
			assert not cr & CR_ADEN, 'DIFSEL written while enabled'
			self.reg[off] = value
		elif off in self.reg:
			self.reg[off] = value

	def write_cr(self, value):
		cr = self.reg[0x08]
		assert not value & (CR_JADSTART | CR_JADSTP), 'injected group used'
		power = CR_DEEPPWD | CR_ADVREGEN
		if (value ^ cr) & power:
			assert not cr & (CR_ADEN | CR_ADCAL), 'regulator changed while enabled'
			deep = value & CR_DEEPPWD
			regen = value & CR_ADVREGEN and not deep
			if regen and not cr & CR_ADVREGEN:
				self.t_regen = self.t
			cr = (cr & ~power) | deep | (CR_ADVREGEN if regen else 0)
		cr = (cr & ~CR_ADCALDIF) | (value & CR_ADCALDIF)
		self.reg[0x08] = cr
		regulated = cr & CR_ADVREGEN and self.t - self.t_regen >= 20 - 1e-9
		if value & CR_ADCAL:
			assert not cr & CR_ADEN, 'calibration while enabled'
			assert regulated, 'calibration before the regulator start time'
			assert not value & CR_ADCALDIF, 'differential calibration'
			self.reg[0x08] |= CR_ADCAL
			self.events['cal'] = None if self.fault == 4 else self.t + self.clk_us(116)
		if value & CR_ADEN:
			if cr & CR_ADCAL or (self.t_cal is not None and self.t - self.t_cal < self.clk_us(4) - 1e-9):
				pass  # Ignored (ES0250 2.8.4)
			else:
				assert regulated, 'enable before the regulator start time'
				self.reg[0x08] |= CR_ADEN
				self.events['ready'] = None if self.fault == 5 else self.t + 2
		if value & CR_ADDIS:
			assert cr & CR_ADEN and not cr & CR_ADSTART, 'disable while not enabled or converting'
			self.reg[0x08] |= CR_ADDIS
			self.events['disable'] = self.t + 1
		if value & CR_ADSTART:
			assert cr & CR_ADEN and self.t_ready is not None and not cr & CR_ADDIS, 'conversion before ready'
			sqr = [self.reg[0x30], self.reg[0x34], self.reg[0x38], self.reg[0x3C]]
			count = (sqr[0] & 15) + 1
			inputs = []
			for r in range(1, count + 1):
				reg = 0 if r < 5 else (r - 5) // 5 + 1
				pos = 6 * r if reg == 0 else 6 * (r - 5 * reg)
				inputs.append((sqr[reg] >> pos) & 31)
			self.scans.append(inputs)
			self.reg[0x08] |= CR_ADSTART
			self.scan = {'inputs': inputs, 'rank': 0, 'wait': False}
			self.start_conversion()
		if value & CR_ADSTP and cr & CR_ADSTART:
			self.reg[0x08] |= CR_ADSTP
			self.events['stop'] = None if self.fault == 3 else self.t + 1


def emulate(path, ascr):
	uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
	uc.mem_map(0x08000000, 0x200000)
	uc.mem_map(0x20000000, 0x80000)
	uc.mem_map(0x40020000, 0x10000)
	uc.mem_map(0x48000000, 0x10000)
	uc.mem_map(0x50000000, 0x100000)
	with path.open('rb') as f:
		elf = ELFFile(f)
		for seg in elf.iter_segments():
			if seg['p_type'] == 'PT_LOAD':
				uc.mem_write(seg['p_paddr'], seg.data())
		start = elf.header['e_entry']
	model = Model(uc, ascr)
	uc.hook_add(UC_HOOK_MEM_READ, model.read)
	uc.hook_add(UC_HOOK_MEM_WRITE, model.write)
	uc.reg_write(UC_ARM_REG_SP, 0x2007FFF0)
	uc.emu_start(start | 1, 0, count=40_000_000)
	assert model.done and model.checked >= 60, f'incomplete: {model.checked} checks'
	# Each scan is a one input scan of the first input of the scan that follows
	for i in range(0, len(model.scans), 2):
		dummy, scan = model.scans[i], model.scans[i + 1]
		assert len(dummy) == 1 and dummy[0] == scan[0], f'scan {i // 2} without its first conversion: {dummy} {scan}'
	return model.checked


def main():
	cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
	if not shutil.which(cxx):
		raise SystemExit('NOT RUN: arm-none-eabi-g++ unavailable')
	with tempfile.TemporaryDirectory(prefix='l4-adc-') as td:
		td = Path(td)
		for part, ascr in PARTS:
			for opt in ('-O0', '-Os'):
				flags = ['-mcpu=cortex-m4', '-mthumb', '-mfloat-abi=soft', opt,
						 '-std=gnu++23', '-D' + part, '-ffunction-sections', '-fdata-sections',
						 '-fno-rtti', '-fno-exceptions', '-fno-builtin', '-Wall', '-Wextra', '-Werror',
						 '-Wno-overloaded-virtual', '-Wno-missing-field-initializers']
				flags += ['-I' + str(HERE / 'adc_shim')]
				flags += ['-I' + str(ROOT / d) for d in ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
						  'ARM/ST/include', 'ARM/ST/STM32L4xx/include')]
				objects = []
				for src in ('tests/stm32l4/adc_fw.cpp', os.environ.get('ADC_SRC', 'ARM/ST/STM32L4xx/src/adc_stm32l4.cpp'),
							'src/device.cpp', 'src/device_intrf.cpp'):
					obj = td / (Path(src).stem + '.o')
					run([cxx, *flags, '-c', str(ROOT / src), '-o', str(obj)])
					objects.append(str(obj))
				exe = td / 'adc.elf'
				run([cxx, *flags, '-nostdlib', '-Wl,--gc-sections', '-T' + str(HERE / 'adc_fw.ld'),
					 *objects, '-lgcc', '-o', str(exe)])
				print(f'{part} {opt}: ADC1 {emulate(exe, ascr)} checks PASS', flush=True)
	print('Register model only. Hardware not tested by this script.')


if __name__ == '__main__':
	main()
