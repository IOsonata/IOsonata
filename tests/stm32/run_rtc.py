#!/usr/bin/env python3
"""Build the STM32 RTC driver with rtc_fw.cpp and run it in Unicorn.

Run: python3 tests/stm32/run_rtc.py

RTC_PARTS=STM32L476xx,STM32G474xx runs only these parts, RTC_SCENARIOS=0,2
only these scenarios, RTC_SRC builds another copy of the driver, RTC_KEEP
keeps the firmware images in that folder.

The driver, RtcStm32 of ARM/ST/src/rtc_stm32.cpp, with RtcDevice
(src/miscdev/rtc.cpp), Device (src/device.cpp) and DeviceIntrf
(src/device_intrf.cpp, with ARM/src/iatomic.c for the Cortex-M0) is built at
-O0 and -Os for the parts of PARTS: F0, F4, L1, L4, L4+, G4, H7 and WBA.
Each scenario of rtc_fw.cpp runs in its own emulator, against a model of the
RTC, of its clocks and of the backup domain in time: usDelay of the firmware
(delay_shim/idelay.h) moves the time, a scenario can restart the firmware
with a system reset. Scenario 21 runs TestRtcChip of rtc_fw.cpp, an RTC chip
implementation of RtcDevice on an I2C interface in RAM, without the STM32
RTC. The register layouts, bit positions, EXTI lines and interrupts of each
family are in this script, from the reference manuals. The model applies
their rules:

- backup domain: RTC, backup registers and RCC backup domain control
  written only with the PWR DBP bit set (PWR clock enabled where it has
  one), RTC and TAMP registers accessed only with their bus clock enabled
  (L412, L4S9, G4, H7, WBA); the backup domain reset clears the RTC, the
  backup registers, the LSE and the RTC clock selection (the LSI of the WBA)
- RTC clock selection written only while it is 0; LSEBYP written only with
  the LSE off; the LSE ready 300 ms after LSEON with a crystal (LSEBYP 0) or
  an external oscillator (LSEBYP 1), never without; the LSI ready 80 us
  after LSION, LSI typical frequency of the datasheets (40 kHz F0, 37 kHz L1,
  32 kHz other families) divided by 128 with the LSI predivider; LSE clock
  security and WBA LSESYSEN set only with the LSE ready, WBA LSI1PREDIV
  written only with the LSI1 off
- RTC write protection: 0xCA then 0x53 in WPR, any other key locks; every
  RTC register write except the flag clears needs it
- initialization mode: INITF 2 RTCCLK cycles after INIT with the RTC clock
  running (selected oscillator ready, RTCEN set where it exists); TR, DR,
  PRER and FMT written only in it; PRER in two writes, PREDIV_S then
  PREDIV_A; the calendar counts again 4 RTCCLK cycles after INIT is
  cleared, prescalers from their start
- calendar: asynchronous then synchronous prescaler to 1 Hz, BCD fields,
  12 or 24 hour format (FMT), months and leap years of 2000 to 2099, day of
  the week counted on its own; read from the counters (BYPSHAD) with 1 us
  passing at each read of TR, DR and SSR
- alarms A and B: written with the alarm off (and ALRxWF set where it
  exists), matched at each second, MSKx masks, WDSEL day of the week
- wakeup timer: WUTR and WUCKSEL written with the timer off and WUTWF set;
  RTCCLK divided by 16, 8, 4, 2, or the 1 Hz clock, with 2^16 added for
  WUCKSEL 6 and 7; period WUT + 1
- flags: ISR flags cleared by writing 0 (F0, F4, L1, L4, L4+, H7), SR flags
  cleared in SCR (L412, G4, WBA); interrupts through EXTI rising edge lines
  with their pending bits (F0, F4, L1, L4, G4, H7), or the RTC interrupt
  while a masked flag is set (WBA)

The firmware checks the driver against the model through test registers.
Requires arm-none-eabi-g++ with newlib, unicorn and pyelftools. Not a
simulation of the oscillator accuracy; hardware not tested by this script.

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
import datetime
from fractions import Fraction
import math
import os
from pathlib import Path
import shutil
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

LSE_HZ = 32768
LSE_START_US = 300000
LSI_START_US = 80
HSE_RTC_HZ = 250000		# HSE / 32 of the clock change scenario

# Families. lse: RCC register of the LSE and RTC clock (offset, layout),
# csr: RCC register of the LSI, pwren: PWR clock enable (offset, bit), dbp:
# PWR register of DBP (offset, bit), exti: layout, lines: alarm and wakeup
# EXTI lines, irq: alarm and wakeup interrupts, extra: settings of the LSE
# register kept over a backup domain reset (scenarios 4 and 5)
FAMILY = {
	'F0': dict(gen='isr', lsi=40000, lse=(0x20, 'bdcr'), csr=0x24, pwren=(0x1C, 28), dbp=(0x00, 8), apben=None,
			   exti='v1', lines=(17, 20), irq=(2, 2), extra4=0x10, extra5=0x10),
	'F4': dict(gen='isr', lsi=32000, lse=(0x70, 'bdcr'), csr=0x74, pwren=(0x40, 28), dbp=(0x00, 8), apben=None,
			   exti='v1', lines=(17, 22), irq=(41, 3), extra4=0, extra5=0),
	'L1': dict(gen='isr', lsi=37000, lse=(0x34, 'l1'), csr=0x34, pwren=(0x24, 28), dbp=(0x00, 8), apben=None,
			   exti='v1', lines=(17, 20), irq=(41, 3), extra4=0, extra5=0),
	'L4': dict(gen='isr', lsi=32000, lse=(0x90, 'bdcr'), csr=0x94, pwren=(0x58, 28), dbp=(0x00, 8), apben=None,
			   exti='v2', lines=(18, 20), irq=(41, 3), extra4=0x10, extra5=0x10),
	'G4': dict(gen='rtc3', lsi=32000, lse=(0x90, 'bdcr'), csr=0x94, pwren=(0x58, 28), dbp=(0x00, 8), apben=(0x58, 10),
			   exti='v2', lines=(17, 20), irq=(41, 3), extra4=0x10, extra5=0x10),
	'H7': dict(gen='isr', lsi=32000, lse=(0x70, 'bdcr'), csr=0x74, pwren=None, dbp=(0x00, 8), apben=(0xF4, 16),
			   exti='h7', lines=(17, 19), irq=(41, 3), extra4=0x10, extra5=0x10),
	# WBA: LSEDRV 2, RADIOSTSEL 1, LSI1PREDIV (scenario 4), LSESYSEN with the
	# LSE running and LSI1 running with LSI1PREDIV (scenario 5)
	'WBA': dict(gen='rtc3', lsi=32000, lse=(0xF0, 'wba'), csr=0xF0, pwren=(0x94, 2), dbp=(0x28, 0), apben=(0xA8, 21),
				exti=None, lines=None, irq=(2, 2), extra4=0x10 | 1 << 18 | 1 << 28,
				extra5=0x10 | 1 << 18 | 1 << 7 | 1 << 26 | 1 << 28),
}

# Part define, family folder, CPU, family, backup registers and differences
# from the family
PARTS = [
	('STM32F030x8', 'STM32F0xx', 'M0', 'F0', dict(nbkp=0, alarm_b=False, wakeup=False)),
	('STM32F072xB', 'STM32F0xx', 'M0', 'F0', dict(nbkp=5, alarm_b=False)),
	('STM32F401xC', 'STM32F4xx', 'M4', 'F4', dict(nbkp=20)),
	('STM32L152xE', 'STM32L1xx', 'M3', 'L1', dict(nbkp=32)),
	('STM32L412xx', 'STM32L4xx', 'M4', 'L4', dict(nbkp=32, gen='rtc3', alrwf=False, apben=(0x58, 10), prediv='csr')),
	('STM32L476xx', 'STM32L4xx', 'M4', 'L4', dict(nbkp=32)),
	('STM32L4S9xx', 'STM32L4xx', 'M4', 'L4', dict(nbkp=32, apben=(0x58, 10))),
	('STM32G474xx', 'STM32G4xx', 'M4', 'G4', dict(nbkp=32)),
	('STM32H753xx', 'STM32H7xx', 'M7', 'H7', dict(nbkp=32)),
	('STM32WBA65xx', 'STM32WBAxx', 'M33', 'WBA', dict(nbkp=32, alrwf=False, prediv='bdcr', bin=True)),
]
GCC_CPU = {'M0': 'cortex-m0', 'M3': 'cortex-m3', 'M4': 'cortex-m4', 'M7': 'cortex-m7', 'M33': 'cortex-m33'}

# Scenario: name, crystal of each boot (xtal, tcxo, none, xtal_bdrst until the
# first backup domain reset), preset at the first boot, options: board low
# frequency oscillator RC, LSE frequency
SCENARIOS = {
	0: ('basic', ('xtal',), None),
	1: ('no crystal', ('none',), None),
	2: ('system reset', ('xtal', 'xtal'), None),
	3: ('LSI over a system reset', ('none', 'none'), None),
	4: ('crystal lost', ('xtal', 'none'), 'extra4'),
	5: ('clock change', ('xtal',), 'hse'),
	6: ('running from reset values', ('xtal',), 'lse_reset'),
	7: ('selected not enabled', ('xtal',), 'lse_off'),
	8: ('external oscillator', ('tcxo',), None),
	9: ('12 hour format PM', ('xtal',), 'fmt_pm'),
	10: ('prescaler', ('xtal',), 'prer'),
	11: ('binary mode', ('xtal',), 'bin'),
	12: ('LSI predivider', ('none',), 'prediv'),
	13: ('calendar read', ('xtal',), None, dict(board='rc')),
	14: ('12 hour format AM', ('xtal',), 'fmt_am'),
	15: ('external oscillator 31250 Hz', ('tcxo',), None, dict(lse_hz=31250)),
	16: ('external oscillator 256 kHz', ('tcxo',), None, dict(lse_hz=256000)),
	17: ('HSE to LSE', ('xtal',), 'hse'),
	18: ('LSE lost at the backup domain reset', ('xtal_bdrst',), 'hse'),
	19: ('LSE lost at the backup domain reset, LSI', ('xtal_bdrst',), 'hse'),
	20: ('selected not enabled, LSI', ('xtal',), 'lse_off'),
	21: ('RTC chip', ('xtal',), None),
}

# RTC registers by generation: name -> offset
REGS = {
	'isr': dict(TR=0x00, DR=0x04, CR=0x08, ISR=0x0C, PRER=0x10, WUTR=0x14, ALRMAR=0x1C, ALRMBR=0x20, WPR=0x24,
				SSR=0x28, ALRMASSR=0x44, ALRMBSSR=0x48),
	'rtc3': dict(TR=0x00, DR=0x04, SSR=0x08, ICSR=0x0C, PRER=0x10, WUTR=0x14, CR=0x18, WPR=0x24, ALRMAR=0x40,
				 ALRMASSR=0x44, ALRMBR=0x48, ALRMBSSR=0x4C, SR=0x50, MISR=0x54, SCR=0x5C),
}
RTC_BKP_OFF = 0x50		# ISR generation, in the RTC
TAMP_BKP_OFF = 0x100	# RTC3 generation, in TAMP

# CR bits
WUCKSEL_M = 7
BYPSHAD, FMT = 1 << 5, 1 << 6
ALRAE, ALRBE, WUTE = 1 << 8, 1 << 9, 1 << 10
ALRAIE, ALRBIE, WUTIE = 1 << 12, 1 << 13, 1 << 14
CR_KNOWN = WUCKSEL_M | BYPSHAD | FMT | ALRAE | ALRBE | WUTE | ALRAIE | ALRBIE | WUTIE
# Flags: ISR positions, SR positions
ISR_FLAG = dict(A=1 << 8, B=1 << 9, W=1 << 10)
SR_FLAG = dict(A=1 << 0, B=1 << 1, W=1 << 2)
INIT, INITF, INITS, RSF = 1 << 7, 1 << 6, 1 << 4, 1 << 5
# Alarm register
MSK1, MSK2, MSK3, MSK4, WDSEL, AL_PM = 1 << 7, 1 << 15, 1 << 23, 1 << 31, 1 << 30, 1 << 22

# LSE register layouts: bit positions
LSE_LAYOUT = {
	'bdcr': dict(lseon=0, lserdy=1, lsebyp=2, css=5, rtcsel=8, rtcen=15, bdrst=16, bd=0xFFFFFFFF, writable=0x030183BD),
	'l1': dict(lseon=8, lserdy=9, lsebyp=10, css=11, rtcsel=16, rtcen=22, bdrst=23, bd=0x00FFFF00, writable=0x00C30D00),
	'wba': dict(lseon=0, lserdy=1, lsebyp=2, css=5, rtcsel=8, rtcen=None, bdrst=16, bd=0xFFFFFFFF,
				writable=0x370D73BD),
}
WBA_LSESYSEN, WBA_LSESYSRDY = 1 << 7, 1 << 11
WBA_LSI1ON, WBA_LSI1RDY, WBA_LSI1PREDIV, WBA_LSI2ON, WBA_LSI2RDY = 1 << 26, 1 << 27, 1 << 28, 1 << 29, 1 << 30
CSR_LSION, CSR_LSIRDY, CSR_LSIPREDIV = 1, 2, 1 << 4

EXTI_LAYOUT = {'v1': (0x00, 0x08, 0x14, 0x00), 'v2': (0x00, 0x08, 0x14, 0x00), 'h7': (0x80, 0x00, 0x88, 0x80)}


def run(cmd):
	p = subprocess.run(cmd, capture_output=True, text=True)
	if p.returncode:
		raise RuntimeError(' '.join(cmd) + '\n' + p.stdout + p.stderr)


def bcd(v):
	return (v // 10) << 4 | v % 10


def unbcd(v, tens_mask, what):
	u, t = v & 0xF, (v >> 4) & tens_mask
	assert u <= 9, f'{what}: BCD units {u}'
	return t * 10 + u


def month_days(yy, mo):
	return 29 if mo == 2 and yy % 4 == 0 else (31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31)[mo - 1]


class Calendar:
	"""Calendar fields as the RTC counts them: hour in the format of FMT"""

	def __init__(self):
		self.reset()

	def reset(self):
		self.yy, self.mo, self.dd, self.wdu = 0, 1, 1, 1
		self.hr, self.pm, self.mi, self.se = 0, 0, 0, 0

	def h24(self, fmt):
		return (self.hr % 12) + (12 if self.pm else 0) if fmt else self.hr

	def set_h24(self, h, fmt):
		if fmt:
			self.pm = 1 if h >= 12 else 0
			self.hr = h % 12 or 12
		else:
			self.hr, self.pm = h, 0

	def tr(self):
		return bcd(self.se) | bcd(self.mi) << 8 | bcd(self.hr) << 16 | self.pm << 22

	def dr(self):
		return bcd(self.dd) | bcd(self.mo) << 8 | self.wdu << 13 | bcd(self.yy) << 16

	def write_tr(self, v, fmt):
		assert v & ~0x007F7F7F == 0, f'TR reserved bits 0x{v:08x}'
		se, mi, hr = unbcd(v, 7, 'TR seconds'), unbcd(v >> 8, 7, 'TR minutes'), unbcd(v >> 16, 3, 'TR hours')
		pm = (v >> 22) & 1
		assert se < 60 and mi < 60, f'TR 0x{v:08x}'
		if fmt:
			assert 1 <= hr <= 12, f'TR 12 hour format hour {hr}'
		else:
			assert hr < 24 and pm == 0, f'TR 24 hour format 0x{v:08x}'
		self.se, self.mi, self.hr, self.pm = se, mi, hr, pm

	def write_dr(self, v):
		assert v & ~0x00FFFF3F == 0, f'DR reserved bits 0x{v:08x}'
		dd, mo, yy = unbcd(v, 3, 'DR day'), unbcd(v >> 8, 1, 'DR month'), unbcd(v >> 16, 0xF, 'DR year')
		wdu = (v >> 13) & 7
		assert 1 <= mo <= 12 and 1 <= dd <= month_days(yy, mo), f'DR date 0x{v:08x}'
		assert wdu != 0, 'DR day of the week 0'
		self.dd, self.mo, self.yy, self.wdu = dd, mo, yy, wdu

	def add(self, n, fmt):
		total = self.h24(fmt) * 3600 + self.mi * 60 + self.se + n
		days, rem = divmod(total, 86400)
		h, rem = divmod(rem, 3600)
		self.mi, self.se = divmod(rem, 60)
		self.set_h24(h, fmt)
		if days:
			d = datetime.date(2000 + self.yy, self.mo, self.dd) + datetime.timedelta(days=days)
			assert d.year < 2100, 'calendar past 2099'
			self.yy, self.mo, self.dd = d.year - 2000, d.month, d.day
			self.wdu = (self.wdu - 1 + days) % 7 + 1

	def seconds(self, fmt):
		days = (datetime.date(2000 + self.yy, self.mo, self.dd) - datetime.date(2000, 1, 1)).days
		return days * 86400 + self.h24(fmt) * 3600 + self.mi * 60 + self.se


class Model:
	def __init__(self, part, sc):
		name, fam_name, cpu, fam, diff = part
		f = dict(FAMILY[fam])
		f.update(diff)
		self.f = f
		self.fam = fam
		self.gen = f['gen']
		self.regs = REGS[self.gen]
		self.off = {v: k for k, v in self.regs.items()}
		self.lay = LSE_LAYOUT[f['lse'][1]]
		self.alarm_b = f.get('alarm_b', True)
		self.has_wakeup = f.get('wakeup', True)
		self.alrwf = f.get('alrwf', True)
		self.nbkp = f['nbkp']
		self.sc = sc
		self.crystals = SCENARIOS[sc][1]
		self.preset = SCENARIOS[sc][2]
		self.opts = SCENARIOS[sc][3] if len(SCENARIOS[sc]) > 3 else {}
		self.lse_hz = self.opts.get('lse_hz', LSE_HZ)
		self.uc = None
		self.ctl = [0] * 64
		self.base = dict(rtc=None, rcc=None, pwr=None, exti=None, tamp=None)
		self.mem = {}
		self.t = Fraction(0)
		self.tc = Fraction(0)
		self.boot = 0
		self.checked = 0
		self.boot_checks = 0
		self.done = False
		self.reboot = False
		self.bd_resets = 0
		self.init_entries = 0
		self.cal_read = (0, 0)
		self.extra = f.get(self.preset, 0) if self.preset in ('extra4', 'extra5') else 0
		self.nvic_en = [0] * 8
		self.nvic_latch = [0] * 8
		self.exti = dict(imr=0, rtsr=0, pr=0)
		self.sig = {}
		# Oscillators outside the backup domain
		self.lsi_on = False
		self.lsi_t = Fraction(0)
		self.csr_prediv = False
		self.domain_reset()
		self.apply_preset()

	# ---------------------------------------------------------------- state
	def domain_reset(self):
		self.bd = 0						# LSE register, backup domain bits
		self.lse_t = Fraction(0)
		self.wba_lsi_t = Fraction(0)
		self.cal = Calendar()
		self.cyc = Fraction(0)
		self.cr = 0
		self.prer = 0x007F00FF
		self.wutr = 0xFFFF
		self.alrm = {'A': 0, 'B': 0}
		self.alrmss = {'A': 0, 'B': 0}
		self.flags = set()
		self.init = False
		self.initf_at = None
		self.resume_at = Fraction(0)
		self.prer_writes = 0
		self.bin = 0
		self.locked = True
		self.wpr_state = 0
		self.t_off = {'A': None, 'B': None, 'W': None}
		self.wk_left = None
		self.bkp = [0] * self.nbkp

	def apply_preset(self):
		p = self.preset
		lay = self.lay
		if p == 'extra4':
			self.bd |= self.extra
		elif p in ('hse', 'lse_reset', 'lse_off', 'fmt_pm', 'fmt_am', 'prer', 'bin'):
			# RTC clock selected by earlier software, LSE running
			self.bd |= 1 << lay['lseon']
			self.lse_t = Fraction(-LSE_START_US)
			sel = 3 if p == 'hse' else 1
			self.bd |= sel << lay['rtcsel']
			if lay['rtcen'] is not None and p != 'lse_off':
				self.bd |= 1 << lay['rtcen']
			if p == 'hse':
				self.extra = self.f['extra5']
				self.bd |= self.extra
				self.wba_lsi_t = Fraction(-LSI_START_US)
			elif p == 'fmt_pm':
				self.cr |= FMT
				self.cal.yy, self.cal.mo, self.cal.dd, self.cal.wdu = 25, 12, 31, 3
				self.cal.hr, self.cal.pm, self.cal.mi, self.cal.se = 11, 1, 59, 58
			elif p == 'fmt_am':
				self.cr |= FMT
				self.cal.yy, self.cal.mo, self.cal.dd, self.cal.wdu = 25, 6, 1, 7
				self.cal.hr, self.cal.pm, self.cal.mi, self.cal.se = 12, 0, 30, 0
			elif p == 'prer':
				self.prer = 127 << 16 | 249
				self.cal.yy, self.cal.mo, self.cal.dd, self.cal.wdu = 25, 5, 5, 1
				self.cal.hr, self.cal.mi, self.cal.se = 5, 5, 5
			elif p == 'bin':
				self.bin = 1
				self.cal.yy, self.cal.hr, self.cal.mi, self.cal.se = 25, 9, 8, 7
			elif p == 'lse_off':
				self.cal.yy, self.cal.mo, self.cal.dd, self.cal.wdu = 24, 6, 1, 6
				self.cal.hr, self.cal.mi, self.cal.se = 12, 34, 56
		elif p == 'prediv':
			if self.f.get('prediv') == 'csr':
				self.csr_prediv = True
			else:
				self.bd |= WBA_LSI1PREDIV

	def system_reset(self):
		self.sync(self.t)
		self.mem = {}
		self.nvic_en = [0] * 8
		self.nvic_latch = [0] * 8
		self.exti = dict(imr=0, rtsr=0, pr=0)
		self.sig = {}
		if self.fam != 'WBA':
			# LSI outside the backup domain
			self.lsi_on = False
		self.locked = True
		self.wpr_state = 0
		self.init_entries = 0
		self.boot_checks = 0
		self.reboot = False

	# ---------------------------------------------------------------- clocks
	def crystal(self):
		x = self.crystals[min(self.boot, len(self.crystals) - 1)]
		if x == 'xtal_bdrst':
			# Crystal until the first backup domain reset
			return 'xtal' if self.bd_resets == 0 else 'none'
		return x

	def bit(self, name):
		pos = self.lay[name]
		return 0 if pos is None else 1 << pos

	def lse_ready_at(self):
		if not self.bd & self.bit('lseon'):
			return None
		byp = bool(self.bd & self.bit('lsebyp'))
		x = self.crystal()
		if x == 'none' or byp != (x == 'tcxo'):
			return None
		return self.lse_t + LSE_START_US

	def lsi_ready_at(self):
		if self.fam == 'WBA':
			return self.wba_lsi_t + LSI_START_US if self.bd & WBA_LSI1ON else None
		return self.lsi_t + LSI_START_US if self.lsi_on else None

	def lsi_hz(self):
		div = (self.bd & WBA_LSI1PREDIV) if self.fam == 'WBA' else self.csr_prediv
		return Fraction(self.f['lsi'], 128 if div else 1)

	def sel(self):
		return (self.bd >> self.lay['rtcsel']) & 3

	def rtcclk(self, t):
		if self.bd & self.bit('bdrst'):
			return 0
		if self.lay['rtcen'] is not None and not self.bd & self.bit('rtcen'):
			return 0
		s = self.sel()
		if s == 1:
			r = self.lse_ready_at()
			return self.lse_hz if r is not None and t >= r else 0
		if s == 2:
			r = self.lsi_ready_at()
			return self.lsi_hz() if r is not None and t >= r else 0
		if s == 3:
			return HSE_RTC_HZ
		return 0

	def changes(self, t):
		c = [x for x in (self.lse_ready_at(), self.lsi_ready_at(), self.resume_at) if x is not None and x > t]
		return min(c) if c else None

	# ---------------------------------------------------------------- time
	def per_second(self):
		return ((self.prer >> 16) + 1) * ((self.prer & 0x7FFF) + 1)

	def fmt(self):
		return bool(self.cr & FMT)

	def sync(self, t_new):
		while self.tc < t_new:
			nxt = self.changes(self.tc)
			end = t_new if nxt is None or nxt > t_new else nxt
			f = self.rtcclk(self.tc)
			if f:
				dc = (end - self.tc) * f / 1000000
				if not self.init and self.tc >= self.resume_at and not self.bin:
					self.count(dc)
				if self.cr & WUTE and (self.cr & WUCKSEL_M) < 4:
					self.wk_left -= dc
					while self.wk_left <= 0:
						self.event('W')
						self.wk_left += ((self.wutr & 0xFFFF) + 1) * (16 >> (self.cr & 3))
			self.tc = end

	def count(self, dc):
		per = self.per_second()
		total = self.cyc + dc
		n = math.floor(total / per)
		self.cyc = total - n * per
		if n == 0:
			return
		spre = self.cr & WUTE and (self.cr & WUCKSEL_M) >= 4
		if not (self.cr & (ALRAE | ALRBE)) and not spre:
			self.cal.add(n, self.fmt())
			return
		for _ in range(n):
			self.cal.add(1, self.fmt())
			for a, en in (('A', ALRAE), ('B', ALRBE)):
				if self.cr & en and self.alarm_match(a):
					self.event(a)
			if spre:
				self.wk_left -= 1
				if self.wk_left <= 0:
					self.event('W')
					self.wk_left = self.spre_period()

	def spre_period(self):
		return (self.wutr & 0xFFFF) + 1 + (65536 if self.cr & 2 else 0)

	def alarm_match(self, a):
		r = self.alrm[a]
		c = self.cal
		assert (self.alrmss[a] >> 24) & 0xF == 0, 'alarm sub-second comparison not modelled'
		if not r & MSK1 and (r & 0x7F) != bcd(c.se):
			return False
		if not r & MSK2 and ((r >> 8) & 0x7F) != bcd(c.mi):
			return False
		if not r & MSK3 and (((r >> 16) & 0x3F) != bcd(c.hr) or bool(r & AL_PM) != bool(c.pm)):
			return False
		if not r & MSK4:
			if r & WDSEL:
				if (r >> 24) & 0xF != c.wdu:
					return False
			elif (r >> 24) & 0x3F != bcd(c.dd):
				return False
		return True

	def event(self, kind):
		self.flags.add(kind)
		self.signals()

	def calendar_us(self):
		frac = self.cyc / self.per_second()
		return self.cal.seconds(self.fmt()), int(frac * 1000000)

	# ---------------------------------------------------------------- RTC
	def ss(self):
		a = (self.prer >> 16) + 1
		return (self.prer & 0x7FFF) - math.floor(self.cyc / a)

	def wf(self, kind, en):
		if self.cr & en:
			return False
		t = self.t_off[kind]
		if t is None:
			return True
		f = self.rtcclk(self.t)
		return bool(f) and self.t >= t + Fraction(2000000) / f

	def initf(self):
		return self.init and self.initf_at is not None and self.t >= self.initf_at

	def flag_bits(self, table):
		v = 0
		for k in self.flags:
			v |= table[k]
		return v

	def status(self):
		v = INIT if self.init else 0
		if self.initf():
			v |= INITF
		if self.cal.yy:
			v |= INITS
		if not self.init:
			v |= RSF
		if self.has_wakeup and self.wf('W', WUTE):
			v |= 1 << 2
		if self.alrwf:
			if self.wf('A', ALRAE):
				v |= 1
			if self.alarm_b and self.wf('B', ALRBE):
				v |= 2
		if self.gen == 'isr':
			v |= self.flag_bits(ISR_FLAG)
		elif self.f.get('bin'):
			v |= self.bin << 8
		return v

	def masked(self):
		m = 0
		for k, en in (('A', ALRAIE), ('B', ALRBIE), ('W', WUTIE)):
			if k in self.flags and self.cr & en:
				m |= SR_FLAG[k]
		return m

	def signals(self):
		if self.f['exti'] is None:
			return
		alarm = any(k in self.flags and self.cr & en for k, en in (('A', ALRAIE), ('B', ALRBIE)))
		wk = 'W' in self.flags and bool(self.cr & WUTIE)
		for line, s in ((self.f['lines'][0], alarm), (self.f['lines'][1], wk)):
			if s and not self.sig.get(line) and self.exti['rtsr'] & (1 << line):
				self.exti['pr'] |= 1 << line
			self.sig[line] = s

	def check_rtc_access(self):
		ab = self.f['apben']
		if ab:
			assert self.plain_read(self.base['rcc'] + ab[0], 4) & (1 << ab[1]), 'RTC access without its bus clock'

	def check_bd_write(self, what):
		off, bit = self.f['dbp']
		assert self.plain_read(self.base['pwr'] + off, 4) & (1 << bit), f'{what} written without backup domain access'

	def rtc_read(self, off):
		self.check_rtc_access()
		r = self.regs
		if off in (r['TR'], r['DR'], r['SSR']):
			assert self.cr & BYPSHAD or self.initf(), 'calendar read from the shadow registers'
			self.sync(self.t)
			v = {r['TR']: self.cal.tr(), r['DR']: self.cal.dr(), r['SSR']: self.ss()}[off]
			self.t += 1
			return v
		if off == r['CR']:
			return self.cr
		if off == r.get('ISR', -1) or off == r.get('ICSR', -1):
			return self.status()
		if off == r['PRER']:
			return self.prer
		if off == r['WUTR']:
			return self.wutr
		if off == r['WPR']:
			return 0
		for a in ('A', 'B'):
			if off == r['ALRM' + a + 'R']:
				return self.alrm[a]
			if off == r['ALRM' + a + 'SSR']:
				return self.alrmss[a]
		if off == r.get('SR', -1):
			return self.flag_bits(SR_FLAG)
		if off == r.get('MISR', -1):
			return self.masked()
		if off == r.get('SCR', -1):
			return 0
		raise AssertionError(f'RTC read of offset 0x{off:x}')

	def protected(self, what):
		assert not self.locked, f'RTC {what} written while write protected'

	def rtc_write(self, off, v):
		self.check_rtc_access()
		self.check_bd_write('RTC')
		self.sync(self.t)
		r = self.regs
		if off == r['WPR']:
			if v == 0xCA:
				self.wpr_state = 1
			elif v == 0x53 and self.wpr_state == 1:
				self.locked = False
				self.wpr_state = 0
			else:
				self.locked = True
				self.wpr_state = 0
			return
		if off == r.get('ISR', -1):
			if (v & INIT) != (INIT if self.init else 0):
				self.protected('INIT')
				self.set_init(bool(v & INIT))
			for k, b in ISR_FLAG.items():
				if not v & b:
					self.flags.discard(k)
			self.signals()
			return
		if off == r.get('ICSR', -1):
			if (v & INIT) != (INIT if self.init else 0):
				self.protected('INIT')
				self.set_init(bool(v & INIT))
			if self.f.get('bin'):
				b = (v >> 8) & 3
				if b != self.bin or (v >> 10) & 7:
					self.protected('BIN')
					assert self.initf(), 'BIN written outside the initialization mode'
					self.bin = b
			return
		if off == r.get('SCR', -1):
			for k, b in SR_FLAG.items():
				if v & b:
					self.flags.discard(k)
			self.signals()
			return
		if off in (r.get('SR', -1), r.get('MISR', -1)):
			return
		if off in (r['TR'], r['DR']):
			self.protected('TR/DR')
			assert self.initf(), 'TR/DR written outside the initialization mode'
			if off == r['TR']:
				self.cal.write_tr(v, self.fmt())
			else:
				self.cal.write_dr(v)
			return
		if off == r['PRER']:
			self.protected('PRER')
			assert self.initf(), 'PRER written outside the initialization mode'
			assert v & ~0x007F7FFF == 0, f'PRER 0x{v:08x}'
			self.prer_writes += 1
			assert self.prer_writes <= 2, 'PRER written more than twice in the initialization'
			if self.prer_writes == 1:
				self.prer = (self.prer & 0x7F0000) | (v & 0x7FFF)
			else:
				self.prer = (self.prer & 0x7FFF) | (v & 0x7F0000)
			return
		if off == r['CR']:
			self.protected('CR')
			assert v & ~CR_KNOWN == 0, f'CR bits 0x{v & ~CR_KNOWN:x}'
			old = self.cr
			ch = old ^ v
			if ch & FMT:
				assert self.initf(), 'FMT written outside the initialization mode'
			if ch & WUCKSEL_M:
				assert not old & WUTE and self.wf('W', WUTE), 'WUCKSEL written with the wakeup timer on'
			if v & (ALRBE | ALRBIE):
				assert self.alarm_b, 'alarm B on a part without it'
			if v & (WUTE | WUTIE | WUCKSEL_M):
				assert self.has_wakeup, 'wakeup timer on a part without it'
			self.cr = v
			for k, en in (('A', ALRAE), ('B', ALRBE), ('W', WUTE)):
				if old & en and not v & en:
					self.t_off[k] = self.t
			if v & WUTE and not old & WUTE:
				if (v & WUCKSEL_M) < 4:
					self.wk_left = Fraction(((self.wutr & 0xFFFF) + 1) * (16 >> (v & 3)))
				else:
					self.wk_left = self.spre_period()
			self.signals()
			return
		if off == r['WUTR']:
			self.protected('WUTR')
			assert self.has_wakeup, 'WUTR on a part without the wakeup timer'
			assert not self.cr & WUTE and self.wf('W', WUTE), 'WUTR written with the wakeup timer on or WUTWF clear'
			assert v & ~0xFFFF == 0, f'WUTR 0x{v:08x}'
			self.wutr = v
			return
		for a, en in (('A', ALRAE), ('B', ALRBE)):
			if off in (r['ALRM' + a + 'R'], r['ALRM' + a + 'SSR']):
				assert a == 'A' or self.alarm_b, 'alarm B on a part without it'
				self.protected('alarm ' + a)
				ok = not self.cr & en and (not self.alrwf or self.wf(a, en))
				assert ok or self.initf(), f'alarm {a} written while enabled or before ALR{a}WF'
				if off == r['ALRM' + a + 'R']:
					self.alrm[a] = v
				else:
					assert v & ~0x0F007FFF == 0, f'ALRM{a}SSR 0x{v:08x}'
					self.alrmss[a] = v
				return
		raise AssertionError(f'RTC write of offset 0x{off:x}')

	def set_init(self, on):
		if on:
			self.init = True
			self.init_entries += 1
			self.prer_writes = 0
			f = self.rtcclk(self.t)
			self.initf_at = self.t + Fraction(2000000) / f if f else None
		else:
			self.init = False
			self.initf_at = None
			f = self.rtcclk(self.t)
			self.resume_at = self.t + (Fraction(4000000) / f if f else 0)
			self.cyc = Fraction(0)

	# ---------------------------------------------------------------- backup registers
	def bkp_index(self, addr):
		if self.gen == 'isr':
			i = (addr - self.base['rtc'] - RTC_BKP_OFF) // 4
		else:
			i = (addr - self.base['tamp'] - TAMP_BKP_OFF) // 4
		assert 0 <= i < self.nbkp, f'backup register {i} of {self.nbkp}'
		return i

	# ---------------------------------------------------------------- RCC
	def lse_reg_read(self):
		v = self.bd
		r = self.lse_ready_at()
		if r is not None and self.t >= r:
			v |= self.bit('lserdy')
			if self.fam == 'WBA' and v & WBA_LSESYSEN:
				v |= WBA_LSESYSRDY
		if self.fam == 'WBA':
			lr = self.lsi_ready_at()
			if lr is not None and self.t >= lr:
				v |= WBA_LSI1RDY
			if v & WBA_LSI2ON:
				v |= WBA_LSI2RDY
		if self.lay is LSE_LAYOUT['l1']:
			v |= self.csr_read()
		return v

	def csr_read(self):
		v = 0
		if self.lsi_on:
			v |= CSR_LSION
			if self.t >= self.lsi_t + LSI_START_US:
				v |= CSR_LSIRDY
		if self.csr_prediv:
			v |= CSR_LSIPREDIV
		return v

	def csr_write(self, v):
		on = bool(v & CSR_LSION)
		if on and not self.lsi_on:
			self.lsi_t = self.t
		self.lsi_on = on

	def lse_reg_write(self, v):
		self.sync(self.t)
		lay = self.lay
		if lay is LSE_LAYOUT['l1']:
			self.csr_write(v)
		w = lay['writable']
		bdrst = self.bit('bdrst')
		new = v & w
		cur = self.bd & w
		if new == cur:
			return
		self.check_bd_write('RCC backup domain control')
		if self.bd & bdrst:
			# Held in reset: only the reset bit
			self.bd = new & bdrst
			return
		if new & bdrst:
			self.bd_resets += 1
			self.domain_reset()
			self.bd = bdrst
			self.signals()
			return
		lseon, byp = self.bit('lseon'), self.bit('lsebyp')
		if (new ^ cur) & byp:
			assert not cur & lseon, 'LSEBYP written with the LSE on'
		if new & lseon and not cur & lseon:
			self.lse_t = self.t
		sel_m = 3 << lay['rtcsel']
		if cur & sel_m:
			new = (new & ~sel_m) | (cur & sel_m)
		self.bd = new
		r = self.lse_ready_at()
		ready = r is not None and self.t >= r
		if new & self.bit('css') and not cur & self.bit('css'):
			assert ready, 'LSE clock security enabled with the LSE not ready'
		if self.fam == 'WBA':
			if new & WBA_LSESYSEN and not cur & WBA_LSESYSEN:
				assert ready, 'LSESYSEN set with the LSE not ready'
			if (new ^ cur) & WBA_LSI1PREDIV:
				assert not (cur | new) & WBA_LSI1ON, 'LSI1PREDIV written with the LSI1 on'
			if new & WBA_LSI1ON and not cur & WBA_LSI1ON:
				self.wba_lsi_t = self.t

	# ---------------------------------------------------------------- memory
	def plain_read(self, addr, size):
		return (self.mem.get(addr & ~3, 0) >> (8 * (addr & 3))) & ((1 << (8 * size)) - 1)

	def plain_write(self, addr, size, value):
		sh = 8 * (addr & 3)
		mask = ((1 << (8 * size)) - 1) << sh
		self.mem[addr & ~3] = (self.mem.get(addr & ~3, 0) & ~mask) | ((value << sh) & mask)

	def irq_level(self):
		lv = 0
		f = self.f
		if f['exti'] is None:
			if self.masked():
				lv |= 1 << f['irq'][0]
		else:
			act = self.exti['pr'] & self.exti['imr']
			if act & (1 << f['lines'][0]):
				lv |= 1 << f['irq'][0]
			if act & (1 << f['lines'][1]):
				lv |= 1 << f['irq'][1]
		return lv

	def where(self, addr):
		b = self.base
		if b['rtc'] is not None and b['rtc'] <= addr < b['rtc'] + 0x400:
			if self.gen == 'isr' and addr >= b['rtc'] + RTC_BKP_OFF:
				return 'bkp', addr
			return 'rtc', addr - b['rtc']
		if b['tamp'] and b['tamp'] <= addr < b['tamp'] + 0x400:
			assert addr >= b['tamp'] + TAMP_BKP_OFF, f'TAMP offset 0x{addr - b["tamp"]:x}'
			return 'bkp', addr
		if b['rcc'] is not None and b['rcc'] <= addr < b['rcc'] + 0x400:
			return 'rcc', addr - b['rcc']
		if b['pwr'] is not None and b['pwr'] <= addr < b['pwr'] + 0x400:
			return 'pwr', addr - b['pwr']
		if b['exti'] is not None and b['exti'] <= addr < b['exti'] + 0x400:
			return 'exti', addr - b['exti']
		return None, addr

	# ---------------------------------------------------------------- MMIO
	def read(self, uc, offset, size, region):
		addr = region + offset
		if self.done or self.reboot:
			return 0
		if CONTROL <= addr < CONTROL + 0x100:
			return self.control_read((addr - CONTROL) // 4) & 0xFFFFFFFF
		if NVIC <= addr < NVIC + 0x1000:
			return self.nvic_read(addr - NVIC, size)
		kind, off = self.where(addr)
		if kind is None:
			return self.plain_read(addr, size)
		assert size == 4, f'{kind} access of {size} bytes at 0x{addr:08x}'
		if kind == 'rtc':
			return self.rtc_read(off)
		if kind == 'bkp':
			self.check_rtc_access()
			return self.bkp[self.bkp_index(addr)]
		if kind == 'rcc':
			if off == self.f['lse'][0]:
				return self.lse_reg_read()
			if off == self.f['csr']:
				return self.csr_read()
			return self.plain_read(addr, 4)
		if kind == 'exti':
			return self.exti_read(off)
		return self.plain_read(addr, 4)

	def write(self, uc, offset, size, value, region):
		addr = region + offset
		value &= 0xFFFFFFFF
		if self.done or self.reboot:
			return
		if CONTROL <= addr < CONTROL + 0x100:
			self.control_write(uc, (addr - CONTROL) // 4, value)
			return
		if NVIC <= addr < NVIC + 0x1000:
			self.nvic_write(addr - NVIC, size, value)
			return
		kind, off = self.where(addr)
		if kind is None:
			self.plain_write(addr, size, value)
			return
		assert size == 4, f'{kind} access of {size} bytes at 0x{addr:08x}'
		if kind == 'rtc':
			self.rtc_write(off, value)
		elif kind == 'bkp':
			self.check_rtc_access()
			self.check_bd_write('backup register')
			self.bkp[self.bkp_index(addr)] = value
		elif kind == 'rcc':
			if off == self.f['lse'][0]:
				self.lse_reg_write(value)
			elif off == self.f['csr']:
				self.sync(self.t)
				self.csr_write(value)
			else:
				self.plain_write(addr, 4, value)
		elif kind == 'pwr':
			pe = self.f['pwren']
			if pe:
				assert self.plain_read(self.base['rcc'] + pe[0], 4) & (1 << pe[1]), 'PWR written without its clock'
			self.plain_write(addr, 4, value)
		elif kind == 'exti':
			self.exti_write(off, value)

	def exti_regs(self):
		imr, rtsr, pr, _ = EXTI_LAYOUT[self.f['exti']]
		return imr, rtsr, pr

	def exti_read(self, off):
		imr, rtsr, pr = self.exti_regs()
		if off == imr:
			return self.exti['imr']
		if off == rtsr:
			return self.exti['rtsr']
		if off == pr:
			return self.exti['pr']
		return self.plain_read(self.base['exti'] + off, 4)

	def exti_write(self, off, v):
		imr, rtsr, pr = self.exti_regs()
		if off == imr:
			self.exti['imr'] = v
		elif off == rtsr:
			self.exti['rtsr'] = v
		elif off == pr:
			self.exti['pr'] &= ~v
		else:
			self.plain_write(self.base['exti'] + off, 4, v)

	def nvic_read(self, off, size):
		if 0x100 <= off < 0x120 or 0x180 <= off < 0x1A0:
			return self.nvic_en[(off & 0x1F) // 4]
		if 0x200 <= off < 0x220 or 0x280 <= off < 0x2A0:
			n = (off & 0x1F) // 4
			lv = (self.irq_level() >> (32 * n)) & 0xFFFFFFFF
			return self.nvic_latch[n] | lv
		return self.plain_read(NVIC + off, size)

	def nvic_write(self, off, size, v):
		n = (off & 0x1F) // 4
		if 0x100 <= off < 0x120:
			self.nvic_en[n] |= v
		elif 0x180 <= off < 0x1A0:
			self.nvic_en[n] &= ~v
		elif 0x200 <= off < 0x220:
			self.nvic_latch[n] |= v
		elif 0x280 <= off < 0x2A0:
			self.nvic_latch[n] &= ~v
		else:
			self.plain_write(NVIC + off, size, v)

	# ---------------------------------------------------------------- test registers
	def control_read(self, n):
		f = self.f
		self.sync(self.t)
		if n == 0:
			return self.sc
		if n == 4:
			return self.boot
		if n == 6:
			return int(self.t)
		if n == 7:
			self.cal_read = self.calendar_us()
			return self.cal_read[0]
		if n == 8:
			return self.cal_read[1]
		if n == 9:
			return int(self.rtcclk(self.t)) if not self.init else 0
		if n == 10:
			return self.bd_resets
		if n == 11:
			return self.init_entries
		if n == 12:
			return 1 if self.locked else 0
		if n == 13:
			return self.sel()
		if n == 14:
			r = self.lse_ready_at()
			v = (1 if self.bd & self.bit('lseon') else 0) | (2 if r is not None and self.t >= r else 0)
			v |= 4 if self.bd & self.bit('lsebyp') else 0
			v |= 8 if (self.bd & WBA_LSI1ON if self.fam == 'WBA' else self.lsi_on) else 0
			v |= 16 if (self.bd & self.extra) == self.extra else 0
			v |= 32 if self.lay['rtcen'] is None or self.bd & self.bit('rtcen') else 0
			return v
		if n == 16:
			return 2 if self.alarm_b else 1
		if n == 17:
			return self.nbkp
		if n == 18:
			return 1 if self.has_wakeup else 0
		if n == 19:
			return 1 if f['irq'][0] == f['irq'][1] else 2
		if n == 20:
			return f['irq'][0]
		if n == 21:
			return f['irq'][1]
		if n == 22:
			return int(self.lsi_hz())
		if n == 23:
			if self.opts.get('board') == 'rc':
				return 0
			return 2 if self.crystals[0] == 'tcxo' else 1
		if n == 24:
			return 0 if self.lay['rtcen'] is None else 1
		if n == 25:
			return 1 if self.fam == 'WBA' else 0
		if n == 31:
			return LSE_START_US
		if n == 32:
			return self.lse_hz
		return self.ctl[n]

	def control_write(self, uc, n, value):
		self.ctl[n] = value
		if n == 1:
			assert not value & 0x80000000, \
				f'firmware check {value & 0x7fffffff} failed, return address 0x{uc.reg_read(arm_const.UC_ARM_REG_LR):08x}'
			self.checked += 1
			self.boot_checks += 1
		elif n == 2:
			assert value == self.boot_checks, f'{value} checks run, {self.boot_checks} seen'
			self.done = True
			uc.emu_stop()
		elif n == 3:
			self.t += value
			self.sync(self.t)
		elif n == 5:
			self.reboot = True
			uc.emu_stop()
		elif 26 <= n <= 30:
			self.base[('rtc', 'rcc', 'pwr', 'exti', 'tamp')[n - 26]] = value or None


def emulate(path, cpu, model):
	uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
	uc.ctl_set_cpu_model(getattr(arm_const, 'UC_CPU_ARM_CORTEX_' + cpu))
	uc.mem_map(0x08000000, 0x200000)
	uc.mem_map(0x20000000, 0x80000)
	model.uc = uc
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
	uc.emu_start(start | 1, 0, count=200_000_000)


def scenario(exe, part, opt, sc):
	model = Model(part, sc)
	try:
		while True:
			emulate(exe, part[2], model)
			if model.done:
				break
			assert model.reboot, f'incomplete: {model.checked} checks'
			model.system_reset()
			model.boot += 1
			assert model.boot < 4, 'too many system resets'
	except (AssertionError, UcError) as e:
		raise SystemExit(f'FAIL {part[0]} {opt} {SCENARIOS[sc][0]} (boot {model.boot}): {e}')
	return model


def main():
	cxx = os.environ.get('ARM_CXX', 'arm-none-eabi-g++')
	cc = os.environ.get('ARM_CC', 'arm-none-eabi-gcc')
	if not shutil.which(cxx) or not shutil.which(cc):
		raise SystemExit('NOT RUN: arm-none-eabi-g++ or arm-none-eabi-gcc unavailable')
	with tempfile.TemporaryDirectory(prefix='stm32-rtc-') as td:
		td = Path(td)
		sel = os.environ.get('RTC_PARTS')
		only = os.environ.get('RTC_SCENARIOS')
		for part in PARTS:
			name, family, cpu = part[0], part[1], part[2]
			if sel and name not in sel.split(','):
				continue
			for opt in ('-O0', '-Os'):
				common = [f'-mcpu={GCC_CPU[cpu]}', '-mthumb', '-mfloat-abi=soft', opt, '-g', '-D' + name,
						  '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra', '-Werror',
						  '-Wno-missing-field-initializers', '-I' + str(HERE / 'delay_shim')]
				common += ['-I' + str(ROOT / d) for d in ('include', 'ARM/include', 'ARM/CMSIS/Core/Include',
						   'ARM/ST/include', 'ARM/ST/' + family + '/include')]
				flags = common + ['-std=gnu++23', '-fno-rtti', '-fno-exceptions']
				# ARM/src/iatomic.c gives the atomic functions of the Cortex-M0
				# (device_intrf.cpp), defined as the library does: GCC built-in
				# declarations with other integer types
				cflags = common + ['-std=gnu17', '-Wno-builtin-declaration-mismatch']
				objects = []
				for src in ('tests/stm32/rtc_fw.cpp', os.environ.get('RTC_SRC', 'ARM/ST/src/rtc_stm32.cpp'),
							'src/miscdev/rtc.cpp', 'src/device.cpp', 'src/device_intrf.cpp', 'ARM/src/iatomic.c'):
					obj = td / (Path(src).stem + '.o')
					if src.endswith('.c'):
						run([cc, *cflags, '-c', str(ROOT / src), '-o', str(obj)])
					else:
						run([cxx, *flags, '-c', str(ROOT / src), '-o', str(obj)])
					objects.append(str(obj))
				exe = td / 'rtc.elf'
				run([cxx, *flags, '--specs=nano.specs', '--specs=nosys.specs', '-nostartfiles', '-Wl,--gc-sections',
					 '-Wl,--no-warn-mismatch', '-T' + str(HERE / 'rtc_fw.ld'), *objects, '-o', str(exe)])
				if os.environ.get('RTC_KEEP'):
					shutil.copy(exe, Path(os.environ['RTC_KEEP']) / f'{name}{opt}.elf')
				results = []
				for sc in SCENARIOS:
					if only and str(sc) not in only.split(','):
						continue
					m = scenario(exe, part, opt, sc)
					if m.checked:
						results.append(f'{sc} {m.checked}')
				print(f'{name} {opt}: ' + ', '.join(results) + ' PASS', flush=True)
	print('RTC model only. Hardware not tested by this script.')


if __name__ == '__main__':
	main()
