#!/usr/bin/env python3
"""Check the SCL rate of the SAM4E TWI driver on a host register model.

Run: python3 tests/sam4e/run_i2c_rate.py

Sam4eI2CSetRate of ARM/Microchip/SAM4E/src/i2c_sam4e.cpp is compiled against
a model of TWI_CWGR. The TWI clock of the datasheet: low period
(CLDIV * 2^CKDIV + 4) MCK cycles, high period (CHDIV * 2^CKDIV + 4) MCK
cycles. For each MCK and each request, the expected CKDIV and CLDIV + CHDIV
are found by a search over every value where:

- the low and high periods are at least the I2C minimums of the mode,
  standard up to 100 kHz, fast above
- SCL is at most 100 kHz in standard mode, 400 kHz in fast mode
- a request above 400 kHz takes 400 kHz, 0 returns 0

The SCL rate closest to the request is expected, the smaller CKDIV then the
higher rate on a tie; the driver returns that rate rounded down to the Hz.
Not a simulation of the bus; hardware not tested by this script.

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""
from fractions import Fraction
from pathlib import Path
import os
import re
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SRC = Path(os.environ.get('SAM4E_I2C_SRC', ROOT / 'ARM/Microchip/SAM4E/src/i2c_sam4e.cpp'))


def function(text, name):
	match = re.search(r'^.*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', text, re.M)
	assert match, name
	start = text.index('{', match.start())
	depth, end = 1, start + 1
	while depth:
		depth += (text[end] == '{') - (text[end] == '}')
		end += 1
	return text[match.start():end]


# I2C timing constants of include/coredev/i2c.h
I2C = {}
for line in (ROOT / 'include/coredev/i2c.h').read_text().splitlines():
	m = re.match(r'#define\s+(I2C_SCL_\w+)\s+(\d+)', line)
	if m:
		I2C[m.group(1)] = int(m.group(2))

CLOCKS = [1000000, 4000000, 8000000, 12000000, 32000000, 48000000, 64000000,
	96000000, 100000000, 120000000, 119999999]
RATES = [0, 1, 10, 100, 1000, 9765, 50000, 90000, 99999, 100000, 100001,
	101000, 150000, 250000, 333333, 380000, 399999, 400000, 400001, 1000000,
	0xFFFFFFFF]


def ceil_div(a, b):
	return -(-a // b)


def expected(mck, rate):
	if rate == 0:
		return None
	rate = min(rate, I2C['I2C_SCL_FAST_MODE_MAX_SPEED'] * 1000)
	fast = rate > I2C['I2C_SCL_STD_MODE_MAX_SPEED'] * 1000
	mode = 'FAST_MODE' if fast else 'STD_MODE'
	top = I2C['I2C_SCL_%s_MAX_SPEED' % mode] * 1000
	lowmin = ceil_div(mck * I2C['I2C_SCL_TLOW_%s_MIN' % mode], 10 ** 9)
	highmin = ceil_div(mck * I2C['I2C_SCL_THIGH_%s_MIN' % mode], 10 ** 9)
	best = None
	for ck in range(8):
		for s in range(511):
			per = (s << ck) + 8
			if per * top < mck:
				continue
			# Some split of s has both periods at their minimum or more
			clmin = max(0, ceil_div(lowmin - 4, 1 << ck))
			chmin = max(0, ceil_div(highmin - 4, 1 << ck))
			if clmin > 255 or chmin > 255 or s < clmin + chmin:
				continue
			err = abs(Fraction(mck, per) - rate)
			if best is None or err < best[0]:
				best = (err, ck, s, lowmin, highmin)
	return best[1:]


code = '''#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstddef>
struct Sam4eTwi { uint32_t TWI_CWGR; };
#define TWI_CWGR_CLDIV(value) (0xffu & (value))
#define TWI_CWGR_CHDIV(value) ((0xffu << 8) & ((value) << 8))
#define TWI_CWGR_CKDIV(value) ((0x7u << 16) & ((value) << 16))
''' + ''.join('#define %s %d\n' % kv for kv in I2C.items()) + '''
struct DevIntrf_t { void *pDevData; };
struct I2CDev_t { struct { uint32_t Rate; } Cfg; DevIntrf_t DevIntrf; };
typedef struct {
	int DevNo;
	uint32_t SamDevId;
	I2CDev_t *pI2cDev;
	Sam4eTwi *pReg;
	void *pPdc;
} SAM4E_I2CDEV;
uint32_t s_Clk;
uint32_t SystemPeriphClockGet(int) { return s_Clk; }
''' + function(SRC.read_text(), 'Sam4eI2CSetRate') + '''
struct Case { uint32_t Mck, Rate, Ck, Sum, LowMin, HighMin; };
static const Case s_Cases[] = {
'''
for mck in CLOCKS:
	for rate in RATES:
		e = expected(mck, rate)
		ck, s, lowmin, highmin = e if e else (0xFFFFFFFF, 0, 0, 0)
		code += '\t{%uU, %uU, %uU, %uU, %uU, %uU},\n' % (mck, rate, ck, s, lowmin, highmin)
code += '''};
int main()
{
	Sam4eTwi reg {};
	I2CDev_t i2c {};
	SAM4E_I2CDEV dev {};
	dev.pI2cDev = &i2c;
	dev.pReg = &reg;
	i2c.DevIntrf.pDevData = &dev;
	int n = 0;
	for (const Case &c : s_Cases)
	{
		s_Clk = c.Mck;
		reg.TWI_CWGR = 0xFFFFFFFF;
		i2c.Cfg.Rate = 12345;
		uint32_t r = Sam4eI2CSetRate(&i2c.DevIntrf, c.Rate);
		if (c.Ck == 0xFFFFFFFF)
		{
			if (r != 0 || i2c.Cfg.Rate != 12345 || reg.TWI_CWGR != 0xFFFFFFFF)
			{
				printf("mck %u rate %u: %u, expected 0\\n", c.Mck, c.Rate, r);
				n++;
			}
			continue;
		}
		uint32_t cl = reg.TWI_CWGR & 0xFF;
		uint32_t ch = (reg.TWI_CWGR >> 8) & 0xFF;
		uint32_t ck = (reg.TWI_CWGR >> 16) & 0x7;
		uint32_t scl = c.Mck / ((c.Sum << c.Ck) + 8);
		if (r != scl || i2c.Cfg.Rate != scl || ck != c.Ck || cl + ch != c.Sum || (reg.TWI_CWGR >> 19) != 0 ||
			(cl << ck) + 4 < c.LowMin || (ch << ck) + 4 < c.HighMin)
		{
			printf("mck %u rate %u: %u CLDIV %u CHDIV %u CKDIV %u, expected %u CLDIV + CHDIV %u CKDIV %u\\n",
				   c.Mck, c.Rate, r, cl, ch, ck, scl, c.Sum, c.Ck);
			n++;
		}
	}
	if (n)
	{
		printf("FAIL SAM4E TWI rate: %d cases\\n", n);
		return 1;
	}
	puts("PASS SAM4E TWI rate: NCLK clocks, NRATE requests, closest SCL in standard and fast mode");
	return 0;
}
'''
code = code.replace('NCLK', str(len(CLOCKS))).replace('NRATE', str(len(RATES)))

with tempfile.TemporaryDirectory(prefix='sam4e-i2c-') as directory:
	work = Path(directory)
	file = work / 'i2c_rate.cpp'
	file.write_text(code)
	for opt in ['-O0', '-O2']:
		exe = work / ('i2c_rate' + opt)
		subprocess.run(shlex.split(os.environ.get('CXX', 'g++')) + ['-std=c++17', opt, '-g', '-Wall', '-Wextra', '-Werror',
			'-Wshadow', '-fsanitize=undefined,address', '-fno-sanitize-recover=all', str(file), '-o', str(exe)], check=True)
		subprocess.run([str(exe)], check=True)
