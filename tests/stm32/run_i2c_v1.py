#!/usr/bin/env python3
"""Check the SCL rate of the STM32 I2C v1 driver on a host register model.

Run: python3 tests/stm32/run_i2c_v1.py

The rate functions of ARM/ST/src/i2c_stm32v1.cpp (F401, L152) are compiled
against a model of the I2C registers. For each PCLK1 and each request, the
expected CCR is found by a search over every valid CCR:

- standard mode, request up to 100 kHz: SCL = PCLK1 / (2 * CCR), CCR from 4
  to 0xFFF and SCL at most 100 kHz
- fast mode, duty 2:1: SCL = PCLK1 / (3 * CCR), CCR from 1 to 0xFFF and SCL
  at most 400 kHz
- a request above 400 kHz takes 400 kHz, 0 takes 100 kHz

The CCR of the SCL rate closest to the request is expected, the higher rate
on a tie; the driver returns that rate rounded down to the Hz. Not a
simulation of the bus; hardware not tested by this script.

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
SRC = Path(os.environ.get('I2C_V1_SRC', ROOT / 'ARM/ST/src/i2c_stm32v1.cpp'))


def function(text, name):
	match = re.search(r'^.*\b' + re.escape(name) + r'\([^;]*?\)\s*\{', text, re.M)
	assert match, name
	start = text.index('{', match.start())
	depth, end = 1, start + 1
	while depth:
		depth += (text[end] == '{') - (text[end] == '}')
		end += 1
	return text[match.start():end]


CLOCKS = [2000000, 4000000, 8000000, 16000000, 21000000, 24000000, 32000000,
	36000000, 41943040, 42000000, 1999999, 43000000]
RATES = [0, 1, 10, 1000, 9765, 50000, 90000, 99999, 100000, 100001, 101000,
	150000, 250000, 333333, 380000, 399999, 400000, 400001, 1000000,
	0xFFFFFFFF]


def expected(clk, rate):
	if clk // 1000000 < 2 or clk // 1000000 > 42:
		return None
	if rate == 0:
		rate = 100000
	rate = min(rate, 400000)
	if rate <= 100000:
		mul, low, top = 2, 4, 100000
	else:
		mul, low, top = 3, 1, 400000
	best = None
	for ccr in range(low, 0x1000):
		if clk > top * mul * ccr:
			continue
		diff = abs(Fraction(clk, mul * ccr) - rate)
		if best is None or diff < best[1]:
			best = (ccr, diff)
	ccr = best[0]
	return (ccr | (0x8000 if mul == 3 else 0)), clk // (mul * ccr)


code = '''#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstddef>
struct I2C_TypeDef { uint32_t CR1, CR2, CCR, TRISE; };
constexpr uint32_t I2C_CR1_PE = 1, I2C_CR1_ACK = 1u << 10, I2C_CR2_FREQ = 0x3F, I2C_CCR_FS = 0x8000;
struct DevIntrf_t { void *pDevData; };
struct I2CDev_t { DevIntrf_t DevIntrf; };
typedef struct {
	I2CDev_t *pDev;
	I2C_TypeDef *pReg;
	uint32_t Addr;
	uint32_t Rate;
	bool bActive;
	bool bRead;
} Stm32I2cv1_t;
uint32_t s_Clk;
uint32_t SystemPeriphClockGet(int) { return s_Clk; }
''' + '\n'.join(function(SRC.read_text(), n) for n in ['Stm32V1Handle', 'Stm32V1Ccr', 'Stm32V1Rate']) + '''
struct Case { uint32_t Clk, Rate, Ccr, Scl; };
static const Case s_Cases[] = {
'''
cases = 0
for clk in CLOCKS:
	for rate in RATES:
		e = expected(clk, rate)
		ccr, scl = e if e else (0xFFFFFFFF, 0)
		code += '\t{%uU, %uU, %uU, %uU},\n' % (clk, rate, ccr, scl)
		cases += 1
code += '''};
int main()
{
	I2C_TypeDef reg {};
	I2CDev_t dev {};
	Stm32I2cv1_t d {};
	d.pDev = &dev;
	d.pReg = &reg;
	dev.DevIntrf.pDevData = &d;
	int n = 0;
	for (const Case &c : s_Cases)
	{
		s_Clk = c.Clk;
		reg.CCR = 0xFFFFFFFF;
		d.Rate = 12345;
		uint32_t r = Stm32V1Rate(&dev.DevIntrf, c.Rate);
		if (c.Ccr == 0xFFFFFFFF)
		{
			if (r != 0 || d.Rate != 12345)
			{
				printf("clk %u rate %u: %u, expected 0\\n", c.Clk, c.Rate, r);
				n++;
			}
			continue;
		}
		if (r != c.Scl || d.Rate != c.Scl || reg.CCR != c.Ccr || !(reg.CR1 & I2C_CR1_PE) || (reg.CR2 & I2C_CR2_FREQ) != c.Clk / 1000000)
		{
			printf("clk %u rate %u: %u CCR 0x%x, expected %u CCR 0x%x\\n", c.Clk, c.Rate, r, reg.CCR, c.Scl, c.Ccr);
			n++;
		}
	}
	if (n)
	{
		printf("FAIL I2C v1 rate: %d cases\\n", n);
		return 1;
	}
	puts("PASS I2C v1 rate: NCLK clocks, NRATE requests, closest SCL in standard and fast mode");
	return 0;
}
'''
code = code.replace('NCLK', str(len(CLOCKS))).replace('NRATE', str(len(RATES)))

with tempfile.TemporaryDirectory(prefix='stm32-i2cv1-') as directory:
	work = Path(directory)
	file = work / 'i2c_v1.cpp'
	file.write_text(code)
	for opt in ['-O0', '-O2']:
		exe = work / ('i2c_v1' + opt)
		subprocess.run(shlex.split(os.environ.get('CXX', 'g++')) + ['-std=c++17', opt, '-g', '-Wall', '-Wextra', '-Werror',
			'-fsanitize=undefined,address', '-fno-sanitize-recover=all', str(file), '-o', str(exe)], check=True)
		subprocess.run([str(exe)], check=True)
