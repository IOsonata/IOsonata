#!/usr/bin/env python3
"""Test the production nRF53 I2S rate table/setter with modeled clock hooks.

This focused host test does not simulate DMA, interrupts or physical clocks.
The table and complete setter are extracted unchanged from the driver.
"""
import os
from pathlib import Path
import shlex
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
source = (ROOT / 'ARM/Nordic/src/i2s_nrfx.cpp').read_text()
start = source.index('typedef struct {', source.index('// HFCLKAUDIO'))
table = source[start:source.index('static bool s_AudioClockStarted;', start)]
start = source.index('static uint32_t nRFxI2SSetRate(')
setter = source[start:source.index('static void nRFxI2SService(', start)]
prefix = r'''
#include <cassert>
#include <cstdio>
#include "coredev/i2s.h"
#define NRF53_SERIES 1
#define I2S_CONFIG_MCKFREQ_MCKFREQ_32MDIV4 0x20000000U
#define I2S_CONFIG_CLKCONFIG_CLKSRC_ACLK 1U
static struct {
	struct { uint32_t CLKCONFIG, MCKFREQ, RATIO; } CONFIG;
} regs;
#define NRF_I2S (&regs)
static bool s_Running, s_AudioClockStarted, clockOk = true;
static uint32_t s_AudioClockReg;
static unsigned starts, stops;
static bool nRF53AudioClockStart() {
	starts++;
	s_AudioClockStarted = clockOk;
	return clockOk;
}
static void nRF53AudioClockStop() { stops++; s_AudioClockStarted = false; }
'''
checks = r'''
int main()
{
	I2SDev_t dev{};
	dev.DevIntrf.pDevData = &dev;
	dev.DevIntrf.SetRate = nRFxI2SSetRate;
	const uint32_t rates[] = {8000, 12000, 16000, 24000, 32000, 48000, 96000};
	const uint32_t ratios[] = {7, 6, 5, 4, 3, 4, 0};
	auto check = [&](uint32_t request, unsigned index) {
		assert(DeviceIntrfSetRate(&dev.DevIntrf, request) == rates[index]);
		assert(dev.Freq == rates[index]);
		assert(dev.MClkFreq == (index == 5 ? 6144000U : 3072000U));
		assert(regs.CONFIG.RATIO == ratios[index]);
		assert(regs.CONFIG.MCKFREQ == (index == 5 ? 0x66666000U : 0x20000000U));
		assert(regs.CONFIG.CLKCONFIG == 1 && s_AudioClockReg == 39846);
	};
	for (unsigned i = 0; i < 7; i++) check(rates[i], i);
	for (unsigned i = 0; i < 6; i++) {
		uint32_t midpoint = (rates[i] + rates[i + 1]) / 2;
		check(midpoint - 1, i);
		check(midpoint, i);
		check(midpoint + 1, i + 1);
	}
	check(1, 0);
	check(UINT32_MAX, 6);
	check(44100, 5);
	check(48001, 5);
	unsigned before = starts;
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 0) == 0);
	s_Running = true;
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 32000) == 0);
	s_Running = false;
	dev.DevIntrf.pDevData = nullptr;
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 32000) == 0);
	dev.DevIntrf.pDevData = &dev;
	assert(starts == before && dev.Freq == 48000);
	clockOk = false;
	s_AudioClockStarted = false;
	assert(DeviceIntrfSetRate(&dev.DevIntrf, 32000) == 0);
	assert(dev.Freq == 48000 && dev.MClkFreq == 6144000);
	assert(regs.CONFIG.MCKFREQ == 0x66666000U && regs.CONFIG.RATIO == 4);
	clockOk = true;
	s_AudioClockStarted = true;
	s_AudioClockReg = 0;
	check(32001, 4);
	assert(stops == 1);
	puts("PASS nRF53 I2S rates: exact, midpoints, bounds, guards and clock failure/retry");
}
'''
with tempfile.TemporaryDirectory() as temp:
    work = Path(temp)
    model = work / 'rate.cpp'
    model.write_text(prefix + table + setter + checks)
    for opt in ('-O0', '-O2'):
        exe = work / ('rate' + opt)
        command = shlex.split(os.environ.get('CXX', 'g++')) + [
            '-std=gnu++17', opt, '-g', '-Wall', '-Wextra', '-Werror',
            '-fsanitize=address,undefined', '-fno-sanitize-recover=all',
            '-I' + str(ROOT / 'tests/nrf5340_audio/hostport/coredev'),
            '-I' + str(ROOT / 'include'), str(model), '-o', str(exe)]
        subprocess.run(command, check=True)
        subprocess.run([str(exe)], check=True)
