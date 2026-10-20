/**-------------------------------------------------------------------------
@file	adc_fw.cpp

@brief	STM32WBA ADC4 driver test firmware.

Production ADC4 driver exercised by run_adc.py; no electrical simulation.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

@license

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

----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stddef.h>
#include "adc_stm32wba.h"
#include "coredev/iopincfg.h"

static volatile uint32_t *const io = (volatile uint32_t *)0x500FF000;
static unsigned checks, configured, analog;
static uint32_t clockHz = 16000000;

extern "C" uint32_t SystemCoreClockGet()
{
	return clockHz;
}

extern "C" uint32_t SystemMicroSecLoopCnt;
uint32_t SystemMicroSecLoopCnt = 1;

extern "C" void IOPinConfig(int port, int pin, int, IOPINDIR, IOPINRES, IOPINTYPE)
{
	if (port == 0 && pin == 3)
	{
		configured++;
	}
}

extern "C" void IOPinDisable(int port, int pin)
{
	if (port == 0 && pin == 3)
	{
		analog++;
	}
}

extern "C" void *memset(void *p, int c, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;
	while (n--)
	{
		*d++ = c;
	}
	return p;
}

extern "C" void *memcpy(void *p, const void *q, size_t n)
{
	volatile unsigned char *d = (volatile unsigned char *)p;
	const volatile unsigned char *s = (const volatile unsigned char *)q;
	while (n--)
	{
		*d++ = *s++;
	}
	return p;
}

void operator delete(void *) noexcept
{
}

void operator delete(void *, size_t) noexcept
{
}

static void Check(bool ok)
{
	io[1] = ++checks | (ok ? 0 : 0x80000000U);
}

extern "C" void ResetHandler()
{
	extern uint32_t _sidata, _sdata, _edata, _sbss, _ebss;
	uint32_t *s = &_sidata;
	for (uint32_t *d = &_sdata; d < &_edata;)
	{
		*d++ = *s++;
	}
	for (uint32_t *d = &_sbss; d < &_ebss;)
	{
		*d++ = 0;
	}
	AdcSTM32WBA adc, other;
	AdcRefVolt_t ref = { ADC_REFVOLT_TYPE_SUPPLY, 3.3f, -1 };
	AdcCfg_t cfg = { ADC_CONV_MODE_SINGLE, &ref, 1, 0, 12, 0, 0, false, 6, nullptr };
	AdcChanCfg_t ch = { 0, 0, ADC_CHAN_TYPE_SINGLE_ENDED, 0x100, 10, false, { 6, ADC_PIN_CONN_NONE }, { -1, ADC_PIN_CONN_NONE }, 0, nullptr };
	AdcData_t data;
	Check(!adc.Read(0, &data) && !adc.StartConversion());
	cfg.bInterrupt = true;
	Check(!adc.Init(cfg));
	cfg.bInterrupt = false;
	cfg.Mode = ADC_CONV_MODE_CONTINUOUS;
	Check(!adc.Init(cfg));
	cfg.Mode = ADC_CONV_MODE_SINGLE;
	clockHz = 1000000;
	Check(!adc.Init(cfg));
	clockHz = 16000000;
	Check(adc.Init(cfg) && adc.Valid());
	Check(!other.Init(cfg));
	Check(!adc.StartConversion());
	Check(adc.OpenChannel(&ch, 1) && configured == 1 && analog == 1);
	Check(adc.Read(0, &data) && data.Chan == 0 && data.Timestamp == 0 && data.Data > 1.64f && data.Data < 1.66f);
	Check(!adc.UpdateData());
	Check(adc.StartConversion() && adc.UpdateData());
	Check(adc.Read(&data, 1) == 1 && !adc.UpdateData());
	Check(adc.Resolution(9) == 10 && adc.Read(0, &data) && data.Data > 1.64f && data.Data < 1.66f);
	Check(adc.Resolution(1) == 6 && adc.Resolution(99) == 12);
	ch.PinP.PinNo = 18;
	Check(!adc.OpenChannel(&ch, 1));
	ch.PinP.PinNo = 6;
	ch.Type = ADC_CHAN_TYPE_DIFFERENTIAL;
	Check(!adc.OpenChannel(&ch, 1));
	ch.Type = ADC_CHAN_TYPE_SINGLE_ENDED;
	ch.Gain = 2;
	Check(!adc.OpenChannel(&ch, 1));
	ch.Gain = 0x100;
	Check(adc.Read(0, &data)); // Failed configuration preserved the channel
	AdcChanCfg_t second = ch;
	second.Chan = 3;
	second.PinP.PinNo = 5;
	Check(adc.OpenChannel(&second, 1) && adc.StartConversion());
	AdcData_t round[2];
	Check(adc.Read(round, 2) == 2 && round[0].Chan == 0 && round[1].Chan == 3 && !adc.UpdateData());
	adc.CloseChannel(3);
	io[0] = 1;
	Check(!adc.Read(0, &data) && !adc.UpdateData()); // Conversion timeout
	io[0] = 0;
	Check(adc.Read(0, &data)); // Next conversion recovers
	io[0] = 2;
	Check(!adc.Read(0, &data)); // Overrun
	io[0] = 0;
	Check(adc.Read(0, &data));
	io[0] = 6;
	Check(!adc.Read(0, &data) && !adc.Valid()); // Stop timeout
	adc.Disable();
	adc.PowerOff();
	Check(!adc.Valid()); // Reset a stuck converter
	io[0] = 0;
	Check(adc.Init(cfg) && adc.OpenChannel(&ch, 1));
	adc.Disable();
	Check(!adc.UpdateData() && adc.Enable());
	Check(adc.Calibrate() && adc.Read(0, &data));
	adc.CloseChannel(0);
	Check(!adc.Read(0, &data) && !adc.StartConversion());
	adc.PowerOff();
	Check(!adc.Valid());
	io[0] = 3;
	Check(!adc.Init(cfg) && !adc.Valid()); // Regulator timeout
	io[0] = 4;
	Check(!adc.Init(cfg) && !adc.Valid()); // Calibration timeout
	io[0] = 5;
	Check(!adc.Init(cfg) && !adc.Valid()); // Ready timeout
	io[0] = 0;
	Check(other.Init(cfg) && other.OpenChannel(&ch, 1) && other.Read(0, &data));
	other.PowerOff();
	io[2] = checks;
	while (1)
	{
	}
}
