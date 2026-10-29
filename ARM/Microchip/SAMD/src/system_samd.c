/**-------------------------------------------------------------------------
@file	system_samd.c

@brief	SAM D11/D21 system clock initialization

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <stdbool.h>
#include <stdint.h>
#include "samdxxx.h"

#include "coredev/system_core_clock.h"
#include "clock_samd.h"

#if !defined(__SAMD11D14AM__) && !defined(__SAMD21J18A__)
#error Unsupported SAM D device
#endif

#define SAMD_GCLK_STATUS		(*(volatile uint8_t *)0x40000C01UL)
#define SAMD_GCLK_CLKCTRL		(*(volatile uint16_t *)0x40000C02UL)
#define SAMD_GCLK_CLKCTRL_CLKEN	(1U << 14)
#define SAMD_GCLK_CLKCTRL_WRTLOCK	(1U << 15)
#define SAMD_GCLK_CLKCTRL_GEN(n)	((uint16_t)(n) << 8)
#define SAMD_GCLK_CLKCTRL_ID_MASK	0x3FU
#define SAMD_GCLK_GENCTRL		(*(volatile uint32_t *)0x40000C04UL)
#define SAMD_GCLK_STATUS_SYNCBUSY	(1U << 7)
#define SAMD_GCLK_GENCTRL_OSC8M	(0x06UL << 8)
#define SAMD_GCLK_GENCTRL_GENEN	(1UL << 16)
#define SAMD_GCLK_TIMEOUT		100000UL

#define SAMD_SYSCTRL_OSC8M		(*(volatile uint32_t *)0x40000820UL)
#define SAMD_OSC8M_PRESC_MASK	(3UL << 8)
#define SAMD_OSC8M_ENABLE		(1UL << 1)
#define SAMD_OSC8M_FREQ			8000000UL

// CPU clock after reset is OSC8M divided by eight.
uint32_t SystemCoreClock = 1000000UL;
uint32_t SystemnsDelayFactor = 1UL;

__attribute__((weak)) McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, 1000000UL, 0UL, 0UL },
	{ OSC_TYPE_RC, 32768UL, 0UL, 0UL },
	false
};

static bool SamdGclkWait(void)
{
	uint32_t timeout;

	timeout = SAMD_GCLK_TIMEOUT;
	while ((SAMD_GCLK_STATUS & SAMD_GCLK_STATUS_SYNCBUSY) != 0U)
	{
		if (--timeout == 0U)
		{
			return false;
		}
	}

	return true;
}

bool SamdGclkChannelEnable(uint8_t Channel, uint8_t Generator)
{
	uint16_t control;

	if (Channel > SAMD_GCLK_CLKCTRL_ID_MASK || Generator > 7U ||
		!SamdGclkWait())
	{
		return false;
	}

	// Select the channel without changing its configuration.
	*(volatile uint8_t *)0x40000C02UL = Channel;
	if (!SamdGclkWait())
	{
		return false;
	}

	control = SAMD_GCLK_CLKCTRL;
	if ((control & SAMD_GCLK_CLKCTRL_WRTLOCK) != 0U)
	{
		return (control & (SAMD_GCLK_CLKCTRL_CLKEN | (0xFU << 8))) ==
			(SAMD_GCLK_CLKCTRL_CLKEN | SAMD_GCLK_CLKCTRL_GEN(Generator));
	}

	SAMD_GCLK_CLKCTRL = Channel | SAMD_GCLK_CLKCTRL_GEN(Generator) |
		SAMD_GCLK_CLKCTRL_CLKEN;
	if (!SamdGclkWait())
	{
		return false;
	}

	control = SAMD_GCLK_CLKCTRL;
	return (control & (SAMD_GCLK_CLKCTRL_ID_MASK |
		SAMD_GCLK_CLKCTRL_CLKEN | (0xFU << 8))) ==
		(Channel | SAMD_GCLK_CLKCTRL_GEN(Generator) |
		SAMD_GCLK_CLKCTRL_CLKEN);
}

bool SamdGclkChannelDisable(uint8_t Channel)
{
	uint16_t control;

	if (Channel > SAMD_GCLK_CLKCTRL_ID_MASK || !SamdGclkWait())
	{
		return false;
	}

	*(volatile uint8_t *)0x40000C02UL = Channel;
	if (!SamdGclkWait())
	{
		return false;
	}

	control = SAMD_GCLK_CLKCTRL;
	if ((control & SAMD_GCLK_CLKCTRL_WRTLOCK) != 0U)
	{
		return (control & SAMD_GCLK_CLKCTRL_CLKEN) == 0U;
	}

	SAMD_GCLK_CLKCTRL = Channel | (control & (0xFU << 8));
	if (!SamdGclkWait())
	{
		return false;
	}

	return (SAMD_GCLK_CLKCTRL & SAMD_GCLK_CLKCTRL_CLKEN) == 0U;
}

void SystemInit(void)
{
	// The initial implementation retains the reset clock configuration.
	// External oscillators and USB clocks are not configured here.
}

void SystemCoreClockUpdate(void)
{
	uint32_t osc8m;
	uint32_t prescaler;
	uint32_t generator;

	osc8m = SAMD_SYSCTRL_OSC8M;
	if ((osc8m & SAMD_OSC8M_ENABLE) == 0UL)
	{
		return;
	}

	prescaler = (osc8m & SAMD_OSC8M_PRESC_MASK) >> 8;
	// Report OSC8M only when generator zero uses that source.
	generator = SAMD_GCLK_GENCTRL;
	if ((generator & 0x0FUL) != 0UL ||
		(generator & (0x1FUL << 8)) != SAMD_GCLK_GENCTRL_OSC8M ||
		(generator & SAMD_GCLK_GENCTRL_GENEN) == 0UL)
	{
		return;
	}

	SystemCoreClock = SAMD_OSC8M_FREQ >> prescaler;
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	(void)Idx;
	return 0UL;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	(void)Idx;
	(void)Freq;
	return 0UL;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t osc8m;

	if (ClkSrc != OSC_TYPE_RC || OscFreq != SAMD_OSC8M_FREQ)
	{
		return false;
	}

	// Keep GCLK0 on OSC8M while changing its prescaler.
	osc8m = SAMD_SYSCTRL_OSC8M;
	if ((osc8m & SAMD_OSC8M_ENABLE) == 0UL || !SamdGclkWait())
	{
		return false;
	}

	SAMD_SYSCTRL_OSC8M = osc8m & ~SAMD_OSC8M_PRESC_MASK;
	SAMD_GCLK_GENCTRL = SAMD_GCLK_GENCTRL_OSC8M |
		SAMD_GCLK_GENCTRL_GENEN;
	if (!SamdGclkWait())
	{
		return false;
	}

	SystemCoreClockUpdate();
	return SystemCoreClock == SAMD_OSC8M_FREQ;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	(void)ClkSrc;
	(void)OscFreq;
	return false;
}

void SystemOscInit(void)
{
}
