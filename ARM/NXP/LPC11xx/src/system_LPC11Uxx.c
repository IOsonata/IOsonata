/*--------------------------------------------------------------------------
File   : system_LPC11Uxx.c

Author : Hoang Nguyen Hoan          Dec. 9, 2014

Desc   : Implementation of CMSIS Cortex-M0 Device for LPC11Uxx Device Series
		 Added SystemSetCoreClock for on the fly clock setting

Copyright (c) 2014, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/

#include <stdint.h>
#include <stdbool.h>

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "LPC11Uxx.h"


#define OSC_FREQ				XTAL_FREQ_12MHZ	// Installed oscillator frequency
#define IRC_FREQ				(12000000UL)	// Internal RC freq */

#define SYSTEM_CORE_CLOCK				48000000UL		// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(93UL)			// TODO: Adjustment value

#define LPC11_CLOCK_WAIT_MAX		100000U

// Select system clock oscillator source
#define DEF_CLK_SRC				IRC_FREQ

__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_XTAL, 12000000, 20},
	{OSC_TYPE_RC, 4000000, 20},
	true
};

uint32_t SystemCoreClock = IRC_FREQ;	// System Clock Frequency (Core Clock)
uint32_t SystemMainClkFreq = IRC_FREQ;		// System clock frequency, reset default IRC
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;

static uint32_t s_SysOscFreq = OSC_FREQ;

// Nominal watchdog analog frequencies, UM10462 Table 14. Index zero is undefined.
static const uint32_t s_WdtOscFreq[] = {
	0, 600000, 1050000, 1400000,
	1750000, 2100000, 2400000, 2700000,
	3000000, 3250000, 3500000, 3750000,
	4000000, 4200000, 4400000, 4600000,
};

static inline uint32_t GetSysPllClk(void)
{
	switch (LPC_SYSCON->SYSPLLCLKSEL & SYSPLLCLKSEL_MASK)
	{
	case SYSPLLCLKSEL_SYSOSC:
		return s_SysOscFreq;
	case SYSPLLCLKSEL_IRC:
		return IRC_FREQ;
	default:
		return 0;
	}
}

// Update clock frequencies from the selected source and AHB divider.
void SystemCoreClockUpdate(void)
{
	uint32_t sysclk = 0;
	uint32_t mainclk = LPC_SYSCON->MAINCLKSEL & MAINCLKSEL_MASK;
	uint32_t div = LPC_SYSCON->SYSAHBCLKDIV & 0xffU;

	switch (mainclk)
	{
	case MAINCLKSEL_PLLIN:
		sysclk = GetSysPllClk();
		break;
	case MAINCLKSEL_PLLOUT:
		// PLL output is FIN * M; P selects the internal CCO frequency.
		sysclk = GetSysPllClk() * ((LPC_SYSCON->SYSPLLCTRL & 0x1fU) + 1U);
		break;
	case MAINCLKSEL_WDT:
	{
		uint32_t ctrl = LPC_SYSCON->WDTOSCCTRL;
		uint32_t wdtdiv = 2U * ((ctrl & 0x1fU) + 1U);

		sysclk = s_WdtOscFreq[(ctrl >> 5) & 0xfU] / wdtdiv;
		break;
	}
	case MAINCLKSEL_IRC:
		sysclk = IRC_FREQ;
		break;
	}

	SystemMainClkFreq = sysclk;
	// Divider zero disables the system clock; do not retain a stale frequency.
	SystemCoreClock = div != 0 ? sysclk / div : 0;
}

// A register-read budget bounds startup even if an oscillator or PLL fails.
static bool Lpc11ClockReady(volatile const uint32_t *pReg)
{
	for (uint32_t count = 0; count < LPC11_CLOCK_WAIT_MAX; count++)
	{
		if (*pReg & 1U)
		{
			return true;
		}
	}
	return false;
}

static bool Lpc11ClockLatch(volatile uint32_t *pReg)
{
	*pReg = 0;
	*pReg = 1;

	return Lpc11ClockReady(pReg);
}

// Startup cannot use timers or delay calibration that depend on the old clock.
static void Lpc11ClockDelay(uint32_t Reads)
{
	for (uint32_t count = 0; count < Reads; count++)
	{
		(void)LPC_SYSCON->SYSAHBCLKDIV;
	}
}

static bool Lpc11MainClockIRC(void)
{
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_IRC;
	if (!Lpc11ClockLatch(&LPC_SYSCON->MAINCLKUEN))
	{
		SystemMainClkFreq = 0;
		SystemCoreClock = 0;
		return false;
	}
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	SystemMainClkFreq = IRC_FREQ;
	SystemCoreClock = IRC_FREQ;

	return true;
}

static uint32_t Lpc11SetCoreClock(OSC_TYPE Type, uint32_t ClkFreq, bool UsbClock)
{
	bool external = Type != OSC_TYPE_RC;
	uint32_t sysclk = IRC_FREQ;
	uint32_t pll = 0;
	uint32_t div = 0;
	uint32_t fclko = 0;

	if (external)
	{
		// ClkFreq describes the physical input, not a requested CPU rate.
		if (Type != OSC_TYPE_XTAL && Type != OSC_TYPE_TCXO)
		{
			return 0;
		}
		if (ClkFreq < 10000000 || ClkFreq > 25000000)
		{
			return 0;
		}
		sysclk = (uint32_t)ClkFreq;
	}

	// Plan both PLLs before touching the running clock. USB requires 48 MHz.
	for (uint32_t p = 0; p < 4 && div == 0; p++)
	{
		for (uint32_t m = 1; m <= 32; m++)
		{
			uint32_t output = m * sysclk;
			uint64_t cco = (uint64_t)output * (2U << p);

			if (cco < 156000000ULL || cco > 320000000ULL ||
				output % SYSTEM_CORE_CLOCK != 0)
			{
				continue;
			}
			fclko = output;
			div = output / SYSTEM_CORE_CLOCK;
			pll = (m - 1U) | (p << 5);
			break;
		}
	}
	if (div == 0)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();

	// Three flash clocks are required at 48 MHz. Preserve reserved bits.
	LPC_SYSCON->SYSAHBCLKCTRL |= 1U << 3;
	LPC_FLASHCTRL->FLASHCFG = (LPC_FLASHCTRL->FLASHCFG & ~3U) | 2U;
	LPC_SYSCON->PDRUNCFG &= ~(PDRUNCFG_IRC_PD | PDRUNCFG_IRCOUT_PD);
	Lpc11ClockDelay(1024U);
	if (!Lpc11MainClockIRC())
	{
		// Do not power down a PLL that may still be clocking the CPU.
		EnableInterrupt(state);
		return 0;
	}

	// Callers must stop USB and other clock-dependent peripherals first.
	LPC_SYSCON->USBCLKDIV = 0;
	LPC_SYSCON->PDRUNCFG |= PDRUNCFG_USBPAD_PD | PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD;
	if (external)
	{
		LPC_SYSCON->PDRUNCFG |= PDRUNCFG_SYSOSC_PD;
		LPC_SYSCON->SYSOSCCTRL = (LPC_SYSCON->SYSOSCCTRL & ~3U) |
			(sysclk > 20000000U ? SYSOSCCTRL_FREQRANGE15 : SYSOSCCTRL_FREQRANGE0) |
			(Type == OSC_TYPE_TCXO ? SYSOSCCTRL_BYPASS : 0U);
		LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSOSC_PD;
		// At least 1 ms at the 12 MHz IRC; PLL lock is checked separately.
		Lpc11ClockDelay(12000U);
		s_SysOscFreq = sysclk;
	}
	LPC_SYSCON->SYSPLLCLKSEL = external ? SYSPLLCLKSEL_SYSOSC : SYSPLLCLKSEL_IRC;
	if (!Lpc11ClockLatch(&LPC_SYSCON->SYSPLLCLKUEN))
	{
		goto clock_failed;
	}
	LPC_SYSCON->SYSPLLCTRL = pll;
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSPLL_PD;
	if (!Lpc11ClockReady(&LPC_SYSCON->SYSPLLSTAT))
	{
		goto clock_failed;
	}

	if (external && UsbClock)
	{
		LPC_SYSCON->USBPLLCLKSEL = USBPLLCLKSEL_SYSOSC;
		if (!Lpc11ClockLatch(&LPC_SYSCON->USBPLLCLKUEN))
		{
			goto clock_failed;
		}
		LPC_SYSCON->USBPLLCTRL = pll;
		LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_USBPLL_PD;
		if (!Lpc11ClockReady(&LPC_SYSCON->USBPLLSTAT))
		{
			goto clock_failed;
		}
	}

	// Install the divider before connecting the faster PLL output.
	LPC_SYSCON->SYSAHBCLKDIV = div;
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLOUT;
	if (!Lpc11ClockLatch(&LPC_SYSCON->MAINCLKUEN))
	{
		goto clock_failed;
	}
	if (external && UsbClock)
	{
		LPC_SYSCON->USBCLKSEL = USBCLKSEL_PLLOUT;
		if (!Lpc11ClockLatch(&LPC_SYSCON->USBCLKUEN))
		{
			goto clock_failed;
		}
		LPC_SYSCON->USBCLKDIV = div;
		LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_USBPAD_PD;
	}
	LPC_SYSCON->SYSAHBCLKCTRL |= 1U << 16;
	SystemMainClkFreq = fclko;
	SystemCoreClock = fclko / div;
	EnableInterrupt(state);

	return SystemCoreClock;

clock_failed:
	LPC_SYSCON->USBCLKDIV = 0;
	LPC_SYSCON->PDRUNCFG |= PDRUNCFG_USBPAD_PD;
	if (Lpc11MainClockIRC())
	{
		LPC_SYSCON->PDRUNCFG |= PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD;
	}
	else
	{
		// A failed clock-source latch leaves the active frequency unknown.
		SystemMainClkFreq = 0;
		SystemCoreClock = 0;
	}
	EnableInterrupt(state);

	return 0;
}

/**
 * Select the physical PLL input for the fixed 48 MHz CPU target.
 * USB follows g_McuOsc.bUSBClk and is disabled for IRC input.
 * Callers must stop clock-dependent peripherals before changing clocks.
 * Failure preserves the requested oscillator descriptor. PLL lock failure
 * leaves the CPU on IRC; SystemCoreClock reports that fallback frequency.
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (ClkSrc == OSC_TYPE_RC && OscFreq != IRC_FREQ)
	{
		return false;
	}
	if (Lpc11SetCoreClock(ClkSrc, OscFreq, g_McuOsc.bUSBClk) == 0)
	{
		return false;
	}
	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;

	return true;
}

uint32_t SystemCoreClockSet(bool Crystal, int ClkFreq)
{
	return Lpc11SetCoreClock(Crystal ? OSC_TYPE_XTAL : OSC_TYPE_RC,
		(uint32_t)ClkFreq, Crystal);
}

uint32_t SystemSetCoreClock(bool Crystal, int ClkFreq)
{
	return SystemCoreClockSet(Crystal, ClkFreq);
}

void SystemInit (void)
{
	SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq);
	SystemCoreClockUpdate();
}

/**
 * @brief Get peripheral bus clock frequency.
 *
 * The AHB/APB bus uses the divided system clock, shared with the CPU.
 * UART and SSP function-clock dividers are managed by their drivers.
 *
 * @param Idx : 0 for the system bus clock.
 * @return Frequency in Hz, or zero for an invalid index.
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 0 ? SystemCoreClock : 0;
}

/**
 * @brief Independent peripheral bus clock changes are not supported.
 *
 * Changing SYSAHBCLKDIV also changes the CPU clock. Do not silently change
 * it through a peripheral-only request.
 *
 * @return Zero; no clock registers are changed.
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	(void)Idx;
	(void)Freq;

	return 0;
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}


