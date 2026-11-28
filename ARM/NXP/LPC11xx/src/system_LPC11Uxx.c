/*--------------------------------------------------------------------------
File   : system_LPC11Uxx.c

Author : Hoang Nguyen Hoan          Dec. 9, 2014

Desc   : Implementation of CMSIS Cortex-M0 Device for LPC11Uxx Device Series

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
#include "idelay.h"
#include "coredev/interrupt.h"
#include "LPC11Uxx.h"


#define IRC_FREQ				(12000000UL)	// Internal RC freq
#define IRC_ACCURACY			10000U			// ppm, UM10462 +/-1 percent

#define SYSTEM_CORE_CLOCK				48000000UL		// Fixed CPU and USB target
#define SYSTEM_NSDELAY_CORE_FACTOR		(93UL)			// TODO: Adjustment value

#define LPC11_CLOCK_WAIT_MAX		100000U
#define LPC11_WDTOSC_ACCURACY		400000U			// ppm, +/-40 percent
#define LPC11_FLASH_1CLK_MAX		20000000UL		// Flash access in 1 system clock
#define LPC11_FLASH_2CLK_MAX		40000000UL		// Flash access in 2 system clocks

__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_XTAL, 12000000, 20},
	{OSC_TYPE_RC, 0, 0},
	true
};

// Calibrated delay-loop iterations per second, defined in ResetEntry.c
extern uint32_t SystemMicroSecLoopCnt;

uint32_t SystemCoreClock = IRC_FREQ;	// System Clock Frequency (Core Clock)
uint32_t SystemMainClkFreq = IRC_FREQ;	// Main clock set by the clock setup, reset default IRC
uint32_t SystemnsDelayFactor = 1000000UL;

// Nominal watchdog analog frequencies, UM10462 Table 14. Index zero is undefined.
static const uint32_t s_WdtOscFreq[] = {
	0, 600000, 1050000, 1400000,
	1750000, 2100000, 2400000, 2700000,
	3000000, 3250000, 3500000, 3750000,
	4000000, 4200000, 4400000, 4600000,
};

static uint32_t s_HfUsers;
static uint32_t s_LfUsers;
static uint32_t s_LfFreq;				// Watchdog oscillator started by SystemInit
static uint32_t s_PerformanceDiv;		// SYSAHBCLKDIV set by SystemInit
static SYSTEM_POWER_MODE s_PowerMode;

// CPU clock from the main clock the clock setup set and the AHB divider
void SystemCoreClockUpdate(void)
{
	uint32_t div;

	div = LPC_SYSCON->SYSAHBCLKDIV & 0xffU;
	// Divider zero stops the system clock
	SystemCoreClock = div != 0U ? SystemMainClkFreq / div : 0U;
	(void)ArmDelayClockSetDivisors(SystemCoreClock,
		16000000UL, ((uint64_t)SYSTEM_NSDELAY_CORE_FACTOR * SYSTEM_CORE_CLOCK / 1000UL));
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

// Main clock on the IRC, AHB divider 1. Interrupts are disabled.
static bool Lpc11MainClockIRC(void)
{
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_IRC;
	if (!Lpc11ClockLatch(&LPC_SYSCON->MAINCLKUEN))
	{
		// The main clock source is not known
		SystemMainClkFreq = 0;
		SystemCoreClockUpdate();
		return false;
	}
	LPC_SYSCON->SYSAHBCLKDIV = 1;
	SystemMainClkFreq = IRC_FREQ;
	SystemCoreClockUpdate();

	return true;
}

// The caller has selected IRC and holds the interrupt lock.
static bool Lpc11ConnectPll(OSC_TYPE Type, uint32_t sysclk, uint32_t pll,
		uint32_t div, bool UsbClock)
{
	bool external = Type != OSC_TYPE_RC;

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
	}
	LPC_SYSCON->SYSPLLCLKSEL = external ? SYSPLLCLKSEL_SYSOSC : SYSPLLCLKSEL_IRC;
	if (!Lpc11ClockLatch(&LPC_SYSCON->SYSPLLCLKUEN))
	{
		return false;
	}
	LPC_SYSCON->SYSPLLCTRL = pll;
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_SYSPLL_PD;
	if (!Lpc11ClockReady(&LPC_SYSCON->SYSPLLSTAT))
	{
		return false;
	}

	if (external && UsbClock)
	{
		LPC_SYSCON->USBPLLCLKSEL = USBPLLCLKSEL_SYSOSC;
		if (!Lpc11ClockLatch(&LPC_SYSCON->USBPLLCLKUEN))
		{
			return false;
		}
		LPC_SYSCON->USBPLLCTRL = pll;
		LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_USBPLL_PD;
		if (!Lpc11ClockReady(&LPC_SYSCON->USBPLLSTAT))
		{
			return false;
		}
	}

	// Install the divider before connecting the faster PLL output.
	LPC_SYSCON->SYSAHBCLKDIV = div;
	LPC_SYSCON->MAINCLKSEL = MAINCLKSEL_PLLOUT;
	if (!Lpc11ClockLatch(&LPC_SYSCON->MAINCLKUEN))
	{
		return false;
	}
	if (external && UsbClock)
	{
		LPC_SYSCON->USBCLKSEL = USBCLKSEL_PLLOUT;
		if (!Lpc11ClockLatch(&LPC_SYSCON->USBCLKUEN))
		{
			return false;
		}
		LPC_SYSCON->USBCLKDIV = div;
		LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_USBPAD_PD;
	}
	LPC_SYSCON->SYSAHBCLKCTRL |= 1U << 16;
	return true;
}

// The 48 MHz CPU clock from the PLL. ClkFreq is the crystal or external clock
// input, the IRC is 12 MHz. Returns the CPU frequency, 0 when the input cannot
// give 48 MHz or the PLL does not lock, the CPU is then on the 12 MHz IRC.
static uint32_t Lpc11SetCoreClock(OSC_TYPE Type, uint32_t ClkFreq, bool UsbClock)
{
	bool external;
	uint32_t sysclk;
	uint32_t pll;
	uint32_t div;
	uint32_t fclko;
	uint32_t state;

	external = Type != OSC_TYPE_RC;
	sysclk = IRC_FREQ;
	pll = 0;
	div = 0;
	fclko = 0;
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

	state = DisableInterrupt();

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

	if (Lpc11ConnectPll(Type, sysclk, pll, div, UsbClock))
	{
		SystemMainClkFreq = fclko;
		SystemCoreClockUpdate();
		EnableInterrupt(state);
		return SystemCoreClock;
	}

	LPC_SYSCON->USBCLKDIV = 0;
	LPC_SYSCON->PDRUNCFG |= PDRUNCFG_USBPAD_PD;
	if (Lpc11MainClockIRC())
	{
		LPC_SYSCON->PDRUNCFG |= PDRUNCFG_SYSPLL_PD | PDRUNCFG_USBPLL_PD;
	}
	EnableInterrupt(state);

	return 0;
}

// CPU clock from g_McuOsc.CoreOsc. A missing crystal or a PLL that does not
// lock gives 48 MHz from the IRC, without USB, and the descriptor then
// describes the IRC.
static void Lpc11CoreClockInit(void)
{
	if (g_McuOsc.CoreOsc.Type != OSC_TYPE_RC &&
		Lpc11SetCoreClock(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq, g_McuOsc.bUSBClk) != 0)
	{
		return;
	}
	if (Lpc11SetCoreClock(OSC_TYPE_RC, IRC_FREQ, false) == 0)
	{
		return;
	}
	if (g_McuOsc.CoreOsc.Type != OSC_TYPE_RC)
	{
		// External accuracy and crystal load do not describe the IRC.
		// Application calibration of an RC descriptor is kept.
		g_McuOsc.CoreOsc.Accuracy = IRC_ACCURACY;
		g_McuOsc.CoreOsc.LoadCap = 0;
	}
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	g_McuOsc.CoreOsc.Freq = IRC_FREQ;
}

// Watchdog oscillator from g_McuOsc.LowPwrOsc, the closest nominal rate to
// its frequency. It stays powered down with a frequency of 0, the default.
// The watchdog is neither enabled nor fed.
static void Lpc11LowFreqInit(void)
{
	uint32_t best;
	uint32_t value;
	uint32_t error;
	uint32_t rate;
	uint32_t diff;

	s_LfFreq = 0;
	if (g_McuOsc.LowPwrOsc.Type != OSC_TYPE_RC || g_McuOsc.LowPwrOsc.Freq == 0)
	{
		return;
	}
	best = 0;
	value = 0;
	error = UINT32_MAX;
	for (uint32_t sel = 1; sel < 16; sel++)
	{
		for (uint32_t div = 0; div < 32; div++)
		{
			rate = s_WdtOscFreq[sel] / (2U * (div + 1U));
			diff = rate > g_McuOsc.LowPwrOsc.Freq ? rate - g_McuOsc.LowPwrOsc.Freq :
				g_McuOsc.LowPwrOsc.Freq - rate;
			if (diff < error)
			{
				error = diff;
				best = rate;
				value = (sel << 5) | div;
			}
		}
	}
	LPC_SYSCON->WDTOSCCTRL = value;
	LPC_SYSCON->PDRUNCFG &= ~PDRUNCFG_WDTOSC_PD;
	g_McuOsc.LowPwrOsc.Freq = best;
	g_McuOsc.LowPwrOsc.Accuracy = LPC11_WDTOSC_ACCURACY;
	s_LfFreq = best;
}

void SystemInit(void)
{
	Lpc11CoreClockInit();
	Lpc11LowFreqInit();
	s_PerformanceDiv = LPC_SYSCON->SYSAHBCLKDIV & 0xffU;
	s_PowerMode = SYSTEM_POWER_MODE_PERFORMANCE;
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
 * @brief Peripheral bus clock frequency in effect.
 *
 * The bus clock is the CPU clock, SYSAHBCLKDIV divides both. A peripheral
 * request does not change it.
 *
 * @return Frequency in Hz of the bus clock, 0 for an invalid index.
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	(void)Freq;

	return SystemPeriphClockGet(Idx);
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

bool SystemHighFreqClockEnable(void)
{
	uint32_t state;
	bool ready;

	state = DisableInterrupt();
	ready = s_HfUsers != UINT32_MAX && SystemMainClkFreq != 0U;
	if (ready)
	{
		s_HfUsers++;
	}
	EnableInterrupt(state);
	return ready;
}

void SystemHighFreqClockDisable(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_HfUsers != 0U)
	{
		s_HfUsers--;
	}
	EnableInterrupt(state);
}

uint32_t SystemHighFreqClockStartTime(void)
{
	// The oscillator is started in SystemInit, Enable never waits for it
	return 0U;
}

// The watchdog oscillator, when g_McuOsc.LowPwrOsc gave SystemInit a frequency
bool SystemLowFreqClockEnable(void)
{
	uint32_t state;
	bool ready;

	state = DisableInterrupt();
	ready = s_LfUsers != UINT32_MAX && s_LfFreq != 0U;
	if (ready)
	{
		s_LfUsers++;
	}
	EnableInterrupt(state);
	return ready;
}

void SystemLowFreqClockDisable(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_LfUsers != 0U)
	{
		s_LfUsers--;
	}
	EnableInterrupt(state);
}

uint32_t SystemLowFreqClockStartTime(void)
{
	// The oscillator is started in SystemInit, Enable never waits for it
	return 0U;
}

// Flash access time for a system clock frequency. FLASHCFG is in the flash
// controller, its clock was enabled by the clock setup.
static void Lpc11FlashTime(uint32_t Freq)
{
	uint32_t tim;

	tim = Freq <= LPC11_FLASH_1CLK_MAX ? 0U : (Freq <= LPC11_FLASH_2CLK_MAX ? 1U : 2U);
	LPC_FLASHCTRL->FLASHCFG = (LPC_FLASHCTRL->FLASHCFG & ~3U) | tim;
}

/**
 * @brief	CPU running mode.
 *
 * LOW_POWER doubles SYSAHBCLKDIV: the CPU, AHB and APB clocks, and the timers
 * and the SysTick on them, run at half the frequency. The UART, SSP and USB
 * clocks come from the main clock through their own dividers and do not
 * change. The flash access time follows the system clock.
 */
bool SystemPowerModeSet(SYSTEM_POWER_MODE Mode)
{
	uint32_t state;
	uint32_t div;
	uint32_t freq;

	if (__get_IPSR() != 0U ||
		(Mode != SYSTEM_POWER_MODE_PERFORMANCE && Mode != SYSTEM_POWER_MODE_LOW_POWER) ||
		s_PerformanceDiv == 0U || s_PerformanceDiv > 0x7fU)
	{
		return false;
	}
	div = Mode == SYSTEM_POWER_MODE_LOW_POWER ? s_PerformanceDiv << 1 : s_PerformanceDiv;
	freq = SystemMainClkFreq / div;
	state = DisableInterrupt();
	if (Mode != s_PowerMode)
	{
		// The longer flash access before the faster clock, after the slower one
		if (Mode == SYSTEM_POWER_MODE_PERFORMANCE)
		{
			Lpc11FlashTime(freq);
		}
		LPC_SYSCON->SYSAHBCLKDIV = div;
		if (Mode == SYSTEM_POWER_MODE_LOW_POWER)
		{
			Lpc11FlashTime(freq);
		}
		SystemCoreClockUpdate();
		s_PowerMode = Mode;
	}
	EnableInterrupt(state);
	return true;
}

SYSTEM_POWER_MODE SystemPowerModeGet(void)
{
	return s_PowerMode;
}
