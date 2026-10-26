/**-------------------------------------------------------------------------
@file	system_ra4m1.c

@brief	Native IOsonata RA4M1 system startup. No FSP or board initialization.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include "ra4m1xxx.h"
#include "coredev/interrupt.h"
#define RA4M1_PRCR_KEY      0xA500U
#define RA4M1_PRCR_PRC0     0x01U
#define RA4M1_PRCR_PRC1     0x02U
#define RA4M1_OSCSF_HOCO    0x01U
#define RA4M1_OSCSF_MOSC    0x08U
#define RA4M1_OSCSF_PLL     0x20U
#define RA4M1_MODE_BUSY     0x10U
#define RA4M1_CK_HOCO       0U
#define RA4M1_CK_MOCO       1U
#define RA4M1_CK_LOCO       2U
#define RA4M1_CK_MOSC       3U
#define RA4M1_CK_SOSC       4U
#define RA4M1_CK_PLL        5U
#define RA4M1_MOCO_HZ       8000000UL
#define RA4M1_LOCO_HZ       32768UL
#define RA4M1_RESET_DIV     0x44044444UL
#define RA4M1_FAST_DIV      0x10010100UL
#define RA4M1_SLOW_DIV      0x00000000UL

#if RA4M1_MOSC_WAIT > 9U || RA4M1_SOSC_DRIVE > 3U
#error "Invalid RA4M1 oscillator control setting"
#endif
#if RA4M1_SOSC_STARTUP_US == 0 || RA4M1_SOSC_STARTUP_US > 60000000UL
#error "RA4M1 SOSC startup delay must be 1..60000000 us"
#endif
#if RA4M1_STARTUP_TIMEOUT == 0 || RA4M1_NSDELAY_FACTOR == 0
#error "RA4M1 startup timeout and delay factor must be nonzero"
#endif

#define RA4M1_USB_SCKE       0x0400U

// Reset is MOCO /16, not HOCO. ResetEntry initializes this storage before
// calling SystemInit, including an application's strong g_McuOsc override.
uint32_t SystemCoreClock = 500000UL;
uint32_t SystemnsDelayFactor = RA4M1_NSDELAY_FACTOR;
volatile uint32_t g_Ra4m1StartupError;
extern uint32_t SystemMicroSecLoopCnt;
extern void (* const __Vectors[])(void);

__WEAK McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, 48000000UL, 0U, 0U },
	{ OSC_TYPE_RC, 32768UL, 0U, 0U },
	false
};

// Only an external oscillator's frequency is not recoverable from registers.
// Keep its configured nominal frequency independently of the selected source.
static uint32_t s_MainOscFreq;

typedef struct __Ra4m1_Clock_Plan {
	uint32_t SourceHz;
	uint32_t Dividers;
	uint8_t Source;
	uint8_t Hoco;
	uint8_t Pll;
	uint8_t FlashWait;
} Ra4m1ClockPlan_t;

static bool Ra4m1Wait8(volatile const uint8_t *Register, uint8_t Mask, uint8_t Value)
{
	uint32_t n = RA4M1_STARTUP_TIMEOUT;
	do {
		if (((*Register) & Mask) == Value)
			return true;
	} while (--n != 0U);
	return false;
}

static bool Ra4m1Wait16(volatile const uint16_t *Register, uint16_t Mask, uint16_t Value)
{
	uint32_t n = RA4M1_STARTUP_TIMEOUT;
	do {
		if (((*Register) & Mask) == Value)
			return true;
	} while (--n != 0U);
	return false;
}

// Conservative minimum wait: each iteration includes a volatile decrement
// and a NOP. No dependency on SysTick, interrupts, calibrated nsDelay, or FPU.
// This is deliberately a lower-bound delay, not a microsecond time service.
// Only called with ICLK <= 48 MHz (or reset MOCO /16).
static void Ra4m1DelayUs(uint32_t Us)
{
#ifdef RA4M1_TEST_DELAY_US
	RA4M1_TEST_DELAY_US(Us);
#else
	while (Us-- != 0U)
	{
		volatile uint32_t n = 64U;
		do { __NOP(); } while (--n != 0U);
	}
#endif
}

static uint32_t Ra4m1HocoHz(uint8_t Control)
{
	switch ((Control >> 3U) & 7U)
	{
		case 0U: return 24000000UL;
		case 2U: return 32000000UL;
		case 4U: return 48000000UL;
		case 5U: return 64000000UL;
		default: return 0U;
	}
}

static uint32_t Ra4m1PllHz(uint32_t Input, uint8_t Control)
{
	const uint32_t factor = (Control & 31U) + 1U;
	const uint32_t divCode = Control >> 6U;
	if (Input < 4000000UL || Input > 12500000UL ||
		factor < 8U || factor > 31U || (Control & 0x20U) != 0U ||
		(divCode != 1U && divCode != 2U))
		return 0U;
	const uint32_t freq = (Input * factor) >> divCode;
	return freq >= 24000000UL && freq <= 64000000UL ? freq : 0U;
}

// Prefer /2, then /4; all arithmetic stays within uint32_t. RA4M1 has an
// output divider in PLLCCR2, NOT the RE01 PLL input-divider arrangement.
static bool Ra4m1Pll48(uint32_t Input, uint8_t *pControl)
{
	if (Input < 4000000UL || Input > 12500000UL)
		return false;
	for (uint32_t divCode = 1U; divCode <= 2U; ++divCode)
	{
		const uint32_t numerator = 48000000UL << divCode;
		if (numerator % Input != 0U)
			continue;
		const uint32_t factor = numerator / Input;
		if (factor >= 8U && factor <= 31U)
		{
			*pControl = (uint8_t)((divCode << 6U) | (factor - 1U));
			return true;
		}
	}
	return false;
}

static bool Ra4m1MakePlan(OSC_TYPE Type, uint32_t Freq, bool Usb,
						 Ra4m1ClockPlan_t *pPlan)
{
	Ra4m1ClockPlan_t plan = {0};
	plan.SourceHz = Freq;
	if (Type == OSC_TYPE_RC)
	{
		switch (Freq)
		{
			case 8000000UL:  plan.Source = RA4M1_CK_MOCO; break;
			case 24000000UL: plan.Source = RA4M1_CK_HOCO; plan.Hoco = 0U; break;
			case 32000000UL: plan.Source = RA4M1_CK_HOCO; plan.Hoco = 0x10U; break;
			case 48000000UL: plan.Source = RA4M1_CK_HOCO; plan.Hoco = 0x20U; break;
			default: return false;
		}
	}
	else if (Type == OSC_TYPE_XTAL || Type == OSC_TYPE_TCXO)
	{
		if (Freq < 1000000UL || Freq > 20000000UL)
			return false;
		plan.Source = RA4M1_CK_MOSC;
		if (Ra4m1Pll48(Freq, &plan.Pll))
		{
			plan.Source = RA4M1_CK_PLL;
			plan.SourceHz = 48000000UL;
		}
	}
	else
		return false;

	if (Usb && plan.SourceHz != 48000000UL)
		return false;
	// Fast plan: ICLK/PCLKA/C/D = 48 MHz; PCLKB/FCLK = 24 MHz.
	// Bits [18:16], although reserved, must mirror PCKB[2:0].
	// At <=32 MHz all clock domains can use the undivided source.
	plan.Dividers = plan.SourceHz > 32000000UL ? RA4M1_FAST_DIV : RA4M1_SLOW_DIV;
	plan.FlashWait = plan.SourceHz > 32000000UL ? 1U : 0U;
	*pPlan = plan;
	return true;
}

static bool Ra4m1ValidLf(OSC_TYPE Type, uint32_t Freq)
{
	return (Type == OSC_TYPE_RC || Type == OSC_TYPE_XTAL) && Freq == 32768UL;
}

static bool Ra4m1SetDividers(uint32_t Value)
{
	R_SYSTEM->SCKDIVCR =  Value;
	__DSB();
	const bool ok = R_SYSTEM->SCKDIVCR == Value;
	__ISB();
	return ok;
}

static bool Ra4m1SetSource(uint8_t Source)
{
	R_SYSTEM->SCKSCR =  Source;
	__DSB();
	const bool ok = Ra4m1Wait8(&R_SYSTEM->SCKSCR, 7U, Source);
	__ISB();
	return ok;
}

static bool Ra4m1StartHoco(uint8_t Config)
{
	if ((R_SYSTEM->HOCOCR & 1U) == 0U)
	{
		if (!Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_HOCO, RA4M1_OSCSF_HOCO))
			return false;
		if (R_SYSTEM->HOCOCR2 == Config)
		{
			// RA4M1 changes HOCOWTCR only at 64 MHz.
			return true;
		}
		R_SYSTEM->HOCOCR =  1U;
	}
	if (!Ra4m1Wait8(&R_SYSTEM->HOCOCR, 1U, 1U) ||
		!Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_HOCO, 0U))
		return false;
	R_SYSTEM->HOCOCR2 =  Config;
	// RA4M1 FSP only programs HOCOWTCR for 64 MHz, to 6.
	if (Config == 0x28U)
	{
		R_SYSTEM->HOCOWTCR =  6U;
	}
	if (R_SYSTEM->HOCOCR2 != Config)
		return false;
	Ra4m1DelayUs(2U);
	R_SYSTEM->HOCOCR =  0U;
	return Ra4m1Wait8(&R_SYSTEM->HOCOCR, 1U, 0U) &&
		Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_HOCO, RA4M1_OSCSF_HOCO);
}

static bool Ra4m1StartMain(OSC_TYPE Type, uint32_t Freq)
{
	const uint8_t config = Type == OSC_TYPE_TCXO ? 0x40U :
						  (Freq <= 10000000UL ? 0x08U : 0U);
	const uint8_t wait = Type == OSC_TYPE_TCXO ? 0U : RA4M1_MOSC_WAIT;
	// CPU is on MOCO. Stop PLL before changing its input oscillator.
	R_SYSTEM->PLLCR =  1U;
	if (!Ra4m1Wait8(&R_SYSTEM->PLLCR, 1U, 1U) ||
		!Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_PLL, 0U))
		return false;
	R_SYSTEM->MOSCCR =  1U;
	if (!Ra4m1Wait8(&R_SYSTEM->MOSCCR, 1U, 1U) ||
		!Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_MOSC, 0U))
		return false;
	R_SYSTEM->MOMCR =  config;
	R_SYSTEM->MOSCWTCR =  wait;
	if (R_SYSTEM->MOMCR != config ||
		R_SYSTEM->MOSCWTCR != wait)
		return false;
	s_MainOscFreq = Freq;
	R_SYSTEM->MOSCCR =  0U;
	return Ra4m1Wait8(&R_SYSTEM->MOSCCR, 1U, 0U) &&
		Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_MOSC, RA4M1_OSCSF_MOSC);
}

// Caller has interrupts masked, peripherals/DMA quiesced, and USB SCKE=0.
// Low-frequency system-clock / Subosc-mode transitions are not part of this
// initial port. Refuse them rather than changing their backup-clock policy.
static Ra4m1StartupError_t Ra4m1ApplyPlan(const Ra4m1ClockPlan_t *pPlan,
									   OSC_TYPE Type, uint32_t Freq, bool Usb)
{
	const uint8_t source = R_SYSTEM->SCKSCR & 7U;
	if (source == RA4M1_CK_LOCO || source == RA4M1_CK_SOSC || source > RA4M1_CK_PLL ||
		(R_SYSTEM->SOPCCR & (1U | RA4M1_MODE_BUSY)) != 0U ||
		R_SYSTEM->OSTDCR != 0U ||
		(R_SYSTEM->OSTDSR & 1U) != 0U)
		return RA4M1_STARTUP_BAD_ENTRY;
	// USBFS is still in module-stop at reset. Do not access its registers
	// during system clock configuration: SCKE is managed by UsbCtrlrStart.
	if (!Ra4m1Wait8(&R_SYSTEM->OPCCR, RA4M1_MODE_BUSY, 0U))
		return RA4M1_STARTUP_POWER_MODE;

	// In low-voltage mode HOCO must remain running. The emitted option word
	// ensures this from reset; tolerate an existing image with HOCO stopped
	// only by enabling its valid configured oscillator before changing mode.
	if ((R_SYSTEM->OPCCR & 3U) == 2U &&
		(R_SYSTEM->HOCOCR & 1U) != 0U)
	{
		if (Ra4m1HocoHz(R_SYSTEM->HOCOCR2) == 0U)
			return RA4M1_STARTUP_HOCO;
		R_SYSTEM->HOCOCR =  0U;
	}
	if ((R_SYSTEM->HOCOCR & 1U) == 0U &&
		!Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_HOCO, RA4M1_OSCSF_HOCO))
		return RA4M1_STARTUP_HOCO;

	R_FCACHE->FCACHEE =  0U;
	if (!Ra4m1Wait16(&R_FCACHE->FCACHEE, 1U, 0U))
		return RA4M1_STARTUP_FLASH;
	// Down-divide the current source before selecting MOCO: no transient
	// overclock even when the previous CPU source was an external PLL.
	if (!Ra4m1SetDividers(RA4M1_RESET_DIV))
		return RA4M1_STARTUP_CLOCK;
	R_SYSTEM->MOCOCR =  0U;
	if (!Ra4m1Wait8(&R_SYSTEM->MOCOCR, 1U, 0U))
		return RA4M1_STARTUP_MOCO;
	Ra4m1DelayUs(2U);
	if (!Ra4m1SetSource(RA4M1_CK_MOCO))
		return RA4M1_STARTUP_CLOCK;

	R_SYSTEM->OPCCR =  0U;
	if (!Ra4m1Wait8(&R_SYSTEM->OPCCR, 0x13U, 0U))
		return RA4M1_STARTUP_POWER_MODE;
	// CPU is 500 kHz, cache disabled, power mode is now High-speed.
	R_SYSTEM->MEMWAIT =  pPlan->FlashWait;
	if (R_SYSTEM->MEMWAIT != pPlan->FlashWait)
		return RA4M1_STARTUP_FLASH;

	if (pPlan->Source == RA4M1_CK_HOCO)
	{
		if (!Ra4m1StartHoco(pPlan->Hoco))
			return RA4M1_STARTUP_HOCO;
	}
	else if (pPlan->Source == RA4M1_CK_MOSC || pPlan->Source == RA4M1_CK_PLL)
	{
		if (!Ra4m1StartMain(Type, Freq))
			return RA4M1_STARTUP_MOSC;
		if (pPlan->Source == RA4M1_CK_PLL)
		{
			R_SYSTEM->PLLCCR2 =  pPlan->Pll;
			if (R_SYSTEM->PLLCCR2 != pPlan->Pll)
				return RA4M1_STARTUP_PLL;
			Ra4m1DelayUs(2U); // PLLMUL-to-PLL-start minimum is 1 us.
			R_SYSTEM->PLLCR =  0U;
			if (!Ra4m1Wait8(&R_SYSTEM->PLLCR, 1U, 0U) ||
				!Ra4m1Wait8(&R_SYSTEM->OSCSF, RA4M1_OSCSF_PLL, RA4M1_OSCSF_PLL))
				return RA4M1_STARTUP_PLL;
		}
	}

	if (Usb)
	{
		const uint8_t usbSource = pPlan->Source == RA4M1_CK_HOCO ? 1U : 0U;
		if (usbSource != 0U && R_SYSTEM->HOCOUTCR != 0U)
			return RA4M1_STARTUP_USB_CLOCK;
		R_SYSTEM->USBCKCR =  usbSource;
		if (R_SYSTEM->USBCKCR != usbSource)
			return RA4M1_STARTUP_USB_CLOCK;
	}
	if (!Ra4m1SetDividers(pPlan->Dividers) || !Ra4m1SetSource(pPlan->Source))
		return RA4M1_STARTUP_CLOCK;

	R_FCACHE->FCACHEIV =  1U;
	if (!Ra4m1Wait16(&R_FCACHE->FCACHEIV, 1U, 0U))
		return RA4M1_STARTUP_FLASH;
	R_FCACHE->FCACHEE =  1U;
	return Ra4m1Wait16(&R_FCACHE->FCACHEE, 1U, 1U) ?
		RA4M1_STARTUP_OK : RA4M1_STARTUP_FLASH;
}

static uint32_t Ra4m1SourceHz(uint8_t Source)
{
	const uint8_t ready = R_SYSTEM->OSCSF;
	if ((R_SYSTEM->OSTDSR & 1U) != 0U)
	{
		// OSTDF redirects MOSC to MOCO without a software selector write.
		// A failed PLL input leaves the PLL free-running: its rate is unknown.
		if (Source == RA4M1_CK_MOSC)
			return RA4M1_MOCO_HZ;
		if (Source == RA4M1_CK_PLL)
			return 0U;
	}
	switch (Source)
	{
		case RA4M1_CK_HOCO:
			return (R_SYSTEM->HOCOCR & 1U) == 0U &&
				(ready & RA4M1_OSCSF_HOCO) != 0U ?
				Ra4m1HocoHz(R_SYSTEM->HOCOCR2) : 0U;
		case RA4M1_CK_MOCO:
			return (R_SYSTEM->MOCOCR & 1U) == 0U ? RA4M1_MOCO_HZ : 0U;
		case RA4M1_CK_LOCO:
			return (R_SYSTEM->LOCOCR & 1U) == 0U ? RA4M1_LOCO_HZ : 0U;
		case RA4M1_CK_MOSC:
			return (R_SYSTEM->MOSCCR & 1U) == 0U &&
				(ready & RA4M1_OSCSF_MOSC) != 0U ? s_MainOscFreq : 0U;
		case RA4M1_CK_SOSC:
			return (R_SYSTEM->SOSCCR & 1U) == 0U ? 32768UL : 0U;
		case RA4M1_CK_PLL:
			if ((R_SYSTEM->PLLCR & 1U) != 0U ||
				(ready & (RA4M1_OSCSF_PLL | RA4M1_OSCSF_MOSC)) !=
						 (RA4M1_OSCSF_PLL | RA4M1_OSCSF_MOSC))
				return 0U;
			return Ra4m1PllHz(s_MainOscFreq, R_SYSTEM->PLLCCR2);
		default: return 0U;
	}
}

static uint32_t Ra4m1DividedClock(unsigned Shift)
{
	const uint32_t state = DisableInterrupt();
	const uint8_t source = R_SYSTEM->SCKSCR & 7U;
	const uint32_t div = (R_SYSTEM->SCKDIVCR >> Shift) & 7U;
	const uint32_t freq = Ra4m1SourceHz(source);
	EnableInterrupt(state);
	return div <= 6U ? freq >> div : 0U;
}

void SystemCoreClockUpdate(void)
{
	// Read/report only: never change flash timing in a clock query.
	SystemCoreClock = Ra4m1DividedClock(24U);
}

uint32_t SystemCoreClockGet(void)
{
	return Ra4m1DividedClock(24U);
}

uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx >= 0 && Idx < 4 ? Ra4m1DividedClock((unsigned)(3 - Idx) * 4U) : 0U;
}

uint32_t SystemFlashClockGet(void)
{
	return Ra4m1DividedClock(28U);
}

uint32_t SystemUsbClockGet(void)
{
	const uint32_t state = DisableInterrupt();
	uint32_t hz = 0U;
	if (g_McuOsc.bUSBClk)
	{
		const bool hoco = (R_SYSTEM->USBCKCR & 1U) != 0U;
		hz = Ra4m1SourceHz(hoco ? RA4M1_CK_HOCO : RA4M1_CK_PLL);
		if (hoco && R_SYSTEM->HOCOUTCR != 0U)
			hz = 0U;
	}
	EnableInterrupt(state);
	return hz == 48000000UL ? hz : 0U;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	Ra4m1ClockPlan_t plan;
	if (!Ra4m1MakePlan(ClkSrc, Freq, g_McuOsc.bUSBClk, &plan))
	{
		g_Ra4m1StartupError = RA4M1_STARTUP_BAD_OSC;
		return false;
	}
	const uint32_t state = DisableInterrupt();
	const uint16_t protect = R_SYSTEM->PRCR & 0x0BU;
	R_SYSTEM->PRCR =  RA4M1_PRCR_KEY | protect | 3U;
	Ra4m1StartupError_t result = RA4M1_STARTUP_PROTECT;
	if ((R_SYSTEM->PRCR & 3U) == 3U)
		result = Ra4m1ApplyPlan(&plan, ClkSrc, Freq, g_McuOsc.bUSBClk);
	R_SYSTEM->PRCR =  RA4M1_PRCR_KEY | protect;
	if ((R_SYSTEM->PRCR & 0x0BU) != protect)
		result = RA4M1_STARTUP_PROTECT;
	if (result == RA4M1_STARTUP_OK)
	{
		g_McuOsc.CoreOsc.Type = ClkSrc;
		g_McuOsc.CoreOsc.Freq = Freq;
	}
	SystemCoreClockUpdate();
	SystemMicroSecLoopCnt = (SystemCoreClock + 8000000UL) / 16000000UL;
	g_Ra4m1StartupError = result;
	EnableInterrupt(state);
	return result == RA4M1_STARTUP_OK;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	if (!Ra4m1ValidLf(ClkSrc, Freq))
	{
		g_Ra4m1StartupError = RA4M1_STARTUP_BAD_LF_OSC;
		return false;
	}
	const uint32_t state = DisableInterrupt();
	const uint16_t protect = R_SYSTEM->PRCR & 0x0BU;
	R_SYSTEM->PRCR =  RA4M1_PRCR_KEY | protect | RA4M1_PRCR_PRC0;
	bool ok = (R_SYSTEM->PRCR & RA4M1_PRCR_PRC0) != 0U;
	if (ok && ClkSrc == OSC_TYPE_RC)
	{
		if ((R_SYSTEM->LOCOCR & 1U) != 0U)
		{
			R_SYSTEM->LOCOCR =  0U;
			ok = Ra4m1Wait8(&R_SYSTEM->LOCOCR, 1U, 0U);
		}
		if (ok)
			Ra4m1DelayUs(100U); // LOCO has no oscillator-ready flag.
	}
	else if (ok)
	{
		// Preserve a running sub-clock and the retained RTC. Its consumers
		// choose LOCO/SOSC themselves; SystemInit does not reset the RTC.
		if ((R_SYSTEM->SOSCCR & 1U) != 0U)
		{
			R_SYSTEM->SOMCR =  RA4M1_SOSC_DRIVE;
			ok = R_SYSTEM->SOMCR == RA4M1_SOSC_DRIVE;
			if (ok)
			{
				R_SYSTEM->SOSCCR =  0U;
				ok = Ra4m1Wait8(&R_SYSTEM->SOSCCR, 1U, 0U);
			}
		}
		if (ok)
			Ra4m1DelayUs(RA4M1_SOSC_STARTUP_US);
	}
	R_SYSTEM->PRCR =  RA4M1_PRCR_KEY | protect;
	ok = ok && (R_SYSTEM->PRCR & 0x0BU) == protect;
	if (ok)
	{
		g_McuOsc.LowPwrOsc.Type = ClkSrc;
		g_McuOsc.LowPwrOsc.Freq = Freq;
	}
	g_Ra4m1StartupError = ok ? RA4M1_STARTUP_OK : RA4M1_STARTUP_LF_CLOCK;
	EnableInterrupt(state);
	return ok;
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	// No independent runtime bus-divider policy in this initial port. Return
	// failure for a change, not a fictitious successful frequency setting.
	const uint32_t actual = SystemPeriphClockGet(Idx);
	return Freq == actual ? actual : 0U;
}

void SystemOscInit(void)
{
	// Oscillator configuration is performed by SystemInit/clock selectors.
	// RA4M1 has no software-programmable crystal load capacitance here.
}

static void Ra4m1StartupStop(Ra4m1StartupError_t Error)
{
	g_Ra4m1StartupError = Error;
	__disable_irq();
	for (;;) { __NOP(); }
}

void SystemInit(void)
{
	const uint32_t state = DisableInterrupt();
	g_Ra4m1StartupError = RA4M1_STARTUP_OK;
	SCB->VTOR = (uint32_t)(uintptr_t)__Vectors;
	// Enable CP10/CP11 before any application/RTOS floating-point use.
	SCB->CPACR |= 0x00F00000UL;
	__DSB();
	__ISB();

	// Cold-start baseline only. No fixed peripheral-to-vector assignments.
	for (unsigned i = 0U; i < RA4M1_IRQ_COUNT; ++i)
	{
		NVIC_DisableIRQ((IRQn_Type)i);
		R_ICU->IELSR[i] = 0U;
		NVIC_ClearPendingIRQ((IRQn_Type)i);
	}
	Ra4m1ClockPlan_t plan;
	if (!Ra4m1ValidLf(g_McuOsc.LowPwrOsc.Type, g_McuOsc.LowPwrOsc.Freq))
		Ra4m1StartupStop(RA4M1_STARTUP_BAD_LF_OSC);
	if (!Ra4m1MakePlan(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq,
					  g_McuOsc.bUSBClk, &plan))
		Ra4m1StartupStop(RA4M1_STARTUP_BAD_OSC);
	if (!SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq))
		Ra4m1StartupStop((Ra4m1StartupError_t)g_Ra4m1StartupError);
	if (!SystemLowFreqClockSelect(g_McuOsc.LowPwrOsc.Type, g_McuOsc.LowPwrOsc.Freq))
		Ra4m1StartupStop((Ra4m1StartupError_t)g_Ra4m1StartupError);
	SystemCoreClockUpdate();
	EnableInterrupt(state);
}
