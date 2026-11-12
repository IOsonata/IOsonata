/**-------------------------------------------------------------------------
@file	system_sam4e.c

@brief	CMSIS System initializations


@author	Hoang Nguyen Hoan
@date	June. 24, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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

#include "sam4e.h"

#include "system_sam4e.h"
#include "coredev/system_core_clock.h"

#define SYSTEM_CORE_CLOCK				120000000UL	// Default core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(27UL)		// Adjustment value for nanosec delay

#define MAINOSC_FREQ_MIN		3000000
#define MAINOSC_FREQ_MAX		20000000

#define USB_FREQ				48000000
#define PLLA_FREQ				240000000UL

#define SAM4E_CLOCK_POLLS		1000000U

__WEAK McuOsc_t g_McuOsc = {
	{ OSC_TYPE_XTAL, 12000000, 20, 0},
	{ OSC_TYPE_RC, 32000, 20, 0 },
	false,
	0
};

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;
static bool Sam4eClockWait(uint32_t Mask, uint32_t Value)
{
	uint32_t i;

	for (i = 0; i < SAM4E_CLOCK_POLLS; i++)
	{
		if ((SAM4E_PMC->PMC_SR & Mask) == Value)
		{
			return true;
		}
	}
	return false;
}

static bool Sam4eClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	uint32_t div;
	uint32_t mul;
	uint32_t bestdiv = 0;
	uint32_t bestmul = 0;
	uint32_t bestfreq = 0;
	uint32_t rate;
	uint32_t mor;
	uint32_t rc = CKGR_MOR_MOSCRCF_4_MHz;
	volatile uint32_t delay;

	if (ClkSrc == OSC_TYPE_RC)
	{
		Freq = Freq <= 6000000 ? 4000000 : (Freq <= 10000000 ? 8000000 : 12000000);
		if (Freq == 8000000)
		{
			rc = CKGR_MOR_MOSCRCF_8_MHz;
		}
		else if (Freq == 12000000)
		{
			rc = CKGR_MOR_MOSCRCF_12_MHz;
		}
	}
	else if ((ClkSrc != OSC_TYPE_XTAL && ClkSrc != OSC_TYPE_TCXO) ||
			Freq < MAINOSC_FREQ_MIN ||
			Freq > (ClkSrc == OSC_TYPE_XTAL ? MAINOSC_FREQ_MAX : 50000000U))
	{
		return false;
	}

	// Do not retune a source feeding an enabled USB or programmable clock.
	if ((SAM4E_PMC->PMC_SCSR & (PMC_SCSR_UDP | PMC_SCSR_PCK0 |
			PMC_SCSR_PCK1 | PMC_SCSR_PCK2)) ||
			(SAM4E_PMC->PMC_WPMR & PMC_WPMR_WPEN))
	{
		return false;
	}
	// PLL input: 3..32 MHz, output: 80..240 MHz, multiplier: 2..81.
	// Choose the fastest legal core clock at or below 120 MHz.
	for (div = 1; div <= 16; div++)
	{
		if (Freq < 3000000U * div || Freq > 32000000U * div)
		{
			continue;
		}
		for (mul = 2; mul <= 81; mul++)
		{
			if (Freq * mul > PLLA_FREQ * div)
			{
				break;
			}
			rate = Freq * mul / div;
			if (rate < 80000000U || rate <= bestfreq)
			{
				continue;
			}
			if (g_McuOsc.bUSBClk && (Freq * mul % (USB_FREQ * div)))
			{
				continue;
			}
			bestdiv = div;
			bestmul = mul;
			bestfreq = rate;
		}
	}
	if (bestdiv == 0)
	{
		return false;
	}

	// Raise flash wait states before changing frequency. Preserve other settings.
	SAM4E_EFC->EEFC_FMR = (SAM4E_EFC->EEFC_FMR & ~EEFC_FMR_FWS_Msk) |
			EEFC_FMR_FWS(5) | EEFC_FMR_CLOE;
	// Use SLCK while changing MAINCK and PLLA, including repeated selections.
	SAM4E_PMC->PMC_MCKR &= ~PMC_MCKR_CSS_Msk;
	if (!Sam4eClockWait(PMC_SR_MCKRDY, PMC_SR_MCKRDY))
	{
		return false;
	}
	SAM4E_PMC->CKGR_PLLAR = CKGR_PLLAR_ONE;
	// MCK now runs at SLCK or slower; allow at least two SLCK cycles.
	for (delay = 0; delay < 16; delay++)
	{
	}
	mor = (SAM4E_PMC->CKGR_MOR & ~(CKGR_MOR_KEY_Msk | CKGR_MOR_MOSCSEL)) |
			CKGR_MOR_KEY_PASSWD | CKGR_MOR_MOSCRCEN;
	SAM4E_PMC->CKGR_MOR = mor;
	if (!Sam4eClockWait(PMC_SR_MOSCRCS | PMC_SR_MOSCSELS, PMC_SR_MOSCRCS))
	{
		return false;
	}
	mor &= ~(CKGR_MOR_MOSCRCF_Msk | CKGR_MOR_MOSCXTEN | CKGR_MOR_MOSCXTBY |
			CKGR_MOR_MOSCXTST_Msk);
	mor |= rc | CKGR_MOR_MOSCXTST(0xff);
	if (ClkSrc == OSC_TYPE_XTAL)
	{
		mor |= CKGR_MOR_MOSCXTEN;
	}
	else if (ClkSrc == OSC_TYPE_TCXO)
	{
		mor |= CKGR_MOR_MOSCXTBY;
	}
	SAM4E_PMC->CKGR_MOR = mor;
	if (!Sam4eClockWait(PMC_SR_MOSCRCS, PMC_SR_MOSCRCS))
	{
		return false;
	}
	if (ClkSrc != OSC_TYPE_RC)
	{
		if (ClkSrc == OSC_TYPE_XTAL && !Sam4eClockWait(PMC_SR_MOSCXTS, PMC_SR_MOSCXTS))
		{
			return false;
		}
		SAM4E_PMC->CKGR_MOR = mor | CKGR_MOR_MOSCSEL;
		if (!Sam4eClockWait(PMC_SR_MOSCSELS, PMC_SR_MOSCSELS))
		{
			return false;
		}
	}
	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = Freq;
	SAM4E_PMC->CKGR_PLLAR = CKGR_PLLAR_ONE | CKGR_PLLAR_PLLACOUNT(0x3f) |
			CKGR_PLLAR_DIVA(bestdiv) | CKGR_PLLAR_MULA(bestmul - 1);
	if (!Sam4eClockWait(PMC_SR_LOCKA, PMC_SR_LOCKA))
	{
		return false;
	}
	// PRES and CSS must be written separately, each followed by MCKRDY.
	SAM4E_PMC->PMC_MCKR = (SAM4E_PMC->PMC_MCKR &
			~(PMC_MCKR_PRES_Msk | PMC_MCKR_PLLADIV2)) | PMC_MCKR_PRES_CLK_2;
	if (!Sam4eClockWait(PMC_SR_MCKRDY, PMC_SR_MCKRDY))
	{
		return false;
	}
	SAM4E_PMC->PMC_MCKR = (SAM4E_PMC->PMC_MCKR & ~PMC_MCKR_CSS_Msk) |
			PMC_MCKR_CSS_PLLA_CLK;
	if (!Sam4eClockWait(PMC_SR_MCKRDY, PMC_SR_MCKRDY))
	{
		return false;
	}
	if (g_McuOsc.bUSBClk)
	{
		SAM4E_PMC->PMC_USB = PMC_USB_USBDIV(bestfreq / USB_FREQ - 1);
	}
	return true;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	bool result;

	result = Sam4eClockSelect(ClkSrc, Freq);
	SystemCoreClockUpdate();
	return result;
}

void SystemInit(void)
{
	// Preserve read-to-clear wake flags before clock initialization.
	(void)SystemResetReason();
	SystemOscInit();
	SAM4E_WDT->WDT_MR = WDT_MR_WDDIS;
	(void)SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq);

#if (__FPU_USED == 1)
	SCB->CPACR |= (3UL << 20) | (3UL << 22);
	__DSB();
	__ISB();
#endif
}

void SystemCoreClockUpdate( void )
{
	uint32_t clk = 0;
	uint32_t mckr = SAM4E_PMC->PMC_MCKR;
	uint32_t tmp = 0;
	uint32_t div;

	switch (mckr & PMC_MCKR_CSS_Msk)
	{
		case PMC_MCKR_CSS_SLOW_CLK:	// SLCK
			if ( SAM4E_SUPC->SUPC_SR & SUPC_SR_OSCSEL)
			{
				clk = 32768;
			}
			else
			{
				clk = 32000;
			}
		break;

		case PMC_MCKR_CSS_PLLA_CLK:
		case PMC_MCKR_CSS_MAIN_CLK:	// MAINCK
			if (SAM4E_PMC->CKGR_MOR & CKGR_MOR_MOSCSEL)
			{
				clk = g_McuOsc.CoreOsc.Freq;
			}
			else
			{
				switch (SAM4E_PMC->CKGR_MOR & CKGR_MOR_MOSCRCF_Msk)
				{
					case CKGR_MOR_MOSCRCF_4_MHz:
						clk = 4000000;
						break;

					case CKGR_MOR_MOSCRCF_8_MHz:
						clk = 8000000;
						break;

					case CKGR_MOR_MOSCRCF_12_MHz:
						clk = 12000000;
						break;

					default:
						break;
				}
			}
			break;

	}
	if ((mckr & PMC_MCKR_CSS_Msk) == PMC_MCKR_CSS_PLLA_CLK)
	{
		tmp = SAM4E_PMC->CKGR_PLLAR;
		div = (tmp & CKGR_PLLAR_DIVA_Msk) >> CKGR_PLLAR_DIVA_Pos;
		tmp = (tmp & CKGR_PLLAR_MULA_Msk) >> CKGR_PLLAR_MULA_Pos;
		clk = div != 0 && tmp != 0 ? (uint64_t)clk * (tmp + 1U) / div : 0;
		if (mckr & PMC_MCKR_PLLADIV2)
		{
			clk /= 2;
		}
	}

	mckr &= PMC_MCKR_PRES_Msk;
	if (mckr == PMC_MCKR_PRES_CLK_3)
	{
		clk /= 3;
	}
	else
	{
		clk >>= (mckr >> PMC_MCKR_PRES_Pos);
	}

	SystemCoreClock = clk;
	if (clk != 0)
	{
		SystemnsDelayFactor = (SYSTEM_NSDELAY_CORE_FACTOR * SYSTEM_CORE_CLOCK + clk / 2) / clk;
	}

}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}

/**
 * @brief	Get peripheral clock frequency
 *
 * Peripheral clock on the SAM4E is the same as MCK which is the core clock
 *
 * @param	Idx : Zero based peripheral clock number. Many processors can
 * 				  have more than 1 peripheral clock settings.
 *
 * @return	Peripheral clock frequency in Hz.
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 0 ? SystemCoreClock : 0;
}

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Peripheral clock on the SAM4E is the same as MCK which is the core clock
 * there is no settings to change freq.
 *
 * @param	Idx  : Zero based peripheral clock number. Many processors can
 * 				   have more than 1 peripheral clock settings.
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	(void)Freq;
	return SystemPeriphClockGet(Idx);
}

