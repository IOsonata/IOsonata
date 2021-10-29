/**-------------------------------------------------------------------------
@file	system_sam4l.c

@brief	CMSIS System initializations

Clock scheme

Generic clock source (OSCSEL)
	0 - RCSYS
	1 - OSC32K
	2 - DFPLL0
	3 - OSC0
	4 - RC80M
	5 - RCFAST
	6 - RC1M
	7 - CLK_CPU
	8 - CLK_HSB
	9 - CLK_PBA
	10 - CLK_PBB
	11 - CLK_PBC
	12 - CLK_PBD
	13 - RC32K
	14 - Reserved
	15 - CLK_1K
	16 - PLL0
	17 - HRP
	18 - FP
	19-20 - GCLK_IN[0-1]
	21 - GCLK11

PLL0 clock source (PLLOSC)
	0 - OSC0
	1 - Generic clock 9

High Resolution/Fractional Prescaler clock source (CKSEL)
	0 - OSC0
	1 - PLL0
	2 - DFPLL0
	3 - Reserved
	4 - RC80M


@author	Hoang Nguyen Hoan
@date	June. 30, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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

#include "sam4lxxx.h"

#include "system_sam4l.h"
#include "coredev/system_core_clock.h"
#include "interrupt.h"

#define SYSTEM_CORE_CLOCK				48000000UL	// Default core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(27UL)		// Adjustment value for nanosec delay

#define MAINOSC_FREQ_MIN		3000000
#define MAINOSC_FREQ_MAX		20000000

#define USB_FREQ				48000000

#if defined(SAM4LSXA) || defined(SAM4LSXB) || defined(SAM4LSXC)
#define FVCO_FREQ_MAX			192000000UL
#else
#define FVCO_FREQ_MAX			240000000UL
#endif

#define PERIPH_CLOCK_MAX		3

// Flash waitstate

__WEAK MCU_OSC g_McuOsc = {
	OSC_TYPE_XTAL,
	12000000,
	OSC_TYPE_RC,
	32000
};

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;
static uint32_t s_PllFreq = SYSTEM_CORE_CLOCK;

static const uint32_t s_Fvco[] = { 192000000UL, 96000000UL, 48000000UL };
static const int s_FvcoCnt = sizeof(s_Fvco) / sizeof(uint32_t);

/**
 * @brief	Get system low frequency oscillator type
 *
 * @return	Return oscillator type either internal RC or external crystal/osc
 */
OSC_TYPE GetLowFreqOscType()
{
	return g_McuOsc.LFType;
}

/**
 * @brief	Get system high frequency oscillator type
 *
 * @return	Return oscillator type either internal RC or external crystal/osc
 */
OSC_TYPE GetHighFreqOscType()
{
	return g_McuOsc.HFType;
}

bool FlashWaitReady(uint32_t Timeout)
{
	bool res = false;

	while (Timeout != 0 && res == false)
	{
		res = SAM4L_HFLASHC->FLASHCALW_FSR & FLASHCALW_FSR_FRDY;
		Timeout--;
	}

	return res;
}

void FlashSendCommand(uint32_t Cmd, int PgNo)
{
	uint32_t d;

	if (FlashWaitReady(1000000))
	{
		d = SAM4L_HFLASHC->FLASHCALW_FCMD & ~FLASHCALW_FCMD_CMD_Msk;

		if (PgNo >= 0)
		{
			d = FLASHCALW_FCMD_KEY_KEY | FLASHCALW_FCMD_PAGEN(PgNo) | Cmd;
		}
		else
		{
			d |= (FLASHCALW_FCMD_KEY_KEY | Cmd);
		}

		SAM4L_HFLASHC->FLASHCALW_FCMD = d;
		FlashWaitReady(1000000);
	}
}

void SetFlashWaitState(uint32_t CoreFreq)
{
	if (CoreFreq < 12000000UL)
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSDIS, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR & ~(FLASHCALW_FCR_FWS | FLASHCALW_FCR_WS1OPT)) |
										FLASHCALW_FCR_FWS_0;

	}
	else if (CoreFreq < 24000000UL)
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSDIS, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR & ~FLASHCALW_FCR_FWS) |
										FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	}
	else if (CoreFreq < 48000000UL)
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSEN, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR & ~(FLASHCALW_FCR_FWS | FLASHCALW_FCR_WS1OPT)) |
										FLASHCALW_FCR_FWS_0;
	}
	else
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSEN, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR & ~FLASHCALW_FCR_FWS) |
										FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	}
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	if (Freq < MAINOSC_FREQ_MIN || Freq > 20000000)
	{
		return false;
	}

	g_McuOsc.HFType = ClkSrc;

	if (ClkSrc == OSC_TYPE_RC)
	{
		if (Freq < 8000000)
		{
			g_McuOsc.HFFreq = 4000000;
		}
		else if (Freq < 16000000)
		{
			g_McuOsc.HFFreq = 8000000;
		}
		else
		{
			g_McuOsc.HFFreq = 12000000;
		}
	}
	else
	{
		g_McuOsc.HFFreq = Freq;
	}

	SystemInit();

	return true;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (ClkSrc == OSC_TYPE_RC)
	{
		g_McuOsc.LFType = OSC_TYPE_RC;
		g_McuOsc.LFFreq = 32000;
	}
	else
	{
		g_McuOsc.LFType = OSC_TYPE_XTAL;
		g_McuOsc.LFFreq = 32768;
	}

	return true;
}

// Fvco = (PLLMUL+1)/PLLDIV • Fref, if PLLDIV >0
// Fvco = 2•(PLLMUL+1) • Fref, if PLLDIV = 0
// Fvco = SYSTEM_CORE_CLOCK = (PLLMUL+1)/PLLDIV • Fref
// SYSTEM_CORE_CLOCK / Fref = (PLLMUL+1)/PLLDIV
// SYSTEM_CORE_CLOCK * PLLDIV / Fref - 1 = PLLMUL
/// @return PLL frequency
void SystemSetPLL()
{
	uint32_t div = 0;
	uint32_t mul = 0;
	uint32_t fvco = 0;//(96000000UL / SYSTEM_CORE_CLOCK) * SYSTEM_CORE_CLOCK;
	uint32_t pllopt = 0;

	for (int i = 15; i > 0; i--)
	{
		for (int j = 1; j < 16; j++)
		{
			uint32_t f = g_McuOsc.HFFreq * (i + 1) / j;
			for (int x = 0; x < s_FvcoCnt; x++)
			{
				if (f == s_Fvco[x] && (f > fvco))
				{
					fvco = f;
					div = j;
					mul = i;
				}
			}
		}
	}

	s_PllFreq = (mul + 1) * g_McuOsc.HFFreq / div;

	if (s_PllFreq > 160000000)
	{
		pllopt = 1;
	}

//	uint32_t d =  SCIF_PLL_PLLDIV(div) | SCIF_PLL_PLLMUL(mul) |
//  	   	   	   	   SCIF_PLL_PLLOPT(pllopt) | SCIF_PLL_PLLCOUNT_Msk |
//  	   	   	   	   SCIF_PLL_PLLEN;
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu) |
			SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL - (uint32_t)SAM4L_SCIF);

	SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL = SCIF_PLL_PLLDIV(div) | SCIF_PLL_PLLMUL(mul) |
			   	   	   	   	   	   	   SCIF_PLL_PLLOPT(pllopt) | SCIF_PLL_PLLCOUNT_Msk |
			   	   	   	   	   	   	   SCIF_PLL_PLLEN;
	while (!(SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_PLL0LOCK));
}

void SystemInit()
{
	uint32_t temp = 0;

	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
		| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_PBBMASK - (uint32_t)SAM4L_PM);
	SAM4L_PM->PM_PBBMASK |= PM_PBBMASK_HCACHE;

	SAM4L_HCACHE->HCACHE_CTRL = HCACHE_CTRL_CEN_YES;
	while (!(SAM4L_HCACHE->HCACHE_SR & HCACHE_SR_CSTS_EN));

	// WDT is on by default.  Disable it.
	SAM4L_WDT->WDT_CTRL &= ~WDT_CTRL_EN;

	FlashSendCommand(FLASHCALW_FCMD_CMD_HSEN, -1);
	SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR & ~FLASHCALW_FCR_FWS) |
									FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;

	if (g_McuOsc.HFType == OSC_TYPE_RC)
	{
		// Internal RC
		switch (g_McuOsc.HFFreq)
		{
			case 1000000:
				SAM4L_BSCIF->BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
					| BSCIF_UNLOCK_ADDR((uint32_t)&SAM4L_BSCIF->BSCIF_RC1MCR - (uint32_t)SAM4L_BSCIF);
				SAM4L_BSCIF->BSCIF_RC1MCR |= BSCIF_RC1MCR_CLKOE;
				while ((SAM4L_BSCIF->BSCIF_RC1MCR & BSCIF_RC1MCR_CLKOE) == 0);
				break;

			case 80000000:
			default:
				g_McuOsc.HFFreq = 80000000;
				// USING 80M
				if ((SAM4L_SCIF->SCIF_RC80MCR & SCIF_RC80MCR_EN) == 0)
				{
					SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
						| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_RC80MCR - (uint32_t)SAM4L_SCIF);
					SAM4L_SCIF->SCIF_RC80MCR |= SCIF_RC80MCR_EN;
					while ((SAM4L_SCIF->SCIF_RC80MCR & SCIF_RC80MCR_EN) == 0);
				}
				SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_CPUSEL - (uint32_t)SAM4L_PM);
				SAM4L_PM->PM_CPUSEL = 1 | PM_CPUSEL_CPUDIV;

				break;

			case 4000000:
			case 8000000:
			case 12000000:
				// using RCFAST

				temp = SAM4L_SCIF->SCIF_RCFASTCFG & ~SCIF_RCFASTCFG_FRANGE_Msk;

				SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
					| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_RCFASTCFG - (uint32_t)SAM4L_SCIF);
				SAM4L_SCIF->SCIF_RCFASTCFG = temp | SCIF_RCFASTCFG_EN
					| SCIF_RCFASTCFG_FRANGE(g_McuOsc.HFFreq / 4000000 - 1);
				while ((SAM4L_SCIF->SCIF_RCFASTCFG & (SCIF_RCFASTCFG_EN)) == 0);

				SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
				SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_RCFAST;
				break;
		}
	}
	else
	{
		// External crystal/tcxo
		uint32_t gain = g_McuOsc.HFFreq / 4000000;
		uint32_t mode = 0;	// Default external oscillator TXO

		if (g_McuOsc.HFType == OSC_TYPE_XTAL)
		{
			// external crystal
			mode = (0x1u << SCIF_OSCCTRL0_MODE_Pos);
		}
		SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
			| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_OSCCTRL0 - (uint32_t)SAM4L_SCIF);
		SAM4L_SCIF->SCIF_OSCCTRL0 = SCIF_OSCCTRL0_OSCEN | mode | SCIF_OSCCTRL0_GAIN(gain) | SCIF_OSCCTRL0_STARTUP(2);
		while ((SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_OSC0RDY) == 0);

		SystemSetPLL();

		// PLL reg = 0x3f030109
		uint32_t cpudiv = SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL;

		if (s_PllFreq > SYSTEM_CORE_CLOCK)
		{
//			cpudiv = s_PllFreq / 48000000;
//			SystemCoreClock = s_PllFreq / cpudiv;

			cpudiv = 30 - __CLZ(s_PllFreq / SYSTEM_CORE_CLOCK);

			SystemCoreClock = s_PllFreq / (1 << (cpudiv + 1));

			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
				| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_CPUSEL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_CPUSEL = cpudiv | PM_CPUSEL_CPUDIV;
		}

		if (SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_PLL0LOCK)
		{
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
				| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_PLL0;
		}
	}


	SetFlashWaitState(SystemCoreClock);

	// Low freq clock
	if (g_McuOsc.LFType == OSC_TYPE_RC)
	{
		uint32_t rc32ctrl = SAM4L_BSCIF->BSCIF_RC32KCR;
		rc32ctrl &= BSCIF_RC32KCR_FCD;
		rc32ctrl |= BSCIF_RC32KCR_TCEN | BSCIF_RC32KCR_EN32K | BSCIF_RC32KCR_REF | BSCIF_RC32KCR_EN;

		SAM4L_BSCIF->BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
			| BSCIF_UNLOCK_ADDR((uint32_t)&SAM4L_BSCIF->BSCIF_RC32KCR - (uint32_t)SAM4L_BSCIF);
		SAM4L_BSCIF->BSCIF_RC32KCR = rc32ctrl;
		while (!(SAM4L_BSCIF->BSCIF_PCLKSR & BSCIF_PCLKSR_RC32KRDY));
	}
	else
	{
		// External Xtal/txco
		uint32_t oscctrl = BSCIF_OSCCTRL32_STARTUP(2) | BSCIF_OSCCTRL32_SELCURR(10) |
						   BSCIF_OSCCTRL32_EN32K | BSCIF_OSCCTRL32_OSC32EN;

		if (g_McuOsc.LFType == OSC_TYPE_XTAL)
		{
			oscctrl |= BSCIF_OSCCTRL32_MODE(1);
		}


		SAM4L_BSCIF->BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
			| BSCIF_UNLOCK_ADDR((uint32_t)&SAM4L_BSCIF->BSCIF_OSCCTRL32 - (uint32_t)SAM4L_BSCIF);
		SAM4L_BSCIF->BSCIF_OSCCTRL32 =  oscctrl;
		while (!(SAM4L_BSCIF->BSCIF_PCLKSR & BSCIF_PCLKSR_OSC32RDY));
	}

	SystemCoreClockUpdate();
}

void SystemCoreClockUpdate(void)
{
	if (g_McuOsc.HFType == OSC_TYPE_RC)
	{

	}
	else
	{
		// XTAL
		uint32_t pll = SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL;
		uint32_t mul = (pll & SCIF_PLL_PLLMUL_Msk) >> SCIF_PLL_PLLMUL_Pos;
		uint32_t div = (pll & SCIF_PLL_PLLDIV_Msk) >> SCIF_PLL_PLLDIV_Pos;
		uint32_t fvco = div > 0 ? (mul + 1) * g_McuOsc.HFFreq / div : ((mul + 1) << 1) * g_McuOsc.HFFreq;
		uint32_t cpusel = SAM4L_PM->PM_CPUSEL;

		if (pll & SCIF_PLL_PLLOPT(2))
		{
			fvco >>= 1;
		}

		if (cpusel & PM_CPUSEL_CPUDIV)
		{
			SystemCoreClock = fvco / (1 << ((cpusel & PM_CPUSEL_CPUSEL_Msk) + 1));
		}
		else
		{
			SystemCoreClock = fvco;
		}
	}
	SetFlashWaitState(SystemCoreClock);
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
	return SystemCoreClock;
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
	return SystemPeriphClockGet(Idx);
}

