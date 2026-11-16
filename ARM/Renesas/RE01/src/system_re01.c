/**-------------------------------------------------------------------------
@file	system_re01.c

@brief	Implementation of CMSIS SystemInit for Renesas RE01 Device Series

Note: 	USB operation requires PLL as clock source running at 48MHz
		PLL operation requires boost mode and main clock (crystal or ext osc)
		Therefore main clock source (8-32MHz) must be chosen to allows PLL to
		generate 48MHz

Note:	Running at max freq 64MHz cause limitation on UART timing.  Many standard
		Baudrate would not work.

@author	Hoang Nguyen Hoan
@date	Nov. 11, 2021

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
#include <stdlib.h>
#include "coredev/interrupt.h"

#include "re01xxx.h"
#include "coredev/system_core_clock.h"

#define SYSTEM_CORE_CLOCK_MAX			64000000UL
#define SYSTEM_NSDELAY_CORE_FACTOR		(34UL)
#define RE01_OSC_WAIT_COUNT				1000000UL

#define SYSTEM_SCKSCR_CKSEL_HOCO	(0)		// High speed RC
#define SYSTEM_SCKSCR_CKSEL_MOCO	(1)		// Mid speed RC
#define SYSTEM_SCKSCR_CKSEL_LOCO	(2)		// Low speed RC
#define SYSTEM_SCKSCR_CKSEL_MCO		(3)		// Main Clock osc
#define SYSTEM_SCKSCR_CKSEL_SCO		(4)		// Sub-clock osc
#define SYSTEM_SCKSCR_CKSEL_PLL		(5)		// PLL

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK_MAX;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;

// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
	{
		OSC_TYPE_RC,
		48000000,		// Default to 48MHz because at 64MHz many UART baudrate could not be matched
		20, 0
	},
	{
		OSC_TYPE_RC,
		32768, 20, 0
	},
	true,
	0
};

static uint32_t s_PeriphSrcFreq = 0;

static bool Re01ClockIdle(void)
{
	// Running modules keep the source and dividers used by their drivers.
	return (SYSTEM->MSTPCRA & SYSTEM_MSTPCRA_MSTPA22_Msk) != 0 &&
		(MSTP->MSTPCRB & 0xfc4c0b60UL) == 0xfc4c0b60UL &&
		(MSTP->MSTPCRC & 0x8640e403UL) == 0x8640e403UL &&
		(MSTP->MSTPCRD & 0x105341ffUL) == 0x105341ffUL &&
		(SYSTEM->CKOCR & SYSTEM_CKOCR_CKOEN_Msk) == 0 &&
		FLASH->FENTRYR == 0;
}

static void SetFlashWaitState(uint32_t CoreFreq)
{
	if (CoreFreq > 32000000U)
	{
		FLASH->FLWT = 1;
	}
	else
	{
		FLASH->FLWT = 0;
	}
}

static bool Re01WaitOsc(uint8_t Mask, bool bStable)
{
	for (uint32_t retry = RE01_OSC_WAIT_COUNT; retry > 0; retry--)
	{
		if (((SYSTEM->OSCSF & Mask) != 0) == bStable)
		{
			return true;
		}
	}
	return false;
}

static bool EnterBoostMode(void)
{
	if (SYSTEM->PWSTF_b.BOOSTM)
	{
		return true;
	}
	if (SYSTEM->PWSTF != SYSTEM_PWSTF_ALLPWONM_Msk ||
		(SYSTEM->SYOCDCR & (SYSTEM_SYOCDCR_DOCDF_Msk | SYSTEM_SYOCDCR_DBGEN_Msk)))
	{
		return false;
	}
	uint16_t sbycr = SYSTEM->SBYCR;
	uint8_t dpsbycr = SYSTEM->DPSBYCR;
	uint8_t snzcr = SYSTEM->SNZCR;
	uint8_t power = SYSTEM->PWSTCR;
	// Consume a stale event before the WFE power transition.
	__SEV();
	__WFE();
	// Boost entry uses Software Standby as a power-mode transition.
	SYSTEM->SBYCR = SYSTEM_SBYCR_SSBY_Msk;
	SYSTEM->DPSBYCR_b.DPSBY = 0;
	SYSTEM->SNZCR_b.SNZE = 0;
	SYSTEM->PWSTCR = 5;
	if (SYSTEM->PWSTCR != 5)
	{
		SYSTEM->SBYCR = sbycr;
		SYSTEM->DPSBYCR = dpsbycr;
		SYSTEM->SNZCR = snzcr;
		SYSTEM->PWSTCR = power;
		return false;
	}
	__DSB();
	__WFE();

	uint32_t retry = RE01_OSC_WAIT_COUNT;
	while (!SYSTEM->PWSTF_b.BOOSTM && retry > 0)
	{
		retry--;
	}
	bool ready = SYSTEM->PWSTF_b.BOOSTM != 0;
	SYSTEM->SBYCR = sbycr;
	SYSTEM->DPSBYCR = dpsbycr;
	SYSTEM->SNZCR = snzcr;
	SYSTEM->PWSTCR = power;
	return ready;
}

bool Re01LeaveBoostMode(void)
{
	uint32_t state = DisableInterrupt();
	if ((SYSTEM->PWSTF & SYSTEM_PWSTF_BOOSTM_Msk) == 0)
	{
		EnableInterrupt(state);
		return true;
	}
	if (SYSTEM->PWSTF != (SYSTEM_PWSTF_ALLPWONM_Msk | SYSTEM_PWSTF_BOOSTM_Msk) ||
		(SYSTEM->SYOCDCR & (SYSTEM_SYOCDCR_DOCDF_Msk | SYSTEM_SYOCDCR_DBGEN_Msk)))
	{
		EnableInterrupt(state);
		return false;
	}
	uint16_t protection = SYSTEM->PRCR & 0xfU;
	uint16_t standby = SYSTEM->SBYCR;
	uint8_t deepstandby = SYSTEM->DPSBYCR;
	uint8_t snooze = SYSTEM->SNZCR;
	uint8_t power = SYSTEM->PWSTCR;
	uint8_t operating = SYSTEM->OPCCR;
	uint8_t suboperating = SYSTEM->SOPCCR;
	SYSTEM->PRCR = 0xa503U | protection;

	// PLL and clocks above 32 MHz require Boost. Move to MOCO first.
	SYSTEM->MOCOCR = 0;
	SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MOCO;
	bool ready = SYSTEM->MOCOCR == 0 && SYSTEM->SCKSCR_b.CKSEL == SYSTEM_SCKSCR_CKSEL_MOCO;
	if (ready)
	{
		SYSTEM->PLLCR = SYSTEM_PLLCR_PLLSTP_Msk;
		SYSTEM->HOCOCR = 1;
		ready = SYSTEM->PLLCR == SYSTEM_PLLCR_PLLSTP_Msk && SYSTEM->HOCOCR == 1 &&
			Re01WaitOsc(SYSTEM_OSCSF_PLLSF_Msk | SYSTEM_OSCSF_HOCOSF_Msk, false);
		g_McuOsc.bUSBClk = false;
	}
	if (ready)
	{
		// Consume a stale event before selecting the transition request.
		__SEV();
		__WFE();
		SYSTEM->SOPCCR_b.SOPCM = 0;
		SYSTEM->OPCCR_b.OPCM = 0;
		SYSTEM->SBYCR = SYSTEM_SBYCR_SSBY_Msk;
		SYSTEM->DPSBYCR_b.DPSBY = 0;
		SYSTEM->SNZCR_b.SNZE = 0;
		SYSTEM->PWSTCR = 4;
		ready = SYSTEM->PWSTCR == 4;
		SYSTEM->PRCR = 0xa500U | protection;
		if (ready)
		{
			__DSB();
			__WFE();
			uint32_t retry = RE01_OSC_WAIT_COUNT;
			while ((SYSTEM->PWSTF & (SYSTEM_PWSTF_BOOSTM_Msk | SYSTEM_PWSTF_VBBM_Msk)) && retry > 0)
			{
				retry--;
			}
			ready = SYSTEM->PWSTF == SYSTEM_PWSTF_ALLPWONM_Msk;
		}
	}
	SYSTEM->PRCR = 0xa503U | protection;
	SYSTEM->SBYCR = standby;
	SYSTEM->DPSBYCR = deepstandby;
	SYSTEM->SNZCR = snooze;
	SYSTEM->PWSTCR = power;
	if (!ready)
	{
		SYSTEM->OPCCR = operating;
		SYSTEM->SOPCCR = suboperating;
	}
	SYSTEM->PRCR = 0xa500U | protection;
	SystemCoreClockUpdate();
	SetFlashWaitState(SystemCoreClock);
	EnableInterrupt(state);
	return ready;
}

// USB operation requires PLL clock source
// Target PLL for 48MHz operating for compatibility with USB
// PLL operation can only work in Boost mode
static uint32_t ConfigPLL(uint32_t SrcFreq)
{
	uint32_t tf = 64000000;

	if (g_McuOsc.bUSBClk)
	{
		tf = 48000000;
	}
	// Make sure PLL is stopped
	if (SYSTEM->SCKSCR_b.CKSEL == SYSTEM_SCKSCR_CKSEL_PLL)
	{
		return 0;
	}
	SYSTEM->PLLCR = SYSTEM_PLLCR_PLLSTP_Msk;
	if (!Re01WaitOsc(SYSTEM_OSCSF_PLLSF_Msk, false))
	{
		return 0;
	}

	for (int div = 1; div < 5; div++)
	{
		for (int mul = 2; mul < 9; mul++)
		{
			// PLL Freq = (SrcFreq / div) * mul;
			if ((uint64_t)SrcFreq * mul == (uint64_t)tf * div)
			{
				if (!EnterBoostMode())
				{
					return 0;
				}
				SYSTEM->PLLCCR = ((div - 1) << SYSTEM_PLLCCR_PLIDIV_Pos) |
								 ((mul - 1) << SYSTEM_PLLCCR_PLLMUL_Pos);
				SYSTEM->PLLCR = 0;	// Start PLL

				if (!Re01WaitOsc(SYSTEM_OSCSF_PLLSF_Msk, true))
				{
					SYSTEM->PLLCR = SYSTEM_PLLCR_PLLSTP_Msk;
					return 0;
				}
				return tf;
			}
		}
	}

	return 0;
}

void SystemCoreClockUpdate(void)
{
	uint8_t clksrc = SYSTEM->SCKSCR_b.CKSEL;
	uint32_t divider = (SYSTEM->SCKDIVCR & SYSTEM_SCKDIVCR_ICK_Msk) >> SYSTEM_SCKDIVCR_ICK_Pos;

	switch (clksrc)
	{
		case SYSTEM_SCKSCR_CKSEL_HOCO:
			if (SYSTEM->HOCOCR || !(SYSTEM->OSCSF & SYSTEM_OSCSF_HOCOSF_Msk))
			{
				SystemCoreClock = 0;
				break;
			}
			switch (SYSTEM->HOCOMCR & SYSTEM_HOCOMCR_HCFRQ_Msk)
			{
				case 0:
					SystemCoreClock = 24000000UL;
					break;
				case 1:
					SystemCoreClock = 32000000UL;
					break;
				case 2:
					SystemCoreClock = 48000000UL;
					break;
				case 3:
					SystemCoreClock = 64000000UL;
					break;
			}
			break;
		case SYSTEM_SCKSCR_CKSEL_MOCO:
			SystemCoreClock = SYSTEM->MOCOCR ? 0 : 2000000UL;
			break;
		case SYSTEM_SCKSCR_CKSEL_LOCO:
			SystemCoreClock = SYSTEM->LOCOCR ? 0 : 32768UL;
			break;
		case SYSTEM_SCKSCR_CKSEL_SCO:
			SystemCoreClock = SYSTEM->SOSCCR ? 0 : 32768UL;
			break;
		case SYSTEM_SCKSCR_CKSEL_MCO:
			SystemCoreClock = SYSTEM->MOSCCR || !(SYSTEM->OSCSF & SYSTEM_OSCSF_MOSCSF_Msk) ?
				0 : g_McuOsc.CoreOsc.Freq;
			break;
		case SYSTEM_SCKSCR_CKSEL_PLL:
			if (SYSTEM->MOSCCR || SYSTEM->PLLCR ||
				(SYSTEM->OSCSF & (SYSTEM_OSCSF_MOSCSF_Msk | SYSTEM_OSCSF_PLLSF_Msk)) !=
				(SYSTEM_OSCSF_MOSCSF_Msk | SYSTEM_OSCSF_PLLSF_Msk))
			{
				SystemCoreClock = 0;
				break;
			}
			SystemCoreClock = (uint64_t)g_McuOsc.CoreOsc.Freq * (SYSTEM->PLLCCR_b.PLLMUL + 1) /
							  (SYSTEM->PLLCCR_b.PLIDIV + 1);
			break;
		default:
			SystemCoreClock = 0;
			break;
	}

	s_PeriphSrcFreq = SystemCoreClock;

	SystemCoreClock = divider <= 6 ? SystemCoreClock >> divider : 0;

	// Reporting clocks must not change divider or flash settings.
}


static bool Re01SetCoreClock(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	FLASH->FLWT = 1;
	SYSTEM->MOCOCR = 0;
	SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MOCO;
	if (SYSTEM->MOCOCR || SYSTEM->SCKSCR_b.CKSEL != SYSTEM_SCKSCR_CKSEL_MOCO)
	{
		return false;
	}
	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;
	SYSTEM->SCKDIVCR = (SYSTEM->SCKDIVCR & ~(SYSTEM_SCKDIVCR_ICK_Msk | SYSTEM_SCKDIVCR_PCKB_Msk)) |
		(1UL << SYSTEM_SCKDIVCR_PCKB_Pos);

	if (ClkSrc == OSC_TYPE_RC && OscFreq <= 32000000UL && !Re01LeaveBoostMode())
	{
		return false;
	}

	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_RC)
	{
		g_McuOsc.bUSBClk = false;
		if (g_McuOsc.CoreOsc.Freq <= 2000000UL)
		{
			return true;
		}
		uint8_t hcfrq = g_McuOsc.CoreOsc.Freq <= 24000000UL ? 0 :
			(g_McuOsc.CoreOsc.Freq <= 32000000UL ? 1 :
			(g_McuOsc.CoreOsc.Freq <= 48000000UL ? 2 : 3));
		if (hcfrq >= 2 && !EnterBoostMode())
		{
			return false;
		}
		SYSTEM->HOCOCR = 1;
		if (!Re01WaitOsc(SYSTEM_OSCSF_HOCOSF_Msk, false))
		{
			return false;
		}
		SYSTEM->HOCOMCR = hcfrq;
		SYSTEM->HOCOCR = 0;
		if (!Re01WaitOsc(SYSTEM_OSCSF_HOCOSF_Msk, true))
		{
			return false;
		}
		SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_HOCO;
		return SYSTEM->SCKSCR_b.CKSEL == SYSTEM_SCKSCR_CKSEL_HOCO;
	}
	if ((g_McuOsc.CoreOsc.Type != OSC_TYPE_XTAL && g_McuOsc.CoreOsc.Type != OSC_TYPE_TCXO) ||
		g_McuOsc.CoreOsc.Freq < 8000000UL || g_McuOsc.CoreOsc.Freq > 32000000UL)
	{
		g_McuOsc.bUSBClk = false;
		return false;
	}
	SYSTEM->PLLCR = SYSTEM_PLLCR_PLLSTP_Msk;
	if (!Re01WaitOsc(SYSTEM_OSCSF_PLLSF_Msk, false))
	{
		g_McuOsc.bUSBClk = false;
		return false;
	}
	SYSTEM->MOSCCR = 1;
	if (!Re01WaitOsc(SYSTEM_OSCSF_MOSCSF_Msk, false))
	{
		g_McuOsc.bUSBClk = false;
		return false;
	}
	SYSTEM->MOMCR = SYSTEM_MOMCR_OSCLPEN_Msk | (4 << SYSTEM_MOMCR_MODRV_Pos) |
		(g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO ? SYSTEM_MOMCR_MOSEL_Msk : 0);
	SYSTEM->MOSCCR = 0;
	if (!Re01WaitOsc(SYSTEM_OSCSF_MOSCSF_Msk, true))
	{
		g_McuOsc.bUSBClk = false;
		return false;
	}
	if (ConfigPLL(g_McuOsc.CoreOsc.Freq) != 0)
	{
		SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_PLL;
		return SYSTEM->SCKSCR_b.CKSEL == SYSTEM_SCKSCR_CKSEL_PLL;
	}
	// A failed PLL leaves a live main clock selected.
	SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MCO;
	g_McuOsc.bUSBClk = false;
	return false;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if ((ClkSrc != OSC_TYPE_RC && ClkSrc != OSC_TYPE_XTAL && ClkSrc != OSC_TYPE_TCXO) ||
		(ClkSrc != OSC_TYPE_RC && (OscFreq < 8000000UL || OscFreq > 32000000UL)))
	{
		return false;
	}
	if (ClkSrc == OSC_TYPE_RC)
	{
		OscFreq = OscFreq <= 13000000UL ? 2000000UL :
			(OscFreq <= 28000000UL ? 24000000UL :
			(OscFreq <= 40000000UL ? 32000000UL :
			(OscFreq <= 56000000UL ? 48000000UL : 64000000UL)));
	}
	uint32_t state = DisableInterrupt();
	if (!Re01ClockIdle() || (SYSTEM->PWSTF &
		(SYSTEM_PWSTF_ALLPWONM_Msk | SYSTEM_PWSTF_VBBM_Msk)) != SYSTEM_PWSTF_ALLPWONM_Msk)
	{
		EnableInterrupt(state);
		return false;
	}
	uint16_t protection = SYSTEM->PRCR & 0xfU;
	SYSTEM->PRCR = 0xa503U | protection;
	bool ready = Re01SetCoreClock(ClkSrc, OscFreq);
	SYSTEM->PRCR = 0xa500U | protection;
	SystemCoreClockUpdate();
	SetFlashWaitState(SystemCoreClock);
	if (SYSTEM->SCKSCR_b.CKSEL == SYSTEM_SCKSCR_CKSEL_HOCO ||
		SYSTEM->SCKSCR_b.CKSEL == SYSTEM_SCKSCR_CKSEL_MOCO)
	{
		g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
		g_McuOsc.CoreOsc.Freq = s_PeriphSrcFreq;
	}
	EnableInterrupt(state);
	return ready;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (ClkSrc != OSC_TYPE_RC && ClkSrc != OSC_TYPE_XTAL)
	{
		return false;
	}
	(void)OscFreq;
	uint32_t state = DisableInterrupt();
	uint16_t protection = SYSTEM->PRCR & 0xfU;
	SYSTEM->PRCR = 0xa501U | protection;
	if (ClkSrc == OSC_TYPE_RC)
	{
		SYSTEM->LOCOCR = 0;
	}
	else
	{
		SYSTEM->SOSCCR = 0;
	}
	bool ready = false;
	for (uint32_t retry = RE01_OSC_WAIT_COUNT; retry > 0; retry--)
	{
		if ((ClkSrc == OSC_TYPE_RC ? SYSTEM->LOCOCR : SYSTEM->SOSCCR) == 0)
		{
			ready = true;
			break;
		}
	}
	SYSTEM->PRCR = 0xa500U | protection;
	if (ready)
	{
		// Starting a source does not stop the oscillator used by RTC or AGT.
		g_McuOsc.LowPwrOsc.Type = ClkSrc;
		g_McuOsc.LowPwrOsc.Freq = 32768;
	}
	EnableInterrupt(state);
	return ready;
}

void SystemInit(void)
{
	uint16_t protection = SYSTEM->PRCR & 0xfU;
	SYSTEM->PRCR = 0xa503U | protection;
	if (SYSTEM->RSTSR0 & SYSTEM_RSTSR0_DPSRSTF_Msk)
	{
		// Wake configuration belongs to the previous boot. Keep the wake flags
		// for SystemResetReason and the retained pin levels until reconfigured.
		SYSTEM->SBYCR = 0;
		SYSTEM->DPSBYCR_b.DPSBY = 0;
		SYSTEM->PWSTCR = 0;
		SYSTEM->SNZCR_b.SNZE = 0;
		SYSTEM->DPSIER0 = 0;
		SYSTEM->DPSIER1 = 0;
	}
	(void)Re01SetCoreClock(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq);
	SYSTEM->PRCR = 0xa500U | protection;
	(void)SystemLowFreqClockSelect(g_McuOsc.LowPwrOsc.Type, g_McuOsc.LowPwrOsc.Freq);
	SystemCoreClockUpdate();
	SetFlashWaitState(SystemCoreClock);
}

/**
 * @brief	Get high frequency clock frequency (HCLK)
 *
 * @return	HCLK clock frequency in Hz.
 */
uint32_t SystemHFClockGet()
{
	return s_PeriphSrcFreq;
}

/**
 * @brief	Get peripheral clock frequency (PCLK)
 *
 * Most often the PCLK numbering starts from 1 (PCLK1, PCLK2,...).
 * Therefore the clock Idx parameter = 0 indicate PCK1, 1 indicate PCLK2
 *
 * @param	Idx : Zero based peripheral clock number. Many processors can
 * 				  have more than 1 peripheral clock settings.
 *
 * @return	Peripheral clock frequency in Hz.
 * 			0 - Bad clock number
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t clk = 0;

	if (Idx == 0)
	{
		clk = SystemCoreClock;
	}
	else if (Idx == 1)
	{
		uint32_t div = (SYSTEM->SCKDIVCR & SYSTEM_SCKDIVCR_PCKB_Msk) >> SYSTEM_SCKDIVCR_PCKB_Pos;
		clk = div <= 6 ? s_PeriphSrcFreq >> div : 0;
	}

	return clk;
}

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Most often the PCLK numbering starts from 1 (PCLK1, PCLK2,...).
 * Therefore the clock Idx parameter = 0 indicate PCK1, 1 indicate PCLK2
 *
 * @param	Idx  : Zero based peripheral clock number. Many processors can
 * 				   have more than 1 peripheral clock settings.
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 * 			0 - Failed
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	if ((Idx != 0 && Idx != 1) || Freq == 0 || s_PeriphSrcFreq == 0)
	{
		return 0;
	}
	if (Idx == 1 && Freq > 32000000UL)
	{
		Freq = 32000000UL;
	}
	uint32_t state = DisableInterrupt();
	uint32_t reg = SYSTEM->SCKDIVCR;
	uint32_t ick = (reg & SYSTEM_SCKDIVCR_ICK_Msk) >> SYSTEM_SCKDIVCR_ICK_Pos;
	uint32_t pckb = (reg & SYSTEM_SCKDIVCR_PCKB_Msk) >> SYSTEM_SCKDIVCR_PCKB_Pos;
	if (ick > 6 || pckb > 6)
	{
		EnableInterrupt(state);
		return 0;
	}
	uint32_t div = 6;
	uint32_t error = UINT32_MAX;
	for (uint32_t candidate = Idx == 1 ? ick : 0; candidate <= 6; candidate++)
	{
		uint32_t rate = s_PeriphSrcFreq >> candidate;
		if (Idx == 1 && rate > 32000000UL)
		{
			continue;
		}
		uint32_t difference = rate > Freq ? rate - Freq : Freq - rate;
		if (difference < error)
		{
			error = difference;
			div = candidate;
		}
	}
	if (Idx == 0)
	{
		ick = div;
		if (pckb < ick)
		{
			pckb = ick;
		}
	}
	else
	{
		pckb = div < ick ? ick : div;
	}
	while (pckb < 6 && (s_PeriphSrcFreq >> pckb) > 32000000UL)
	{
		pckb++;
	}
	uint32_t next = (reg & ~(SYSTEM_SCKDIVCR_ICK_Msk | SYSTEM_SCKDIVCR_PCKB_Msk)) |
		(ick << SYSTEM_SCKDIVCR_ICK_Pos) | (pckb << SYSTEM_SCKDIVCR_PCKB_Pos);
	if (next != reg && !Re01ClockIdle())
	{
		EnableInterrupt(state);
		return 0;
	}
	if ((s_PeriphSrcFreq >> ick) > SystemCoreClock)
	{
		SetFlashWaitState(s_PeriphSrcFreq >> ick);
	}
	uint16_t protection = SYSTEM->PRCR & 0xFU;
	SYSTEM->PRCR = 0xA501U | protection;
	SYSTEM->SCKDIVCR = next;
	bool ready = SYSTEM->SCKDIVCR == next;
	SYSTEM->PRCR = 0xA500U | protection;
	SystemCoreClockUpdate();
	SetFlashWaitState(SystemCoreClock);
	uint32_t actual = SystemPeriphClockGet(Idx);
	EnableInterrupt(state);
	return ready ? actual : 0;
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}
