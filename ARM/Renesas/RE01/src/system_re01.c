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
#include <assert.h>

#include "re01xxx.h"
#include "coredev/system_core_clock.h"

#define SYSTEM_CORE_CLOCK_MAX			64000000UL	// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(34UL)		// TODO: Adjustment value for nanosec delay
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
		20
	},
	{
		OSC_TYPE_RC,
		32768, 20
	},
	true
};

static uint32_t s_PeriphSrcFreq = 0;

#if 0
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
#endif

void SetFlashWaitState(uint32_t CoreFreq)
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

bool EnterBoostMode()
{
	if (SYSTEM->PWSTF_b.BOOSTM)
	{
		return true;
	}
	uint16_t sbycr = SYSTEM->SBYCR;
	uint8_t dpsbycr = SYSTEM->DPSBYCR;
	uint8_t snzcr = SYSTEM->SNZCR;
    /* Set the software standby mode. (step1) */
    SYSTEM->SBYCR_b.SSBYMP  = 0U;

    /* Set the software standby mode. (step2) */
    SYSTEM->SBYCR_b.SSBY    = 1U;

    /* Set the software standby mode. (step3) */
    SYSTEM->DPSBYCR_b.DPSBY = 0U;

    /* Disable the snooze mode. */
    SYSTEM->SNZCR_b.SNZE    = 0U;

    /* Transition from normal mode to Boost mode. */
    SYSTEM->PWSTCR = 0x05U;

    /* Returns an error because the PWSTCR.PWST[2:0] bits could not be modified. */
    if(0x05U != SYSTEM->PWSTCR)
    {
		SYSTEM->SBYCR = sbycr;
		SYSTEM->DPSBYCR = dpsbycr;
		SYSTEM->SNZCR = snzcr;
        return false;
    }

    /* Execute WFE instruction */
    __WFE();

    /* Wait the transition from normal mode to Boost mode. */
	uint32_t retry = RE01_OSC_WAIT_COUNT;
	while (!SYSTEM->PWSTF_b.BOOSTM && retry > 0)
	{
		retry--;
	}
	bool ready = SYSTEM->PWSTF_b.BOOSTM != 0;
	SYSTEM->SBYCR = sbycr;
	SYSTEM->DPSBYCR = dpsbycr;
	SYSTEM->SNZCR = snzcr;
	return ready;
}

// USB operation requires PLL clock source
// Target PLL for 48MHz operating for compatibility with USB
// PLL operation can only work in Boost mode
uint32_t ConfigPLL(uint32_t SrcFreq)
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
	uint32_t div = 1 << ((SYSTEM->SCKDIVCR & SYSTEM_SCKDIVCR_ICK_Msk) >> SYSTEM_SCKDIVCR_ICK_Pos);

	switch (clksrc)
	{
		case SYSTEM_SCKSCR_CKSEL_HOCO:
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
			SystemCoreClock = 2000000UL;
			break;
		case SYSTEM_SCKSCR_CKSEL_LOCO:
		case SYSTEM_SCKSCR_CKSEL_SCO:
			SystemCoreClock = 32768UL;
			break;
		case SYSTEM_SCKSCR_CKSEL_MCO:
			SystemCoreClock = g_McuOsc.CoreOsc.Freq;
			break;
		case SYSTEM_SCKSCR_CKSEL_PLL:
			SystemCoreClock = (uint64_t)g_McuOsc.CoreOsc.Freq * (SYSTEM->PLLCCR_b.PLLMUL + 1) /
							  (SYSTEM->PLLCCR_b.PLIDIV + 1);
			break;
		default:
			assert(0);
	}

	s_PeriphSrcFreq = SystemCoreClock;

	SystemCoreClock /= div;

	// Reporting clocks must not change divider or flash settings.
}


void SystemInit(void)
{
    SYSTEM->PRCR = 0xA503U;

	FLASH->FLWT = 1;
	// Move off HOCO/PLL before stopping or reprogramming either oscillator.
	SYSTEM->MOCOCR = 0;
	SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MOCO;
	(void)SYSTEM->SCKSCR;
	// PCLKA follows ICLK. PCLKB is limited to 32 MHz, even in boost mode.
	SYSTEM->SCKDIVCR = (SYSTEM->SCKDIVCR & ~(SYSTEM_SCKDIVCR_ICK_Msk | SYSTEM_SCKDIVCR_PCKB_Msk)) |
						  (1UL << SYSTEM_SCKDIVCR_PCKB_Pos);

    if (g_McuOsc.CoreOsc.Type == OSC_TYPE_RC)
	{
    	if (g_McuOsc.bUSBClk)
    	{
    		// USB requires external crystal.
    		// Disable USB support when using RC
    		g_McuOsc.bUSBClk = false;
    	}

    	if (g_McuOsc.CoreOsc.Freq <= 2000000UL)
    	{
			SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MOCO;
    	}
    	else
    	{
    		uint8_t hcfrq = 0;
    		if (g_McuOsc.CoreOsc.Freq <= 24000000UL)
    		{

    		}
    		else if (g_McuOsc.CoreOsc.Freq <= 32000000UL)
    		{
    			hcfrq = 1;
    		}
    		else if (g_McuOsc.CoreOsc.Freq <= 48000000UL)
    		{
    			hcfrq = 2;
    		}
    		else
    		{
    			hcfrq = 3;
    		}

    		// Freq higher than 32MHz requires boost mode
			if (g_McuOsc.CoreOsc.Freq > 32000000)
			{
				if (!EnterBoostMode())
				{
					goto init_done;
				}
			}

    		SYSTEM->HOCOCR = 1;	// Stop HOCO
			if (!Re01WaitOsc(SYSTEM_OSCSF_HOCOSF_Msk, false))
			{
				goto init_done;
			}
			SYSTEM->HOCOMCR = hcfrq;
    		SYSTEM->HOCOCR = 0;	// Start HOCO
			if (!Re01WaitOsc(SYSTEM_OSCSF_HOCOSF_Msk, true))
			{
				goto init_done;
			}

			SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_HOCO;
    	}
	}
	else
	{
		// Main clock range 8-32MHz
		if (g_McuOsc.CoreOsc.Freq < 8000000UL || g_McuOsc.CoreOsc.Freq > 32000000UL)
		{
			g_McuOsc.bUSBClk = false;
			goto init_done;
		}

		if (g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO)
		{
			// External input clock
		    SYSTEM->MOMCR = SYSTEM_MOMCR_OSCLPEN_Msk | (4 << SYSTEM_MOMCR_MODRV_Pos) | SYSTEM_MOMCR_MOSEL_Msk;
		}
		else
		{
			// Crystal
		    SYSTEM->MOMCR = SYSTEM_MOMCR_OSCLPEN_Msk | (4 << SYSTEM_MOMCR_MODRV_Pos);
		}

		SYSTEM->MOSCCR = 0;	// Start main clock oscillator

		if (!Re01WaitOsc(SYSTEM_OSCSF_MOSCSF_Msk, true))
		{
			g_McuOsc.bUSBClk = false;
			goto init_done;
		}

		uint32_t pllfreq = ConfigPLL(g_McuOsc.CoreOsc.Freq);
		if (pllfreq != 0)
		{
			SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_PLL;
		}
		else
		{
			// Keep a live main clock when PLL/boost setup fails.
			SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MCO;
			g_McuOsc.bUSBClk = false;
		}
	}

init_done:
    if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC)
    {
    	SYSTEM->LOCOCR = 0;
    }
    else
    {
    	SYSTEM->SOSCCR = 0;	// Enable 32KHz crystal
    }

    SYSTEM->PRCR = 0xA500U;

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
		uint32_t div = 1 << ((SYSTEM->SCKDIVCR & SYSTEM_SCKDIVCR_PCKB_Msk) >> SYSTEM_SCKDIVCR_PCKB_Pos);
		clk = s_PeriphSrcFreq / div;
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
	uint32_t div = 0;
	while (div < 6 && (s_PeriphSrcFreq >> div) > Freq)
	{
		div++;
	}
	if ((s_PeriphSrcFreq >> div) > Freq)
	{
		return 0;
	}
	uint32_t reg = SYSTEM->SCKDIVCR;
	uint32_t ick = (reg & SYSTEM_SCKDIVCR_ICK_Msk) >> SYSTEM_SCKDIVCR_ICK_Pos;
	uint32_t pckb = (reg & SYSTEM_SCKDIVCR_PCKB_Msk) >> SYSTEM_SCKDIVCR_PCKB_Pos;
	if (Idx == 0)
	{
		ick = div;
		if (pckb < ick)
		{
			pckb = ick;
		}
		// Set wait states before increasing the CPU clock.
		if ((s_PeriphSrcFreq >> ick) > SystemCoreClock)
		{
			SetFlashWaitState(s_PeriphSrcFreq >> ick);
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
	uint16_t protection = SYSTEM->PRCR & 0xFU;
	SYSTEM->PRCR = 0xA501U | protection;
	SYSTEM->SCKDIVCR = (reg & ~(SYSTEM_SCKDIVCR_ICK_Msk | SYSTEM_SCKDIVCR_PCKB_Msk)) |
						  (ick << SYSTEM_SCKDIVCR_ICK_Pos) | (pckb << SYSTEM_SCKDIVCR_PCKB_Pos);
	SYSTEM->PRCR = 0xA500U | protection;
	SystemCoreClockUpdate();
	SetFlashWaitState(SystemCoreClock);
	return SystemPeriphClockGet(Idx);
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}
