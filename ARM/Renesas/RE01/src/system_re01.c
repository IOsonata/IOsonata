/**-------------------------------------------------------------------------
@file	system_re01.c

@brief	Implementation of CMSIS SystemInit for Renesas RE01 Device Series

Note: 	USB operation requires PLL as clock source running at 48MHz
		PLL operation requires boost mode and main clock (crystal or ext osc)
		Therefore main clock source (8-32MHz) must be chosen to allows PLL to
		generate 48MHz

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

#include "RE01xxx.h"
#include "coredev/system_core_clock.h"

#define SYSTEM_CORE_CLOCK_MAX			64000000UL	// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(34UL)		// TODO: Adjustment value for nanosec delay

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
	OSC_TYPE_RC,
	64000000,
	OSC_TYPE_RC,
	32768
};

static uint32_t s_PeriphSrcFreq = 0;

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

// USB operation requires PLL clock source
// Target PLL for 48MHz operating for compatibility with USB
// PLL operation can only work in Boost mode
uint32_t ConfigPLL(uint32_t SrcFreq)
{
	// Make sure PLL is stopped
	SYSTEM->PLLCR = SYSTEM_PLLCR_PLLSTP_Msk;

	// TODO: Enter Boost mode

	for (int div = 1; div < 5; div++)
	{
		uint32_t divfreq = SrcFreq / div;
		for (int mul = 2; mul < 9; mul++)
		{
			// PLL Freq = (SrcFreq / div) * mul;
			uint32_t f = divfreq * mul;

			if (f == 48000000UL)
			{
				SYSTEM->PLLCCR = ((div - 1) << SYSTEM_PLLCCR_PLIDIV_Pos) |
								 ((mul - 1) << SYSTEM_PLLCCR_PLLMUL_Pos);
				SYSTEM->PLLCR = 0;	// Start PLL

				while ((SYSTEM->OSCSF & SYSTEM_OSCSF_PLLSF_Msk) == 0);

				return 48000000UL;
			}
		}
	}

	return 0;
}

void SystemCoreClockUpdate(void)
{
	uint8_t clksrc = SYSTEM->SCKSCR;
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
			SystemCoreClock = g_McuOsc.HFFreq;
			break;
		case SYSTEM_SCKSCR_CKSEL_PLL:
			SystemCoreClock = (g_McuOsc.HFFreq / (SYSTEM->PLLCCR_b.PLIDIV + 1)) * (SYSTEM->PLLCCR_b.PLLMUL + 1);
			break;
		default:
			assert(0);
	}

	s_PeriphSrcFreq = SystemCoreClock;

	SystemCoreClock /= div;

	// Update Flash wait state to current core freq.
	SetFlashWaitState(SystemCoreClock);
	SystemPeriphClockSet(0, SystemCoreClock);
	SystemPeriphClockSet(1, s_PeriphSrcFreq);
}


void SystemInit(void)
{
    SYSTEM->PRCR = 0xA503U;

    if (g_McuOsc.HFType == OSC_TYPE_RC)
	{
    	if (g_McuOsc.HFFreq <= 2000000UL)
    	{
			SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_MOCO;
    	}
    	else
    	{
    		uint8_t hcfrq = 0;
    		if (g_McuOsc.HFFreq <= 24000000UL)
    		{

    		}
    		else if (g_McuOsc.HFFreq <= 32000000UL)
    		{
    			hcfrq = 1;
    		}
    		else if (g_McuOsc.HFFreq <= 48000000UL)
    		{
    			hcfrq = 2;
    		}
    		else
    		{
    			hcfrq = 3;
    		}

    		SYSTEM->HOCOMCR = hcfrq;
    		SYSTEM->HOCOCR = 0;	// Start HOCO
			while ((SYSTEM->OSCSF & SYSTEM_OSCSF_HOCOSF_Msk) == 0);

			SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_HOCO;
    	}
	}
	else
	{
		// Main clock range 8-32MHz

		if (g_McuOsc.HFType == OSC_TYPE_TCXO)
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

		while ((SYSTEM->OSCSF & SYSTEM_OSCSF_MOSCSF_Msk) == 0);

		uint32_t pllfreq = ConfigPLL(g_McuOsc.HFFreq);
		assert(pllfreq==48000000UL);

		// Select source clock PLL 48MHz
		SYSTEM->SCKSCR = SYSTEM_SCKSCR_CKSEL_PLL;
	}

    if (g_McuOsc.LFType == OSC_TYPE_RC)
    {
    	SYSTEM->LOCOCR = 0;
    }
    else
    {
    	SYSTEM->SOSCCR = 0;	// Enable 32KHz crystal
    }

	SystemCoreClockUpdate();
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
	uint32_t clk = 0;

	if (Idx == 0)
	{
		clk =  SystemCoreClock;
	}
	else if (Idx == 1 && Freq > 0)
	{
		uint32_t div = s_PeriphSrcFreq / Freq;

		if (div > 0)
		{
			SYSTEM->SCKDIVCR_b.PCKB = div - 1;
			clk = s_PeriphSrcFreq / div;
		}
		else
		{
			clk = s_PeriphSrcFreq;
		}
	}

	return clk;
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}
