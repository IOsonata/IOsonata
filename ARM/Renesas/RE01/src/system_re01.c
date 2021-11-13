/**-------------------------------------------------------------------------
@file	system_re01.c

@brief	Implementation of CMSIS SystemInit for Renesas RE01 Device Series


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

#include "RE01xxx.h"
#include "coredev/system_core_clock.h"

#define DEFAULT_RC_FREQ		32000000UL
#define XTAL_FREQ			32000000UL

#define OSC_FREQ_MAX		32000000UL		// Max oscillator freq internal or external

#define SYSTEM_CORE_CLOCK_MAX			64000000UL	// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(34UL)	// TODO: Adjustment value for nanosec delay

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK_MAX;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;

// Overload this variable in application firmware to change oscillator
__WEAK MCU_OSC g_McuOsc = {
	OSC_TYPE_RC,
	64000000,
	OSC_TYPE_XTAL,
	32768
};

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
}

uint32_t FindPllCfg(uint32_t SrcFreq)
{
	return 0;
}

void SystemCoreClockUpdate(void)
{
#if 0
	uint32_t cfgr = RCC->CFGR;
	uint32_t pllcfgr = RCC->PLLCFGR;
	uint32_t clk = DEFAULT_RC_FREQ;

	if (pllcfgr & RCC_PLLCFGR_PLLSRC_MSI)
	{
		int ridx = (RCC->CR & RCC_CR_MSIRANGE_Msk) >> RCC_CR_MSIRANGE_Pos;
		SystemCoreClock = MSIRangeTable[ridx];
	}
	else if (pllcfgr & RCC_PLLCFGR_PLLSRC_HSE)
	{
		SystemCoreClock = g_McuOsc.HFFreq;//s_ClkSrcFreq;
	}
	else if (pllcfgr & RCC_PLLCFGR_PLLSRC_HSI)
	{
		SystemCoreClock = 16000000;
	}

	if (cfgr & RCC_CFGR_SWS_PLL)
	{
		uint32_t m = ((pllcfgr & RCC_PLLCFGR_PLLM_Msk) >> RCC_PLLCFGR_PLLM_Pos) + 1;
		uint32_t n = (pllcfgr & RCC_PLLCFGR_PLLN_Msk) >> RCC_PLLCFGR_PLLN_Pos;
		uint32_t r = (((pllcfgr & RCC_PLLCFGR_PLLR_Msk) >> RCC_PLLCFGR_PLLR_Pos) + 1) << 1;
		SystemCoreClock = (SystemCoreClock * n / (m * r));
	}

	if (SystemCoreClock > 80000000)
	{
		PWR->CR5 &= ~PWR_CR5_R1MODE;
	}
#endif
	// Update Flash wait state to current core freq.
	SetFlashWaitState(SystemCoreClock);
	SystemPeriphClockSet(0, SystemCoreClock >> 1);
	SystemPeriphClockSet(1, SystemCoreClock >> 1);
}


void SystemInit(void)
{
	if (g_McuOsc.HFType == OSC_TYPE_RC)
	{
		switch (g_McuOsc.HFFreq)
		{
			case 2000000:	// 2MHz
				break;
			case 24000000:	// 24MHz
				break;
			case 32000000:	// 32MHz
				break;
			case 48000000:	// 48MHz
				break;
			case 64000000:	// 64MHz
			default :
				break;
		}
	}
	else
	{

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
	//uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;

	//return tmp & 8 ? SystemCoreClock >> ((tmp & 7) + 1) : SystemCoreClock;
	return SystemCoreClock;
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
	if (Idx < 0 || Idx > 1)
	{
		return 0;
	}

	return SystemCoreClock;
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
	if (Idx < 0 || Idx > 1 || Freq > SYSTEM_CORE_CLOCK_MAX)
	{
		return 0;
	}
#if 0
	uint32_t div = (SystemCoreClock + (Freq >> 1))/ Freq;
	uint32_t f = 0;
	uint32_t ppreval = 0;

	if (div < 2)
	{
		// Div 1
		f = SystemCoreClock;
	}
	else if (div < 4)
	{
		// Div 2
		ppreval = 4;
		f = SystemCoreClock >> 1;
	}
	else if (div < 8)
	{
		// Div 4
		ppreval = 5;
		f = SystemCoreClock >> 2;
	}
	else if (div < 16)
	{
		// Div 8
		ppreval = 6;
		f = SystemCoreClock >> 3;
	}
	else
	{
		// Div 16
		ppreval = 7;
		f = SystemCoreClock >> 4;
	}

	if (Idx == 1)
	{
		RCC->CFGR &= ~RCC_CFGR_PPRE2_Msk;
		RCC->CFGR |= ppreval << RCC_CFGR_PPRE2_Pos;
	}
	else
	{
		RCC->CFGR &= ~RCC_CFGR_PPRE1_Msk;
		RCC->CFGR |= ppreval << RCC_CFGR_PPRE1_Pos;
	}
#endif
	return 0;
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}
