/**-------------------------------------------------------------------------
@file	system_stm32l4plus.c

@brief	Implementation of CMSIS SystemInit for STM32L4+ Device Series


@author	Hoang Nguyen Hoan
@date	June 5, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include "stm32l4xx.h"
#include "coredev/system_core_clock.h"

#define DEFAULT_RC_FREQ		48000000
#define XTAL_FREQ			16000000

#define HSE_OSC_FREQ_MIN	4000000UL		// 4 MHz
#define HSE_OSC_FREQ_MAX	48000000UL		// 48 MHz

#define HSI_RC_OSC_FREQ		16000000UL		// 16 MHz

#define MSI_RC_OSC_FREQ_MIN	100000UL		// 100 KHz
#define MSI_RC_OSC_FREQ_MAX	48000000UL		// 48 MHz

#define OSC_FREQ_MAX		48000000UL		// Max oscillator freq internal or external

#define SYSTEM_CORE_CLOCK_MAX			120000000UL	// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(34UL)	// TODO: Adjustment value for nanosec delay

#define PLLM_DIV_MAX		16
#define PLLN_MUL_MIN		8
#define PLLN_MUL_MAX		128
#define PLLR_DIV_MAX		8
#define VCO_FREQ_MIN		64000000
#define VCO_FREQ_MAX		344000000

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK_MAX;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;

// Keep this name for compatibility with STM code
const uint32_t MSIRangeTable[] = {
	100000, 200000, 400000, 800000, 1000000, 2000000, 4000000, 8000000, 16000000, 24000000, 32000000, 48000000
};
const uint8_t AHBPrescTable[16] = {
	0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U
};
const uint8_t APBPrescTable[8] = {
	0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U
};

// Overload this variable in application firmware to change oscillator
__WEAK MCU_OSC g_McuOsc = {
	OSC_TYPE_RC,
	48000000,
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
	uint32_t tmp = FLASH->ACR & ~FLASH_ACR_LATENCY;

	if (CoreFreq <= 16000000)
	{
		tmp |= FLASH_ACR_LATENCY_0WS;
	}
	else if (CoreFreq <= 32000000)
	{
		tmp |= FLASH_ACR_LATENCY_1WS;
	}
	else if (CoreFreq <= 48000000)
	{
		tmp |= FLASH_ACR_LATENCY_2WS;
	}
	else if (CoreFreq <= 64000000)
	{
		tmp |= FLASH_ACR_LATENCY_3WS;
	}
	else
	{
		tmp |= FLASH_ACR_LATENCY_4WS;
	}

	FLASH->ACR = tmp;
}

uint32_t GetMsiRange(uint32_t Freq)
{
	int retval = sizeof(MSIRangeTable) / sizeof(uint32_t) - 1;

	do {
		if (MSIRangeTable[retval] <= Freq)
		{
			break;
		}
	} while (retval >= 0);

	return (uint32_t)retval << RCC_CR_MSIRANGE_Pos;
}

uint32_t FindPllCfg(uint32_t SrcFreq)
{
	//
	// VCO = ClkIn * N/M
	// PLLCLK = VCO / R
	// VCO must be within 64-344 MHz

	// find matching clock mul/div
	uint32_t pllm = 1;
	int32_t cdiff = SYSTEM_CORE_CLOCK_MAX;
	uint32_t plln = 8;
	uint32_t pllr = 0;
	uint32_t pllcfgr = 0;

	for (int m = 1; m <= PLLM_DIV_MAX; m++)
	{
		uint32_t clk = SrcFreq / m;

		for (int n = PLLN_MUL_MIN; n <= PLLN_MUL_MAX && clk >= 4000000 && clk <= 48000000; n++)
		{
			uint32_t vco = clk * n;

			for (int r = 2; r <= PLLR_DIV_MAX && vco >= (SYSTEM_CORE_CLOCK_MAX << 1) && vco <= VCO_FREQ_MAX; r += 2)
			{
				uint32_t sysclk = vco / r;

				if (sysclk <= SYSTEM_CORE_CLOCK_MAX)
				{
					int diff = SYSTEM_CORE_CLOCK_MAX - sysclk;
					if (diff < cdiff)
					{
						cdiff = diff;
						pllm = m - 1;
						plln = n;
						pllr = (r >> 1) - 1;
					}
				}
			}
		}
	}

	pllcfgr = (pllm << RCC_PLLCFGR_PLLM_Pos) | (plln << RCC_PLLCFGR_PLLN_Pos) |
			  (pllr << RCC_PLLCFGR_PLLP_Pos) | (pllr << RCC_PLLCFGR_PLLQ_Pos) |
			  (pllr << RCC_PLLCFGR_PLLR_Pos);
	return pllcfgr;
}

void SystemCoreClockUpdate(void)
{
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
	// Update Flash wait state to current core freq.
	SetFlashWaitState(SystemCoreClock);
	SystemPeriphClockSet(0, SystemCoreClock >> 1);
	SystemPeriphClockSet(1, SystemCoreClock >> 1);
}

/**
 * @brief	Select core clock oscillator type
 *
 * @param	ClkSrc : Clock source selection
 *						OSC_TYPE_RC - Internal RC
 *						OSC_TYPE_XTAL - External crystal
 *						OSC_TYPE_CTXO -	External oscillator
 * @param	OscFreq : Oscillator frequency
 *
 * @return	true - success
 *
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (OscFreq > OSC_FREQ_MAX)
	{
		return false;
	}

	if (ClkSrc == OSC_TYPE_RC)
	{
		if (OscFreq < MSI_RC_OSC_FREQ_MIN)
		{
			return false;
		}
	}
	else
	{
		if (OscFreq < HSE_OSC_FREQ_MIN)
		{
			return false;
		}
	}

	g_McuOsc.HFType = ClkSrc;
	g_McuOsc.HFFreq = OscFreq;

	SystemInit();

	return true;//SYSTEM_CORE_CLOCK;
}

void SystemInit(void)
{
	uint32_t cfgr = 0;
	uint32_t pllcfgr = 0;

#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
  /* enable FPU if available and used */
  SCB->CPACR |= ((3UL << 10*2) |             /* set CP10 Full Access               */
                 (3UL << 11*2)  );           /* set CP11 Full Access               */
#endif

	RCC->CFGR = 0;
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLREN;

	RCC->CR |= RCC_CR_MSION;
	RCC->CR &= ~(RCC_CR_PLLON | RCC_CR_HSEBYP | RCC_CR_CSSON | RCC_CR_HSEON);
	while ((RCC->CR & RCC_CR_PLLRDY) != 0);
	RCC->CIER = 0;

	// Flash wait state to max core freq.
	SetFlashWaitState(SYSTEM_CORE_CLOCK_MAX);

	// internal default 48MHz RC, ready for USB clock
	RCC->CR &= ~RCC_CR_MSIRANGE_Msk;
	RCC->CR |= GetMsiRange(DEFAULT_RC_FREQ) | RCC_CR_MSIRGSEL;

	// Select MSI 48MHz USB clock
	RCC->CCIPR |= RCC_CCIPR_CLK48SEL_Msk;

	// Always select PLL for max core frequency
	cfgr |= RCC_CFGR_SW_PLL;

	switch (g_McuOsc.HFType)
	{
		case OSC_TYPE_TCXO:
			RCC->CR |= RCC_CR_HSEBYP;

		case OSC_TYPE_XTAL:
			RCC->CR |= RCC_CR_HSION;

			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			RCC->CR |= RCC_CR_HSEON;

			while ((RCC->CR & RCC_CR_HSERDY) == 0);

			pllcfgr |= RCC_PLLCFGR_PLLSRC_HSE;

		default:	// MSI
			pllcfgr |= RCC_PLLCFGR_PLLSRC_MSI;
	}

	pllcfgr |= FindPllCfg(g_McuOsc.HFFreq);

	RCC->PLLCFGR = pllcfgr;

	RCC->CR |= RCC_CR_PLLON;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	RCC->CFGR = cfgr;

	if (g_McuOsc.LFType == OSC_TYPE_XTAL)
	{
		if ((RCC->APB1ENR1 & RCC_APB1ENR1_PWREN) == 0)
		{
			RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
			while ((RCC->APB1ENR1 & RCC_APB1ENR1_PWREN) == 0);
		}

		if ((PWR->CR1 & PWR_CR1_DBP) == 0)
		{
			PWR->CR1 |= PWR_CR1_DBP;
			while ((PWR->CR1 & PWR_CR1_DBP) == 0);
		}

		RCC->BDCR |= RCC_BDCR_BDRST;
		RCC->BDCR &= ~RCC_BDCR_BDRST;

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx)
		RCC->BDCR |= RCC_BDCR_LSESYSDIS;
#endif
		RCC->BDCR &= ~RCC_BDCR_RTCSEL_Msk;

		RCC->BDCR |= RCC_BDCR_LSEON;

		// if stuck here, board does not have 32768 Hz crystal
		while ((RCC->BDCR & RCC_BDCR_LSERDY) == 0);

#if defined(STM32L4P5xx) || defined(STM32L4Q5xx)
		RCC->BDCR &= ~RCC_BDCR_LSESYSDIS;
#endif

		RCC->BDCR |= RCC_BDCR_RTCSEL_0 | RCC_BDCR_RTCEN;	// RTC source LSE
		RCC->APB1ENR1 &= ~RCC_APB1ENR1_PWREN;
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
	uint32_t tmp = (RCC->CFGR & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos;

	return tmp & 8 ? SystemCoreClock >> ((tmp & 7) + 1) : SystemCoreClock;
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

	uint32_t tmp = 0;

	if (Idx == 1)
	{
		tmp = (RCC->CFGR & RCC_CFGR_PPRE2_Msk) >> RCC_CFGR_PPRE2_Pos;
	}
	else
	{
		tmp = (RCC->CFGR & RCC_CFGR_PPRE1_Msk) >> RCC_CFGR_PPRE1_Pos;
	}

	return tmp & 4 ? SystemCoreClock >> ((tmp & 3) + 1) : SystemCoreClock;
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

	return f;
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}
