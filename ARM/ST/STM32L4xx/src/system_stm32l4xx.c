/**-------------------------------------------------------------------------
@file	system_stm32l4xx.c

@brief	Implementation of CMSIS SystemInit for STM32L4xx Device Series


@author	Hoang Nguyen Hoan
@date	June 5, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------*/

#include <stdbool.h>
#include <stdlib.h>

#include "stm32l4xx.h"
#include "coredev/system_core_clock.h"

#define DEFAULT_RC_FREQ		48000000
#define XTAL_FREQ			16000000

#define SYSTEM_CORE_CLOCK_MAX			80000000UL	// TODO: Adjust value for CPU with fixed core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(40UL)		// TODO: Adjustment value for nanosec delay

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK_MAX;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;

static OSC_TYPE s_Clksrc = OSC_TYPE_RC;
static uint32_t s_ClkSrcFreq = DEFAULT_RC_FREQ;
static const uint32_t s_MsiClkRange[] = {
	100000, 200000, 400000, 800000, 1000000, 2000000, 4000000, 8000000, 16000000, 24000000, 32000000, 48000000
};

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
	int retval = sizeof(s_MsiClkRange) / sizeof(uint32_t) - 1;

	do {
		if (s_MsiClkRange[retval] <= Freq)
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

	for (int m = 1; m <= 8; m++)
	{
		uint32_t clk = SrcFreq / m;

		for (int n = 8; n <= 86 && clk >= 4000000 && clk <= 16000000; n++)
		{
			uint32_t vco = clk * n;

			for (int r = 2; r <= 8 && vco >= 64000000 && vco <= 344000000; r += 2)
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
		SystemCoreClock = s_MsiClkRange[ridx];
	}
	else if (pllcfgr & RCC_PLLCFGR_PLLSRC_HSE)
	{
		SystemCoreClock = s_ClkSrcFreq;
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

	// Update Flash wait state to current core freq.
	SetFlashWaitState(SystemCoreClock);
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t cfgr = 0;
	uint32_t pllcfgr = 0;

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

	s_Clksrc = ClkSrc;

	switch (ClkSrc)
	{
		case OSC_TYPE_TCXO:
			RCC->CR |= RCC_CR_HSEBYP;
		case OSC_TYPE_XTAL:
			s_ClkSrcFreq = OscFreq;

			RCC->CR |= RCC_CR_HSION;

			while ((RCC->CR & RCC_CR_HSIRDY) == 0);

			//RCC->CR |= RCC_CR_CSSON;
			RCC->CR |= RCC_CR_HSEON;

			while ((RCC->CR & RCC_CR_HSERDY) == 0);

			pllcfgr |= RCC_PLLCFGR_PLLSRC_HSE;

		default:	// MSI
			s_ClkSrcFreq = DEFAULT_RC_FREQ;
			pllcfgr |= RCC_PLLCFGR_PLLSRC_MSI;
	}

	pllcfgr |= FindPllCfg(s_ClkSrcFreq);

	RCC->PLLCFGR = pllcfgr;

	RCC->CR |= RCC_CR_PLLON;
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLREN;

	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	RCC->CFGR = cfgr;

	SystemCoreClockUpdate();

	return true;//SYSTEM_CORE_CLOCK;
}

void SystemInit(void)
{
	SystemCoreClockSelect(OSC_TYPE_RC, DEFAULT_RC_FREQ);
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
		RCC->CFGR &= RCC_CFGR_PPRE2_Msk;
		RCC->CFGR |= ppreval << RCC_CFGR_PPRE2_Pos;
	}
	else
	{
		RCC->CFGR &= RCC_CFGR_PPRE1_Msk;
		RCC->CFGR |= ppreval << RCC_CFGR_PPRE1_Pos;
	}

	return f;
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}
