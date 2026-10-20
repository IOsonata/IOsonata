/**-------------------------------------------------------------------------
@file	system_stm32l1xx.c

@brief	STM32L152RE: HSI16 * 6 / 3, 32 MHz CPU and APB clocks.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "stm32l1xx.h"
#include "coredev/system_core_clock.h"

#define HSI_FREQ 16000000UL
#ifndef HSE_VALUE
#define HSE_VALUE 8000000UL
#endif
#define CLOCK_POLLS 2000000UL

extern void (* const __Vectors[])(void);
uint32_t SystemCoreClock = 2097152UL;
const uint8_t PLLMulTable[9] = {3, 4, 6, 8, 12, 16, 24, 32, 48};
const uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
const uint8_t APBPrescTable[8] = {0, 0, 0, 0, 1, 2, 3, 4};
__WEAK McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, HSI_FREQ, 10000, 0 },
	{ OSC_TYPE_XTAL, 32768, 20, 0 },
	false
};

static bool WaitFlag(volatile uint32_t *pReg, uint32_t Mask, uint32_t Value)
{
	for (uint32_t i = 0; i < CLOCK_POLLS; i++)
	{
		if ((*pReg & Mask) == Value)
		{
			return true;
		}
	}
	return false;
}

void SystemCoreClockUpdate(void)
{
	uint32_t cfgr = RCC->CFGR;
	uint32_t freq;

	switch (cfgr & RCC_CFGR_SWS)
	{
		case RCC_CFGR_SWS_HSI:
			freq = HSI_FREQ;
			break;
		case RCC_CFGR_SWS_HSE:
			freq = HSE_VALUE;
			break;
		case RCC_CFGR_SWS_PLL:
		{
			uint32_t mul = (cfgr & RCC_CFGR_PLLMUL) >> RCC_CFGR_PLLMUL_Pos;
			uint32_t div = ((cfgr & RCC_CFGR_PLLDIV) >> RCC_CFGR_PLLDIV_Pos) + 1U;
			uint32_t input = (cfgr & RCC_CFGR_PLLSRC) ? HSE_VALUE : HSI_FREQ;

			freq = mul < 9U ? input * PLLMulTable[mul] / div : 0U;
			break;
		}
		default:
			freq = 32768UL << (((RCC->ICSCR & RCC_ICSCR_MSIRANGE) >> RCC_ICSCR_MSIRANGE_Pos) + 1U);
			break;
	}
	SystemCoreClock = freq >> AHBPrescTable[(cfgr & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
}

void SystemInit(void)
{
	SCB->VTOR = (uint32_t)__Vectors;
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;
	if (WaitFlag(&PWR->CSR, PWR_CSR_VOSF, 0) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	PWR->CR = (PWR->CR & ~PWR_CR_VOS) | PWR_CR_VOS_0;
	if (WaitFlag(&PWR->CSR, PWR_CSR_VOSF, 0) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	// 64-bit flash access must be enabled before prefetch and one wait state.
	FLASH->ACR |= FLASH_ACR_ACC64;
	FLASH->ACR |= FLASH_ACR_LATENCY | FLASH_ACR_PRFTEN;
	if (WaitFlag(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CR |= RCC_CR_HSION;
	if (WaitFlag(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
	if (WaitFlag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CR &= ~RCC_CR_PLLON;
	if (WaitFlag(&RCC->CR, RCC_CR_PLLRDY, 0) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CFGR = (RCC->CFGR & ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMUL | RCC_CFGR_PLLDIV |
		RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) | RCC_CFGR_PLLMUL6 | RCC_CFGR_PLLDIV3;
	RCC->CR |= RCC_CR_PLLON;
	if (WaitFlag(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY))
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
		WaitFlag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL);
	}
	SystemCoreClockUpdate();
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	if (Idx < 0 || Idx > 1)
	{
		return 0;
	}
	uint32_t pos = Idx == 0 ? RCC_CFGR_PPRE1_Pos : RCC_CFGR_PPRE2_Pos;
	return SystemCoreClock >> APBPrescTable[(RCC->CFGR >> pos) & 7U];
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	if (Idx < 0 || Idx > 1)
	{
		return 0;
	}
	uint32_t pos = Idx == 0 ? RCC_CFGR_PPRE1_Pos : RCC_CFGR_PPRE2_Pos;
	uint32_t best = 4U;
	uint32_t error = UINT32_MAX;

	for (uint32_t shift = 0; shift <= 4U; shift++)
	{
		uint32_t rate = SystemCoreClock >> shift;
		uint32_t diff = rate > Freq ? rate - Freq : Freq - rate;

		if (diff < error)
		{
			best = shift;
			error = diff;
		}
	}
	uint32_t pre = best == 0U ? 0U : best + 3U;
	RCC->CFGR = (RCC->CFGR & ~(7U << pos)) | (pre << pos);
	return SystemCoreClock >> best;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	// This initial port supports the internal HSI16 source only.
	return ClkSrc == OSC_TYPE_RC && OscFreq == HSI_FREQ;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	(void)ClkSrc;
	(void)OscFreq;
	return false;
}

void SystemOscInit(void)
{
}
