/**-------------------------------------------------------------------------
@file	system_stm32h7xx.c

@brief	STM32H753 system clock.

400 MHz CPU, 200 MHz AHB, 100 MHz APB1..4, voltage scale 1.
HSI64 avoids a dependency on the Nucleo ST-LINK MCO solder bridges.
Data and stack use DTCM. D-cache is left disabled during bring-up.

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

#include "stm32h7xx.h"
#include "coredev/system_core_clock.h"

#define HSI_FREQ 64000000UL
#define CLOCK_POLLS 2000000UL

extern void (* const __Vectors[])(void);
uint32_t SystemCoreClock = HSI_FREQ;
uint32_t SystemD2Clock = HSI_FREQ;
const uint8_t D1CorePrescTable[16] = {
	0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9
};
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
	uint32_t hsi = HSI_FREQ >> ((RCC->CR & RCC_CR_HSIDIV) >> RCC_CR_HSIDIV_Pos);
	uint32_t freq = hsi;
	uint32_t sw = RCC->CFGR & RCC_CFGR_SWS;
	if (sw == RCC_CFGR_SWS_CSI)
	{
		freq = 4000000UL;
	}
	else if (sw == RCC_CFGR_SWS_HSE)
	{
		freq = g_McuOsc.CoreOsc.Freq;
	}
	else if (sw == RCC_CFGR_SWS_PLL1)
	{
		uint32_t src = RCC->PLLCKSELR & RCC_PLLCKSELR_PLLSRC;
		uint32_t input = src == RCC_PLLCKSELR_PLLSRC_HSE ? g_McuOsc.CoreOsc.Freq :
						 src == RCC_PLLCKSELR_PLLSRC_CSI ? 4000000UL : hsi;
		uint32_t m = (RCC->PLLCKSELR & RCC_PLLCKSELR_DIVM1) >> RCC_PLLCKSELR_DIVM1_Pos;
		uint32_t n = ((RCC->PLL1DIVR & RCC_PLL1DIVR_N1) >> RCC_PLL1DIVR_N1_Pos) + 1U;
		uint32_t p = ((RCC->PLL1DIVR & RCC_PLL1DIVR_P1) >> RCC_PLL1DIVR_P1_Pos) + 1U;
		uint32_t frac = (RCC->PLLCFGR & RCC_PLLCFGR_PLL1FRACEN) ?
			(RCC->PLL1FRACR & RCC_PLL1FRACR_FRACN1) >> RCC_PLL1FRACR_FRACN1_Pos : 0U;
		freq = m == 0U ? 0U : (uint32_t)(((uint64_t)input * (n * 8192U + frac)) / (m * p * 8192U));
	}
	SystemCoreClock = freq >> D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_D1CPRE) >> RCC_D1CFGR_D1CPRE_Pos];
	SystemD2Clock = SystemCoreClock >> D1CorePrescTable[(RCC->D1CFGR & RCC_D1CFGR_HPRE) >> RCC_D1CFGR_HPRE_Pos];
}

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= (3UL << 20) | (3UL << 22);
	__DSB();
	__ISB();
#endif
	SCB->VTOR = (uint32_t)__Vectors;
	// Select the H753 internal LDO while supply configuration is unlocked.
	// NUCLEO-H753ZI supplies VCAP through this regulator.
	if (PWR->CR3 & PWR_CR3_SCUEN)
	{
		PWR->CR3 = (PWR->CR3 & ~(PWR_CR3_SCUEN | PWR_CR3_BYPASS)) | PWR_CR3_LDOEN;
	}
	if (WaitFlag(&PWR->CSR1, PWR_CSR1_ACTVOSRDY, PWR_CSR1_ACTVOSRDY) == false)
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
	RCC->CR &= ~RCC_CR_HSIDIV;
	RCC->CR &= ~RCC_CR_PLL1ON;
	if (WaitFlag(&RCC->CR, RCC_CR_PLL1RDY, 0) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	PWR->D3CR = (PWR->D3CR & ~PWR_D3CR_VOS) | PWR_D3CR_VOS;
	if (WaitFlag(&PWR->D3CR, PWR_D3CR_VOSRDY, PWR_D3CR_VOSRDY) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	// VOS1, 200 MHz AXI: four wait states and two programming cycles.
	FLASH->ACR = (FLASH->ACR & ~(FLASH_ACR_LATENCY | FLASH_ACR_WRHIGHFREQ)) |
		4U | (2U << FLASH_ACR_WRHIGHFREQ_Pos);
	if (WaitFlag(&FLASH->ACR, FLASH_ACR_LATENCY, 4U) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->D1CFGR = RCC_D1CFGR_HPRE_DIV2 | (4U << RCC_D1CFGR_D1PPRE_Pos);
	RCC->D2CFGR = (4U << RCC_D2CFGR_D2PPRE1_Pos) | (4U << RCC_D2CFGR_D2PPRE2_Pos);
	RCC->D3CFGR = 4U << RCC_D3CFGR_D3PPRE_Pos;
	RCC->CFGR &= ~RCC_CFGR_TIMPRE;
	// HSI64 / 32 = 2 MHz; VCO = 800 MHz; PLL1P = 400 MHz.
	RCC->PLLCKSELR = (RCC->PLLCKSELR & ~(RCC_PLLCKSELR_PLLSRC | RCC_PLLCKSELR_DIVM1)) |
		RCC_PLLCKSELR_PLLSRC_HSI | (32U << RCC_PLLCKSELR_DIVM1_Pos);
	RCC->PLL1DIVR = (399U << RCC_PLL1DIVR_N1_Pos) | (1U << RCC_PLL1DIVR_P1_Pos) |
		(7U << RCC_PLL1DIVR_Q1_Pos) | (1U << RCC_PLL1DIVR_R1_Pos);
	RCC->PLL1FRACR = 0;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~(RCC_PLLCFGR_PLL1RGE | RCC_PLLCFGR_PLL1VCOSEL |
		RCC_PLLCFGR_PLL1FRACEN | RCC_PLLCFGR_DIVQ1EN | RCC_PLLCFGR_DIVR1EN)) |
		RCC_PLLCFGR_PLL1RGE_0 | RCC_PLLCFGR_DIVP1EN;
	RCC->CR |= RCC_CR_PLL1ON;
	if (WaitFlag(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY))
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL1;
		WaitFlag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL1);
	}
	SystemCoreClockUpdate();
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

static volatile uint32_t *PeriphPrescaler(int Idx, uint32_t *pPos)
{
	switch (Idx)
	{
		case 0:
			*pPos = RCC_D2CFGR_D2PPRE1_Pos;
			return &RCC->D2CFGR;
		case 1:
			*pPos = RCC_D2CFGR_D2PPRE2_Pos;
			return &RCC->D2CFGR;
		case 2:
			*pPos = RCC_D1CFGR_D1PPRE_Pos;
			return &RCC->D1CFGR;
		case 3:
			*pPos = RCC_D3CFGR_D3PPRE_Pos;
			return &RCC->D3CFGR;
		default:
			return 0;
	}
}

uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t pos;
	volatile uint32_t *reg = PeriphPrescaler(Idx, &pos);
	if (reg == 0)
	{
		return 0;
	}
	uint32_t pre = (*reg >> pos) & 7U;
	return SystemD2Clock >> (pre < 4U ? 0U : pre - 3U);
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	uint32_t pos;
	volatile uint32_t *reg = PeriphPrescaler(Idx, &pos);
	if (reg == 0)
	{
		return 0;
	}
	uint32_t best = 4U;
	uint32_t error = UINT32_MAX;
	for (uint32_t shift = 0; shift <= 4U; shift++)
	{
		uint32_t rate = SystemD2Clock >> shift;
		uint32_t diff = rate > Freq ? rate - Freq : Freq - rate;
		if (rate <= 100000000UL && diff < error)
		{
			best = shift;
			error = diff;
		}
	}
	uint32_t pre = best == 0U ? 0U : best + 3U;
	*reg = (*reg & ~(7U << pos)) | (pre << pos);
	return SystemD2Clock >> best;
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	// This initial port uses HSI64 only.
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
