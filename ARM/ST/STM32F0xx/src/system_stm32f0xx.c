/**-------------------------------------------------------------------------
@file	system_stm32f0xx.c

@brief	Implementation of CMSIS SystemInit for STM32F0xx Device Series


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

#include "stm32f0xx.h"
#include "coredev/system_core_clock.h"

#define SYSTEM_CORE_CLOCK				48000000UL		// STM32F0 max core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(93UL)

#define STM32F0XX_HSE_XTAL_EN			false
#define STM32F0XX_HSE_XTAL_FREQ			0

// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, 48000000, 20},
	{OSC_TYPE_RC, 32768, 20},
	false
};

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;

static uint32_t s_XtlFreq = 0;

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}

void SystemCoreClockUpdate (void)
{
	uint32_t cfgr = RCC->CFGR;

	if (cfgr & RCC_CFGR_SW_PLL)
	{
		uint32_t sysclk = 0;

		if (cfgr & RCC_CFGR_PLLSRC_HSE_PREDIV)
		{
			// Crystal / external clock, divided by full 4-bit PREDIV field
			sysclk = s_XtlFreq / ((RCC->CFGR2 & RCC_CFGR2_PREDIV_Msk) + 1);
		}
		else
		{
			// HSI / 2
			sysclk = 4000000;
		}

		uint32_t m = ((cfgr & RCC_CFGR_PLLMUL_Msk) >> RCC_CFGR_PLLMUL_Pos) + 2;
		sysclk = sysclk * m;

		SystemCoreClock = sysclk;
	}
	else if (cfgr & RCC_CFGR_SWS_HSE)
	{
		// HSE
		SystemCoreClock = s_XtlFreq;
	}
	else
	{
		// HSI 8 MHz
		SystemCoreClock = 8000000;
	}
}

//
// ClkFreq = Crystal frequency (ignored when bCrystal == false)
//
void SystemCoreClockSet(bool bCrystal, uint32_t ClkFreq)
{
	uint32_t cfgr = 0;
	uint32_t cfgr2 = 0;

	// Make sure we are running from HSI while we reconfigure PLL.
	RCC->CR |= RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

	if (bCrystal)
	{
		s_XtlFreq = ClkFreq;

		RCC->CR |= RCC_CR_HSEON;
		while ((RCC->CR & RCC_CR_HSERDY) == 0);

		// HSE is now running; enable Clock Security System to monitor it.
		RCC->CR |= RCC_CR_CSSON;

		cfgr |= RCC_CFGR_PLLSRC_HSE_PREDIV;

		uint32_t div = 1;
		int32_t  cdiff = SYSTEM_CORE_CLOCK;
		uint32_t mul = 2;

		// find best-fit div/mul for target core clock
		for (int i = 1; i <= 16; i++)
		{
			uint32_t clk = ClkFreq / i;

			for (int j = 2; j <= 16; j++)
			{
				uint32_t sysclk = clk * j;

				if (sysclk <= SYSTEM_CORE_CLOCK)
				{
					int diff = SYSTEM_CORE_CLOCK - sysclk;
					if (diff < cdiff)
					{
						cdiff = diff;
						div = i;
						mul = j;
					}
				}
			}
		}
		cfgr |= (mul - 2U) * RCC_CFGR_PLLMUL3;
		cfgr2 |= div - 1;
		RCC->CFGR2 = cfgr2;
	}
	else
	{
		// HSI/2 x 12 = 48 MHz. PREDIV not used on the HSI path.
		s_XtlFreq = 0;
		cfgr |= RCC_CFGR_PLLSRC_HSI_DIV2 | RCC_CFGR_PLLMUL12;
	}

	// 1. Write PLL source/multiplier. SW stays at HSI (bits = 0) so the
	//    core keeps running while PLL spins up.
	RCC->CFGR = cfgr;

	// 2. Start PLL and wait for lock.
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	// 3. Flash MUST have >=1 wait state before SYSCLK exceeds 24 MHz.
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// 4. Bus prescalers: HCLK = SYSCLK, PCLK = HCLK (F0 has a single APB).
	RCC->CFGR &= ~(RCC_CFGR_HPRE_Msk | RCC_CFGR_PPRE_Msk);

	// 5. Switch SYSCLK to PLL and confirm the switch happened.
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);

	RCC->CIR = 0x00000000U;

	SystemCoreClockUpdate();
}

void SystemInit(void)
{
	SystemCoreClockSet(STM32F0XX_HSE_XTAL_EN, STM32F0XX_HSE_XTAL_FREQ);
}
