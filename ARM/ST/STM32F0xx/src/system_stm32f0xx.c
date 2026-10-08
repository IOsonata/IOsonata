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


// Overload this variable in application firmware to change oscillator
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, 8000000, 20},
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

	if ((cfgr & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_PLL)
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
		if (m > 16) m = 16;
		sysclk = sysclk * m;

		SystemCoreClock = sysclk;
	}
	else if ((cfgr & RCC_CFGR_SWS_Msk) == RCC_CFGR_SWS_HSE)
	{
		// HSE
		SystemCoreClock = s_XtlFreq;
	}
	else
	{
		// HSI 8 MHz
		SystemCoreClock = 8000000;
	}
	static const uint8_t ahbshift[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};
	SystemCoreClock >>= ahbshift[(cfgr & RCC_CFGR_HPRE_Msk) >> RCC_CFGR_HPRE_Pos];
}

//
// ClkFreq = Crystal frequency (ignored when bCrystal == false)
//
static void Stm32f0ClockSet(OSC_TYPE Type, uint32_t ClkFreq)
{
	uint32_t cfgr = 0;
	uint32_t cfgr2 = 0;

	// Also handle entry from a bootloader without a peripheral reset.
	// Switch away from PLL before changing its configuration.

	// 1. HSI on and stable (safe fallback clock).
	RCC->CR |= RCC_CR_HSION;
	while ((RCC->CR & RCC_CR_HSIRDY) == 0);

	// 2. Switch SYSCLK to HSI. If we were on PLL, the core drops to 8 MHz.
	RCC->CFGR &= ~RCC_CFGR_SW_Msk;
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != 0);

	// 3. Core is on HSI now — safe to relax flash to 0 WS.
	FLASH->ACR = FLASH_ACR_PRFTBE;

	// 4. Turn PLL off and wait until PLLRDY actually clears. Required
	//    before any PLLSRC/PLLMUL/PREDIV write will take effect.
	RCC->CR &= ~RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) != 0);

	// 5. HSE off (in case previous firmware had it on). Clear HSEBYP too.
	RCC->CR &= ~(RCC_CR_CSSON | RCC_CR_HSEON);
	while ((RCC->CR & RCC_CR_HSERDY) != 0);
	RCC->CR &= ~RCC_CR_HSEBYP;

	// 6. Zero CFGR/CFGR2/CFGR3 so no stale PLLMUL/PREDIV/USART1SW.
	RCC->CFGR  = 0;
	RCC->CFGR2 = 0;
	RCC->CFGR3 = 0;

	// 7. Clear all RCC interrupts.
	RCC->CIR = 0x00FF0000U;

	// -------- From here: known state — HSI 8 MHz, PLL off, HSE off --------

	if (Type != OSC_TYPE_RC)
	{
		s_XtlFreq = ClkFreq;

		if (Type == OSC_TYPE_TCXO) RCC->CR |= RCC_CR_HSEBYP;
		RCC->CR |= RCC_CR_HSEON;
		while ((RCC->CR & RCC_CR_HSERDY) == 0);


		cfgr |= RCC_CFGR_PLLSRC_HSE_PREDIV;

		uint32_t div = 1;
		int32_t  cdiff = SYSTEM_CORE_CLOCK;
		uint32_t mul = 2;

		// find best-fit div/mul for target core clock
		for (int i = 1; i <= 16; i++)
		{
			uint32_t clk = ClkFreq / i;
			if (clk < 1000000U || clk > 24000000U) continue;

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

	// 8. Write PLL source/multiplier. SW stays at HSI (bits 0 in cfgr)
	//    so the core keeps running on HSI while PLL spins up.
	RCC->CFGR = cfgr;

	// 9. Start PLL and wait for lock.
	RCC->CR |= RCC_CR_PLLON;
	while ((RCC->CR & RCC_CR_PLLRDY) == 0);

	// 10. Flash MUST have >=1 wait state before SYSCLK exceeds 24 MHz.
	FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;

	// 11. Switch SYSCLK to PLL and confirm the switch happened.
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW_Msk) | RCC_CFGR_SW_PLL;
	while ((RCC->CFGR & RCC_CFGR_SWS_Msk) != RCC_CFGR_SWS_PLL);

	SystemCoreClockUpdate();
}

// Legacy target entry point retained for existing applications.
void SystemCoreClockSet(bool bCrystal, uint32_t ClkFreq)
{
	if (!SystemCoreClockSelect(bCrystal ? OSC_TYPE_XTAL : OSC_TYPE_RC, ClkFreq))
	{
		Stm32f0ClockSet(OSC_TYPE_RC, 8000000U);
	}
}

bool SystemCoreClockSelect(OSC_TYPE Type, uint32_t Freq)
{
	if (Type != OSC_TYPE_RC && Type != OSC_TYPE_XTAL && Type != OSC_TYPE_TCXO)
	{
		return false;
	}
	if (Type != OSC_TYPE_RC && (Freq < 4000000U || Freq > 32000000U))
	{
		return false;
	}
	Stm32f0ClockSet(Type, Freq);
	return true;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	if (Idx != 0) return 0;
	uint32_t pre = (RCC->CFGR & RCC_CFGR_PPRE_Msk) >> RCC_CFGR_PPRE_Pos;
	return SystemCoreClock >> (pre < 4 ? 0 : pre - 3);
}

void SystemInit(void)
{
	if (!SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq))
	{
		Stm32f0ClockSet(OSC_TYPE_RC, 8000000U);
	}
}
