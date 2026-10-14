/**-------------------------------------------------------------------------
@file	system_stm32g4xx.c

@brief	Implementation of CMSIS SystemInit for STM32G4xx Device Series

SystemInit runs the core from the PLL at the highest frequency up to 170 MHz,
fed by HSI16 or by the HSE oscillator described in g_McuOsc. Above 150 MHz
the regulator runs in range 1 boost mode, as RM0440 requires. AHB, APB1 and
APB2 run undivided.

@author	Hoang Nguyen Hoan
@date	October 13, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <stdint.h>

#include "stm32g4xx.h"
#include "coredev/system_core_clock.h"

#define HSI16_FREQ				16000000UL

#define HSE_OSC_FREQ_MIN		4000000UL		// 4 MHz
#define HSE_OSC_FREQ_MAX		48000000UL		// 48 MHz

#define SYSTEM_CORE_CLOCK_MAX	170000000UL		// Range 1 boost mode
#define SYSTEM_CORE_CLOCK_NORMAL_MAX	150000000UL	// Range 1 normal mode

// PLL limits, RM0440 and the STM32G474 datasheet
#define PLLM_DIV_MAX			16
#define PLLN_MUL_MIN			8
#define PLLN_MUL_MAX			127
#define PLL_IN_FREQ_MIN			2660000UL
#define PLL_IN_FREQ_MAX			16000000UL
#define VCO_FREQ_MIN			96000000UL
#define VCO_FREQ_MAX			344000000UL

// Polls of a ready flag before giving up on an oscillator, at least 50 ms
// at 16 MHz
#define OSC_READY_POLLS			200000

// Polls of the clock switch and PLL ready flags
#define CLK_READY_POLLS			100000

extern void (* const __Vectors[])(void);

uint32_t SystemCoreClock = HSI16_FREQ;

const uint8_t AHBPrescTable[16] = {
	0U, 0U, 0U, 0U, 0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U, 6U, 7U, 8U, 9U
};
const uint8_t APBPrescTable[8] = {
	0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U
};

// Overload this variable in application firmware to describe the board
// oscillators. HSE is used for the core when the type is crystal or external
// oscillator. The low frequency oscillator is started by the drivers using it.
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, HSI16_FREQ, 20, 0},
	{OSC_TYPE_XTAL, 32768, 20, 0},
	false
};

static bool WaitFlag(volatile uint32_t *pReg, uint32_t Mask, uint32_t Val, int Polls)
{
	while (Polls-- > 0)
	{
		if ((*pReg & Mask) == Val)
		{
			return true;
		}
	}

	return false;
}

// Flash wait states for the core frequency, RM0440 table 9. Range 1 boost mode
// above 150 MHz, normal mode below.
static uint32_t FlashWaitState(uint32_t CoreFreq)
{
	uint32_t step = CoreFreq > SYSTEM_CORE_CLOCK_NORMAL_MAX ? 34000000UL : 30000000UL;
	uint32_t ws = 0;

	while (CoreFreq > step * (ws + 1) && ws < 8)
	{
		ws++;
	}

	return ws;
}

static void SetFlashWaitState(uint32_t CoreFreq)
{
	uint32_t ws = FlashWaitState(CoreFreq);

	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | ws |
				 FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN;
	WaitFlag(&FLASH->ACR, FLASH_ACR_LATENCY, ws, CLK_READY_POLLS);
}

// Closest PLL output at or below SYSTEM_CORE_CLOCK_MAX from SrcFreq. Returns
// the PLLCFGR M, N and R fields with PLLREN, and the frequency in *pFreq.
static uint32_t FindPllCfg(uint32_t SrcFreq, uint32_t *pFreq)
{
	uint32_t best = 0;
	uint32_t cfg = 0;

	for (uint32_t m = 1; m <= PLLM_DIV_MAX; m++)
	{
		uint32_t in = SrcFreq / m;

		if (in < PLL_IN_FREQ_MIN || in > PLL_IN_FREQ_MAX)
		{
			continue;
		}

		for (uint32_t n = PLLN_MUL_MIN; n <= PLLN_MUL_MAX; n++)
		{
			uint32_t vco = in * n;

			if (vco < VCO_FREQ_MIN || vco > VCO_FREQ_MAX)
			{
				continue;
			}

			for (uint32_t r = 2; r <= 8; r += 2)
			{
				uint32_t f = vco / r;

				if (f <= SYSTEM_CORE_CLOCK_MAX && f > best)
				{
					best = f;
					cfg = ((m - 1) << RCC_PLLCFGR_PLLM_Pos) | (n << RCC_PLLCFGR_PLLN_Pos) |
						  (((r >> 1) - 1) << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN;
				}
			}
		}
	}

	*pFreq = best;

	return cfg;
}

static void ShortDelay(uint32_t Cnt)
{
	while (Cnt-- > 0)
	{
		__NOP();
	}
}

void SystemCoreClockUpdate(void)
{
	uint32_t cfgr = RCC->CFGR;
	uint32_t clk = HSI16_FREQ;

	switch (cfgr & RCC_CFGR_SWS)
	{
		case RCC_CFGR_SWS_HSE:
			clk = g_McuOsc.CoreOsc.Freq;
			break;
		case RCC_CFGR_SWS_PLL:
		{
			uint32_t pllcfgr = RCC->PLLCFGR;
			uint32_t src = (pllcfgr & RCC_PLLCFGR_PLLSRC) == RCC_PLLCFGR_PLLSRC_HSE ?
						   g_McuOsc.CoreOsc.Freq : HSI16_FREQ;
			uint32_t m = ((pllcfgr & RCC_PLLCFGR_PLLM) >> RCC_PLLCFGR_PLLM_Pos) + 1;
			uint32_t n = (pllcfgr & RCC_PLLCFGR_PLLN) >> RCC_PLLCFGR_PLLN_Pos;
			uint32_t r = (((pllcfgr & RCC_PLLCFGR_PLLR) >> RCC_PLLCFGR_PLLR_Pos) + 1) << 1;

			clk = (uint32_t)((uint64_t)src * n / (m * r));
			break;
		}
		default:	// HSI16
			break;
	}

	SystemCoreClock = clk >> AHBPrescTable[(cfgr & RCC_CFGR_HPRE) >> RCC_CFGR_HPRE_Pos];
}

void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	// Enable CP10 and CP11 before any floating point instruction runs
	SCB->CPACR |= ((3UL << 20U) | (3UL << 22U));
#endif
	SCB->VTOR = (uint32_t)__Vectors;
	__DSB();
	__ISB();

	// Run from HSI16 with the PLL off, the reset state, also after a boot
	// loader or an earlier call left another clock running. Leaving the PLL
	// above 80 MHz goes through HCLK divided by 2.
	RCC->CR |= RCC_CR_HSION;
	WaitFlag(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY, CLK_READY_POLLS);
	if ((RCC->CFGR & RCC_CFGR_SWS) == RCC_CFGR_SWS_PLL)
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_DIV2;
	}
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_HSI;
	WaitFlag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_HSI, CLK_READY_POLLS);
	RCC->CFGR &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
	RCC->CR &= ~RCC_CR_PLLON;
	WaitFlag(&RCC->CR, RCC_CR_PLLRDY, 0, CLK_READY_POLLS);
	RCC->CIER = 0;

	uint32_t src = HSI16_FREQ;
	uint32_t pllsrc = RCC_PLLCFGR_PLLSRC_HSI;

	if (g_McuOsc.CoreOsc.Type != OSC_TYPE_RC &&
		g_McuOsc.CoreOsc.Freq >= HSE_OSC_FREQ_MIN && g_McuOsc.CoreOsc.Freq <= HSE_OSC_FREQ_MAX)
	{
		RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
		if (g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO)
		{
			RCC->CR |= RCC_CR_HSEBYP;
		}
		RCC->CR |= RCC_CR_HSEON;

		if (WaitFlag(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY, OSC_READY_POLLS))
		{
			src = g_McuOsc.CoreOsc.Freq;
			pllsrc = RCC_PLLCFGR_PLLSRC_HSE;
		}
		else
		{
			// No oscillator: stay on HSI16 and record it
			RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEBYP);
			g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
			g_McuOsc.CoreOsc.Freq = HSI16_FREQ;
		}
	}

	uint32_t freq = 0;
	uint32_t pllcfgr = FindPllCfg(src, &freq);

	if (freq == 0)
	{
		SystemCoreClockUpdate();
		return;
	}

	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	(void)RCC->APB1ENR1;
	if (freq > SYSTEM_CORE_CLOCK_NORMAL_MAX)
	{
		PWR->CR5 &= ~PWR_CR5_R1MODE;	// Range 1 boost mode
	}
	else
	{
		PWR->CR5 |= PWR_CR5_R1MODE;
	}

	SetFlashWaitState(freq);

	RCC->PLLCFGR = pllsrc | pllcfgr;
	RCC->CR |= RCC_CR_PLLON;

	if (WaitFlag(&RCC->CR, RCC_CR_PLLRDY, RCC_CR_PLLRDY, CLK_READY_POLLS) == false)
	{
		RCC->CR &= ~RCC_CR_PLLON;
		SystemCoreClockUpdate();
		return;
	}

	// Above 80 MHz, switch with HCLK divided by 2, then wait at least 1 us
	// before running undivided (RM0440 clock switch and boost mode sequence).
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_HPRE) | RCC_CFGR_HPRE_DIV2;
	RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_SW) | RCC_CFGR_SW_PLL;
	WaitFlag(&RCC->CFGR, RCC_CFGR_SWS, RCC_CFGR_SWS_PLL, CLK_READY_POLLS);
	ShortDelay(200);
	RCC->CFGR &= ~RCC_CFGR_HPRE;

	SystemCoreClockUpdate();
}

/**
 * @brief	Select core clock oscillator type
 *
 * @param	ClkSrc : Clock source selection
 *						OSC_TYPE_RC - Internal RC, HSI16
 *						OSC_TYPE_XTAL - External crystal
 *						OSC_TYPE_TCXO -	External oscillator
 * @param	OscFreq : Oscillator frequency
 *
 * @return	true - success
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (ClkSrc == OSC_TYPE_RC)
	{
		OscFreq = HSI16_FREQ;
	}
	else if (OscFreq < HSE_OSC_FREQ_MIN || OscFreq > HSE_OSC_FREQ_MAX)
	{
		return false;
	}

	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;

	SystemInit();

	return true;
}

/**
 * @brief	Get high frequency clock frequency (HCLK)
 *
 * @return	HCLK clock frequency in Hz.
 */
uint32_t SystemHFClockGet(void)
{
	return SystemCoreClock;
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

/**
 * @brief	Get peripheral clock frequency (PCLK)
 *
 * @param	Idx : 0 PCLK1 (APB1), 1 PCLK2 (APB2)
 *
 * @return	Peripheral clock frequency in Hz.
 * 			0 - Bad clock number
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t ppre;

	if (Idx == 0)
	{
		ppre = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
	}
	else if (Idx == 1)
	{
		ppre = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
	}
	else
	{
		return 0;
	}

	return SystemCoreClock >> APBPrescTable[ppre];
}

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Selects the APB prescaler giving the closest frequency, HCLK divided by 1
 * to 16.
 *
 * @param	Idx  : 0 PCLK1 (APB1), 1 PCLK2 (APB2)
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 * 			0 - Bad clock number
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	if (Idx < 0 || Idx > 1)
	{
		return 0;
	}

	// Same divider choice as the STM32L4 port, 0 Hz gives the lowest
	uint32_t div = Freq > 0 ? (SystemCoreClock + (Freq >> 1)) / Freq : 16;
	uint32_t shift;

	if (div < 2)
	{
		shift = 0;
	}
	else if (div < 4)
	{
		shift = 1;
	}
	else if (div < 8)
	{
		shift = 2;
	}
	else if (div < 16)
	{
		shift = 3;
	}
	else
	{
		shift = 4;
	}

	// PPRE: 0 undivided, 4 + n divides by 2^(n + 1)
	uint32_t ppre = shift == 0 ? 0 : (3 + shift);

	if (Idx == 1)
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE2) | (ppre << RCC_CFGR_PPRE2_Pos);
	}
	else
	{
		RCC->CFGR = (RCC->CFGR & ~RCC_CFGR_PPRE1) | (ppre << RCC_CFGR_PPRE1_Pos);
	}

	return SystemCoreClock >> shift;
}
