/**-------------------------------------------------------------------------
@file	system_stm32wbaxx.c

@brief	STM32WBA common Cortex-M33 system initialization for IOsonata ResetEntry.

		SystemInit runs the core from PLL1R at up to 100 MHz. The source is HSE32
		when g_McuOsc describes a 32 MHz crystal or oscillator and it starts,
		HSI16 otherwise. HCLK and the APB buses are not divided. See SystemInit.

@author	Hoang Nguyen Hoan
@date	October 14, 2026

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

#include <stdbool.h>
#include <stdint.h>

#include "stm32wbaxx.h"
#include "system_stm32wbaxx.h"
#include "coredev/system_core_clock.h"

#define HSI16_FREQ				16000000UL
#define HSE32_FREQ				32000000UL		// The only HSE crystal frequency

#define SYSTEM_CORE_CLOCK_MAX	100000000UL

// PLL1 limits, RM0515 and the STM32WBA65 datasheet
#define PLLM_DIV_MAX			8
#define PLLN_MUL_MIN			4
#define PLLN_MUL_MAX			512
#define PLLR_DIV_MAX			128
#define PLL_IN_FREQ_MIN			4000000UL
#define PLL_IN_FREQ_MAX			16000000UL
#define PLL_IN_RANGE0_MAX		8000000UL		// VCO input range 0 is 4 to 8 MHz
#define VCO_FREQ_MIN			128000000UL
#define VCO_FREQ_MAX			544000000UL

// Voltage scaling range 1 is needed above 16 MHz
#define VOS_RANGE1_MIN_FREQ		16000000UL

// Polls of a ready flag before giving up on an oscillator, at least 50 ms
// at 16 MHz
#define OSC_READY_POLLS			200000

// Polls of the clock switch, PLL ready and voltage ready flags
#define CLK_READY_POLLS			100000

extern void (* const __Vectors[])(void);

uint32_t SystemCoreClock = HSI16_FREQ;

const uint8_t AHBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
const uint8_t APBPrescTable[8] = {0U, 0U, 0U, 0U, 1U, 2U, 3U, 4U};
const uint8_t AHB5PrescTable[8] = {1U, 1U, 1U, 1U, 2U, 3U, 4U, 6U};

/* Overload in the application firmware to describe the board oscillators.
 * HSE32 is used for the core clock when the type is crystal or external
 * oscillator and the frequency is 32 MHz. The LSE is not started here.
 */
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, HSI16_FREQ, 20, 0},
	{OSC_TYPE_RC, 32000, 20, 0},
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

/* Flash wait states of the core frequency in voltage scaling range 1,
 * as the ST LL utils: 0 up to 32 MHz, 1 up to 64, 2 up to 96, 3 up to 100.
 */
static uint32_t FlashWaitState(uint32_t CoreFreq)
{
	uint32_t ws = 0;

	while (CoreFreq > 32000000UL * (ws + 1) && ws < 3)
	{
		ws++;
	}

	return ws;
}

static bool SetFlashWaitState(uint32_t CoreFreq)
{
	uint32_t ws = FlashWaitState(CoreFreq) << FLASH_ACR_LATENCY_Pos;

	FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | ws | FLASH_ACR_PRFTEN;

	return WaitFlag(&FLASH->ACR, FLASH_ACR_LATENCY, ws, CLK_READY_POLLS);
}

/* Closest PLL1R output at or below SYSTEM_CORE_CLOCK_MAX from SrcFreq. A lower
 * VCO is kept when two settings give the same output. Returns the frequency, 0
 * when none, and the M, N and R dividers.
 */
static uint32_t FindPllCfg(uint32_t SrcFreq, uint32_t *pM, uint32_t *pN, uint32_t *pR)
{
	uint32_t best = 0;
	uint32_t bestvco = 0;

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

			if (vco < VCO_FREQ_MIN)
			{
				continue;
			}
			if (vco > VCO_FREQ_MAX)
			{
				break;
			}

			for (uint32_t r = 1; r <= PLLR_DIV_MAX; r++)
			{
				uint32_t f = vco / r;

				if (f > SYSTEM_CORE_CLOCK_MAX)
				{
					continue;
				}
				if (f < best)
				{
					break;
				}
				if (f > best || (f == best && vco < bestvco))
				{
					best = f;
					bestvco = vco;
					*pM = m;
					*pN = n;
					*pR = r;
				}
			}
		}
	}

	return best;
}

static void ShortDelay(uint32_t Cnt)
{
	while (Cnt-- > 0)
	{
		__NOP();
	}
}

/* Decode the clock the RCC runs from now */
void SystemCoreClockUpdate(void)
{
	uint32_t clk = HSI16_FREQ;
	uint32_t cr = RCC->CR;

	switch (RCC->CFGR1 & RCC_CFGR1_SWS)
	{
		case RCC_CFGR1_SWS_1:		// HSE32, or HSE32 divided by 2
			clk = HSE32_FREQ >> ((cr & RCC_CR_HSEPRE) >> RCC_CR_HSEPRE_Pos);
			break;
		case (RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0):	// PLL1R
		{
			uint32_t cfgr = RCC->PLL1CFGR;
			uint32_t divr = RCC->PLL1DIVR;
			uint32_t src = HSI16_FREQ;

			if ((cfgr & RCC_PLL1CFGR_PLL1SRC) == (RCC_PLL1CFGR_PLL1SRC_1 | RCC_PLL1CFGR_PLL1SRC_0))
			{
				src = HSE32_FREQ >> ((cr & RCC_CR_HSEPRE) >> RCC_CR_HSEPRE_Pos);
			}

			uint32_t m = ((cfgr & RCC_PLL1CFGR_PLL1M) >> RCC_PLL1CFGR_PLL1M_Pos) + 1;
			uint32_t n = ((divr & RCC_PLL1DIVR_PLL1N) >> RCC_PLL1DIVR_PLL1N_Pos) + 1;
			uint32_t r = ((divr & RCC_PLL1DIVR_PLL1R) >> RCC_PLL1DIVR_PLL1R_Pos) + 1;

			clk = (uint32_t)((uint64_t)src * n / (m * r));
			break;
		}
		default:					// HSI16
			break;
	}

	SystemCoreClock = clk >> AHBPrescTable[(RCC->CFGR2 & RCC_CFGR2_HPRE) >> RCC_CFGR2_HPRE_Pos];
}

/* Core clock setup, run by ResetEntry before main.
 *
 * Returns to HSI16 with no divider first, so that a second call (see
 * SystemCoreClockSelect) starts from the reset state. Then the clock is
 * PLL1R from HSE32 when g_McuOsc.CoreOsc is a 32 MHz crystal or oscillator
 * and it starts, otherwise from HSI16, at the highest frequency up to 100 MHz
 * the PLL dividers give: 100 MHz from either source. The sequence is the ST
 * HAL one (HAL_RCC_OscConfig, HAL_RCC_ClockConfig): voltage scaling range 1
 * and the flash wait states first, then PLL1, the switch through the PLL1R
 * two step divider, and the divider released. HCLK, PCLK1, PCLK2 and PCLK7 are
 * not divided. If the PLL does not lock the clock stays on HSI16.
 */
void SystemInit(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	/* The library is built for the FPU (hard float ABI). Enable CP10 and CP11
	   before any floating point instruction runs. */
	SCB->CPACR |= ((3UL << 20U) | (3UL << 22U));
#endif
	SCB->VTOR = (uint32_t)__Vectors;
	__DSB();
	__ISB();

	RCC->AHB4ENR |= RCC_AHB4ENR_PWREN;
	(void)RCC->AHB4ENR;

	// Run from HSI16 with PLL1 off, the reset state, also after a boot loader
	// or an earlier call left another clock running
	RCC->CR |= RCC_CR_HSION;
	if (WaitFlag(&RCC->CR, RCC_CR_HSIRDY, RCC_CR_HSIRDY, CLK_READY_POLLS) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CFGR1 &= ~RCC_CFGR1_SW;
	if (WaitFlag(&RCC->CFGR1, RCC_CFGR1_SWS, 0, CLK_READY_POLLS) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CR &= ~RCC_CR_PLL1ON;
	if (WaitFlag(&RCC->CR, RCC_CR_PLL1RDY, 0, CLK_READY_POLLS) == false)
	{
		SystemCoreClockUpdate();
		return;
	}
	RCC->CFGR2 &= ~(RCC_CFGR2_HPRE | RCC_CFGR2_PPRE1 | RCC_CFGR2_PPRE2);
	RCC->CFGR3 &= ~RCC_CFGR3_PPRE7;
	RCC->CR &= ~(RCC_CR_HSEON | RCC_CR_HSEPRE);

	uint32_t src = HSI16_FREQ;
	uint32_t pllsrc = RCC_PLL1CFGR_PLL1SRC_1;			// HSI16

	if (g_McuOsc.CoreOsc.Type != OSC_TYPE_RC && g_McuOsc.CoreOsc.Freq == HSE32_FREQ)
	{
		RCC->CR |= RCC_CR_HSEON;

		if (WaitFlag(&RCC->CR, RCC_CR_HSERDY, RCC_CR_HSERDY, OSC_READY_POLLS))
		{
			src = HSE32_FREQ;
			pllsrc = RCC_PLL1CFGR_PLL1SRC_1 | RCC_PLL1CFGR_PLL1SRC_0;	// HSE
		}
		else
		{
			// No oscillator: stay on HSI16 and record it
			RCC->CR &= ~RCC_CR_HSEON;
			g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
			g_McuOsc.CoreOsc.Freq = HSI16_FREQ;
		}
	}

	uint32_t m = 1;
	uint32_t n = PLLN_MUL_MIN;
	uint32_t r = 1;
	uint32_t freq = FindPllCfg(src, &m, &n, &r);

	if (freq == 0)
	{
		SystemCoreClockUpdate();
		return;
	}

	if (freq > VOS_RANGE1_MIN_FREQ)
	{
		PWR->VOSR |= PWR_VOSR_VOS;
		if (WaitFlag(&PWR->VOSR, PWR_VOSR_VOSRDY, PWR_VOSR_VOSRDY, CLK_READY_POLLS) == false ||
			WaitFlag(&PWR->SVMSR, PWR_SVMSR_ACTVOSRDY, PWR_SVMSR_ACTVOSRDY, CLK_READY_POLLS) == false)
		{
			SystemCoreClockUpdate();
			return;
		}
	}

	if (SetFlashWaitState(freq) == false)
	{
		SystemCoreClockUpdate();
		return;
	}

	uint32_t rge = (src / m) > PLL_IN_RANGE0_MAX ? (RCC_PLL1CFGR_PLL1RGE_1 | RCC_PLL1CFGR_PLL1RGE_0) : 0;

	RCC->PLL1CFGR = pllsrc | rge | ((m - 1) << RCC_PLL1CFGR_PLL1M_Pos) | RCC_PLL1CFGR_PLL1REN;
	RCC->PLL1DIVR = ((n - 1) << RCC_PLL1DIVR_PLL1N_Pos) | (1UL << RCC_PLL1DIVR_PLL1P_Pos) |
					(1UL << RCC_PLL1DIVR_PLL1Q_Pos) | ((r - 1) << RCC_PLL1DIVR_PLL1R_Pos);
	RCC->CR |= RCC_CR_PLL1ON;

	if (WaitFlag(&RCC->CR, RCC_CR_PLL1RDY, RCC_CR_PLL1RDY, CLK_READY_POLLS) == false)
	{
		RCC->CR &= ~RCC_CR_PLL1ON;
		SystemCoreClockUpdate();
		return;
	}

	// Switch through the PLL1R two step divider: select the 2 step mode, enable
	// the divider with the 2 extra pulses of the ST HAL for early silicon, then
	// switch, release the divider and wait for it
	RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1RCLKPRESTEP;
	RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1RCLKPRE;
	RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1RCLKPRE;
	RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1RCLKPRE;
	RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1RCLKPRE;
	RCC->PLL1CFGR |= RCC_PLL1CFGR_PLL1RCLKPRE;

	RCC->CFGR1 = (RCC->CFGR1 & ~RCC_CFGR1_SW) | RCC_CFGR1_SW_1 | RCC_CFGR1_SW_0;
	WaitFlag(&RCC->CFGR1, RCC_CFGR1_SWS, RCC_CFGR1_SWS_1 | RCC_CFGR1_SWS_0, CLK_READY_POLLS);

	RCC->PLL1CFGR &= ~RCC_PLL1CFGR_PLL1RCLKPRE;
	WaitFlag(&RCC->PLL1CFGR, RCC_PLL1CFGR_PLL1RCLKPRERDY, RCC_PLL1CFGR_PLL1RCLKPRERDY, CLK_READY_POLLS);
	ShortDelay(100);

	SystemCoreClockUpdate();
}

/**
 * @brief	Select core clock oscillator type
 *
 * @param	ClkSrc : OSC_TYPE_RC - HSI16
 *					 OSC_TYPE_XTAL - 32 MHz crystal
 *					 OSC_TYPE_TCXO - 32 MHz external oscillator
 * @param	OscFreq : Oscillator frequency, 32 MHz for a crystal or oscillator
 *
 * @return	true - success
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (ClkSrc == OSC_TYPE_RC)
	{
		OscFreq = HSI16_FREQ;
	}
	else if (OscFreq != HSE32_FREQ)
	{
		return false;
	}

	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;

	SystemInit();

	return true;
}

/* Core clock frequency (HCLK1) in Hz */
uint32_t SystemHFClockGet(void)
{
	return SystemCoreClock;
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

/* Peripheral clock frequency in Hz, HCLK1 divided by the APB prescaler.
 * Idx : 0 PCLK1 (APB1), 1 PCLK2 (APB2), 2 PCLK7 (APB7). 0 for another Idx.
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t ppre;

	switch (Idx)
	{
		case 0:
			ppre = (RCC->CFGR2 & RCC_CFGR2_PPRE1_Msk) >> RCC_CFGR2_PPRE1_Pos;
			break;
		case 1:
			ppre = (RCC->CFGR2 & RCC_CFGR2_PPRE2_Msk) >> RCC_CFGR2_PPRE2_Pos;
			break;
		case 2:
			ppre = (RCC->CFGR3 & RCC_CFGR3_PPRE7_Msk) >> RCC_CFGR3_PPRE7_Pos;
			break;
		default:
			return 0;
	}

	return SystemCoreClock >> APBPrescTable[ppre];
}

/* Set a peripheral clock (PCLK) frequency
 *
 * Selects the APB prescaler giving the closest frequency, HCLK divided by 1
 * to 16. 0 Hz gives the lowest.
 *
 * Idx : 0 PCLK1 (APB1), 1 PCLK2 (APB2), 2 PCLK7 (APB7)
 *
 * Return the frequency set in Hz, 0 for a bad clock number.
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	if (Idx < 0 || Idx > 2)
	{
		return 0;
	}

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

	switch (Idx)
	{
		case 0:
			RCC->CFGR2 = (RCC->CFGR2 & ~RCC_CFGR2_PPRE1) | (ppre << RCC_CFGR2_PPRE1_Pos);
			break;
		case 1:
			RCC->CFGR2 = (RCC->CFGR2 & ~RCC_CFGR2_PPRE2) | (ppre << RCC_CFGR2_PPRE2_Pos);
			break;
		default:
			RCC->CFGR3 = (RCC->CFGR3 & ~RCC_CFGR3_PPRE7) | (ppre << RCC_CFGR3_PPRE7_Pos);
			break;
	}

	return SystemCoreClock >> shift;
}

