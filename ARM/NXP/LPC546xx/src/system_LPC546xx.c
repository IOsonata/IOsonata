/**-------------------------------------------------------------------------
@file	system_LPC546xx.c

@brief	CMSIS system initialization

Implementation of the CMSIS system initialization for the NXP LPC546xx

@author	Hoang Nguyen Hoan
@date	July 10, 2020

@licanse

Copyright (c) 2020, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <stdbool.h>

#include "LPC546xx.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "clock_lpc546xx.h"
#include "coredev/interrupt.h"

#define NVALMAX (0x100)
#define PVALMAX (0x20)
#define MVALMAX (0x8000)
#define PLL_MDEC_VAL_P (0)                                       /* MDEC is in bits  16:0 */
#define PLL_MDEC_VAL_M (0x1FFFFUL << PLL_MDEC_VAL_P)
#define PLL_NDEC_VAL_P (0)                                       /* NDEC is in bits  9:0 */
#define PLL_NDEC_VAL_M (0x3FFUL << PLL_NDEC_VAL_P)
#define PLL_PDEC_VAL_P (0)                                       /* PDEC is in bits  6:0 */
#define PLL_PDEC_VAL_M (0x7FUL << PLL_PDEC_VAL_P)

#ifndef SYSTEM_CORE_CLOCK_MAX
#error "SYSTEM_CORE_CLOCK_MAX must be defined by the MCU system header"
#endif

// nsDelay loop length in core clocks
#define LPC546XX_NSDELAY_LOOP_CLK	3UL

// System PLL. Fcco = 2 * M * Fin / N must stay within the CCO range. The PLL
// output is Fcco, or Fcco / (2 * P) with the post divider.
#define LPC546XX_PLL_CCO_MIN		275000000UL
#define LPC546XX_PLL_CCO_MAX		550000000UL
#define LPC546XX_PLL_REF_MIN		1000000UL		// Lowest Fin / N used, keeps the PLL reference high
#define LPC546XX_PLL_LOCK_WAIT		100000UL		// SYSPLLSTAT reads before giving up
#define LPC546XX_AUDPLL_REF_MIN		10000UL			// Lowest Fin / N of the audio PLL (NXP SDK)
#define LPC546XX_AUDPLL_LOCK_WAIT	10000000UL		// AUDPLLSTAT reads before giving up, the reference can be slow
#define LPC546XX_PLLCLKSEL_FRO12M	0U
#define LPC546XX_PLLCLKSEL_CLKIN	1U
#define LPC546XX_MAINCLKSELB_MAINA	0U
#define LPC546XX_MAINCLKSELB_PLL	2U

// System oscillator range, FREQRANGE 0 up to 20 MHz, 1 from 15 to 25 MHz
#define LPC546XX_SYSOSC_FREQ_MIN	1000000UL
#define LPC546XX_SYSOSC_FREQ_MAX	25000000UL
#define LPC546XX_SYSOSC_RANGE_LOW	20000000UL

// Voltage domain level registers, one word per domain VD1 to VD6, and the
// domain status register. They are not described in UM10912. Addresses,
// levels and frequency limits are the ones POWER_SetVoltageForFreq and
// POWER_SetPLL write in the NXP SDK power library (libpower, BSD-3-Clause).
// Level n is 0.65 V + n * 50 mV.
#define LPC546XX_VD_LEVEL			((volatile uint32_t *)0x40020000UL)
#define LPC546XX_VD_STATUS			(*(volatile uint32_t *)0x40020054UL)
#define LPC546XX_VD_STATUS_VD3_RDY	(1UL << 5)
#define LPC546XX_VD_STATUS_WAIT		100000UL
#define LPC546XX_VD_1V20			11U
#define LPC546XX_VD_1V25			12U
#define LPC546XX_VD_1V30			13U
#define LPC546XX_VD_1V40			15U
#define LPC546XX_VD_FREQ_MID		100000000UL		// VD1 and VD4 at 1.30 V above this
#define LPC546XX_VD_FREQ_HIGH		180000000UL		// VD1 and VD4 at 1.40 V above this

// FLASHTIM above LPC546XX_VD_FREQ_HIGH. The 1.40 V core voltage gives the
// flash access time of 168 MHz.
#define LPC546XX_FLASHTIM_VD_HIGH	7U

typedef struct {
	uint32_t N;					//!< Pre divider, 1 to NVALMAX
	uint32_t M;					//!< Multiplier, 1 to MVALMAX
	uint32_t P;					//!< Post divider, 0 when not used
	uint32_t Freq;				//!< Output frequency in Hz
} Lpc546xxPll_t;

#if defined(__MCUXPRESSO)
extern void(*const g_pfnVectors[])(void);
#else
extern void *__Vectors;
#endif

__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, 12000000, 0, 0},
	{OSC_TYPE_XTAL, 32768, 0, 0},
	true,
	0
};

// Flash access time is FLASHTIM + 1 system clocks. Entry n is the highest
// system clock for FLASHTIM n.
static const uint32_t s_FlashTimFreq[] = {
	12000000UL, 24000000UL, 36000000UL, 60000000UL, 96000000UL,
	120000000UL, 144000000UL, 168000000UL, 180000000UL
};
// Calibrated delay-loop iterations per second, defined in ResetEntry.c
extern uint32_t SystemMicroSecLoopCnt;

uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;
uint32_t SystemnsDelayFactor = 1000000UL;


static uint32_t s_Lpc546xxAudioPllFreq = 0;

// Main clock set by the clock setup, the 12 MHz FRO after reset
static uint32_t s_Lpc546xxMainFreq = CLK_FRO_12MHZ;
static uint32_t s_HfUsers;
static SYSTEM_POWER_MODE s_PowerMode;

static bool Lpc546xxCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);

// NDEC, PDEC and MDEC encodings of the PLL dividers (UM10912)
static uint32_t pllEncodeN(uint32_t N)
{
	if (N == 1U)
	{
		return 0x302U;
	}
	if (N == 2U)
	{
		return 0x202U;
	}

	uint32_t x = 0x80U;

	for (uint32_t i = N; i <= NVALMAX; i++)
	{
		x = (((x ^ (x >> 2U) ^ (x >> 3U) ^ (x >> 4U)) & 1U) << 7U) | ((x >> 1U) & 0x7FU);
	}

	return x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P);
}

static uint32_t pllEncodeP(uint32_t P)
{
	if (P == 0U)
	{
		return 0x7FU;
	}
	if (P == 1U)
	{
		return 0x62U;
	}
	if (P == 2U)
	{
		return 0x42U;
	}

	uint32_t x = 0x10U;

	for (uint32_t i = P; i <= PVALMAX; i++)
	{
		x = (((x ^ (x >> 2U)) & 1U) << 4U) | ((x >> 1U) & 0xFU);
	}

	return x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P);
}

static uint32_t pllEncodeM(uint32_t M)
{
	if (M == 1U)
	{
		return 0x18003U;
	}
	if (M == 2U)
	{
		return 0x10003U;
	}

	uint32_t x = 0x4000U;

	for (uint32_t i = M; i <= MVALMAX; i++)
	{
		x = (((x ^ (x >> 1U)) & 1U) << 14U) | ((x >> 1U) & 0x3FFFU);
	}

	return x & (PLL_MDEC_VAL_M >> PLL_MDEC_VAL_P);
}

// SYSPLLCTRL value. The bandwidth is set from the total multiplier 2 * M,
// which gives the values of the NXP LPCXpresso54628 180 and 220 MHz setups.
static uint32_t Lpc546xxPllCtrl(uint32_t M, bool bDirectOut)
{
	uint32_t mt = M << 1;
	uint32_t selp = mt < 60U ? (mt >> 1) + 1U : PVALMAX - 1U;
	uint32_t seli;

	if (mt > 16384U)
	{
		seli = 1U;
	}
	else if (mt > 8192U)
	{
		seli = 2U;
	}
	else if (mt > 2048U)
	{
		seli = 4U;
	}
	else if (mt >= 501U)
	{
		seli = 8U;
	}
	else if (mt >= 60U)
	{
		seli = 4096U / (mt + 9U);
	}
	else
	{
		seli = (mt & 0x3CU) + 4U;
	}

	return SYSCON_SYSPLLCTRL_SELI(seli) | SYSCON_SYSPLLCTRL_SELP(selp) | SYSCON_SYSPLLCTRL_SELR(0) |
		   (bDirectOut ? SYSCON_SYSPLLCTRL_DIRECTO_MASK : 0U);
}

// Find the PLL setting with the highest output not above Fout. Direct output
// and small dividers are tried first.
static bool Lpc546xxPllFind(uint32_t Fin, uint32_t Fout, Lpc546xxPll_t *pPll)
{
	pPll->N = 1;
	pPll->M = 1;
	pPll->P = 0;
	pPll->Freq = 0;

	for (uint32_t p = 0; p <= PVALMAX; p++)
	{
		uint32_t div = p == 0U ? 1U : p << 1;
		uint64_t fcco = (uint64_t)Fout * div;

		if (fcco < LPC546XX_PLL_CCO_MIN)
		{
			continue;
		}
		if (fcco > LPC546XX_PLL_CCO_MAX)
		{
			break;
		}

		for (uint32_t n = 1; n <= NVALMAX && Fin / n >= LPC546XX_PLL_REF_MIN; n++)
		{
			uint64_t m = fcco * n / ((uint64_t)Fin << 1);

			if (m == 0U || m > MVALMAX)
			{
				continue;
			}

			uint64_t cco = ((uint64_t)Fin * (m << 1)) / n;

			if (cco < LPC546XX_PLL_CCO_MIN)
			{
				continue;
			}

			uint32_t f = (uint32_t)(cco / div);

			if (f > pPll->Freq)
			{
				pPll->N = n;
				pPll->M = (uint32_t)m;
				pPll->P = p;
				pPll->Freq = f;
			}
			if (f == Fout)
			{
				return true;
			}
		}
	}

	return pPll->Freq != 0U;
}

// Core voltage for a system clock frequency
static void Lpc546xxSetVoltage(uint32_t Freq)
{
	uint32_t vd = LPC546XX_VD_1V20;

	if (Freq > LPC546XX_VD_FREQ_HIGH)
	{
		vd = LPC546XX_VD_1V40;
	}
	else if (Freq > LPC546XX_VD_FREQ_MID)
	{
		vd = LPC546XX_VD_1V30;
	}

	LPC546XX_VD_LEVEL[0] = vd;
	LPC546XX_VD_LEVEL[1] = LPC546XX_VD_1V25;
	LPC546XX_VD_LEVEL[2] = LPC546XX_VD_1V20;
	LPC546XX_VD_LEVEL[3] = vd;
	LPC546XX_VD_LEVEL[4] = LPC546XX_VD_1V20;
	LPC546XX_VD_LEVEL[5] = LPC546XX_VD_1V20;
}

// Flash access time for a system clock frequency
static void Lpc546xxSetFlashTime(uint32_t Freq)
{
	uint32_t tim = LPC546XX_FLASHTIM_VD_HIGH;

	if (Freq <= LPC546XX_VD_FREQ_HIGH)
	{
		tim = 0;
		while (tim < sizeof(s_FlashTimFreq) / sizeof(s_FlashTimFreq[0]) - 1U && Freq > s_FlashTimFreq[tim])
		{
			tim++;
		}
	}

	SYSCON->FLASHCFG = (SYSCON->FLASHCFG & ~SYSCON_FLASHCFG_FLASHTIM_MASK) | SYSCON_FLASHCFG_FLASHTIM(tim);
}

// FRO on, main clock from fro_12m, AHB divider 1
static void Lpc546xxMainClockFro12M(void)
{
	SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_FRO_MASK;
	SYSCON->MAINCLKSELA = SYSCON_MAINCLKSELA_SEL(0);
	SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(LPC546XX_MAINCLKSELB_MAINA);
	SYSCON->AHBCLKDIV = 0;
	s_Lpc546xxMainFreq = CLK_FRO_12MHZ;
}

// Program and start the system PLL. The main clock must not be on the PLL.
static bool Lpc546xxPllStart(uint32_t ClkSel, const Lpc546xxPll_t *pPll)
{
	uint32_t cnt = LPC546XX_VD_STATUS_WAIT;

	// VD3 supplies the PLL
	SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_VD3_MASK;
	while ((LPC546XX_VD_STATUS & LPC546XX_VD_STATUS_VD3_RDY) == 0U && --cnt > 0U);
	if (cnt == 0U)
	{
		return false;
	}

	// The PLL is powered down while it is changed. A divider value is taken
	// when its REQ bit is written.
	SYSCON->PDRUNCFGSET[0] = SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK;
	SYSCON->SYSPLLCLKSEL = SYSCON_SYSPLLCLKSEL_SEL(ClkSel);
	SYSCON->SYSPLLCTRL = Lpc546xxPllCtrl(pPll->M, pPll->P == 0U);

	uint32_t ndec = SYSCON_SYSPLLNDEC_NDEC(pllEncodeN(pPll->N));
	uint32_t pdec = SYSCON_SYSPLLPDEC_PDEC(pllEncodeP(pPll->P));
	uint32_t mdec = SYSCON_SYSPLLMDEC_MDEC(pllEncodeM(pPll->M));

	SYSCON->SYSPLLNDEC = ndec;
	SYSCON->SYSPLLNDEC = ndec | SYSCON_SYSPLLNDEC_NREQ_MASK;
	SYSCON->SYSPLLPDEC = pdec;
	SYSCON->SYSPLLPDEC = pdec | SYSCON_SYSPLLPDEC_PREQ_MASK;
	SYSCON->SYSPLLMDEC = mdec;
	SYSCON->SYSPLLMDEC = mdec | SYSCON_SYSPLLMDEC_MREQ_MASK;

	SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK;

	cnt = LPC546XX_PLL_LOCK_WAIT;
	while ((SYSCON->SYSPLLSTAT & SYSCON_SYSPLLSTAT_LOCK_MASK) == 0U && --cnt > 0U);
	if (cnt == 0U)
	{
		SYSCON->PDRUNCFGSET[0] = SYSCON_PDRUNCFG_PDEN_SYS_PLL_MASK;

		return false;
	}

	return true;
}

// clk_in is the system oscillator when the core oscillator is external
static uint32_t Lpc546xxClkInFreq(void)
{
	return g_McuOsc.CoreOsc.Type != OSC_TYPE_RC &&
		(SYSCON->PDRUNCFG[1] & SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK) == 0 ?
		g_McuOsc.CoreOsc.Freq : 0U;
}

void SystemInit(void)
{
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

#if defined(__MCUXPRESSO)
    SCB->VTOR = (uint32_t) &g_pfnVectors;
#else
    SCB->VTOR = (uint32_t) &__Vectors;
#endif
    SYSCON->ARMTRACECLKDIV = 0;
/* Optionally enable RAM banks that may be off by default at reset */
#if !defined(DONT_ENABLE_DISABLED_RAMBANKS)
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_SRAM1_MASK | SYSCON_AHBCLKCTRL_SRAM2_MASK | SYSCON_AHBCLKCTRL_SRAM3_MASK;
#endif

	// On failure the PLL is tried from the FRO. If it does not lock either,
	// the core stays on the 12 MHz FRO.
	if (Lpc546xxCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq) == false)
	{
		Lpc546xxCoreClockSelect(OSC_TYPE_RC, CLK_FRO_12MHZ);
	}
	s_PowerMode = SYSTEM_POWER_MODE_PERFORMANCE;
}

/**
 * @brief	Select the core clock oscillator. Startup only.
 *
 * The core runs from the system PLL at SYSTEM_CORE_CLOCK_MAX, or the closest
 * frequency below it. The PLL input is the 12 MHz FRO for OSC_TYPE_RC, the
 * system oscillator for OSC_TYPE_XTAL and OSC_TYPE_TCXO (clock on XTALIN).
 * The core voltage and the flash access time are set for that frequency.
 *
 * @param	ClkSrc	: Oscillator type
 * @param	OscFreq	: Oscillator frequency in Hz, 12 MHz for OSC_TYPE_RC
 *
 * @return	true - core on the PLL
 * 			false - invalid oscillator, or PLL failure. The core is then on the
 * 			12 MHz FRO when the PLL was tried.
 */
static bool Lpc546xxCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t clksel = LPC546XX_PLLCLKSEL_FRO12M;
	Lpc546xxPll_t pll;
	uint32_t state;

	if (ClkSrc == OSC_TYPE_RC)
	{
		if (OscFreq != CLK_FRO_12MHZ)
		{
			return false;
		}
	}
	else if (ClkSrc == OSC_TYPE_XTAL || ClkSrc == OSC_TYPE_TCXO)
	{
		if (OscFreq < LPC546XX_SYSOSC_FREQ_MIN || OscFreq > LPC546XX_SYSOSC_FREQ_MAX)
		{
			return false;
		}
		clksel = LPC546XX_PLLCLKSEL_CLKIN;
	}
	else
	{
		return false;
	}

	if (Lpc546xxPllFind(OscFreq, SYSTEM_CORE_CLOCK_MAX, &pll) == false)
	{
		return false;
	}

	state = DisableInterrupt();

	// Run from the FRO while the voltage, the flash access time and the PLL
	// change. The higher voltage and longer flash access are safe at 12 MHz.
	Lpc546xxMainClockFro12M();
	Lpc546xxSetVoltage(pll.Freq);
	Lpc546xxSetFlashTime(pll.Freq);

	if (clksel == LPC546XX_PLLCLKSEL_CLKIN)
	{
		// VD2_ANA supplies the system oscillator
		SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK;
		SYSCON->PDRUNCFGSET[1] = SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
		SYSCON->SYSOSCCTRL = (ClkSrc == OSC_TYPE_TCXO ? SYSCON_SYSOSCCTRL_BYPASS_MASK : 0U) |
							 (OscFreq > LPC546XX_SYSOSC_RANGE_LOW ? SYSCON_SYSOSCCTRL_FREQRANGE_MASK : 0U);
		SYSCON->PDRUNCFGCLR[1] = SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
	}

	if (Lpc546xxPllStart(clksel, &pll) == false)
	{
		if (clksel == LPC546XX_PLLCLKSEL_CLKIN)
		{
			SYSCON->PDRUNCFGSET[1] = SYSCON_PDRUNCFG_PDEN_SYSOSC_MASK;
		}
		Lpc546xxSetFlashTime(CLK_FRO_12MHZ);
		Lpc546xxSetVoltage(CLK_FRO_12MHZ);
		SystemCoreClockUpdate();
		EnableInterrupt(state);

		return false;
	}

	SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(LPC546XX_MAINCLKSELB_PLL);
	s_Lpc546xxMainFreq = pll.Freq;

	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;

	SystemCoreClockUpdate();
	EnableInterrupt(state);

	return true;
}

uint32_t SystemCoreClockGet(void)
{
	return SystemCoreClock;
}

/**
 * @brief	Get peripheral bus clock frequency.
 *
 * The LPC546xx peripherals are on the AHB clock. Flexcomm and timer function
 * clocks have their own selection in their drivers.
 *
 * @param	Idx : 0, the AHB clock
 *
 * @return	Frequency in Hz, 0 for an invalid index
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx == 0 ? SystemCoreClock : 0;
}

/**
 * @brief	Peripheral bus clock frequency in effect.
 *
 * The AHB clock is the CPU clock, AHBCLKDIV divides both. A peripheral
 * request does not change it.
 *
 * @return	Frequency in Hz of the AHB clock, 0 for an invalid index
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	(void)Freq;

	return SystemPeriphClockGet(Idx);
}

// CPU clock from the main clock the clock setup set and the AHB divider
void SystemCoreClockUpdate(void)
{
	SystemCoreClock = s_Lpc546xxMainFreq / ((SYSCON->AHBCLKDIV & 0xFFUL) + 1UL);
	(void)ArmDelayClockSetDivisors(SystemCoreClock,
		16000000UL, (LPC546XX_NSDELAY_LOOP_CLK * 1000000UL));
}

bool SystemHighFreqClockEnable(void)
{
	uint32_t state;
	bool ready;

	state = DisableInterrupt();
	ready = s_HfUsers != UINT32_MAX && s_Lpc546xxMainFreq != 0U;
	if (ready)
	{
		s_HfUsers++;
	}
	EnableInterrupt(state);
	return ready;
}

void SystemHighFreqClockDisable(void)
{
	uint32_t state;

	state = DisableInterrupt();
	if (s_HfUsers != 0U)
	{
		s_HfUsers--;
	}
	EnableInterrupt(state);
}

uint32_t SystemHighFreqClockStartTime(void)
{
	// The oscillator is started in SystemInit, Enable never waits for it
	return 0U;
}

// The low frequency oscillator is the RTC crystal, powered and checked by the
// RTC driver. No driver of this port takes a clock from this request.
bool SystemLowFreqClockEnable(void)
{
	return false;
}

void SystemLowFreqClockDisable(void)
{
}

uint32_t SystemLowFreqClockStartTime(void)
{
	return 0U;
}

/**
 * @brief	CPU running mode.
 *
 * LOW_POWER divides the main clock by two with AHBCLKDIV: the CPU, the AHB
 * clock, CTIMER0 to 2 and SysTick run at half the frequency. The Flexcomm
 * function clocks and CTIMER3 and 4 are on the 12 MHz FRO and do not change.
 * The flash access time follows the CPU clock. The core voltage stays at the
 * level of the PLL output, which still drives the divider.
 */
bool SystemPowerModeSet(SYSTEM_POWER_MODE Mode)
{
	uint32_t state;
	uint32_t div;
	uint32_t freq;

	if (__get_IPSR() != 0U || s_Lpc546xxMainFreq == 0U ||
		(Mode != SYSTEM_POWER_MODE_PERFORMANCE && Mode != SYSTEM_POWER_MODE_LOW_POWER))
	{
		return false;
	}
	div = Mode == SYSTEM_POWER_MODE_LOW_POWER ? 1U : 0U;
	freq = s_Lpc546xxMainFreq / (div + 1U);
	state = DisableInterrupt();
	if (Mode != s_PowerMode)
	{
		// The longer flash access before the faster clock, after the slower one
		if (Mode == SYSTEM_POWER_MODE_PERFORMANCE)
		{
			Lpc546xxSetFlashTime(freq);
		}
		SYSCON->AHBCLKDIV = div;
		if (Mode == SYSTEM_POWER_MODE_LOW_POWER)
		{
			Lpc546xxSetFlashTime(freq);
		}
		SystemCoreClockUpdate();
		s_PowerMode = Mode;
	}
	EnableInterrupt(state);
	return true;
}

SYSTEM_POWER_MODE SystemPowerModeGet(void)
{
	return s_PowerMode;
}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}

// ---- Audio PLL ----

// The audio PLL setting with the output closest to Fout: P from 1, N from 1,
// M rounded and kept in the CCO range. The first exact one is taken. A Fout
// out of the PLL range gets its lowest or highest output.
static bool Lpc546xxAudioPllFind(uint32_t Fin, uint32_t Fout, Lpc546xxPll_t *pPll)
{
	uint64_t best = UINT64_MAX;

	pPll->N = 1;
	pPll->M = 1;
	pPll->P = 1;
	pPll->Freq = 0;

	for (uint32_t p = 1; p <= PVALMAX; p++)
	{
		uint64_t div = (uint64_t)p << 1;

		for (uint32_t n = 1; n <= NVALMAX && Fin / n >= LPC546XX_AUDPLL_REF_MIN; n++)
		{
			uint64_t fin2 = (uint64_t)Fin << 1;
			// M from the CCO Fout x 2P, rounded, then the M range of the CCO.
			// With Fin / N 10 kHz or more, M is 27500 or less.
			uint64_t m = ((uint64_t)Fout * div * n + Fin) / fin2;
			uint64_t mlo = ((uint64_t)LPC546XX_PLL_CCO_MIN * n + fin2 - 1U) / fin2;
			uint64_t mhi = (uint64_t)LPC546XX_PLL_CCO_MAX * n / fin2;

			if (mlo > mhi)
			{
				continue;
			}
			if (m < mlo)
			{
				m = mlo;
			}
			else if (m > mhi)
			{
				m = mhi;
			}

			uint64_t f = (uint64_t)Fin * (m << 1) / n / div;
			uint64_t diff = f > Fout ? f - Fout : Fout - f;

			if (diff < best)
			{
				best = diff;
				pPll->N = n;
				pPll->M = (uint32_t)m;
				pPll->P = p;
				pPll->Freq = (uint32_t)f;
				if (diff == 0U)
				{
					return true;
				}
			}
		}
	}

	return pPll->Freq != 0U;
}

uint32_t Lpc546xxAudioPllStart(uint32_t Freq)
{
	uint32_t fin = Lpc546xxClkInFreq();
	uint32_t clksel = LPC546XX_PLLCLKSEL_CLKIN;
	Lpc546xxPll_t pll;

	if (fin == 0U)
	{
		fin = CLK_FRO_12MHZ;
		clksel = LPC546XX_PLLCLKSEL_FRO12M;
	}

	Lpc546xxAudioPllStop();

	if (Freq == 0U || Lpc546xxAudioPllFind(fin, Freq, &pll) == false)
	{
		return 0;
	}

	// VD3 supplies the PLLs
	uint32_t cnt = LPC546XX_VD_STATUS_WAIT;

	SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_VD3_MASK;
	while ((LPC546XX_VD_STATUS & LPC546XX_VD_STATUS_VD3_RDY) == 0U && --cnt > 0U);
	if (cnt == 0U)
	{
		return 0;
	}

	// The PLL is powered down while it is changed. A divider value is taken
	// when its REQ bit is written. The fractional divider is off (SEL_EXT).
	SYSCON->AUDPLLCLKSEL = SYSCON_AUDPLLCLKSEL_SEL(clksel);
	SYSCON->AUDPLLCTRL = Lpc546xxPllCtrl(pll.M, false);

	uint32_t ndec = SYSCON_AUDPLLNDEC_NDEC(pllEncodeN(pll.N));
	uint32_t pdec = SYSCON_AUDPLLPDEC_PDEC(pllEncodeP(pll.P));
	uint32_t mdec = SYSCON_AUDPLLMDEC_MDEC(pllEncodeM(pll.M));

	SYSCON->AUDPLLNDEC = ndec;
	SYSCON->AUDPLLNDEC = ndec | SYSCON_AUDPLLNDEC_NREQ_MASK;
	SYSCON->AUDPLLPDEC = pdec;
	SYSCON->AUDPLLPDEC = pdec | SYSCON_AUDPLLPDEC_PREQ_MASK;
	SYSCON->AUDPLLMDEC = mdec;
	SYSCON->AUDPLLMDEC = mdec | SYSCON_AUDPLLMDEC_MREQ_MASK;
	SYSCON->AUDPLLFRAC = SYSCON_AUDPLLFRAC_SEL_EXT_MASK;

	SYSCON->PDRUNCFGCLR[1] = SYSCON_PDRUNCFG_PDEN_AUD_PLL_MASK;

	cnt = LPC546XX_AUDPLL_LOCK_WAIT;
	while ((SYSCON->AUDPLLSTAT & SYSCON_AUDPLLSTAT_LOCK_MASK) == 0U && --cnt > 0U);
	if (cnt == 0U)
	{
		SYSCON->PDRUNCFGSET[1] = SYSCON_PDRUNCFG_PDEN_AUD_PLL_MASK;

		return 0;
	}

	s_Lpc546xxAudioPllFreq = pll.Freq;

	return pll.Freq;
}

void Lpc546xxAudioPllStop(void)
{
	SYSCON->PDRUNCFGSET[1] = SYSCON_PDRUNCFG_PDEN_AUD_PLL_MASK;
	s_Lpc546xxAudioPllFreq = 0;
}

uint32_t Lpc546xxAudioPllFreq(void)
{
	return s_Lpc546xxAudioPllFreq;
}
