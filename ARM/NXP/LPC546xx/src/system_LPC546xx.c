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

extern void *__Vectors;

__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, 12000000, 0},
	{OSC_TYPE_RC, 32000, 0},
	true
};

static const uint8_t wdtFreqLookup[32] = {0, 8, 12, 15, 18, 20, 24, 26, 28, 30, 32, 34, 36, 38, 40, 41, 42, 44, 45, 46,
                                            48, 49, 50, 52, 53, 54, 56, 57, 58, 59, 60, 61};
// Flash access time is FLASHTIM + 1 system clocks. Entry n is the highest
// system clock for FLASHTIM n.
static const uint32_t s_FlashTimFreq[] = {
	12000000UL, 24000000UL, 36000000UL, 60000000UL, 96000000UL,
	120000000UL, 144000000UL, 168000000UL, 180000000UL
};
uint32_t SystemCoreClock = DEFAULT_SYSTEM_CLOCK;
uint32_t SystemnsDelayFactor = LPC546XX_NSDELAY_LOOP_CLK * 1000000000UL / DEFAULT_SYSTEM_CLOCK;


/* Get WATCH DOG Clk */
static uint32_t getWdtOscFreq(void)
{
    uint8_t freq_sel, div_sel;
    if (SYSCON->PDRUNCFG[0] & SYSCON_PDRUNCFG_PDEN_WDT_OSC_MASK)
    {
        return 0U;
    }
    else
    {
        div_sel = (uint8_t)((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_DIVSEL_MASK) + 1UL) << 1UL;
        freq_sel = wdtFreqLookup[((SYSCON->WDTOSCCTRL & SYSCON_WDTOSCCTRL_FREQSEL_MASK) >> SYSCON_WDTOSCCTRL_FREQSEL_SHIFT)];
        return ((uint32_t) freq_sel * 50000U)/((uint32_t)div_sel);
    }
}
/* Find decoded N value for raw NDEC value */
static uint32_t pllDecodeN(uint32_t NDEC)
{
    uint32_t n, x, i;

    /* Find NDec */
    switch (NDEC)
    {
        case 0x3FF:
            n = 0UL;
            break;
        case 0x302:
            n = 1UL;
            break;
        case 0x202:
            n = 2UL;
            break;
        default:
            x = 0x080UL;
            n = 0xFFFFFFFFUL;
            for (i = NVALMAX; i >= 3UL; i--)
            {
                x = (((x ^ (x >> 2UL) ^ (x >> 3UL) ^ (x >> 4UL)) & 1UL) << 7UL) | ((x >> 1UL) & 0x7FUL);
                if ((x & (PLL_NDEC_VAL_M >> PLL_NDEC_VAL_P)) == NDEC)
                {
                    /* Decoded value of NDEC */
                    n = i;
                }
                if (n != 0xFFFFFFFFUL)
                {
                    break;
                }
            }
            break;
    }
    return n;
}

/* Find decoded P value for raw PDEC value */
static uint32_t pllDecodeP(uint32_t PDEC)
{
    uint32_t p, x, i;
    /* Find PDec */
    switch (PDEC)
    {
        case 0x7F:
            p = 0UL;
            break;
        case 0x62:
            p = 1UL;
            break;
        case 0x42:
            p = 2UL;
            break;
        default:
            x = 0x10UL;
            p = 0xFFFFFFFFUL;
            for (i = PVALMAX; i >= 3UL; i--)
            {
                x = (((x ^ (x >> 2UL)) & 1UL) << 4UL) | ((x >> 1UL) & 0xFUL);
                if ((x & (PLL_PDEC_VAL_M >> PLL_PDEC_VAL_P)) == PDEC)
                {
                    /* Decoded value of PDEC */
                    p = i;
                }
                if (p != 0xFFFFFFFFUL)
                {
                    break;
                }
            }
            break;
    }
    return p;
}

/* Find decoded M value for raw MDEC value */
static uint32_t pllDecodeM(uint32_t MDEC)
{
    uint32_t m, i, x;

    /* Find MDec */
    switch (MDEC)
    {
        case 0x1FFFF:
            m = 0UL;
            break;
        case 0x18003:
            m = 1UL;
            break;
        case 0x10003:
            m = 2UL;
            break;
        default:
            x = 0x04000UL;
            m = 0xFFFFFFFFUL;
            for (i = MVALMAX; i >= 3UL; i--)
            {
                x = (((x ^ (x >> 1UL)) & 1UL) << 14UL) | ((x >> 1UL) & 0x3FFFUL);
                if ((x & (PLL_MDEC_VAL_M >> PLL_MDEC_VAL_P)) == MDEC)
                {
                    /* Decoded value of MDEC */
                    m = i;
                }
                if (m != 0xFFFFFFFFUL)
                {
                    break;
                }
            }
            break;
    }
    return m;
}

/* Get predivider (N) from PLL NDEC setting */
static uint32_t findPllPreDiv(uint32_t ctrlReg, uint32_t nDecReg)
{
    uint32_t preDiv = 1;

    /* Direct input is not used? */
    if ((ctrlReg & SYSCON_SYSPLLCTRL_DIRECTI_MASK) == 0UL)
    {
        /* Decode NDEC value to get (N) pre divider */
        preDiv = pllDecodeN(nDecReg & 0x3FFUL);
        if (preDiv == 0UL)
        {
            preDiv = 1;
        }
    }
    /* Adjusted by 1, directi is used to bypass */
    return preDiv;
}

/* Get postdivider (P) from PLL PDEC setting */
static uint32_t findPllPostDiv(uint32_t ctrlReg, uint32_t pDecReg)
{
    uint32_t postDiv = 1;

    /* Direct input is not used? */
    if ((ctrlReg & SYSCON_SYSPLLCTRL_DIRECTO_MASK) == 0UL)
    {
        /* Decode PDEC value to get (P) post divider */
        postDiv = 2UL * pllDecodeP(pDecReg & 0x7FUL);
        if (postDiv == 0UL)
        {
            postDiv = 2;
        }
    }
    /* Adjusted by 1, directo is used to bypass */
    return postDiv;
}

/* Get multiplier (M) from PLL MDEC and BYPASS_FBDIV2 settings */
static uint32_t findPllMMult(uint32_t ctrlReg, uint32_t mDecReg)
{
    uint32_t mMult = 1;

    /* Decode MDEC value to get (M) multiplier */
    mMult = pllDecodeM(mDecReg & 0x1FFFFUL);
    if (mMult == 0UL)
    {
        mMult = 1;
    }
    return mMult;
}

// NDEC, PDEC and MDEC encodings, the reverse of the decoders above
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
	return g_McuOsc.CoreOsc.Type != OSC_TYPE_RC ? g_McuOsc.CoreOsc.Freq : 0U;
}

void SystemInit(void)
{
#if ((__FPU_PRESENT == 1) && (__FPU_USED == 1))
  SCB->CPACR |= ((3UL << 10*2) | (3UL << 11*2));    /* set CP10, CP11 Full Access */
#endif /* ((__FPU_PRESENT == 1) && (__FPU_USED == 1)) */

#if defined(__MCUXPRESSO)
    extern void(*const g_pfnVectors[]) (void);
    SCB->VTOR = (uint32_t) &g_pfnVectors;
#else
    extern void *__Vectors;
    SCB->VTOR = (uint32_t) &__Vectors;
#endif
    SYSCON->ARMTRACECLKDIV = 0;
/* Optionally enable RAM banks that may be off by default at reset */
#if !defined(DONT_ENABLE_DISABLED_RAMBANKS)
    SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_SRAM1_MASK | SYSCON_AHBCLKCTRL_SRAM2_MASK | SYSCON_AHBCLKCTRL_SRAM3_MASK;
#endif

	// On failure the PLL is tried from the FRO. If it does not lock either,
	// the core stays on the 12 MHz FRO.
	if (SystemCoreClockSelect(g_McuOsc.CoreOsc.Type, g_McuOsc.CoreOsc.Freq) == false)
	{
		SystemCoreClockSelect(OSC_TYPE_RC, CLK_FRO_12MHZ);
	}
}

/**
 * @brief	Select the core clock oscillator.
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
 * 			false - invalid oscillator, or the PLL did not lock. The core is
 * 			then on the 12 MHz FRO when the PLL was tried.
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t clksel = LPC546XX_PLLCLKSEL_FRO12M;
	Lpc546xxPll_t pll;

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

	// Run from the FRO while the voltage, the flash access time and the PLL
	// change. The higher voltage and longer flash access are safe at 12 MHz.
	Lpc546xxMainClockFro12M();
	Lpc546xxSetVoltage(pll.Freq);
	Lpc546xxSetFlashTime(pll.Freq);

	if (clksel == LPC546XX_PLLCLKSEL_CLKIN)
	{
		// VD2_ANA supplies the system oscillator
		SYSCON->PDRUNCFGCLR[0] = SYSCON_PDRUNCFG_PDEN_VD2_ANA_MASK;
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

		return false;
	}

	SYSCON->MAINCLKSELB = SYSCON_MAINCLKSELB_SEL(LPC546XX_MAINCLKSELB_PLL);

	g_McuOsc.CoreOsc.Type = ClkSrc;
	g_McuOsc.CoreOsc.Freq = OscFreq;

	SystemCoreClockUpdate();

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

/* ----------------------------------------------------------------------------
   -- SystemCoreClockUpdate()
   ---------------------------------------------------------------------------- */

void SystemCoreClockUpdate (void) {
uint32_t clkRate = 0;
    uint32_t prediv, postdiv;
    uint64_t workRate;

    switch (SYSCON->MAINCLKSELB & SYSCON_MAINCLKSELB_SEL_MASK)
    {
        case 0x00: /* MAINCLKSELA clock (main_clk_a)*/
            switch (SYSCON->MAINCLKSELA & SYSCON_MAINCLKSELA_SEL_MASK)
            {
                case 0x00: /* FRO 12 MHz (fro_12m) */
                    clkRate = CLK_FRO_12MHZ;
                    break;
                case 0x01: /* CLKIN Source (clk_in) */
                    clkRate = Lpc546xxClkInFreq();
                    break;
                case 0x02: /* Watchdog oscillator (wdt_clk) */
                    clkRate = getWdtOscFreq();
                    break;
                default: /* = 0x03 = FRO 96 or 48 MHz (fro_hf) */
                    if ((SYSCON->FROCTRL & SYSCON_FROCTRL_SEL_MASK) == SYSCON_FROCTRL_SEL_MASK)
                    {
                        clkRate = CLK_FRO_96MHZ;
                    }
                    else
                    {
                        clkRate = CLK_FRO_48MHZ;
                    }
                    break;
            }
            break;
        case 0x02: /* System PLL clock (pll_clk)*/
            switch (SYSCON->SYSPLLCLKSEL & SYSCON_SYSPLLCLKSEL_SEL_MASK)
            {
                case 0x00: /* FRO 12 MHz (fro_12m) */
                    clkRate = CLK_FRO_12MHZ;
                    break;
                case 0x01: /* CLKIN Source (clk_in) */
                    clkRate = Lpc546xxClkInFreq();
                    break;
                case 0x02: /* Watchdog oscillator (wdt_clk) */
                    clkRate = getWdtOscFreq();
                    break;
                case 0x03: /* RTC oscillator 32 kHz output (32k_clk) */
                    clkRate = CLK_RTC_32K_CLK;
                    break;
                default:
                    break;
            }
            if ((SYSCON->SYSPLLCTRL & SYSCON_SYSPLLCTRL_BYPASS_MASK) == 0UL)
            {
                /* PLL is not in bypass mode, get pre-divider, post-divider, and M divider */
                prediv = findPllPreDiv(SYSCON->SYSPLLCTRL, SYSCON->SYSPLLNDEC);
                postdiv = findPllPostDiv(SYSCON->SYSPLLCTRL, SYSCON->SYSPLLPDEC);
                /* Adjust input clock */
                clkRate = clkRate / prediv;

                /* MDEC used for rate */
                workRate = (uint64_t)(clkRate) * (uint64_t)findPllMMult(SYSCON->SYSPLLCTRL, SYSCON->SYSPLLMDEC);
                clkRate = (uint32_t)(workRate / ((uint64_t)postdiv));
                clkRate = clkRate * 2; /* PLL CCO output is divided by 2 before to M-Divider */
            }
            break;
        case 0x03: /* RTC oscillator 32 kHz output (32k_clk) */
            clkRate = CLK_RTC_32K_CLK;
            break;
        default:
            break;
    }
    SystemCoreClock = clkRate / ((SYSCON->AHBCLKDIV & 0xFFUL) + 1UL);
	if (SystemCoreClock != 0)
	{
		SystemnsDelayFactor = LPC546XX_NSDELAY_LOOP_CLK * 1000000000UL / SystemCoreClock;
	}
}

/* ----------------------------------------------------------------------------
   -- SystemInitHook()
   ---------------------------------------------------------------------------- */

__attribute__ ((weak)) void SystemInitHook (void) {
  /* Void implementation of the weak function. */
}
