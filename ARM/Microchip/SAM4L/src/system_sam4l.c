/**-------------------------------------------------------------------------
 @file	system_sam4l.c

 @brief	CMSIS System initializations

 Clock scheme

 Generic clock source (OSCSEL) (Table 13-8 page 260 in SAM4Lx Datasheet)
 0 - RCSYS
 1 - OSC32K
 2 - DFPLL0
 3 - OSC0
 4 - RC80M
 5 - RCFAST
 6 - RC1M
 7 - CLK_CPU
 8 - CLK_HSB
 9 - CLK_PBA
 10 - CLK_PBB
 11 - CLK_PBC
 12 - CLK_PBD
 13 - RC32K
 14 - Reserved
 15 - CLK_1K
 16 - PLL0
 17 - HRP
 18 - FP
 19-20 - GCLK_IN[0-1]
 21 - GCLK11

 PLL0 clock source (PLLOSC)
 0 - OSC0
 1 - Generic clock 9

 High Resolution/Fractional Prescaler clock source (CKSEL)
 0 - OSC0
 1 - PLL0
 2 - DFPLL0
 3 - Reserved
 4 - RC80M

 DFLL clock source (DFLL0) in Closed Loop Operation
 Generic Clock 9





 @author	Hoang Nguyen Hoan
 @date	June. 30, 2021
 @author	Thinh Tran
 @date	Mar. 18, 2022

 @license

 MIT License

 Copyright (c) 2021 I-SYST inc. All rights reserved.

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

#include "sam4lxxx.h"

#include "system_sam4l.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

#define SYSTEM_CORE_CLOCK				48000000UL	// Default core frequency
#define SYSTEM_NSDELAY_CORE_FACTOR		(27UL)		// Adjustment value for nanosec delay

#define MAINOSC_FREQ_MIN		3000000
#define MAINOSC_FREQ_MAX		20000000

#define USB_FREQ				48000000

#if defined(SAM4LSXA) || defined(SAM4LSXB) || defined(SAM4LSXC)
#define FVCO_FREQ_MAX			192000000UL
#else
#define FVCO_FREQ_MAX			240000000UL
#endif

#define PERIPH_CLOCK_MAX		3

// Generic Clock Sources for Generic clock controller (OSCSEL) in SCIF module
//(Table 13-8 in page 260 in SAM4Lx Datasheet)
#define RCSYS_GEN_CLK_SRC		0
#define OSC32K_GEN_CLK_SRC		1
#define	DFPLL0_GEN_CLK_SRC	 	2
#define	OSC0_GEN_CLK_SRC 		3
#define	RC80M_GEN_CLK_SRC 		4
#define	RCFAST_GEN_CLK_SRC		5
#define RC1M_GEN_CLK_SRC		6
#define CLK_CPU_GEN_CLK_SRC		7
#define CLK_HSB_GEN_CLK_SRC		8
#define CLK_PBA_GEN_CLK_SRC		9
#define	CLK_PBB_GEN_CLK_SRC		10
#define CLK_PBC_GEN_CLK_SRC		11
#define	CLK_PBD_GEN_CLK_SRC		12
#define	RC32K_GEN_CLK_SRC		13
//#define Reserved_Src	14
#define CLK_1K_GEN_CLK_SRC		15
#define PLL0_GEN_CLK_SRC		16
#define HRP_GEN_CLK_SRC			17
#define FP_GEN_CLK_SRC			18
#define GCLK0_GEN_CLK_SRC	19
#define GCLK1_GEN_CLK_SRC	20
#define GCLK11_GEN_CLK_SRC		21

// Clk_sources for Main Clock (Sync Clk Generator) in Power Manager module
// (Table 10-4 in page 117 in SAM4Lx Datasheet)
#define MCLK_RCSYS		0
#define MCLK_OSC0		1
#define MCLK_PLL		2
#define MCLK_DFLL		3
#define MCLK_RC80M		4
#define MCLK_RCFAST		5
#define MCLK_RC1M		6
//#define MCLK_Reserved	7

//// Flash waitstate
//__WEAK MCU_OSC g_McuOsc = {
//	OSC_TYPE_XTAL,
//	12000000,
//	OSC_TYPE_RC,
//	32000
//};

// Default oscillator configuration matches the SAM4L8 Xplained Pro eval kit —
// the official development board for this MCU family.  It carries a 12 MHz
// crystal on OSC0 and a 32.768 kHz crystal on OSC32K (per the SAM4L8 Xplained
// Pro User Guide, feature list).  Custom-board firmware should override this
// __WEAK symbol with its own g_McuOsc matching its hardware.
//
// OscDesc_t fields : {Type, Freq Hz, Accuracy PPM, LoadCap x10 pF}
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_XTAL, 12000000, 20, 180},		// 12 MHz main crystal, ~18 pF
	{OSC_TYPE_XTAL,    32768, 20, 125},		// 32.768 kHz LF crystal, ~12.5 pF
	false									// USB clock not enabled by default
};

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;
static uint32_t s_PllFreq = SYSTEM_CORE_CLOCK;

// f_main : the clock feeding the PM CPU/HSB/PBx dividers.  On reset this is
// RCSYS (~115 kHz), but SystemInit reconfigures it; each init branch updates
// this variable to reflect the new main clock source.  All of f_CPU,
// f_HSB, f_PBA..f_PBD are derived from this.
static uint32_t s_MainClock = SYSTEM_CORE_CLOCK;

// Apply a PM clock divider register (CPUSEL/HSBSEL/PBASEL/PBBSEL/PBCSEL/PBDSEL)
// to a source frequency.  All six registers have the same bit layout:
//   bit 7 : xxxDIV - 0 = no division (out = fin), 1 = divide enabled
//   bits 2:0 : xxxSEL - divider exponent; out = fin >> (xxxSEL + 1)
static inline uint32_t Sam4lApplyClkDiv(uint32_t fin, uint32_t selreg)
{
	if (selreg & PM_CPUSEL_CPUDIV)
	{
		return fin >> ((selreg & PM_CPUSEL_CPUSEL_Msk) + 1u);
	}
	return fin;
}

// Program all synchronous clock dividers (CPU/HSB share CPUSEL; PBA/PBB/PBC/PBD
// have separate registers with identical bit layout) to the same value.  SAM4L
// max f_CPU = f_HSB = f_PBx = 48 MHz at PS0 with HSEN - any f_main > 48 MHz must
// be divided down BEFORE the main clock switch, otherwise buses run out of spec.
static void Sam4lSetSyncClkDividers(uint32_t sel_val)
{
	volatile uint32_t * const regs[] = {
		&SAM4L_PM->PM_CPUSEL,
		&SAM4L_PM->PM_PBASEL,
		&SAM4L_PM->PM_PBBSEL,
		&SAM4L_PM->PM_PBCSEL,
		&SAM4L_PM->PM_PBDSEL,
	};

	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++)
	{
		while ((SAM4L_PM->PM_SR & PM_SR_CKRDY) == 0)
			{};
		SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
				| PM_UNLOCK_ADDR((uint32_t)regs[i] - (uint32_t)SAM4L_PM);
		*regs[i] = sel_val;
	}
}

static const uint32_t s_Fvco[] =
{ 192000000UL, 96000000UL, 48000000UL };
static const int s_FvcoCnt = sizeof(s_Fvco) / sizeof(uint32_t);

bool FlashWaitReady(uint32_t Timeout)
{
	bool res = false;

	while (Timeout != 0 && res == false)
	{
		res = SAM4L_HFLASHC->FLASHCALW_FSR & FLASHCALW_FSR_FRDY;
		Timeout--;
	}

	return res;
}

void FlashSendCommand(uint32_t Cmd, int PgNo)
{
	uint32_t d;

	if (FlashWaitReady(1000000))
	{
		d = SAM4L_HFLASHC->FLASHCALW_FCMD & ~FLASHCALW_FCMD_CMD_Msk;

		if (PgNo >= 0)
		{
			d = FLASHCALW_FCMD_KEY_KEY | FLASHCALW_FCMD_PAGEN(PgNo) | Cmd;
		}
		else
		{
			d |= (FLASHCALW_FCMD_KEY_KEY | Cmd);
		}

		SAM4L_HFLASHC->FLASHCALW_FCMD = d;
		FlashWaitReady(1000000);
	}
}

void SetFlashWaitState(uint32_t CoreFreq)
{
	if (CoreFreq < 12000000UL)
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSDIS, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR
				& ~(FLASHCALW_FCR_FWS | FLASHCALW_FCR_WS1OPT)) |
		FLASHCALW_FCR_FWS_0;

	}
	else if (CoreFreq < 24000000UL)
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSDIS, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR
				& ~FLASHCALW_FCR_FWS) |
		FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	}
	else if (CoreFreq < 48000000UL)
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSEN, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR
				& ~(FLASHCALW_FCR_FWS | FLASHCALW_FCR_WS1OPT)) |
		FLASHCALW_FCR_FWS_0;
	}
	else
	{
		FlashSendCommand(FLASHCALW_FCMD_CMD_HSEN, -1);
		SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR
				& ~FLASHCALW_FCR_FWS) |
		FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	}
}

bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	if (Freq < MAINOSC_FREQ_MIN || Freq > 20000000)
	{
		return false;
	}

	g_McuOsc.CoreOsc.Type = ClkSrc;

	if (ClkSrc == OSC_TYPE_RC)
	{
		if (Freq < 8000000)
		{
			g_McuOsc.CoreOsc.Freq = 4000000;
		}
		else if (Freq < 16000000)
		{
			g_McuOsc.CoreOsc.Freq = 8000000;
		}
		else
		{
			g_McuOsc.CoreOsc.Freq = 12000000;
		}
	}
	else
	{
		g_McuOsc.CoreOsc.Freq = Freq;
	}

	SystemInit();

	return true;
}

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	if (ClkSrc == OSC_TYPE_RC)
	{
		g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
		g_McuOsc.LowPwrOsc.Freq = 32000;
	}
	else
	{
		g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
		g_McuOsc.LowPwrOsc.Freq = 32768;
	}

	return true;
}

// Fvco = (PLLMUL+1)/PLLDIV • Fref, if PLLDIV >0
// Fvco = 2•(PLLMUL+1) • Fref, if PLLDIV = 0
// Fvco = SYSTEM_CORE_CLOCK = (PLLMUL+1)/PLLDIV • Fref
// SYSTEM_CORE_CLOCK / Fref = (PLLMUL+1)/PLLDIV
// SYSTEM_CORE_CLOCK * PLLDIV / Fref - 1 = PLLMUL
/// @return PLL frequency
/*void SystemSetPLL(uint32_t SrcFreq)
 {
 uint32_t div = 0;
 uint32_t mul = 0;
 uint32_t fvco = 0;//(96000000UL / SYSTEM_CORE_CLOCK) * SYSTEM_CORE_CLOCK;
 uint32_t pllopt = 0;

 for (int i = 15; i > 0; i--)
 {
 for (int j = 1; j < 16; j++)
 {
 uint32_t f = SrcFreq * (i + 1) / j;//external freq
 for (int x = 0; x < s_FvcoCnt; x++)
 {
 if (f == s_Fvco[x] && (f > fvco))
 {
 fvco = f;
 div = j;
 mul = i;
 }
 }
 }
 }

 s_PllFreq = (mul + 1) * SrcFreq / div;

 if (s_PllFreq > 160000000)
 {
 pllopt = 1;
 }

 //	uint32_t d =  SCIF_PLL_PLLDIV(div) | SCIF_PLL_PLLMUL(mul) |
 //  	   	   	   	   SCIF_PLL_PLLOPT(pllopt) | SCIF_PLL_PLLCOUNT_Msk |
 //  	   	   	   	   SCIF_PLL_PLLEN;
 SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu) |
 SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL - (uint32_t)SAM4L_SCIF);

 SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL = SCIF_PLL_PLLDIV(div) | SCIF_PLL_PLLMUL(mul) |
 SCIF_PLL_PLLOPT(pllopt) | SCIF_PLL_PLLCOUNT_Msk |
 SCIF_PLL_PLLEN;
 while (!(SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_PLL0LOCK));
 }*/

void SystemSetPLL()
{
	uint32_t div = 0;
	uint32_t mul = 0;
	uint32_t fvco = 0; //(96000000UL / SYSTEM_CORE_CLOCK) * SYSTEM_CORE_CLOCK;
	uint32_t pllopt = 0;

	for (int i = 15; i > 0; i--)
	{
		for (int j = 1; j < 16; j++)
		{
			uint32_t f = g_McuOsc.CoreOsc.Freq * (i + 1) / j;
			for (int x = 0; x < s_FvcoCnt; x++)
			{
				if (f == s_Fvco[x] && (f > fvco))
				{
					fvco = f;
					div = j;
					mul = i;
				}
			}
		}
	}

	s_PllFreq = (mul + 1) * g_McuOsc.CoreOsc.Freq / div;

	if (s_PllFreq > 160000000)
	{
		pllopt = 1;
	}

//	uint32_t d =  SCIF_PLL_PLLDIV(div) | SCIF_PLL_PLLMUL(mul) |
//  	   	   	   	   SCIF_PLL_PLLOPT(pllopt) | SCIF_PLL_PLLCOUNT_Msk |
//  	   	   	   	   SCIF_PLL_PLLEN;
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
			| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL - (uint32_t)SAM4L_SCIF);

	SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL = SCIF_PLL_PLLDIV(div)
			| SCIF_PLL_PLLMUL(mul) | SCIF_PLL_PLLOPT(pllopt)
			| SCIF_PLL_PLLCOUNT_Msk | SCIF_PLL_PLLEN;
	while (!(SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_PLL0LOCK))
		{};
}


/** Configure DFLL0 (in Closed loop mode) to output mul MHz
	 * Input: 1 MHz GenClock[0]
	 * Output: 96 MHz
**/
void ConfigDFLL0Freq(uint16_t mul)
{
	//Wait for Internal clock synchronization
	while ((SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY) == 0)
		{};

	// Set DFLL multiplier register at mul value
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0XAAu)
			| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_DFLL0MUL - (uint32_t)SAM4L_SCIF);
	SAM4L_SCIF->SCIF_DFLL0MUL = SCIF_DFLL0MUL_MUL(mul);// mul MHz

	// Configure DFLL Step_size register COARSE and FINE
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0XAAu)
			| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_DFLL0STEP - (uint32_t)SAM4L_SCIF);
	SAM4L_SCIF->SCIF_DFLL0STEP = SCIF_DFLL0STEP_CSTEP(8) | SCIF_DFLL0STEP_FSTEP(64);

	// Configure DFLL Value register COARSE and FINE
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0XAAu)
			| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_DFLL0VAL - (uint32_t)SAM4L_SCIF);
	SAM4L_SCIF->SCIF_DFLL0VAL = SCIF_DFLL0VAL_COARSE(16) | SCIF_DFLL0VAL_FINE(128);

	// Configure Closed loop operation mode
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0XAAu)
			| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_DFLL0CONF - (uint32_t)SAM4L_SCIF);
	SAM4L_SCIF->SCIF_DFLL0CONF = SCIF_DFLL0CONF_EN
			| SCIF_DFLL0CONF_MODE | SCIF_DFLL0CONF_RANGE(1);//50-110 MHz frequency range

	while ((SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_DFLL0RDY) == 0)
		{}; //Internal clock synchronization
}

void SystemInit()
{
	uint32_t temp = 0;

	SAM4L_PM->PM_UNLOCK =
			PM_UNLOCK_KEY(
					0xAAu) | PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_PBBMASK - (uint32_t)SAM4L_PM);
	SAM4L_PM->PM_PBBMASK |= PM_PBBMASK_HCACHE;

	SAM4L_HCACHE->HCACHE_CTRL = HCACHE_CTRL_CEN_YES;
	while (!(SAM4L_HCACHE->HCACHE_SR & HCACHE_SR_CSTS_EN))
		;

	// WDT is on by default.  Disable it.
	SAM4L_WDT->WDT_CTRL &= ~WDT_CTRL_EN;

	FlashSendCommand(FLASHCALW_FCMD_CMD_HSEN, -1);
	SAM4L_HFLASHC->FLASHCALW_FCR = (SAM4L_HFLASHC->FLASHCALW_FCR
			& ~FLASHCALW_FCR_FWS) |
	FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;

	/*** High freq clock ***/
	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_RC)
	{
		/** Internal RC **/
		switch (g_McuOsc.CoreOsc.Freq)
		{
		/** RC1M RC Osc as source clk **/
		case 1000000:
			/** BSCIF.RC1M --> SCIF.GenClk0 --> DFLL 96 MHz (close loop) --> PM.MCCCTRL 48 MHz **/

			// Activate RC1M clk in BSCIF module
			SAM4L_BSCIF->BSCIF_UNLOCK =
					BSCIF_UNLOCK_KEY(
							0xAAu) | BSCIF_UNLOCK_ADDR((uint32_t)&SAM4L_BSCIF->BSCIF_RC1MCR - (uint32_t)SAM4L_BSCIF);
			SAM4L_BSCIF->BSCIF_RC1MCR = BSCIF_RC1MCR_CLKOE
					| BSCIF_RC1MCR_CLKCAL(0) | BSCIF_RC1MCR_FCD;
			while ((SAM4L_BSCIF->BSCIF_RC1MCR & BSCIF_RC1MCR_CLKOE) == 0)
				{};

			// Allocate RC1M to GenClk0 for DFLL
			SAM4L_SCIF->SCIF_UNLOCK =
					SCIF_UNLOCK_KEY(0xAAu)
					| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_GCCTRL[0] - (uint32_t)SAM4L_SCIF);
			SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL = ~SCIF_GCCTRL_CEN; //disable clock
			SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL = SCIF_GCCTRL_CEN | (SCIF_GCCTRL_DIVEN & 0)
					| SCIF_GCCTRL_OSCSEL(RC1M_GEN_CLK_SRC) | SCIF_GCCTRL_DIV(0); //Enable GenClk with configuration
			while ((SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL & SCIF_GCCTRL_CEN) == 0)
				{};

			/* Configure DFLL at 96MHz */
			ConfigDFLL0Freq(96);

			// Pre-configure ALL synchronous clock dividers /2 BEFORE switching
			// main clock to DFLL.  f_main will be 96 MHz; everything needs /2
			// to stay within the 48 MHz spec max.
			Sam4lSetSyncClkDividers(PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0));

			// Now switch main clock to DFLL.  f_CPU = f_HSB = f_PBx = 48 MHz.
			SAM4L_PM->PM_UNLOCK =
					PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_DFLL0;

			s_MainClock = 96000000UL;
			SystemCoreClock = Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_CPUSEL);
			break;

		/** 4/8/12 MHz RC Osc (RCFAST) as main source clk **/
		case 4000000:
		case 8000000:
		case 12000000:
			/** SCIF.RCFAST (4/8/12 MHz) --> Gen_Clk[0] at 1MHz -->
			 * --> DFLL at 96MHz --> Main_CLK at 48MHz
			**/
			// Unlock write protection
			SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
					| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_RCFASTCFG - (uint32_t)SAM4L_SCIF);

			// Disable RCFAST before enabling it again
//			while ((SAM4L_SCIF->SCIF_RCFASTCFG & SCIF_RCFASTCFG_EN) == 0)
//				{};


			// Enabling RFAST again
			SAM4L_SCIF->SCIF_RCFASTCFG &= ~SCIF_RCFASTCFG_EN;
			while ((SAM4L_SCIF->SCIF_RCFASTCFG & SCIF_RCFASTCFG_EN) == 1)
				{};// wait until fully disable

			temp = SAM4L_SCIF->SCIF_RCFASTCFG & SCIF_RCFASTCFG_FRANGE_Msk;//FRANGE = 0b00
			SAM4L_SCIF->SCIF_RCFASTCFG = temp | SCIF_RCFASTCFG_EN
					| SCIF_RCFASTCFG_FRANGE(g_McuOsc.CoreOsc.Freq / 4000000 - 1);
			while ((SAM4L_SCIF->SCIF_RCFASTCFG & SCIF_RCFASTCFG_EN) == 0)
				{};// wait until fully enabled

			// Allocate RCFAST to GenClk0 for DFLL
			SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
					| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_GCCTRL[0] - (uint32_t)SAM4L_SCIF);

			// Enable & using RCFAST & using divider
			SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL = SCIF_GCCTRL_CEN
					| SCIF_GCCTRL_OSCSEL(RCFAST_GEN_CLK_SRC) | SCIF_GCCTRL_DIVEN;

			// Configure divider of GenClk0 to output 1 MHz
			switch (g_McuOsc.CoreOsc.Freq)
			{
			case 4000000:
				SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL |= SCIF_GCCTRL_DIV(1);
				break;
			case 8000000:
				SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL |= SCIF_GCCTRL_DIV(3);
				break;
			case 12000000:
			default:
				SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL |= SCIF_GCCTRL_DIV(5);
				break;

			}
			while ((SAM4L_SCIF->SCIF_GCCTRL[0].SCIF_GCCTRL & SCIF_GCCTRL_CEN) == 0)
				{};

			/* Configure DFLL at 96MHz */
			ConfigDFLL0Freq(96);

			/* Configure PM module */
			// Pre-configure ALL synchronous clock dividers /2 BEFORE switching
			// to DFLL.  f_main will be 96 MHz; every bus needs /2 for 48 MHz.
			Sam4lSetSyncClkDividers(PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0));

			// Now switch main clock to DFLL
			SAM4L_PM->PM_UNLOCK =
					PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_DFLL0;

			s_MainClock = 96000000UL;
			SystemCoreClock = Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_CPUSEL);
			break;

		/** RC80M RC Osc as main source clk **/
		case 80000000:
		default:
			g_McuOsc.CoreOsc.Freq = 80000000;

			// Unlock the RC80MCR register
			SAM4L_SCIF->SCIF_UNLOCK =SCIF_UNLOCK_KEY(0xAAu)
					| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_RC80MCR - (uint32_t)SAM4L_SCIF);
			SAM4L_SCIF->SCIF_RC80MCR &= ~SCIF_RC80MCR_EN;//disable first
			// Enable RC80M register
			SAM4L_SCIF->SCIF_RC80MCR |= SCIF_RC80MCR_EN;
			while ((SAM4L_SCIF->SCIF_RC80MCR & SCIF_RC80MCR_EN) == 0)
				{};

			// Pre-configure ALL synchronous clock dividers /2 BEFORE switching
			// main clock to RC80M.  f_main will be 80 MHz; every bus needs /2
			// to stay within 48 MHz spec (result = 40 MHz).
			Sam4lSetSyncClkDividers(PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0));

			// Now switch main clock to RC80M
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_RC80M;

			s_MainClock = 80000000UL;
			SystemCoreClock = Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_CPUSEL);
			break;
		}
	}
	else
	{
		/** External Crystal or TCXO **/
		uint32_t gain = g_McuOsc.CoreOsc.Freq / 4000000;
		uint32_t mode = 0;	// if g_McuOsc.HFType == OSC_TYPE_TCXO external clock

		if (g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL)
		{
			// external crystal
			mode = (0x1u << SCIF_OSCCTRL0_MODE_Pos);
		}

		SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAu)
				| SCIF_UNLOCK_ADDR((uint32_t)&SAM4L_SCIF->SCIF_OSCCTRL0 - (uint32_t)SAM4L_SCIF);
		SAM4L_SCIF->SCIF_OSCCTRL0 = SCIF_OSCCTRL0_OSCEN | mode
				| SCIF_OSCCTRL0_GAIN(gain) | SCIF_OSCCTRL0_STARTUP(2);
		while ((SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_OSC0RDY) == 0)
			{};

		// Configure PLL clock
		// TODO: Use GenClk9 as input ref_clk as well
		SystemSetPLL();

		// Pre-configure ALL synchronous clock dividers so every bus stays
		// within 48 MHz after the MCCTRL switch.  If PLL is already <= 48 MHz,
		// no division is needed; otherwise pick the smallest power-of-2 divider.
		uint32_t sel_val = 0;	// no division

		if (s_PllFreq > SYSTEM_CORE_CLOCK)
		{
			uint32_t div_exp = 30 - __CLZ(s_PllFreq / SYSTEM_CORE_CLOCK);
			sel_val = PM_CPUSEL_CPUSEL(div_exp) | PM_CPUSEL_CPUDIV;
		}

		Sam4lSetSyncClkDividers(sel_val);

		// Only switch main clock once the PLL has actually locked.  If it did
		// not lock, main clock stays on the previous source and SystemCoreClock
		// is left alone (reported value will be stale but matches nothing dangerous).
		if (SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_PLL0LOCK)
		{
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_PLL0;

			s_MainClock = s_PllFreq;
			SystemCoreClock = Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_CPUSEL);
		}
	}

	SetFlashWaitState(SystemCoreClock);

	/** Low-freq clock configuration **/
	if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC)
	{
		uint32_t rc32ctrl = SAM4L_BSCIF->BSCIF_RC32KCR;
		rc32ctrl &= BSCIF_RC32KCR_FCD;
		rc32ctrl |= BSCIF_RC32KCR_TCEN
				| BSCIF_RC32KCR_EN32K | BSCIF_RC32KCR_REF | BSCIF_RC32KCR_EN;

		SAM4L_BSCIF->BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAu)
				| BSCIF_UNLOCK_ADDR((uint32_t)&SAM4L_BSCIF->BSCIF_RC32KCR - (uint32_t)SAM4L_BSCIF);
		SAM4L_BSCIF->BSCIF_RC32KCR = rc32ctrl;
		while (!(SAM4L_BSCIF->BSCIF_PCLKSR & BSCIF_PCLKSR_RC32KRDY))
			{};
	}
	else
	{
		// External XTAL/TCXO
		uint32_t oscctrl = BSCIF_OSCCTRL32_STARTUP(
				2) | BSCIF_OSCCTRL32_SELCURR(10) |
				BSCIF_OSCCTRL32_EN32K | BSCIF_OSCCTRL32_OSC32EN;

		if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL)
		{
			oscctrl |= BSCIF_OSCCTRL32_MODE(1);
		}

		SAM4L_BSCIF->BSCIF_UNLOCK =
				BSCIF_UNLOCK_KEY(
						0xAAu) | BSCIF_UNLOCK_ADDR((uint32_t)&SAM4L_BSCIF->BSCIF_OSCCTRL32 - (uint32_t)SAM4L_BSCIF);
		SAM4L_BSCIF->BSCIF_OSCCTRL32 = oscctrl;
		while (!(SAM4L_BSCIF->BSCIF_PCLKSR & BSCIF_PCLKSR_OSC32RDY))
			{};
	}

	//SystemCoreClockUpdate();
}

void SystemCoreClockUpdate(void)
{
	// All main-clock sources ultimately route through PM_CPUSEL.  Recompute
	// f_CPU from whatever s_MainClock currently holds plus the live CPUSEL.
	SystemCoreClock = Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_CPUSEL);

	SetFlashWaitState(SystemCoreClock);
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}

/**
 * @brief	Get peripheral clock frequency
 *
 * Returns the clock of the selected peripheral bus on SAM4L:
 *   Idx 0 -> PBA (USART, SPI, TWIM, ADC...)
 *   Idx 1 -> PBB (PDCA, HRAMC1, USBC, CRCCU, HMATRIX...)
 *   Idx 2 -> PBC (CHIPID, FREQM, GPIO, PM, SCIF...)
 *   Idx 3 -> PBD (AST, WDT, EIC, PICOUART, BPM, BSCIF...)
 *
 * Each PBx bus has its own divider off f_main (independent of f_CPU).
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	switch (Idx)
	{
		case 0:  return Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_PBASEL);
		case 1:  return Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_PBBSEL);
		case 2:  return Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_PBCSEL);
		case 3:  return Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_PBDSEL);
		default: return 0;
	}
}

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Not implemented.  Peripheral bus dividers are set by SystemInit; changing
 * them at runtime requires re-applying flash waitstates and coordinating with
 * every active peripheral.
 *
 * @param	Idx  : Zero based peripheral clock number.
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	return SystemPeriphClockGet(Idx);
}

