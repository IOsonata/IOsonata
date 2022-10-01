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
#include "interrupt.h"

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

// Flash waitstate
__WEAK McuOsc_t g_McuOsc = {
	{OSC_TYPE_RC, 1000000, 20},
	{OSC_TYPE_RC, 32000, 20},
	false
};

uint32_t SystemCoreClock = SYSTEM_CORE_CLOCK;
uint32_t SystemnsDelayFactor = SYSTEM_NSDELAY_CORE_FACTOR;
static uint32_t s_PllFreq = SYSTEM_CORE_CLOCK;

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

			// Select DFLL as Main_Clk_Source in PM module
			SAM4L_PM->PM_UNLOCK =
					PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL |= PM_MCCTRL_MCSEL_DFLL0;

			// CPU_Clk_Select Register Set Divider
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_CPUSEL - (uint32_t)SAM4L_PM);
			// wait until PM_SR.CKRDY is high
			while ((SAM4L_PM->PM_SR & PM_SR_CKRDY) == 0)
				{};
			SAM4L_PM->PM_CPUSEL = PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0);// f_cpu = f_ref / 2 = 48 MHz
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
			// Select DFLL as Main_Clk_Source in PM module
			SAM4L_PM->PM_UNLOCK =
					PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL |= PM_MCCTRL_MCSEL_DFLL0;

			// CPU_Clk_Select Register Set Divider
			SAM4L_PM->PM_UNLOCK =
					PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_CPUSEL - (uint32_t)SAM4L_PM);
			while ((SAM4L_PM->PM_SR & PM_SR_CKRDY) == 0)
				{};	// wait until PM_SR.CKRDY is high
			SAM4L_PM->PM_CPUSEL = PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0);// f_cpu = f_ref / 2 = 48 MHz
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

			// Select RC80M as Main_Clk_Source in PM module
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL |= PM_MCCTRL_MCSEL_RC80M;

			// CPU_Clk_Select Register Set Divider
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_CPUSEL - (uint32_t)SAM4L_PM);
			while ((SAM4L_PM->PM_SR & PM_SR_CKRDY) == 0)
				{};// wait until PM_SR.CKRDY is high
			SAM4L_PM->PM_CPUSEL = PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(0);// f_cpu = RC80M_clk / 2^(0+1) = 40MHz
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

		// PLL reg = 0x3f030109
		uint32_t cpudiv = SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL;

		if (s_PllFreq > SYSTEM_CORE_CLOCK)
		{
//			cpudiv = s_PllFreq / 48000000;
//			SystemCoreClock = s_PllFreq / cpudiv;

			cpudiv = 30 - __CLZ(s_PllFreq / SYSTEM_CORE_CLOCK);

			SystemCoreClock = s_PllFreq / (1 << (cpudiv + 1));

			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_CPUSEL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_CPUSEL = cpudiv | PM_CPUSEL_CPUDIV;
		}

		if (SAM4L_SCIF->SCIF_PCLKSR & SCIF_PCLKSR_PLL0LOCK)
		{
			SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAu)
					| PM_UNLOCK_ADDR((uint32_t)&SAM4L_PM->PM_MCCTRL - (uint32_t)SAM4L_PM);
			SAM4L_PM->PM_MCCTRL = PM_MCCTRL_MCSEL_PLL0;
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

	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_RC)
	{
		// TODO: Read back the SystemCoreClock
		printf("SystemCoreClock = %d\n", SystemCoreClock);
	}
	else if (g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL)
	{
		// XTAL
		uint32_t pll = SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL;
		uint32_t mul = (pll & SCIF_PLL_PLLMUL_Msk) >> SCIF_PLL_PLLMUL_Pos;
		uint32_t div = (pll & SCIF_PLL_PLLDIV_Msk) >> SCIF_PLL_PLLDIV_Pos;
		uint32_t fvco =
				div > 0 ?
						(mul + 1) * g_McuOsc.CoreOsc.Freq / div :
						((mul + 1) << 1) * g_McuOsc.CoreOsc.Freq;
		uint32_t cpusel = SAM4L_PM->PM_CPUSEL;

		if (pll & SCIF_PLL_PLLOPT(2))
		{
			fvco >>= 1;
		}

		if (cpusel & PM_CPUSEL_CPUDIV)
		{
			SystemCoreClock = fvco
					/ (1 << ((cpusel & PM_CPUSEL_CPUSEL_Msk) + 1));
		}
		else
		{
			SystemCoreClock = fvco;
		}
	}
	else if (g_McuOsc.CoreOsc.Type == OSC_TYPE_TCXO)
	{
		printf("TCXO: external oscillator\n");
	}
	else
	{
		printf("Wrong setting\n");
		while(1);
	}

	SetFlashWaitState(SystemCoreClock);
}

uint32_t SystemCoreClockGet()
{
	return SystemCoreClock;
}

/**
 * @brief	Get peripheral clock frequency
 *
 * Peripheral clock on the SAM4E is the same as MCK which is the core clock
 *
 * @param	Idx : Zero based peripheral clock number. Many processors can
 * 				  have more than 1 peripheral clock settings.
 *
 * @return	Peripheral clock frequency in Hz.
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	// TODO: Get the clock of the peripheral APBx
	return SystemCoreClock;
}

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Peripheral clock on the SAM4E is the same as MCK which is the core clock
 * there is no settings to change freq.
 *
 * @param	Idx  : Zero based peripheral clock number. Many processors can
 * 				   have more than 1 peripheral clock settings.
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	// TODO: Configure the clock for peripherals APBx
	return SystemPeriphClockGet(Idx);
}

