/**-------------------------------------------------------------------------
@file	flexcomm_lpc546xx.cpp

@brief	LPC546xx Flexcomm interface selection and interrupt dispatch

		The Flexcomm interrupt vectors are shared by the USART, SPI and I2C
		functions. Each vector calls the handler set in g_SharedIntrf by the
		driver of the selected function.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "LPC546xx.h"
#include "device_intrf.h"
#include "coredev/shared_intrf.h"
#include "flexcomm_lpc546xx.h"

// FCLKSEL value of the 12 MHz FRO
#define LPC546XX_FCLKSEL_FRO12M		0U

typedef struct {
	FLEXCOMM_Type *pReg;		//!< Flexcomm registers
	IRQn_Type IrqNo;			//!< Interrupt number
	int CtrlIdx;				//!< AHBCLKCTRL and PRESETCTRL index
	uint32_t ClkMask;			//!< AHBCLKCTRL bit
	uint32_t RstMask;			//!< PRESETCTRL bit
} Lpc546xxFlexcomm_t;

static const Lpc546xxFlexcomm_t s_Lpc546xxFlexcomm[] = {
	{FLEXCOMM0, FLEXCOMM0_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM0_MASK, SYSCON_PRESETCTRL_FC0_RST_MASK},
	{FLEXCOMM1, FLEXCOMM1_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM1_MASK, SYSCON_PRESETCTRL_FC1_RST_MASK},
	{FLEXCOMM2, FLEXCOMM2_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM2_MASK, SYSCON_PRESETCTRL_FC2_RST_MASK},
	{FLEXCOMM3, FLEXCOMM3_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM3_MASK, SYSCON_PRESETCTRL_FC3_RST_MASK},
	{FLEXCOMM4, FLEXCOMM4_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM4_MASK, SYSCON_PRESETCTRL_FC4_RST_MASK},
	{FLEXCOMM5, FLEXCOMM5_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM5_MASK, SYSCON_PRESETCTRL_FC5_RST_MASK},
	{FLEXCOMM6, FLEXCOMM6_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM6_MASK, SYSCON_PRESETCTRL_FC6_RST_MASK},
	{FLEXCOMM7, FLEXCOMM7_IRQn, 1, SYSCON_AHBCLKCTRL_FLEXCOMM7_MASK, SYSCON_PRESETCTRL_FC7_RST_MASK},
	{FLEXCOMM8, FLEXCOMM8_IRQn, 2, SYSCON_AHBCLKCTRL_FLEXCOMM8_MASK, SYSCON_PRESETCTRL_FC8_RST_MASK},
	{FLEXCOMM9, FLEXCOMM9_IRQn, 2, SYSCON_AHBCLKCTRL_FLEXCOMM9_MASK, SYSCON_PRESETCTRL_FC9_RST_MASK},
};

#define LPC546XX_FLEXCOMM_CNT		((int)(sizeof(s_Lpc546xxFlexcomm) / sizeof(Lpc546xxFlexcomm_t)))

// One shared interrupt entry per Flexcomm, index is the Flexcomm number
const int g_SharedIntrfMaxCnt = LPC546XX_FLEXCOMM_CNT;
SharedIntrf_t g_SharedIntrf[LPC546XX_FLEXCOMM_CNT];

static void Lpc546xxFlexcommDispatch(int FcNo);

static void Lpc546xxFlexcommDispatch(int FcNo)
{
	if (g_SharedIntrf[FcNo].pIntrf != NULL && g_SharedIntrf[FcNo].Handler != NULL)
	{
		g_SharedIntrf[FcNo].Handler(FcNo, g_SharedIntrf[FcNo].pIntrf);
	}
}

int Lpc546xxFlexcommCount(void)
{
	return LPC546XX_FLEXCOMM_CNT;
}

uint32_t Lpc546xxFlexcommSelect(int FcNo, LPC546XX_FLEXCOMM_FUNC Func)
{
	if (FcNo < 0 || FcNo >= LPC546XX_FLEXCOMM_CNT || Func < LPC546XX_FLEXCOMM_USART || Func > LPC546XX_FLEXCOMM_I2C)
	{
		return 0;
	}

	const Lpc546xxFlexcomm_t *fc = &s_Lpc546xxFlexcomm[FcNo];

	Lpc546xxFlexcommClock(FcNo, true);

	// USARTPRESENT, SPIPRESENT and I2CPRESENT are consecutive bits. The lock
	// is checked before the reset, which would clear it.
	uint32_t psel = fc->pReg->PSELID;

	if ((psel & (FLEXCOMM_PSELID_USARTPRESENT_MASK << ((uint32_t)Func - 1U))) == 0U ||
		((psel & FLEXCOMM_PSELID_LOCK_MASK) != 0U && (psel & FLEXCOMM_PSELID_PERSEL_MASK) != (uint32_t)Func))
	{
		return 0;
	}

	SYSCON->PRESETCTRLSET[fc->CtrlIdx] = fc->RstMask;
	SYSCON->PRESETCTRLCLR[fc->CtrlIdx] = fc->RstMask;

	fc->pReg->PSELID = FLEXCOMM_PSELID_PERSEL(Func);
	SYSCON->FCLKSEL[FcNo] = LPC546XX_FCLKSEL_FRO12M;

	return CLK_FRO_12MHZ;
}

void Lpc546xxFlexcommClock(int FcNo, bool bOn)
{
	if (FcNo < 0 || FcNo >= LPC546XX_FLEXCOMM_CNT)
	{
		return;
	}

	const Lpc546xxFlexcomm_t *fc = &s_Lpc546xxFlexcomm[FcNo];

	if (bOn)
	{
		SYSCON->AHBCLKCTRLSET[fc->CtrlIdx] = fc->ClkMask;
	}
	else
	{
		SYSCON->AHBCLKCTRLCLR[fc->CtrlIdx] = fc->ClkMask;
	}
}

IRQn_Type Lpc546xxFlexcommIrqNo(int FcNo)
{
	return s_Lpc546xxFlexcomm[FcNo].IrqNo;
}

extern "C" void FLEXCOMM0_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(0);
}

extern "C" void FLEXCOMM1_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(1);
}

extern "C" void FLEXCOMM2_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(2);
}

extern "C" void FLEXCOMM3_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(3);
}

extern "C" void FLEXCOMM4_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(4);
}

extern "C" void FLEXCOMM5_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(5);
}

extern "C" void FLEXCOMM6_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(6);
}

extern "C" void FLEXCOMM7_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(7);
}

extern "C" void FLEXCOMM8_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(8);
}

extern "C" void FLEXCOMM9_IRQHandler(void)
{
	Lpc546xxFlexcommDispatch(9);
}
