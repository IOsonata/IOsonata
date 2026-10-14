/**-------------------------------------------------------------------------
@file	timer_lpc546xx.cpp

@brief	LPC546xx CTIMER clocks, descriptors and interrupt vectors.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include "timer_lpcxx.h"

LpcTimer_t g_LpcTimer[] = {
	{ .pReg = CTIMER3, .IrqNo = CTIMER3_IRQn, .bAsync = true,
	  .ClkMask = ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER3_MASK, .RstMask = ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER3_MASK, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = CTIMER4, .IrqNo = CTIMER4_IRQn, .bAsync = true,
	  .ClkMask = ASYNC_SYSCON_ASYNCAPBCLKCTRL_CTIMER4_MASK, .RstMask = ASYNC_SYSCON_ASYNCPRESETCTRL_CTIMER4_MASK, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = CTIMER0, .IrqNo = CTIMER0_IRQn, .bAsync = false,
	  .ClkMask = SYSCON_AHBCLKCTRL_CTIMER0_MASK, .RstMask = SYSCON_PRESETCTRL_CTIMER0_RST_MASK, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = CTIMER1, .IrqNo = CTIMER1_IRQn, .bAsync = false,
	  .ClkMask = SYSCON_AHBCLKCTRL_CTIMER1_MASK, .RstMask = SYSCON_PRESETCTRL_CTIMER1_RST_MASK, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = CTIMER2, .IrqNo = CTIMER2_IRQn, .bAsync = false,
	  .ClkMask = SYSCON_AHBCLKCTRL_CTIMER2_MASK, .RstMask = SYSCON_PRESETCTRL_CTIMER2_RST_MASK, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
};

const int g_LpcTimerCount = sizeof(g_LpcTimer) / sizeof(g_LpcTimer[0]);

uint32_t LpcTimerBaseFreq(const LpcTimer_t *pDev)
{
	return pDev->bAsync ? CLK_FRO_12MHZ : SystemCoreClock;
}

void LpcTimerClockEnable(const LpcTimer_t *pDev)
{
	// Clock on and peripheral reset. CTIMER3 and 4 are on the async APB
	// bridge, which runs from the 12 MHz FRO.
	if (pDev->bAsync)
	{
		SYSCON->ASYNCAPBCTRL = SYSCON_ASYNCAPBCTRL_ENABLE_MASK;
		ASYNC_SYSCON->ASYNCAPBCLKSELA = ASYNC_SYSCON_ASYNCAPBCLKSELA_SEL(1U);
		ASYNC_SYSCON->ASYNCAPBCLKCTRLSET = pDev->ClkMask;
		ASYNC_SYSCON->ASYNCPRESETCTRLSET = pDev->RstMask;
		ASYNC_SYSCON->ASYNCPRESETCTRLCLR = pDev->RstMask;
	}
	else
	{
		SYSCON->AHBCLKCTRLSET[1] = pDev->ClkMask;
		SYSCON->PRESETCTRLSET[1] = pDev->RstMask;
		SYSCON->PRESETCTRLCLR[1] = pDev->RstMask;
	}

}

extern "C" void CTIMER0_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[2]);
}

extern "C" void CTIMER1_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[3]);
}

extern "C" void CTIMER2_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[4]);
}

extern "C" void CTIMER3_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[0]);
}

extern "C" void CTIMER4_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[1]);
}
