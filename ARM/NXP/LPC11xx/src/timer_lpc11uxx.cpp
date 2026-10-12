/**-------------------------------------------------------------------------
@file	timer_lpc11uxx.cpp

@brief	LPC11Uxx CT16B/CT32B clocks, descriptors and interrupt vectors.

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

// All four timers use the system AHB clock. Narrow counters precede wide ones.
LpcTimer_t g_LpcTimer[] = {
	{ .pReg = LPC_CT16B0, .IrqNo = TIMER_16_0_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 7, .RstMask = 0, .CounterMask = 0xFFFFU, .PrescalerMax = 0xFFFFU, },
	{ .pReg = LPC_CT16B1, .IrqNo = TIMER_16_1_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 8, .RstMask = 0, .CounterMask = 0xFFFFU, .PrescalerMax = 0xFFFFU, },
	{ .pReg = LPC_CT32B0, .IrqNo = TIMER_32_0_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 9, .RstMask = 0, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = LPC_CT32B1, .IrqNo = TIMER_32_1_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 10, .RstMask = 0, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
};

const int g_LpcTimerCount = sizeof(g_LpcTimer) / sizeof(g_LpcTimer[0]);

uint32_t LpcTimerBaseFreq(const LpcTimer_t *pDev)
{
	(void)pDev;
	return SystemCoreClock;
}

void LpcTimerClockEnable(const LpcTimer_t *pDev)
{
	// CT16B/CT32B have no PRESETCTRL bits. The shared driver resets via TCR.
	LPC_SYSCON->SYSAHBCLKCTRL |= pDev->ClkMask;
	(void)LPC_SYSCON->SYSAHBCLKCTRL;
}

extern "C" void TIMER16_0_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[0]);
}

extern "C" void TIMER16_1_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[1]);
}

extern "C" void TIMER32_0_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[2]);
}

extern "C" void TIMER32_1_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[3]);
}

