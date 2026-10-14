/**-------------------------------------------------------------------------
@file	timer_lpc17xx.cpp

@brief	LPC17xx TIMER0..3 clocks, descriptors and interrupt vectors.

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
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

// Four equal 32-bit timers; DevNo 0..3 maps to TIMER0..3.
LpcTimer_t g_LpcTimer[] = {
	{ .pReg = LPC_TIM0, .IrqNo = TIMER0_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 1, .RstMask = 0, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = LPC_TIM1, .IrqNo = TIMER1_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 2, .RstMask = 0, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = LPC_TIM2, .IrqNo = TIMER2_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 22, .RstMask = 0, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
	{ .pReg = LPC_TIM3, .IrqNo = TIMER3_IRQn, .bAsync = false,
	  .ClkMask = 1UL << 23, .RstMask = 0, .CounterMask = UINT32_MAX, .PrescalerMax = UINT32_MAX, },
};

const int g_LpcTimerCount = sizeof(g_LpcTimer) / sizeof(g_LpcTimer[0]);
static const int s_TimerPclk[] = {
	LPC17_PCLK_TIMER0, LPC17_PCLK_TIMER1, LPC17_PCLK_TIMER2, LPC17_PCLK_TIMER3
};

uint32_t LpcTimerBaseFreq(const LpcTimer_t *pDev)
{
	return SystemPeriphClockGet(s_TimerPclk[pDev - g_LpcTimer]);
}

void LpcTimerClockEnable(const LpcTimer_t *pDev)
{
	uint32_t state = DisableInterrupt();

	// Preserve the selected peripheral divider and other peripherals' power.
	LPC_SC->PCONP |= pDev->ClkMask;
	(void)LPC_SC->PCONP;
	EnableInterrupt(state);
}

extern "C" void TIMER0_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[0]);
}

extern "C" void TIMER1_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[1]);
}

extern "C" void TIMER2_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[2]);
}

extern "C" void TIMER3_IRQHandler(void)
{
	LpcTimerIrqHandler(&g_LpcTimer[3]);
}
