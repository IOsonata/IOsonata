/**-------------------------------------------------------------------------
@file	timer_lpc_target.h

@brief	LPC17xx vendor timer register access for the shared LPC driver.

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
#ifndef IOSONATA_TIMER_LPC_TARGET_H
#define IOSONATA_TIMER_LPC_TARGET_H
#include "LPC17xx.h"
typedef LPC_TIM_TypeDef LpcTimerReg_t;

static inline volatile uint32_t *LpcTimerMatchReg(LpcTimerReg_t *pReg, unsigned Index)
{
	switch (Index)
	{
		case 0:
			return &pReg->MR0;
		case 1:
			return &pReg->MR1;
		case 2:
			return &pReg->MR2;
		default:
			return &pReg->MR3;
	}
}

static inline void LpcTimerDisablePwm(LpcTimerReg_t *pReg)
{
	// TIMER0..3 have no PWM control register.
	(void)pReg;
}
#endif
