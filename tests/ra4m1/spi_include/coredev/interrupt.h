/**-------------------------------------------------------------------------
@file	interrupt.h

@brief	Host test replacement of coredev/interrupt.h for spi_ra4m1.cpp.

		DisableInterrupt and EnableInterrupt keep a PRIMASK of the model.
		Re-enabling interrupts calls TestIrqPoint, where time passes in the
		model and pending interrupts are taken.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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
#ifndef __INTERRUPT_H__
#define __INTERRUPT_H__

#include <stdint.h>

extern uint32_t g_TestPrimask;
void TestIrqPoint(void);

static inline uint32_t DisableInterrupt(void)
{
	uint32_t state = g_TestPrimask;

	g_TestPrimask = 1;

	return state;
}

static inline void EnableInterrupt(uint32_t State)
{
	g_TestPrimask = State;
	if (State == 0)
	{
		TestIrqPoint();
	}
}

#endif
