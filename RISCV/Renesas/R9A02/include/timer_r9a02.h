/**-------------------------------------------------------------------------
@file	timer_r9a02.h

@brief	R9A02G021 timer device numbers

TimerCfg_t.DevNo, from low frequency, low power to high frequency:

	DevNo 0		: Machine timer, 64-bit up counter on MTCLK (LOCO, 32768 Hz),
				  1 trigger (mtimecmp). R9A02G021 User's Manual R01UH1036EJ0110
				  2.5.4.
	DevNo 1 to 8: TAU channels 0 to 7, 16-bit counter on CK00 or CK01
				  (PCLKB / 2^n, n = 0 to 15), 1 trigger each (the interval
				  period). Manual chapter 18.

The 32-bit interval timer TML32 has no readable counter in 32-bit mode
(19.2) and is not a timer device.

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
#ifndef __TIMER_R9A02_H__
#define __TIMER_R9A02_H__

#include "r9a02g021.h"

#define R9A02_TIMER_MTIMER_DEVNO		0
#define R9A02_TIMER_TAU_DEVNO(Chan)		(1 + (Chan))
#define R9A02_TIMER_LOWFREQ_CNT			1
#define R9A02_TIMER_HIGHFREQ_CNT		R9A02_TAU_CHAN_COUNT
#define R9A02_TIMER_DEV_CNT				(R9A02_TIMER_LOWFREQ_CNT + R9A02_TIMER_HIGHFREQ_CNT)

#endif
