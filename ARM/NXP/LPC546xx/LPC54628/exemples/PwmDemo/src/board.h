/**-------------------------------------------------------------------------
@example	board.h

@brief	LPCXpresso54628 board definitions for PwmDemo

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

// PWM of the demo on SCT0, DevNo 0: channel 0 is SCT0_OUT4 on LED1 (P3_14,
// function 1), channel 1 SCT0_OUT6 on LED3 (P2_2, function 3). The LEDs are
// active low: the duty cycle is the time the LED is on.
#define PWM_DEVNO			0
#define PWM_CHAN_CFG		\
	{ .Chan = 0, .Pol = PWM_POL_LOW, .Port = 3, .Pin = 14, .PinOp = IOPINOP_FUNC1 }, \
	{ .Chan = 1, .Pol = PWM_POL_LOW, .Port = 2, .Pin = 2, .PinOp = IOPINOP_FUNC3 },

#endif // __BOARD_H__
