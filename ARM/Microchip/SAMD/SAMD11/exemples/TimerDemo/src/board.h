/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D11 Xplained Pro timer demo

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D11 Xplained Pro. Trigger 0 toggles LED0. The
UART and printf do not fit the 16 KB flash with the timer: no console, the
counts and periods are in g_TriggerCount and g_Period for the debugger.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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

#define TIMER_DEMO_NO_UART

// LED0 on PA16, active low
#define LED_PINS_MAP		{ \
	{ IOPORTA, 16, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL } \
}

// DevNo 0 is the RTC on the 32.768 kHz clock, 1 trigger. DevNo 1 and 2 are
// TC1 and TC2 on the main clock, 2 triggers each, DevNo 3 is TCC0 on the
// main clock, 4 triggers.
#define TIMER_DEMO_DEVNO	0
#define TIMER_DEMO_FREQ		0

// Cortex-M0+ priorities 0 to 3
#define TIMER_DEMO_INT_PRIO	1

#endif // __BOARD_H__
