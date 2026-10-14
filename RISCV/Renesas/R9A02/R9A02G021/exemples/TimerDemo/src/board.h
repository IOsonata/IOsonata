/**-------------------------------------------------------------------------
@file	board.h

@brief	Timer demo on the FPB-R9A02G021

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
#ifndef __BOARD_H__
#define __BOARD_H__

#include "coredev/iopincfg.h"

// The library defaults are kept: HOCO 48 MHz core clock, LOCO low
// frequency clock. Define MCUOSC here for another setting, for example the
// 32.768 kHz crystal fitted on the board:
// #define MCUOSC	{ {OSC_TYPE_RC, 48000000, 0, 0}, {OSC_TYPE_XTAL, 32768, 0, 0}, false }

// FPB-R9A02G021 LED1 P107 and LED2 P100, active high
#define LED1_PORT			1
#define LED1_PIN			7
#define LED1_PINOP			IOPINOP_GPIO

#define LED2_PORT			1
#define LED2_PIN			0
#define LED2_PINOP			IOPINOP_GPIO

#define LED_PINS_MAP		{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Timer DevNo 0 is the machine timer at 32768 Hz, DevNo 1 to 8 are the TAU
// channels 0 to 7. Freq 0 selects the default rate of the device.
#ifndef TIMER_DEVNO
#define TIMER_DEVNO			0
#endif
#ifndef TIMER_FREQ
#define TIMER_FREQ			0
#endif

#endif
