/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The supplied pin assignments are examples, not a development-board definition.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

// Keep the library's internal oscillator defaults. Define MCUOSC here only
// when the application needs a different oscillator configuration.

// LED or oscilloscope outputs
#define LED0_PORT			1
#define LED0_PIN			2
#define LED0_PINOP			IOPINOP_GPIO

#define LED1_PORT			1
#define LED1_PIN			11
#define LED1_PINOP			IOPINOP_GPIO

#define LED2_PORT			1
#define LED2_PIN			12
#define LED2_PINOP			IOPINOP_GPIO

#define LED_PINS_MAP	{ \
	{LED0_PORT, LED0_PIN, LED0_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// One Timer interface and one logical device-number space.
// Select any timer with DevNo (0..9). Freq=0 uses that device's default rate.
// Low-frequency devices come first; TimerGetHighFreqDevNo() returns the
// first high-frequency device number. No hardware-family selector is needed.
#ifndef TIMER_DEVNO
#define TIMER_DEVNO			0
#endif
#ifndef TIMER_FREQ
#define TIMER_FREQ			0
#endif

#endif // __BOARD_H__
