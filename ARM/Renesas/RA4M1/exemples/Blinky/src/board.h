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

// Optional button: ground P000 to stop Blinky. No debounce is provided.
#define BUT1_PORT			0
#define BUT1_PIN			0
#define BUT1_PINOP			IOPINOP_GPIO
#define BUT1_INT			6
#define BUT1_INT_PRIO		IRQ_PRIO_NORMAL
#define BUT1_SENSE			IOPINSENSE_LOW_TRANSITION

#define BUTTON_PINS_MAP	{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
