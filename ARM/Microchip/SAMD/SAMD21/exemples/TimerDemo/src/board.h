/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro timer demo

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D21 Xplained Pro. The trigger events toggle LED0
and are printed on the EDBG COM port.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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

// EDBG virtual COM port on SERCOM3, function C: RX PA23 pad 1, TX PA22 pad 0.
// Configure RX before TX in the pin map.
#define UART_DEVNO			3
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			23
#define UART_RX_PINOP		IOPINOP_PERIPHC
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			22
#define UART_TX_PINOP		IOPINOP_PERIPHC

// LED0 on PB30, active low
#define LED_PINS_MAP		{ \
	{ IOPORTB, 30, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL } \
}

// DevNo 0 is the RTC on the 32.768 kHz clock, 1 trigger. DevNo 1 to 5 are
// TC3 to TC7 on the main clock, 2 triggers each. DevNo 6 to 8 are TCC0
// (4 triggers), TCC1 and TCC2 (2 triggers each) on the main clock.
#define TIMER_DEMO_DEVNO	0
#define TIMER_DEMO_FREQ		0

// Cortex-M0+ priorities 0 to 3
#define TIMER_DEMO_INT_PRIO	1

#endif // __BOARD_H__
