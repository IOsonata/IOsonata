/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPC11U35 TimerDemo

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

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

// Same LED pins as the LPC11U35 Blinky example.
#define LED_PINS_MAP { \
	{0, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 4, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// DevNo 0/1: CT16B0/CT16B1; DevNo 2/3: CT32B0/CT32B1.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO		0
#endif
#ifndef TIMER_DEMO_FREQ
#define TIMER_DEMO_FREQ		10000
#endif
#define TIMER_DEMO_INT_PRIO	3

// Same RX/TX pins as UartPrbsTxTest, no hardware flow control.
#define TIMER_DEMO_UART
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			18
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			19
#define UART_TX_PINOP		IOPINOP_FUNC1

#endif
