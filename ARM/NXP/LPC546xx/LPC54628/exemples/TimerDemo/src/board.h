/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPCXpresso54628

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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

// LED1 P3_14, LED2 P3_3 and LED3 P2_2, active low. Each one toggles on its
// timer trigger.
#define LED_PINS_MAP	{ \
	{3, 14, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{3, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{2, 2, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Virtual timers 0 and 1 are CTIMER3 and 4 on the 12 MHz FRO, 2 to 4 are
// CTIMER0 to 2 on the system clock. Each has 3 triggers.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO	0
#endif
#define TIMER_DEMO_FREQ		10000
#define TIMER_DEMO_INT_PRIO	2

// Output on the LPC-Link2 VCOM, Flexcomm 0, P0_29 RXD and P0_30 TXD
#define TIMER_DEMO_UART
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			29
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			30
#define UART_TX_PINOP		IOPINOP_FUNC1

#endif
