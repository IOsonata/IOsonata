/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, PeiCounterDemo on the nRF52840 DK

The console is UART0 on the interface MCU VCOM0 port of the nRF52840 DK, the
SysLog output. The presses of BUTTON1 are counted, LED1 is toggled at each
limit event.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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

// Console, UART0 on the VCOM0 port of the interface MCU
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			6
#define UART_TX_PINOP		1

// Button counted, BUTTON1 P0.11
#define PEI_COUNTER_DEMO_BUTTON		{ 0, 11, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL }

// LED toggled at each limit event, LED1 P0.13, active low
#define PEI_COUNTER_DEMO_LED		{ 0, 13, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }

#endif // __BOARD_H__
