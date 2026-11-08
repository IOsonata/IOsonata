/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, PeiCounterDemo on the BLYST-NANO-DK

The console is UART0 on the BLYST-NANO-DK UART pins, the SysLog output. The
presses of BUT1 are counted, LED1 is toggled at each limit event.

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
#include "blystnano_boards.h"

// Console, UART0, the SysLog output
#define UART_DEVNO			0
#define UART_RX_PORT		BLYST_NANO_DK_UART_RX_PORT
#define UART_RX_PIN			BLYST_NANO_DK_UART_RX_PIN
#define UART_RX_PINOP		BLYST_NANO_DK_UART_RX_PINOP
#define UART_TX_PORT		BLYST_NANO_DK_UART_TX_PORT
#define UART_TX_PIN			BLYST_NANO_DK_UART_TX_PIN
#define UART_TX_PINOP		BLYST_NANO_DK_UART_TX_PINOP

// Button counted, BUT1 P0.02
#define PEI_COUNTER_DEMO_BUTTON		{ BLYST_NANO_DK_BUT1_PORT, BLYST_NANO_DK_BUT1_PIN, BLYST_NANO_DK_BUT1_PINOP, \
									  IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL }

// LED toggled at each limit event, LED1 P0.30, active low
#define PEI_COUNTER_DEMO_LED		{ BLYST_NANO_DK_LED1_PORT, BLYST_NANO_DK_LED1_PIN, BLYST_NANO_DK_LED1_PINOP, \
									  IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }

#endif // __BOARD_H__
