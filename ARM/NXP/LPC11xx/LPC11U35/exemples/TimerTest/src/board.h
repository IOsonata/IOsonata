/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
LPC11U35. USART on PIO0_18 (RX) and PIO0_19 (TX), the IBK-IDAP-M
connector pins.

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

// Keep the library's oscillator defaults. Define MCUOSC here only when the
// application needs a different oscillator configuration.

// UART device 0. Configure RX before TX in the pin map.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			18
#define UART_RX_PINOP		IOPINOP_FUNC1

#define UART_TX_PORT		0
#define UART_TX_PIN			19
#define UART_TX_PINOP		IOPINOP_FUNC1

#define UART_BAUDRATE 1000000

// The Cortex-M0 has 4 interrupt levels, 0 to 3
#define TIMER_TEST_IRQ_PRIO	3

#endif
