/**-------------------------------------------------------------------------
@example	board.h

@brief	Board definitions for nRF54L15 DK and IBK-NRF54L15 PowerOffDemo

The DK uses UARTE30 on VCOM1, P0.01 RX and P0.00 TX, with Button 1
on P1.09. The IBK-NRF54L15 (schematic 03-231201-01 revision 0) has
P0.00 RXD, P0.01 TXD, S1 on P1.08 and S2 on P0.03.

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

// Select BOARD using the project preprocessor definitions when required.
#define NORDIC_NRF54L15_DK	1
#define IBK_NRF54L15		2

#ifndef BOARD
#define BOARD			IBK_NRF54L15
#endif

// The console is UARTE30 on both boards.
#define UART_DEVNO			0

#if BOARD == IBK_NRF54L15

// IBK-NRF54L15: P0.00 RXD, P0.01 TXD (IDAP-Link header).
#define UART_RX_PORT		0
#define UART_RX_PIN			0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			1
#define UART_TX_PINOP		1

// S1 is active low on P1.08. S2 is active low on P0.03.
#define WAKE_BUT_PORT		1
#define WAKE_BUT_PIN		8
#define WAKE_BUT_PINOP		0

#elif BOARD == NORDIC_NRF54L15_DK

// Nordic DK console (VCOM1).
#define UART_RX_PORT		0
#define UART_RX_PIN			1
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			0
#define UART_TX_PINOP		1

// Button 1, active low.
#define WAKE_BUT_PORT		1
#define WAKE_BUT_PIN		9
#define WAKE_BUT_PINOP		0

#else
#error "Unsupported nRF54L15 PowerOffDemo board"
#endif

#endif
