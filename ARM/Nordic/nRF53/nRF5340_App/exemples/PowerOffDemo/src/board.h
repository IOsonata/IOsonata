/**-------------------------------------------------------------------------
@example	board.h

@brief	Board definitions for nRF5340 Audio DK and standard DK PowerOffDemo

The Audio DK (PCA10121) uses UARTE0, P1.04 RX, P1.05 TX and
Button 1 (volume down) P0.02. The standard DK (PCA10095) uses
P0.22 RX, P0.20 TX and Button 1 P0.23.

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

// Audio DK is the default board. Define NRF5340_DK_PCA10095 in the
// project symbols to use the standard nRF5340 DK instead.
#define UART_DEVNO			0

#if defined(NRF5340_DK_PCA10095)
// Standard nRF5340 DK (PCA10095), VCOM0 and Button 1.
#define UART_RX_PORT		0
#define UART_RX_PIN			22
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			20
#define UART_TX_PINOP		1
#define WAKE_BUT_PORT		0
#define WAKE_BUT_PIN		23
#define WAKE_BUT_PINOP		0
#else
// nRF5340 Audio DK (PCA10121), VCOM and Button 1 (volume down).
#define UART_RX_PORT		1
#define UART_RX_PIN			4
#define UART_RX_PINOP		0
#define UART_TX_PORT		1
#define UART_TX_PIN			5
#define UART_TX_PINOP		0
#define WAKE_BUT_PORT		0
#define WAKE_BUT_PIN		2
#define WAKE_BUT_PINOP		0
#define POWER_DEMO_PIN_DIAG	1
#endif

// Test two sections in RAM1. Normal data and stack are in RAM;
// IPC0 starts at 0x20070000 and must not be modified.
#define POWER_DEMO_RAM_RET_TEST	1
#define POWER_DEMO_RAM_RET_ADDR	0x20040000UL
#define POWER_DEMO_RAM_RET_SIZE	32768UL
#define POWER_DEMO_RAM_DROP_ADDR	0x20048000UL
#define POWER_DEMO_RAM_DROP_SIZE	32768UL

#endif
