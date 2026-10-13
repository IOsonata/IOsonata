/**-------------------------------------------------------------------------
@file	board.h

@brief	UARTA loopback on the FPB-R9A02G021

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

#include "coredev/iopincfg.h"

// The library defaults are kept: HOCO 48 MHz core clock, LOCO low
// frequency clock. Define MCUOSC here for another setting, for example the
// 32.768 kHz crystal fitted on the board:
// #define MCUOSC	{ {OSC_TYPE_RC, 48000000, 0, 0}, {OSC_TYPE_XTAL, 32768, 0, 0}, false }

// UARTA1 is UART_DEVNO 4 (UARTA0 is 3): RxDA1 P008, TxDA1 P403 on the
// Arduino header, PSEL 0x06 (UARTA) is IOPINOP_FUNC5. Connect a 3.3 V serial
// adapter. The clock is the HOCO, 200 to 153600 bps.
#define UART_DEVNO			4
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		IOPINOP_FUNC5

#define UART_TX_PORT		4
#define UART_TX_PIN			3
#define UART_TX_PINOP		IOPINOP_FUNC5

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define UART_BAUDRATE		115200

#endif
