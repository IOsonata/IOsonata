/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D11 Xplained Pro UART loopback

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D11 Xplained Pro.

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

// EDBG virtual COM port on SERCOM2, function D: RX PA11 pad 3, TX PA10 pad 2.
// Configure RX before TX in the pin map.
#define UART_DEVNO			2
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			11
#define UART_RX_PINOP		IOPINOP_PERIPHD
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			10
#define UART_TX_PINOP		IOPINOP_PERIPHD

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Received bytes are sent back, from a terminal on the EDBG COM port at
// 1000000 baud

#endif // __BOARD_H__
