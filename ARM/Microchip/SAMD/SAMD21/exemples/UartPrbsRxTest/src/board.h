/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro UART PRBS receive test

Receives the PRBS8 stream of Python/uartprbs_tx.py on the EDBG COM port at
1000000 baud and counts the bytes that are not the PRBS successor of the one
before (errcnt, in the debugger).

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
#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
// Pin map names of the older examples
#define UART_NO				UART_DEVNO
#define UART_PORTPINS		IOPinCfg_t s_UartPortPins[] = UART_PINS
#define UART_PORTPIN_COUNT	(sizeof(s_UartPortPins) / sizeof(s_UartPortPins[0]))
// Interrupt mode, no DMA path on SAM D
#define UART_INT_MODE		true
#define UART_DMA_MODE		false
#define UART_INT_PRIO		2
// No output per byte, it would stall the reception
#define UART_PRBS_VERBOSE	0

#endif // __BOARD_H__
