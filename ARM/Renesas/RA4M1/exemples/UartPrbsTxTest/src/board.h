/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The supplied pin assignments are examples, not a development-board definition.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

// Keep the library's internal oscillator defaults. Define MCUOSC here only
// when the application needs a different oscillator configuration.

// UART device 0. Configure RX before TX in the pin map.
#define UART_DEVNO			0
#define UART_RX_PORT		1
#define UART_RX_PIN			0
#define UART_RX_PINOP		IOPINOP_FUNC3

#define UART_TX_PORT		1
#define UART_TX_PIN			1
#define UART_TX_PINOP		IOPINOP_FUNC3

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Names consumed by the existing shared UART loopback example.
#define UART_NO				UART_DEVNO
#define UART_PORTPINS		IOPinCfg_t s_UartPortPins[] = UART_PINS
#define UART_PORTPIN_COUNT	(sizeof(s_UartPortPins) / sizeof(s_UartPortPins[0]))

#ifndef UART_BAUDRATE
#define UART_BAUDRATE		115200
#endif
#ifndef UART_INT_MODE
#define UART_INT_MODE		true
#endif
#define UART_DMA_MODE		false
#define UARTFIFOSIZE		CFIFO_MEMSIZE(256)

#endif // __BOARD_H__
