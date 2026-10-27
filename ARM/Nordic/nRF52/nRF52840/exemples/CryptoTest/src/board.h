/**-------------------------------------------------------------------------
@file	board.h

@brief	CryptoTest board definitions.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include "blueio_board.h"

// BLUEIO-WIZARD console pin mapping; set pins for other nRF52840 boards.
#define UART_DEVNO		0
#define UART_RX_PORT	BLUEIO_UART_RX_PORT
#define UART_RX_PIN		BLUEIO_UART_RX_PIN
#define UART_RX_PINOP	BLUEIO_UART_RX_PINOP
#define UART_TX_PORT	BLUEIO_UART_TX_PORT
#define UART_TX_PIN		BLUEIO_UART_TX_PIN
#define UART_TX_PINOP	BLUEIO_UART_TX_PINOP

#endif
