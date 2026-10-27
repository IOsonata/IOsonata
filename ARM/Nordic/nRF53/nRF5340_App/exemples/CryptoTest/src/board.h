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

#include "coredev/iopincfg.h"

// nRF5340 Audio DK by default; define NRF5340_DK_PCA10095 for DK.
#define UART_RETARGET
#define UART_DEVNO 0
#define UART_RX_PINOP 0
#define UART_TX_PINOP 0
#if defined(NRF5340_DK_PCA10095)
#define UART_RX_PORT 0
#define UART_RX_PIN 22
#define UART_TX_PORT 0
#define UART_TX_PIN 20
#else
#define UART_RX_PORT 1
#define UART_RX_PIN 4
#define UART_TX_PORT 1
#define UART_TX_PIN 5
#endif

#endif
