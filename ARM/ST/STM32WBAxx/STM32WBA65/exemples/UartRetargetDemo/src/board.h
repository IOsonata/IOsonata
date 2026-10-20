/**-------------------------------------------------------------------------
@file	board.h

@brief	STM32WBA65I-DK1 board definitions: USART1 on the ST-LINK virtual COM port.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#define BOARD STM32WBA65I_DK1
#include "stm32_boards.h"

#define UART_DEVNO BOARD_VCP_DEVNO

#define UART_RX_PORT IOPORTA
#define UART_RX_PIN 8
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTB
#define UART_TX_PIN 12
#define UART_TX_PINOP IOPINOP_FUNC7
#define UART_CTS_PORT -1
#define UART_CTS_PIN -1
#define UART_CTS_PINOP 0
#define UART_RTS_PORT -1
#define UART_RTS_PIN -1
#define UART_RTS_PINOP 0

#endif
