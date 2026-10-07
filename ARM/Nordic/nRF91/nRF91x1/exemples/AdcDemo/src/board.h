/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, SAADC demo on the nRF9161 and nRF9151 DK

The console is UART0 on the interface MCU VCOM port.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

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

// Console, UART0 to the interface MCU (VCOM0)
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			26
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			27
#define UART_TX_PINOP		1

// No flow control: the pins are left unconfigured
#define UART_CTS_PORT		0
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		1

// Analog inputs AIN0 and AIN1 (P0.13 and P0.14)
#define AIN0_PORT			0
#define AIN0_PIN			13
#define AIN1_PORT			0
#define AIN1_PIN			14

#endif
