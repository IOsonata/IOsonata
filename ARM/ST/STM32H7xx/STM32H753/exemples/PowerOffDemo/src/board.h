/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the NUCLEO-H753ZI

The console is USART3, the ST-LINK virtual COM port, RX PD9, TX PD8, the
SysLog output. The wake up button is the blue user button B1 on PC13, WKUP4,
to the supply with a pull down: the MCU wakes up when it is pressed.

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026

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

// Board, one of the names in stm32_boards.h
#define BOARD				NUCLEO_H753ZI

#include "stm32_boards.h"

// Console, the SysLog output, 1000000 baud 8N1
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_RX_PORT		IOPORTD
#define UART_RX_PIN			9
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTD
#define UART_TX_PIN			8
#define UART_TX_PINOP		IOPINOP_FUNC7

// Wake up button, user button B1. Button to the supply, wakes up on a high level.
#define WAKE_BUT_PORT		IOPORTC
#define WAKE_BUT_PIN		13
#define WAKE_BUT_PINOP		IOPINOP_GPIO
#define WAKE_BUT_SENSE		IOPINSENSE_HIGH_TRANSITION

#endif // __BOARD_H__
