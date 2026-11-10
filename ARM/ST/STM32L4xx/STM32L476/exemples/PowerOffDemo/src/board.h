/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the B-L475E-IOT01A

This STM32L476 project builds for the B-L475E-IOT01A and B-L475E-IOT01A2
(STM32L475VG). The console is USART1, the ST-LINK virtual COM port, RX PB7,
TX PB6, the SysLog output. The wake up button is the blue user button B1 on
PC13, WKUP2, to ground with a pull up: the MCU wakes up when it is pressed.

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
#define BOARD				B_L475E_IOT01A

#include "stm32_boards.h"

// Console, the SysLog output, 1000000 baud 8N1
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_RX_PORT		IOPORTB
#define UART_RX_PIN			7
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			6
#define UART_TX_PINOP		IOPINOP_FUNC7

// Wake up button, user button B1. Button to ground, wakes up on a low level.
#define WAKE_BUT_PORT		IOPORTC
#define WAKE_BUT_PIN		13
#define WAKE_BUT_PINOP		IOPINOP_GPIO
#define WAKE_BUT_SENSE		IOPINSENSE_LOW_TRANSITION

#endif // __BOARD_H__
