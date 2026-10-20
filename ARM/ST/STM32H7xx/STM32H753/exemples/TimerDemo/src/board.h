/**-------------------------------------------------------------------------
@file	board.h

@brief	NUCLEO-H753ZI board definitions for TimerDemo: VCP UART, LEDs and the timer used.

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
#define BOARD NUCLEO_H753ZI
#include "stm32_boards.h"
#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_PINS BOARD_VCP_PINS
#define UART_INT_MODE true
#define UART_DMA_MODE false
#define LED_PINS_MAP BOARD_LED_PINS
// DevNo 0 TIM6, 1 TIM7, 2 TIM13, 3 TIM14, 4 TIM16, 5 TIM17,
// 6 TIM12, 7 TIM15, 8 TIM2, 9 TIM3, 10 TIM4, 11 TIM5, 12 TIM1, 13 TIM8.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO 8
#endif
#define TIMER_DEMO_FREQ 100000
#define TIMER_DEMO_UART
#define UART_RX_PORT IOPORTD
#define UART_RX_PIN 9
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTD
#define UART_TX_PIN 8
#define UART_TX_PINOP IOPINOP_FUNC7
#endif
