/**-------------------------------------------------------------------------
@file	board.h

@brief	NUCLEO-L152RE board definitions for TimerDemo: VCP UART, LED and the timer used.

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
#define BOARD NUCLEO_L152RE
#include "stm32_boards.h"
#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_PINS BOARD_VCP_PINS
#define UART_INT_MODE true
#define UART_DMA_MODE false
#define LED_PINS_MAP BOARD_LED_PINS
// DevNo 5 selects TIM2 with four triggers.
#ifndef TIMER_DEMO_DEVNO
#define TIMER_DEMO_DEVNO 5
#endif
#define TIMER_DEMO_FREQ 100000
#define UART_RX_PORT IOPORTA
#define UART_RX_PIN 3
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTA
#define UART_TX_PIN 2
#define UART_TX_PINOP IOPINOP_FUNC7

// Runtime selectors in this port are stubs; use startup readback and timer operation.
#define CLOCK_DEMO_SOURCES { {"Startup readback", OSC_TYPE_RC, 0U, 1U, UINT32_MAX, false} }
#define CLOCK_DEMO_APB_DIVIDERS {}
#define CLOCK_DEMO_TIMER_DEVNO TimerGetLowFreqDevCount()
#define CLOCK_DEMO_CYCLES 3U
#define CLOCK_DEMO_IRQ_PRIORITY 1
#define CLOCK_DEMO_STOP_UART 1
#define CLOCK_DEMO_DEFER_TIMER 1
#define CLOCK_DEMO_CORE_READONLY 1
#define CLOCK_DEMO_APB_READONLY 1

#endif
