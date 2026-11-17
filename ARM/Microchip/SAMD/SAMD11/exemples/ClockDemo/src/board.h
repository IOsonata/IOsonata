/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D11 Xplained Pro.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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

// EDBG virtual COM port on SERCOM2, function D: RX PA11 pad 3, TX PA10 pad 2.
// Configure RX before TX in the pin map.
#define UART_DEVNO			2
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			11
#define UART_RX_PINOP		IOPINOP_PERIPHD
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			10
#define UART_TX_PINOP		IOPINOP_PERIPHD

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Select Standby/resume checks or the clock-selection checks to fit D11 flash.
#ifndef CLOCK_DEMO_RESUME_TEST
#define CLOCK_DEMO_RESUME_TEST 1
#endif

#if CLOCK_DEMO_RESUME_TEST
// SW0 is PA14, the dedicated NMI input (active low).
#define CLOCK_DEMO_WAKE_PORT IOPORTA
#define CLOCK_DEMO_WAKE_PIN 14
#define CLOCK_DEMO_WAKE_NAME "SW0"
#define CLOCK_DEMO_SLEEP_APB_DIVIDER 1U
#define CLOCK_DEMO_CORE_READONLY 1
#define CLOCK_DEMO_SOURCES { {"Startup clock", OSC_TYPE_RC, 0U, 48000000U, 48000000U, false} }
#else
#define CLOCK_DEMO_SOURCES { {"OSC8M", OSC_TYPE_RC, 8000000U, 1U, UINT32_MAX} }
#endif

// LF selection omitted because a running DFLL/DPLL may own its reference.
#define CLOCK_DEMO_APB_DIVIDERS {1U}
#define CLOCK_DEMO_TIMER_DEVNO 4
#define CLOCK_DEMO_CYCLES 3U
#define CLOCK_DEMO_IRQ_PRIORITY 1
#define CLOCK_DEMO_STOP_UART 1
#define CLOCK_DEMO_DEFER_TIMER 1

#endif // __BOARD_H__
