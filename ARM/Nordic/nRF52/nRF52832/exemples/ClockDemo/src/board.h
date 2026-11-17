/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the BLYST-NANO-DK

The console is UART0 on the BLYST-NANO-DK UART pins, the SysLog output. The
wake up button is BUT1.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

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
#include "blystnano_boards.h"

// Console, UART0, the SysLog output
#define UART_DEVNO			0
#define UART_RX_PORT		BLYST_NANO_DK_UART_RX_PORT
#define UART_RX_PIN			BLYST_NANO_DK_UART_RX_PIN
#define UART_RX_PINOP		BLYST_NANO_DK_UART_RX_PINOP
#define UART_TX_PORT		BLYST_NANO_DK_UART_TX_PORT
#define UART_TX_PIN			BLYST_NANO_DK_UART_TX_PIN
#define UART_TX_PINOP		BLYST_NANO_DK_UART_TX_PINOP

// Wake up button, to ground
#define WAKE_BUT_PORT		BLYST_NANO_DK_BUT1_PORT
#define WAKE_BUT_PIN		BLYST_NANO_DK_BUT1_PIN
#define WAKE_BUT_PINOP		BLYST_NANO_DK_BUT1_PINOP


// Bare-metal HFXO requests/releases are balanced. No BLE stack is started.
#define CLOCK_DEMO_SOURCES { {"Request HFXO", OSC_TYPE_XTAL, 0U, 1U, UINT32_MAX}, {"Release HFXO", OSC_TYPE_RC, 0U, 1U, UINT32_MAX} }
#define CLOCK_DEMO_APB_DIVIDERS {1U}
#define CLOCK_DEMO_LF_SOURCES { {"LF RC", OSC_TYPE_RC, 32768U} }
#define CLOCK_DEMO_TIMER_DEVNO TimerGetLowFreqDevCount()
#define CLOCK_DEMO_CYCLES 3U
#define CLOCK_DEMO_IRQ_PRIORITY 1
#define CLOCK_DEMO_STOP_UART 1
#define CLOCK_DEMO_DEFER_TIMER 1

#endif
