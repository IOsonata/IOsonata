/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the nRF54LM20 DK

The console is UARTE20 (UART_DEVNO 1) on P1.17 RX and P1.16 TX, the SysLog
output. The wake up button is Button 0, P1.26.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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

// Console, the SysLog output
#define UART_DEVNO			1
#define UART_RX_PORT		1
#define UART_RX_PIN			17
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			16
#define UART_TX_PINOP		1

// Wake up button, to ground
#define WAKE_BUT_PORT		1
#define WAKE_BUT_PIN		26
#define WAKE_BUT_PINOP		0

// RAM1 is outside the default .data/.bss/stack allocation.
// Two independent 32 KB sections exercise retention on and off.
#define POWER_DEMO_RAM_RET_TEST	1
#define POWER_DEMO_RAM_RET_ADDR	0x20040000UL
#define POWER_DEMO_RAM_RET_SIZE	32768UL
#define POWER_DEMO_RAM_DROP_ADDR	0x20048000UL
#define POWER_DEMO_RAM_DROP_SIZE	32768UL

#endif
