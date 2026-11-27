/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the nRF52840 DK

The console is UART0 on the interface MCU VCOM0 port of the nRF52840 DK, the
SysLog output. The wake up button is Button 1 (P0.11).

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

// Console, UART0, the SysLog output. P0.8 and P0.6 go to the interface MCU
// VCOM0 on the nRF52840 DK.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			6
#define UART_TX_PINOP		1

// System OFF console: use legacy UART polling so TX can be drained
// synchronously before the peripheral and clock stop.
#define POWER_DEMO_UART_INT	false
#define POWER_DEMO_UART_DMA	false
#define POWER_DEMO_UART_OFF	1

// Wake up button, to ground
#define WAKE_BUT_PORT		0
#define WAKE_BUT_PIN		11
#define WAKE_BUT_PINOP		0
#define WAKE_BUT_SYSTEM_OFF	1

// Sample preserved across the System OFF wake reset.
#define POWER_DEMO_RAM_RET_TEST	1
static uint32_t s_RetainedRam[4] __attribute__((section(".noinit.retention")));
#define POWER_DEMO_RAM_RET_SIZE	sizeof(s_RetainedRam)
#define POWER_DEMO_RAM_RET_ADDR	((uintptr_t)s_RetainedRam)

#endif
