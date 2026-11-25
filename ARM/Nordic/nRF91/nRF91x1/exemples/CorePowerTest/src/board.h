/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the nRF9151 DK or nRF9161 DK

The console is UART0 on the VCOM0 pins of the DK, P0.26 RX and P0.27 TX, the
SysLog output. The wake up button is Button 1, P0.08.

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

// Define NRF91X1_DK in the project settings to use the DK pin map.
#if defined(NRF91X1_DK)
#define UART_RX_PIN			26
#define UART_TX_PIN			27
#define WAKE_BUT_PIN		8
#else
// Nordic Thingy:91 X, UART0 to nRF5340 USB bridge and Button 1.
#define UART_RX_PIN			0
#define UART_TX_PIN			1
#define WAKE_BUT_PIN		26
#ifndef UART_BAUDRATE
#define UART_BAUDRATE			115200
#endif
#endif

// Console, the SysLog output
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PINOP		1

// Wake up button, to ground
#define WAKE_BUT_PORT		0
#define WAKE_BUT_PINOP		0
#define WAKE_BUT_SYSTEM_OFF	1

// One VMC RAM section, excluded from startup initialization.
#define POWER_DEMO_RAM_RET_TEST	1
#define POWER_DEMO_RAM_RET_SIZE	8192U
static uint32_t s_RetainedRam[POWER_DEMO_RAM_RET_SIZE / sizeof(uint32_t)]
	__attribute__((section(".noinit.retention"), aligned(POWER_DEMO_RAM_RET_SIZE)));
#define POWER_DEMO_RAM_RET_ADDR	((uintptr_t)s_RetainedRam)

#endif
