/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, power off demo on the nRF54L15 DK

The console is UARTE30 (UART_DEVNO 0) on the VCOM1 pins of the DK, P0.01 RX
and P0.00 TX, the SysLog output. The wake up button is Button 1, P1.09.

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
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			1
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			0
#define UART_TX_PINOP		1

// Wake up button, to ground
#define WAKE_BUT_PORT		1
#define WAKE_BUT_PIN		9
#define WAKE_BUT_PINOP		0

#endif
