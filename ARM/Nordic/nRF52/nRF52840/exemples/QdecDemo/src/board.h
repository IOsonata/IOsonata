/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, quadrature decoder demo on the nRF52840 DK

The console is UART0 on the interface MCU VCOM0 port of the nRF52840 DK, the
SysLog output. The decoder inputs are the two buttons, Button 1 (P0.11) phase
A and Button 2 (P0.12) phase B. For an encoder, change the pins and set a
shorter QDEC_DEMO_SAMPLE_PERIOD.

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

// Decoder inputs, Button 1 and Button 2
#define QDEC_PHA_PORT		0
#define QDEC_PHA_PIN		11
#define QDEC_PHA_PINOP		0
#define QDEC_PHB_PORT		0
#define QDEC_PHB_PIN		12
#define QDEC_PHB_PINOP		0

#endif
