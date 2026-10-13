/**-------------------------------------------------------------------------
@file	board.h

@brief	SPI master loopback on the FPB-R9A02G021

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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

// The library defaults are kept: HOCO 48 MHz core clock, LOCO low
// frequency clock. Define MCUOSC here for another setting, for example the
// 32.768 kHz crystal fitted on the board:
// #define MCUOSC	{ {OSC_TYPE_RC, 48000000, 0, 0}, {OSC_TYPE_XTAL, 32768, 0, 0}, false }

// SAU UART0 on the on-board J-Link OB virtual COM port (J5 and J6 at 2-3):
// TxD0_B P302, RxD0_B P303. PSEL 0x11 (SAU) is IOPINOP_FUNC16.
// Configure RX before TX in the pin map.
#define UART_DEVNO			0
#define UART_RX_PORT		3
#define UART_RX_PIN			3
#define UART_RX_PINOP		IOPINOP_FUNC16

#define UART_TX_PORT		3
#define UART_TX_PIN			2
#define UART_TX_PINOP		IOPINOP_FUNC16

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// No RTS and CTS
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

// SPI20, SAU1 channel 0: SCK20 P101, SI20 P102, SO20 P103, PSEL 0x11 (SAU)
// is IOPINOP_FUNC16. Chip select on P104 as GPIO. UART2 uses the same SAU
// channels and cannot run with it.
#define SPI_MASTER_DEVNO		4
#define SPI_MASTER_SCK_PORT		1
#define SPI_MASTER_SCK_PIN		1
#define SPI_MASTER_SCK_PINOP	IOPINOP_FUNC16
#define SPI_MASTER_MISO_PORT	1
#define SPI_MASTER_MISO_PIN		2
#define SPI_MASTER_MISO_PINOP	IOPINOP_FUNC16
#define SPI_MASTER_MOSI_PORT	1
#define SPI_MASTER_MOSI_PIN		3
#define SPI_MASTER_MOSI_PINOP	IOPINOP_FUNC16
#define SPI_MASTER_CS_PORT		1
#define SPI_MASTER_CS_PIN		4
#define SPI_MASTER_CS_PINOP		IOPINOP_GPIO

#define SPI_MASTER_RATE			1000000

#define SPI_LOOPBACK_WIRING		"Connect P103 MOSI (SO20) to P102 MISO (SI20)"

#endif
