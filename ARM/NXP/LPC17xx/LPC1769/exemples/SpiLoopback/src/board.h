/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPC1769

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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

// LPC1769 MCU pin assignments; locate these signals on your board.
// Release console: UART0 at 1000000 baud, RX P0.3 and TX P0.2.
// Debug uses semihosting. The shared example initializes UART in both builds.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			3
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			2
#define UART_TX_PINOP		IOPINOP_FUNC1
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

// SSP0: SCK P0.15, MISO P0.17, MOSI P0.18; GPIO chip select P0.16.
#define SPI_MASTER_DEVNO		0
#define SPI_MASTER_SCK_PORT		0
#define SPI_MASTER_SCK_PIN		15
#define SPI_MASTER_SCK_PINOP	IOPINOP_FUNC2
#define SPI_MASTER_MISO_PORT	0
#define SPI_MASTER_MISO_PIN		17
#define SPI_MASTER_MISO_PINOP	IOPINOP_FUNC2
#define SPI_MASTER_MOSI_PORT	0
#define SPI_MASTER_MOSI_PIN		18
#define SPI_MASTER_MOSI_PINOP	IOPINOP_FUNC2
#define SPI_MASTER_CS_PORT		0
#define SPI_MASTER_CS_PIN		16
#define SPI_MASTER_CS_PINOP		IOPINOP_GPIO
#define SPI_MASTER_RATE		1000000
#define SPI_MASTER_INT_ENABLE	false
#define SPI_MASTER_DMA_ENABLE	false

#define SPI_LOOPBACK_WIRING		"Connect P0.18 MOSI to P0.17 MISO"

#endif
