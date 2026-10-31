/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro SPI master loopback

The SPI master on the EXT1 SPI pins reads back what it sends through a
jumper from MOSI (EXT1 16) to MISO (EXT1 17). Results on the EDBG COM port
at 1000000 baud.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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

// EDBG virtual COM port on SERCOM3, function C: RX PA23 pad 1, TX PA22 pad 0.
// Configure RX before TX in the pin map.
#define UART_DEVNO			3
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			23
#define UART_RX_PINOP		IOPINOP_PERIPHC
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			22
#define UART_TX_PINOP		IOPINOP_PERIPHC
// No flow control
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO
#define SPI_LOOPBACK_WIRING	"Jumper EXT1 16 (MOSI PA06) to EXT1 17 (MISO PA04)"
// SPI master on SERCOM0: MOSI PA06 pad 2 function D, MISO PA04 pad 0 function D, SCK PA07 pad 3 function D, CS on GPIO PA05
#define SPI_MASTER_DEVNO		0
#define SPI_MASTER_SCK_PORT	IOPORTA
#define SPI_MASTER_SCK_PIN		7
#define SPI_MASTER_SCK_PINOP	IOPINOP_PERIPHD
#define SPI_MASTER_MISO_PORT	IOPORTA
#define SPI_MASTER_MISO_PIN		4
#define SPI_MASTER_MISO_PINOP	IOPINOP_PERIPHD
#define SPI_MASTER_MOSI_PORT	IOPORTA
#define SPI_MASTER_MOSI_PIN		6
#define SPI_MASTER_MOSI_PINOP	IOPINOP_PERIPHD
#define SPI_MASTER_CS_PORT		IOPORTA
#define SPI_MASTER_CS_PIN		5
#define SPI_MASTER_CS_PINOP		IOPINOP_GPIO
#define SPI_MASTER_INT_ENABLE	false
#define SPI_MASTER_DMA_ENABLE	false

#endif // __BOARD_H__
