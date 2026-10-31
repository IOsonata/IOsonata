/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro SPI master and slave loopback

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
One SERCOM is the SPI master, another one the slave, wired together on
the SAM D21 Xplained Pro. The results go to the EDBG COM port.

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

// Wiring:
//	master MOSI PA06 (EXT1 16) -> slave MOSI PB22 (EXT3 16)
//	master MISO PA04 (EXT1 17) <- slave MISO PB16 (EXT3 17)
//	master SCK  PA07 (EXT1 18) -> slave SCK  PB23 (EXT3 18)
//	master CS   PA05 (EXT1 15) -> slave SS   PB17 (EXT3 15)
#define SPI_LOOPBACK_WIRING		"Wire PA06-PB22, PA04-PB16, PA07-PB23, PA05-PB17"

// Master on SERCOM0, function D: MISO PA04 pad 0, MOSI PA06 pad 2, SCK
// PA07 pad 3, CS on GPIO PA05
#define SPI_MASTER_DEVNO		0
#define SPI_MASTER_SCK_PORT		IOPORTA
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

// Slave on SERCOM5: MISO PB16 pad 0 and SS PB17 pad 1 on function C, MOSI
// PB22 pad 2 and SCK PB23 pad 3 on function D
#define SPI_SLAVE_DEVNO			5
#define SPI_SLAVE_SCK_PORT		IOPORTB
#define SPI_SLAVE_SCK_PIN		23
#define SPI_SLAVE_SCK_PINOP		IOPINOP_PERIPHD
#define SPI_SLAVE_MISO_PORT		IOPORTB
#define SPI_SLAVE_MISO_PIN		16
#define SPI_SLAVE_MISO_PINOP	IOPINOP_PERIPHC
#define SPI_SLAVE_MOSI_PORT		IOPORTB
#define SPI_SLAVE_MOSI_PIN		22
#define SPI_SLAVE_MOSI_PINOP	IOPINOP_PERIPHD
#define SPI_SLAVE_CS_PORT		IOPORTB
#define SPI_SLAVE_CS_PIN		17
#define SPI_SLAVE_CS_PINOP		IOPINOP_PERIPHC

// The SAM D port runs the SPI master in polling, interrupt, DMA and DMA with
// interrupt mode, the slave on interrupts without DMA.
#define SPI_TEST_MASTER_MASK	0x0F
#define SPI_TEST_SLAVE_MASK		0x01

// Cortex-M0+ priorities 0 to 3, the slave above the master
#define SPI_MASTER_INT_PRIO		2
#define SPI_SLAVE_INT_PRIO		1

#endif // __BOARD_H__
