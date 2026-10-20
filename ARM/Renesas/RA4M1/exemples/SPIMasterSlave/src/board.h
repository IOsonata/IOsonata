/**-------------------------------------------------------------------------
@file	board.h

@brief	RA4M1 shared SPI master/slave example board configuration.

		SCI0 on P410/P411 is the console. RSPI0 is the master and RSPI1 is
		the slave, wired together with four jumper wires. The pins below are
		on the Renesas EK-RA4M1 headers, with no board modification.

@author	Hoang Nguyen Hoan
@date	Oct. 14, 2026

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

// SCI0 console, receive pin first in the pin map.
#define UART_DEVNO				0
#define UART_RX_PORT			4
#define UART_RX_PIN				10
#define UART_RX_PINOP			IOPINOP_FUNC3
#define UART_TX_PORT			4
#define UART_TX_PIN				11
#define UART_TX_PINOP			IOPINOP_FUNC3

// No RTS and CTS
#define UART_CTS_PORT			-1
#define UART_CTS_PIN			-1
#define UART_CTS_PINOP			IOPINOP_GPIO
#define UART_RTS_PORT			-1
#define UART_RTS_PIN			-1
#define UART_RTS_PINOP			IOPINOP_GPIO

// Master RSPI0, SPI function PSEL 6 (IOPINOP_FUNC5):
// RSPCKA P102, MISOA P100, MOSIA P101. The chip select is a GPIO driven
// by the driver, P103.
// EK-RA4M1: P102 J2-6, P100 J1-1, P101 J3-8, P103 J1-29.
#define SPI_MASTER_DEVNO		0
#define SPI_MASTER_SCK_PORT		1
#define SPI_MASTER_SCK_PIN		2
#define SPI_MASTER_SCK_PINOP	IOPINOP_FUNC5
#define SPI_MASTER_MISO_PORT	1
#define SPI_MASTER_MISO_PIN		0
#define SPI_MASTER_MISO_PINOP	IOPINOP_FUNC5
#define SPI_MASTER_MOSI_PORT	1
#define SPI_MASTER_MOSI_PIN		1
#define SPI_MASTER_MOSI_PINOP	IOPINOP_FUNC5
#define SPI_MASTER_CS_PORT		1
#define SPI_MASTER_CS_PIN		3
#define SPI_MASTER_CS_PINOP		IOPINOP_GPIO

// The slave interrupt takes every byte, so keep the rate moderate.
#define SPI_MASTER_RATE			400000

// The master is polled first. Set SPI_MASTER_INT_ENABLE to true to run the
// interrupt transfers. There is no DMA on this port.
#define SPI_MASTER_INT_ENABLE	false
#define SPI_MASTER_DMA_ENABLE	false

// Slave RSPI1: RSPCKB P111, MISOB P110, MOSIB P203, SSLB0 P112.
// EK-RA4M1: P111 J1-33, P110 J1-9, P203 J1-12, P112 J1-40. P108 and P109
// are shared with the on-board debugger and are not used.
#define SPI_SLAVE_DEVNO			1
#define SPI_SLAVE_SCK_PORT		1
#define SPI_SLAVE_SCK_PIN		11
#define SPI_SLAVE_SCK_PINOP		IOPINOP_FUNC5
#define SPI_SLAVE_MISO_PORT		1
#define SPI_SLAVE_MISO_PIN		10
#define SPI_SLAVE_MISO_PINOP	IOPINOP_FUNC5
#define SPI_SLAVE_MOSI_PORT		2
#define SPI_SLAVE_MOSI_PIN		3
#define SPI_SLAVE_MOSI_PINOP	IOPINOP_FUNC5
#define SPI_SLAVE_CS_PORT		1
#define SPI_SLAVE_CS_PIN		12
#define SPI_SLAVE_CS_PINOP		IOPINOP_FUNC5

// The RSPI slave works with CPHA = 1 only: modes 1 and 3.
#define SPI_TEST_MODE_MASK		0x0A

#define SPI_SLAVE_INT_ENABLE	true
#define SPI_SLAVE_DMA_ENABLE	false
#define SPI_SLAVE_INT_PRIO		5
#define SPI_MASTER_INT_PRIO		6

#define SPI_LOOPBACK_WIRING		"Connect P102-P111 SCK, P101-P203 MOSI, P110-P100 MISO, P103-P112 CS " \
								"(J2-6 to J1-33, J3-8 to J1-12, J1-9 to J1-1, J1-29 to J1-40)"

#endif
