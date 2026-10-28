/**-------------------------------------------------------------------------
@example	board.h

@brief	LPCXpresso54628 board definitions for SPIMasterSlave

The master is Flexcomm 3 on the host expansion header J14, the slave is
Flexcomm 9 on the Arduino header (J9). Wire the two buses together and
install JP6, see README.md.

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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

// Output on Flexcomm 0, P0_29 RXD and P0_30 TXD: the LPC-Link2 VCOM, or the
// P4 header when JP6 turns the VCOM off. No RTS and CTS.
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			29
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			30
#define UART_TX_PINOP		IOPINOP_FUNC1
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

// Both Flexcomms have DMA channels, all eight mode combinations run
#define SPI_MASTER_HAS_DMA		1
#define SPI_SLAVE_HAS_DMA		1
#define SPI_MASTER_DEVNO		3
#define SPI_SLAVE_DEVNO			9
#define SPI_MASTER_INT_PRIO		7
#define SPI_SLAVE_INT_PRIO		6
#define SPI_TEST_MODE_MASK		0x0F
#define SPI_MASTER_RATE			1000000
#define SPI_LOOPBACK_WIRING		"J14-4 P0_0 to J9-9 P3_20 SCK, J14-3 P0_2 to J9-11 P3_22 MISO, " \
								"J14-2 P0_3 to J9-13 P3_21 MOSI, J14-1 P0_1 to J9-15 P3_30 CS, JP6 on"

// Master, Flexcomm 3 on J14: P0_0 SCK function 2, P0_2 MISO and P0_3 MOSI
// function 1, chip select P0_1 as GPIO
#define SPI_MASTER_SCK_PORT		0
#define SPI_MASTER_SCK_PIN		0
#define SPI_MASTER_SCK_PINOP	IOPINOP_FUNC2
#define SPI_MASTER_MISO_PORT	0
#define SPI_MASTER_MISO_PIN		2
#define SPI_MASTER_MISO_PINOP	IOPINOP_FUNC1
#define SPI_MASTER_MOSI_PORT	0
#define SPI_MASTER_MOSI_PIN		3
#define SPI_MASTER_MOSI_PINOP	IOPINOP_FUNC1
#define SPI_MASTER_CS_PORT		0
#define SPI_MASTER_CS_PIN		1
#define SPI_MASTER_CS_PINOP		IOPINOP_GPIO

// Slave, Flexcomm 9 on the Arduino header: P3_20 SCK (D13), P3_22 MISO
// (D12), P3_21 MOSI (D11) and P3_30 SSEL0 (D10), all function 1, the pins
// of the NXP SDK SPI board to board examples
#define SPI_SLAVE_SCK_PORT		3
#define SPI_SLAVE_SCK_PIN		20
#define SPI_SLAVE_SCK_PINOP		IOPINOP_FUNC1
#define SPI_SLAVE_MISO_PORT		3
#define SPI_SLAVE_MISO_PIN		22
#define SPI_SLAVE_MISO_PINOP	IOPINOP_FUNC1
#define SPI_SLAVE_MOSI_PORT		3
#define SPI_SLAVE_MOSI_PIN		21
#define SPI_SLAVE_MOSI_PINOP	IOPINOP_FUNC1
#define SPI_SLAVE_CS_PORT		3
#define SPI_SLAVE_CS_PIN		30
#define SPI_SLAVE_CS_PINOP		IOPINOP_FUNC1

#endif
