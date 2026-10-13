/**-------------------------------------------------------------------------
@file	board.h

@brief	SPI master and slave on the same FPB-R9A02G021

SPI20 is the master, SPI21 the slave, both on SAU1. Wiring:
	P101 (SCK20)	to P204 (SCK21)		SCK
	P103 (SO20)		to P205 (SI21)		MOSI
	P206 (SO21)		to P102 (SI20)		MISO
	P104 (GPIO)		to P202 (IRQ2_C)	chip select
P101 to P104 are on the Arduino header, P202 and P204 to P206 on the Pmod
connectors: check the pins against the board schematic.

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

#include "coredev/iopincfg.h"

// The library defaults are kept: HOCO 48 MHz core clock and PCLKB, LOCO
// low frequency clock. The slave operation clock is then 24 MHz, its SCK at
// most 3 MHz.

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

// No RTS and CTS
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		IOPINOP_GPIO

// Master SPI20, SAU1 channel 0: SCK20 P101, SI20 P102, SO20 P103, chip
// select P104 as GPIO
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

// Master receptions in the SAU1_ENDI0 interrupt by the DTC, transmissions
// polled
#define SPI_MASTER_INT_ENABLE	true
#define SPI_MASTER_DMA_ENABLE	true

// Slave SPI21, SAU1 channel 1: SCK21 P204, SO21 P206 (MISO), SI21 P205
// (MOSI), chip select P202 as GPIO input on IRQ2. The slave needs the
// interrupt; with DMA the DTC moves its bytes, false for one interrupt
// per byte. Its priority is above the master's: the master interrupt at
// the end of a DTC reception waits for the last byte, a slave interrupt
// held that long would be late.
#define SPI_SLAVE_DEVNO			5
#define SPI_SLAVE_SCK_PORT		2
#define SPI_SLAVE_SCK_PIN		4
#define SPI_SLAVE_SCK_PINOP		IOPINOP_FUNC16
#define SPI_SLAVE_MISO_PORT		2
#define SPI_SLAVE_MISO_PIN		6
#define SPI_SLAVE_MISO_PINOP	IOPINOP_FUNC16
#define SPI_SLAVE_MOSI_PORT		2
#define SPI_SLAVE_MOSI_PIN		5
#define SPI_SLAVE_MOSI_PINOP	IOPINOP_FUNC16
#define SPI_SLAVE_CS_PORT		2
#define SPI_SLAVE_CS_PIN		2
#define SPI_SLAVE_CS_PINOP		IOPINOP_GPIO

#define SPI_SLAVE_INT_ENABLE	true
#define SPI_SLAVE_DMA_ENABLE	true
#define SPI_SLAVE_INT_PRIO		7
#define SPI_MASTER_INT_PRIO		6

#define SPI_LOOPBACK_WIRING		"Connect P101-P204 SCK, P103-P205 MOSI, P206-P102 MISO, P104-P202 CS"

#endif
