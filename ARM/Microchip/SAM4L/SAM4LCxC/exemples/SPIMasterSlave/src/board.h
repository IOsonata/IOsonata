/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM4L8 Xplained Pro SPI slave loopback wiring.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

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
#include "coredev/system_core_clock.h"

#define MCUOSC { \
	{ OSC_TYPE_XTAL, 12000000, 20, 180 }, \
	{ OSC_TYPE_XTAL, 32768, 20, 125 }, true }

// Four adjacent caps; EXT4 I2C jumpers remain in place.
// EXT1 10 PB13 GPIO-CS <-> 12 PA24 SPI-NPCS0 (A)
// CS cap runs along the even-numbered column (10-12), not across a row.
// PA23 is unused: its pad remained low despite a high GPIO output latch.
// EXT1 17 PA21 GPIO-SCK <-> 18 PC30 SPI-SCK (B)
// EXT2  7 PC04 SPI-MISO (A) <-> 8 PC05 GPIO-MISO
// EXT2 15 PB11 GPIO-MOSI <-> 16 PA22 SPI-MOSI (A)
// Reference: SAM4L8 Xplained Pro user guide, tables 4-1/4-2.
// These pins also reach EXT5; disconnect any LCD extension for this test.
#define SPI_LOOPBACK_WIRING "Caps: EXT1 10-12 (same column), 17-18; EXT2 7-8, 15-16"
#define SPI_MASTER_DMA_ENABLE	false
#define SPI_MASTER_INT_ENABLE	true
#define SPI_MASTER_SOFTWARE 	true
#define SPI_MASTER_DEVNO 		0
#define SPI_MASTER_RATE 		100000
#define SPI_MASTER_SCK_PORT		IOPORTA
#define SPI_MASTER_SCK_PIN		21
#define SPI_MASTER_SCK_PINOP	IOPINOP_GPIO
#define SPI_MASTER_MISO_PORT	IOPORTC
#define SPI_MASTER_MISO_PIN		5
#define SPI_MASTER_MISO_PINOP	IOPINOP_GPIO
#define SPI_MASTER_MOSI_PORT	IOPORTB
#define SPI_MASTER_MOSI_PIN		11
#define SPI_MASTER_MOSI_PINOP	IOPINOP_GPIO
#define SPI_MASTER_CS_PORT		IOPORTB
#define SPI_MASTER_CS_PIN		13
#define SPI_MASTER_CS_PINOP 	IOPINOP_GPIO


#define SPI_SLAVE_DMA_ENABLE 	false
#define SPI_SLAVE_INT_ENABLE 	true
#define SPI_SLAVE_DEVNO 		0
#define SPI_SLAVE_SCK_PORT		IOPORTC
#define SPI_SLAVE_SCK_PIN		30
#define SPI_SLAVE_SCK_PINOP		IOPINOP_PERIPHB
#define SPI_SLAVE_MISO_PORT		IOPORTC
#define SPI_SLAVE_MISO_PIN 		4
#define SPI_SLAVE_MISO_PINOP 	IOPINOP_PERIPHA
#define SPI_SLAVE_MOSI_PORT 	IOPORTA
#define SPI_SLAVE_MOSI_PIN 		22
#define SPI_SLAVE_MOSI_PINOP 	IOPINOP_PERIPHA
#define SPI_SLAVE_CS_PORT 		IOPORTA
#define SPI_SLAVE_CS_PIN 		24
#define SPI_SLAVE_CS_PINOP 		IOPINOP_PERIPHA

// SAM4L8 Xplained Pro Virtual COM Port: USART1 on PC26/PC27, peripheral A.
#define UART_DEVNO				1
#define UART_RX_PORT			IOPORTC
#define UART_RX_PIN				26
#define UART_RX_PINOP			IOPINOP_PERIPHA
#define UART_TX_PORT			IOPORTC
#define UART_TX_PIN				27
#define UART_TX_PINOP			IOPINOP_PERIPHA
#define UART_CTS_PORT			-1
#define UART_CTS_PIN			-1
#define UART_CTS_PINOP			IOPINOP_GPIO
#define UART_RTS_PORT			-1
#define UART_RTS_PIN			-1
#define UART_RTS_PINOP			IOPINOP_GPIO

#endif // __BOARD_H__


