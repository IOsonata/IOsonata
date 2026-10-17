/**-------------------------------------------------------------------------
@example	board.h

@brief	Board definitions of the STM32WBA65 SPI master and slave example

		The slave is SPI2 on the Arduino header of the STM32WBA65I-DK1, AF5:
		SCK D13 (PB10), MISO D12 (PA9), MOSI D11 (PC3), NSS D10 (PB9).
		The master is the software SPI on four other header pins: SCK D3
		(PB13), MOSI D6 (PB0), MISO A0 (PA4), chip select D8 (PA10).
		Wire D3 to D13, D6 to D11, A0 to D12 and D8 to D10.
		The result is printed on the first ST-LINK virtual COM port, USART1.

		Not tested on hardware.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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

// Board, one of the names in stm32_boards.h
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"

#define SPI_LOOPBACK_WIRING			"Wire D3-D13 SCK, D6-D11 MOSI, A0-D12 MISO, D8-D10 CS"

// First ST-LINK virtual COM port, USART1, RX PA8 and TX PB12, AF7
#define UART_DEVNO					1
#define UART_RX_PORT				IOPORTA
#define UART_RX_PIN					8
#define UART_RX_PINOP				IOPINOP_FUNC7
#define UART_TX_PORT				IOPORTB
#define UART_TX_PIN					12
#define UART_TX_PINOP				IOPINOP_FUNC7
#define UART_CTS_PORT				-1
#define UART_CTS_PIN				-1
#define UART_CTS_PINOP				IOPINOP_GPIO
#define UART_RTS_PORT				-1
#define UART_RTS_PIN				-1
#define UART_RTS_PINOP				IOPINOP_GPIO

// Master: software SPI, plain GPIO on these pins
#define SPI_MASTER_SOFTWARE			true
#define SPI_MASTER_DEVNO			0
#define SPI_MASTER_SCK_PORT			IOPORTB
#define SPI_MASTER_SCK_PIN			13
#define SPI_MASTER_SCK_PINOP		IOPINOP_GPIO
#define SPI_MASTER_MISO_PORT		IOPORTA
#define SPI_MASTER_MISO_PIN			4
#define SPI_MASTER_MISO_PINOP		IOPINOP_GPIO
#define SPI_MASTER_MOSI_PORT		IOPORTB
#define SPI_MASTER_MOSI_PIN			0
#define SPI_MASTER_MOSI_PINOP		IOPINOP_GPIO
#define SPI_MASTER_CS_PORT			IOPORTA
#define SPI_MASTER_CS_PIN			10
#define SPI_MASTER_CS_PINOP			IOPINOP_GPIO
#define SPI_MASTER_RATE				500000

// Slave: SPI2 (DevNo 1), the fourth pin is the hardware NSS
#define SPI_SLAVE_DEVNO				1
#define SPI_SLAVE_SCK_PORT			IOPORTB
#define SPI_SLAVE_SCK_PIN			10
#define SPI_SLAVE_SCK_PINOP			IOPINOP_FUNC5
#define SPI_SLAVE_MISO_PORT			IOPORTA
#define SPI_SLAVE_MISO_PIN			9
#define SPI_SLAVE_MISO_PINOP		IOPINOP_FUNC5
#define SPI_SLAVE_MOSI_PORT			IOPORTC
#define SPI_SLAVE_MOSI_PIN			3
#define SPI_SLAVE_MOSI_PINOP		IOPINOP_FUNC5
#define SPI_SLAVE_CS_PORT			IOPORTB
#define SPI_SLAVE_CS_PIN			9
#define SPI_SLAVE_CS_PINOP			IOPINOP_FUNC5

// Start with the interrupt slave. Set SPI_SLAVE_DMA_ENABLE to true for GPDMA.
#define SPI_SLAVE_INT_ENABLE		true
#define SPI_SLAVE_DMA_ENABLE		false

#endif
