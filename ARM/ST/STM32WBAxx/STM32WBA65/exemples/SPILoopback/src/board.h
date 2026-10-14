/**-------------------------------------------------------------------------
@example	board.h

@brief	Board definitions of the STM32WBA65 SPI loopback example

		SPI1 master on the Arduino header of the STM32WBA65I-DK1, from
		stm32_boards.h: SCK PB4, MISO PB3, MOSI PA15, AF5, chip select PA12
		driven as a GPIO. Connect MOSI (PA15) to MISO (PB3) with a jumper.
		The result is printed on the first ST-LINK virtual COM port, USART1.

		Not tested on hardware.

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

// Board, one of the names in stm32_boards.h
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"

#define SPI_LOOPBACK_WIRING	"Jumper PA15 (MOSI) <-> PB3 (MISO)"

// Polling master: the driver has no interrupt or DMA mode
#define SPI_MASTER_DMA_ENABLE		false
#define SPI_MASTER_INT_ENABLE		false

#define SPI_MASTER_DEVNO			BOARD_SPI_DEVNO
#define SPI_MASTER_SCK_PORT			BOARD_SPI_SCK_PORT
#define SPI_MASTER_SCK_PIN			BOARD_SPI_SCK_PIN
#define SPI_MASTER_SCK_PINOP		BOARD_SPI_SCK_PINOP
#define SPI_MASTER_MISO_PORT		BOARD_SPI_MISO_PORT
#define SPI_MASTER_MISO_PIN			BOARD_SPI_MISO_PIN
#define SPI_MASTER_MISO_PINOP		BOARD_SPI_MISO_PINOP
#define SPI_MASTER_MOSI_PORT		BOARD_SPI_MOSI_PORT
#define SPI_MASTER_MOSI_PIN			BOARD_SPI_MOSI_PIN
#define SPI_MASTER_MOSI_PINOP		BOARD_SPI_MOSI_PINOP
#define SPI_MASTER_CS_PORT			BOARD_SPI_CS_PORT
#define SPI_MASTER_CS_PIN			BOARD_SPI_CS_PIN
#define SPI_MASTER_CS_PINOP			BOARD_SPI_CS_PINOP
#define SPI_MASTER_RATE				1000000

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

#endif
