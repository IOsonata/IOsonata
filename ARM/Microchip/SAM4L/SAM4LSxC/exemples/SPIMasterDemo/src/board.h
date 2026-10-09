/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM4LS C-package example SPI master demo wiring.

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

// Example wiring only. Check these pins and clocks against your SAM4LS board.
// SAM4LS hardware validation has not been performed.

#include "coredev/iopincfg.h"
#include "coredev/system_core_clock.h"

// Use the library default internal clocks; define MCUOSC for your board.

// SAM4LS C-package example dedicated SPI signals.
// CS is intentionally GPIO: the IOsonata SPI API owns CS across a complete
// Start/Tx/Rx/Stop transaction rather than one hardware character.
#define SPI_MASTER_DEVNO		0
#define SPI_MASTER_SCK_PORT		IOPORTC
#define SPI_MASTER_SCK_PIN		30
#define SPI_MASTER_SCK_PINOP		IOPINOP_PERIPHB
#define SPI_MASTER_MISO_PORT		IOPORTA
#define SPI_MASTER_MISO_PIN		21
#define SPI_MASTER_MISO_PINOP		IOPINOP_PERIPHA
#define SPI_MASTER_MOSI_PORT		IOPORTA
#define SPI_MASTER_MOSI_PIN		22
#define SPI_MASTER_MOSI_PINOP		IOPINOP_PERIPHA
#define SPI_MASTER_CS_PORT		IOPORTC
#define SPI_MASTER_CS_PIN		3
#define SPI_MASTER_CS_PINOP		IOPINOP_GPIO

// SAM4LS C-package example UART adapter: USART1 on PC26/PC27, peripheral A.
#define UART_DEVNO			1
#define UART_RX_PORT		IOPORTC
#define UART_RX_PIN			26
#define UART_RX_PINOP		IOPINOP_PERIPHA
#define UART_TX_PORT		IOPORTC
#define UART_TX_PIN			27
#define UART_TX_PINOP		IOPINOP_PERIPHA
#define UART_CTS_PORT		-1
#define UART_CTS_PIN			-1
#define UART_CTS_PINOP		IOPINOP_GPIO
#define UART_RTS_PORT		-1
#define UART_RTS_PIN			-1
#define UART_RTS_PINOP		IOPINOP_GPIO

#endif // __BOARD_H__
