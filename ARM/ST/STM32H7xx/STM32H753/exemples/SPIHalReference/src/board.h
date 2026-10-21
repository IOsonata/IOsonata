/**-------------------------------------------------------------------------
@file	board.h

@brief	NUCLEO-H753ZI board definitions for SPIHalReference.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#define BOARD NUCLEO_H753ZI
#include "stm32_boards.h"

#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_RX_PORT IOPORTD
#define UART_RX_PIN 9
#define UART_RX_PINOP IOPINOP_FUNC7
#define UART_TX_PORT IOPORTD
#define UART_TX_PIN 8
#define UART_TX_PINOP IOPINOP_FUNC7
#define UART_CTS_PORT -1
#define UART_CTS_PIN -1
#define UART_CTS_PINOP 0
#define UART_RTS_PORT -1
#define UART_RTS_PIN -1
#define UART_RTS_PINOP 0

#define SPI_LOOPBACK_WIRING "PA5-PC10 SCK, PA6-PC11 MISO, PB5-PC12 MOSI, PD14-PA15 CS"

#define SPI_MASTER_SCK_PORT IOPORTA
#define SPI_MASTER_SCK_PIN 5
#define SPI_MASTER_SCK_PINOP IOPINOP_FUNC5

#define SPI_MASTER_MISO_PORT IOPORTA
#define SPI_MASTER_MISO_PIN 6
#define SPI_MASTER_MISO_PINOP IOPINOP_FUNC5

#define SPI_MASTER_MOSI_PORT IOPORTB
#define SPI_MASTER_MOSI_PIN 5
#define SPI_MASTER_MOSI_PINOP IOPINOP_FUNC5

#define SPI_MASTER_CS_PORT IOPORTD
#define SPI_MASTER_CS_PIN 14
#define SPI_MASTER_CS_PINOP IOPINOP_GPIO

#define SPI_SLAVE_SCK_PORT IOPORTC
#define SPI_SLAVE_SCK_PIN 10
#define SPI_SLAVE_SCK_PINOP IOPINOP_FUNC6

#define SPI_SLAVE_MISO_PORT IOPORTC
#define SPI_SLAVE_MISO_PIN 11
#define SPI_SLAVE_MISO_PINOP IOPINOP_FUNC6

#define SPI_SLAVE_MOSI_PORT IOPORTC
#define SPI_SLAVE_MOSI_PIN 12
#define SPI_SLAVE_MOSI_PINOP IOPINOP_FUNC6

#define SPI_SLAVE_CS_PORT IOPORTA
#define SPI_SLAVE_CS_PIN 15
#define SPI_SLAVE_CS_PINOP IOPINOP_FUNC6

#endif
