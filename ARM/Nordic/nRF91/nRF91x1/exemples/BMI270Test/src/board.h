/**-------------------------------------------------------------------------
@file	board.h

@brief	BMI270Test board definitions.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#define UART_DEVNO			0
#define UART_RX_PORT			0
#define UART_RX_PIN			0
#define UART_RX_PINOP			1
#define UART_TX_PORT			0
#define UART_TX_PIN			1
#define UART_TX_PINOP			1

#define I2C_DEVNO			2
#define I2C_SDA_PORT			0
#define I2C_SDA_PIN			9
#define I2C_SDA_PINOP			1
#define I2C_SCL_PORT			0
#define I2C_SCL_PIN			8
#define I2C_SCL_PINOP			1

#define SPI_DEVNO			3
#define SPI_SCK_PORT			0
#define SPI_SCK_PIN			13
#define SPI_SCK_PINOP			1
#define SPI_MISO_PORT			0
#define SPI_MISO_PIN			15
#define SPI_MISO_PINOP			1
#define SPI_MOSI_PORT			0
#define SPI_MOSI_PIN			14
#define SPI_MOSI_PINOP			1

/* Keep the shared SPI3 CS ordering identical to the Thingy:91 X schematic/DTS. */
#define SPI_FLASH_CS_PORT		0
#define SPI_FLASH_CS_PIN		12
#define SPI_FLASH_CS_PINOP		1
#define SPI_WIFI_CS_PORT		0
#define SPI_WIFI_CS_PIN		17
#define SPI_WIFI_CS_PINOP		1
#define SPI_BMI270_CS_PORT		0
#define SPI_BMI270_CS_PIN		10
#define SPI_BMI270_CS_PINOP		1
#define BMI270_SPI_CS			2

#define BMI270_IRQ_PORT			0
#define BMI270_IRQ_PIN			6

#endif
