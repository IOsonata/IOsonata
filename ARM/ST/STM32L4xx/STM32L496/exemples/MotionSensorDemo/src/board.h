/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	July 26, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __BOARD_H__
#define __BOARD_H__

#include "stm32l4xx.h"

#define STM32L476G_DISCO

// *** STM32L476G-DISCO
#define UART_NO				1
#define UART_RX_PORT		3
#define UART_RX_PIN			6
#define UART_RX_PINOP		(0x72)

#define UART_TX_PORT		3
#define UART_TX_PIN			5
#define UART_TX_PINOP		(0x72)

#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		0

#ifdef QSPI
#define QSPI_DEVNO			0
#define QSPI_SCK_PORT       4		// PE10
#define QSPI_SCK_PIN        10
#define QSPI_SCK_PINOP      (0xA2)	// AF10
#define QSPI_D0_PORT		4		// PE12
#define QSPI_D0_PIN        	12
#define QSPI_D0_PINOP      	(0xA2)	// AF10
#define QSPI_D1_PORT		4		// PE13
#define QSPI_D1_PIN        	13
#define QSPI_D1_PINOP      	(0xA2)	// AF10
#define QSPI_D2_PORT		4		// PE14
#define QSPI_D2_PIN        	14
#define QSPI_D2_PINOP      	(0xA2)	// AF10
#define QSPI_D3_PORT		4		// PE15
#define QSPI_D3_PIN        	15
#define QSPI_D3_PINOP      	(0xA2)	// AF10
#define QSPI_FLASH_CS_PORT	4		// PE11
#define QSPI_FLASH_CS_PIN   11
#define QSPI_FLASH_CS_PINOP (0xA2)	// AF10
#else
#if 0
#define SPI2_DEVNO			0
#define SPI2_SCK_PORT      	4		// PE13
#define SPI2_SCK_PIN       	13
#define SPI2_SCK_PINOP     	(0x52)	// AF5
#define SPI2_MISO_PORT		4		// PE14
#define SPI2_MISO_PIN       14
#define SPI2_MISO_PINOP     (0x52)	// AF5
#define SPI2_MOSI_PORT      4		// PE15
#define SPI2_MOSI_PIN       15
#define SPI2_MOSI_PINOP     (0x52)	// AF5
#else
#define SPI_DEVNO			1
#define SPI_SCK_PORT      	3		// PD1
#define SPI_SCK_PIN       	1
#define SPI_SCK_PINOP     	(0x52)	// AF5
#define SPI_MISO_PORT		3		// PD3
#define SPI_MISO_PIN       3
#define SPI_MISO_PINOP     (0x52)	// AF5
#define SPI_MOSI_PORT      3		// PD4
#define SPI_MOSI_PIN       4
#define SPI_MOSI_PINOP     (0x52)	// AF5
#endif
#define SPI_XL_CS_PORT		4		// PE0
#define SPI_XL_CS_PIN		0
#define SPI_XL_CS_PINOP		1

#define SPI_GY_CS_PORT		3		// PD7
#define SPI_GY_CS_PIN		7
#define SPI_GY_CS_PINOP		0

#define SPI_MG_CS_PORT		2		// PC0
#define SPI_MG_CS_PIN		0
#define SPI_MG_CS_PINOP		1

#define SPI2_FLASH_CS_PORT	4		// PE11
#define SPI2_FLASH_CS_PIN   11
#define SPI2_FLASH_CS_PINOP (0x52)	// AF5
#endif

#define I2C_SCL_PORT		5		// PF1
#define I2C_SCL_PIN			1
#define I2C_SCL_PINOP		(0x42)	// AF4

#define I2C_SDA_PORT		5		// PF0
#define I2C_SDA_PIN			0
#define I2C_SDA_PINOP		(0x42)	// AF4


#endif // __BOARD_H__

