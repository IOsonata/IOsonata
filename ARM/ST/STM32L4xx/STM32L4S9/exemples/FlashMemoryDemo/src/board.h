/**-------------------------------------------------------------------------
@file	board.h

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

// *** STM32L476G-DISCO
#define UART_NO				1
#define UART_RX_PORT		IOPORTD
#define UART_RX_PIN			6
#define UART_RX_PINOP		(0x72)

#define UART_TX_PORT		IOPORTD
#define UART_TX_PIN			5
#define UART_TX_PINOP		(0x72)

#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		0

#define QSPI_DEVNO			3
#define QSPI_SCK_PORT       IOPORTE		// PE10
#define QSPI_SCK_PIN        10
#define QSPI_SCK_PINOP      (0xA2)	// AF10
#define QSPI_D0_PORT		IOPORTE		// PE12
#define QSPI_D0_PIN        	12
#define QSPI_D0_PINOP      	(0xA2)	// AF10
#define QSPI_D1_PORT		IOPORTE		// PE13
#define QSPI_D1_PIN        	13
#define QSPI_D1_PINOP      	(0xA2)	// AF10
#define QSPI_D2_PORT		IOPORTE		// PE14
#define QSPI_D2_PIN        	14
#define QSPI_D2_PINOP      	(0xA2)	// AF10
#define QSPI_D3_PORT		IOPORTE		// PE15
#define QSPI_D3_PIN        	15
#define QSPI_D3_PINOP      	(0xA2)	// AF10
#define QSPI_FLASH_CS_PORT	IOPORTE		// PE11
#define QSPI_FLASH_CS_PIN   11
#define QSPI_FLASH_CS_PINOP (0xA2)	// AF10

#define SPI0_DEVNO			0
#define SPI0_SCK_PORT       IOPORTE		// PE13
#define SPI0_SCK_PIN        13
#define SPI0_SCK_PINOP      (0x52)	// AF5
#define SPI0_MISO_PORT		IOPORTE		// PE14
#define SPI0_MISO_PIN		14
#define SPI0_MISO_PINOP		(0x52)	// AF5
#define SPI0_MOSI_PORT      IOPORTE		// PE15
#define SPI0_MOSI_PIN       15
#define SPI0_MOSI_PINOP     (0x52)	// AF5

//#define SPI_DEVNO			3
#define SPI_CLK_PORT		IOPORTB		// PB3
#define SPI_CLK_PIN			3
#define SPI_CLK_PINOP		0x62	// AF6
#define SPI_MISO_PORT		IOPORTB		// PB4
#define SPI_MISO_PIN		4
#define SPI_MISO_PINOP		0x62	// AF6
#define SPI_MOSI_PORT		IOPORTC		// PC12
#define SPI_MOSI_PIN		12
#define SPI_MOSI_PINOP		0x62	// AF6

#if 0
#define SPI_FLASH_CS_PORT	2		// PE12
#define SPI_FLASH_CS_PIN    12
#define SPI_FLASH_CS_PINOP  (0x52)	// AF5
#else
#define SPI_FLASH_CS_PORT	IOPORTA		//PA15
#define SPI_FLASH_CS_PIN	15
#define SPI_FLASH_CS_PINOP	1		// GPIO Output
#endif

#define SPI_DEVNO			2
#define SPI_PHY				SPIPHY_NORMAL	//SPIPHY_QUAD_SDR
#define SPI_CS_MODE			SPICSEL_DRIVER

#if 0
#define SPI_PINS_CFG	{ \
	{QSPI_SCK_PORT, QSPI_SCK_PIN, QSPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D0_PORT, QSPI_D0_PIN, QSPI_D0_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D1_PORT, QSPI_D1_PIN, QSPI_D1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D2_PORT, QSPI_D2_PIN, QSPI_D2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D3_PORT, QSPI_D3_PIN, QSPI_D3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_FLASH_CS_PORT, QSPI_FLASH_CS_PIN, QSPI_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
#else
#define SPI_PINS_CFG	{ \
	{SPI_CLK_PORT, SPI_CLK_PIN, SPI_CLK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_FLASH_CS_PORT, SPI_FLASH_CS_PIN, SPI_FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
}
#endif


#endif // __BOARD_H__

