/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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

#include "nrf.h"
#include "blueio_board.h"

#define BLUEIO840

#define UART_RX_PORT		BLUEIO_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_UART_RX_PINOP
#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT		BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN		BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP		BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP

#define QSPI

#ifdef QSPI
#ifdef BLUEIO840
#define SPI_DEVNO     					4
#define SPI_PHY						SPIPHY_QUAD_SDR
#define QSPI_SCK_PORT        			0
#define QSPI_SCK_PIN         			13
#define QSPI_SCK_PINOP       			1
#define QSPI_D0_PORT       				0
#define QSPI_D0_PIN        				12
#define QSPI_D0_PINOP      				1
#define QSPI_D1_PORT       				1
#define QSPI_D1_PIN        				9
#define QSPI_D1_PINOP      				1
#define QSPI_D2_PORT       				0
#define QSPI_D2_PIN        				11
#define QSPI_D2_PINOP      				1
#define QSPI_D3_PORT       				0
#define QSPI_D3_PIN        				14
#define QSPI_D3_PINOP      				1
#define SPI_CS_MODE					SPICSEL_AUTO

#define FLASH_CS_PORT          		1
#define FLASH_CS_PIN           		8
#define FLASH_CS_PINOP         		0
#endif

#else
#define SPI_DEVNO      					1
#define SPI_PHY							SPIPHY_NORMAL
#define SPI_MISO_PORT       			1
#define SPI_MISO_PIN        			0
#define SPI_MISO_PINOP      			1
#define SPI_MOSI_PORT       			0
#define SPI_MOSI_PIN        			21
#define SPI_MOSI_PINOP      			1
#define SPI_SCK_PORT        			0
#define SPI_SCK_PIN         			19
#define SPI_SCK_PINOP       			1
#define SPI_CS_MODE						SPICSEL_AUTO

#define FLASH_CS_PORT          		0
#define FLASH_CS_PIN           		24
#define FLASH_CS_PINOP         		0

#define FLASH_HOLD_PORT            	0
#define FLASH_HOLD_PIN             	26
#define FLASH_HOLD_PINOP           	0
#endif

#ifdef QSPI
#define SPI_PINS_CFG	{ \
	{QSPI_SCK_PORT, QSPI_SCK_PIN, QSPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D0_PORT, QSPI_D0_PIN, QSPI_D0_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D1_PORT, QSPI_D1_PIN, QSPI_D1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D2_PORT, QSPI_D2_PIN, QSPI_D2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{QSPI_D3_PORT, QSPI_D3_PIN, QSPI_D3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{FLASH_CS_PORT, FLASH_CS_PIN, FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
#else
#define SPI_PINS_CFG	{ \
	{SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
    {SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{FLASH_CS_PORT, FLASH_CS_PIN, FLASH_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
#endif

#define FLASH_SECTOR_SIZE				FLASH_MX25L25645G_SECTSIZE
#define FLASH_CFG(InitCB, WaitCB)		FLASH_MX25L25645G(InitCB, WaitCB)

#endif // __BOARD_H__

