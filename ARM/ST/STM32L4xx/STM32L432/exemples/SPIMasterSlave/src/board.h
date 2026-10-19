/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

// Board, one of the names in stm32_boards.h. This STM32L432 project builds for
// NUCLEO_L432KC.
#define BOARD				NUCLEO_L432KC

#include "stm32_boards.h"

#define SPI_LOOPBACK_WIRING			"Wire A1-D13 SCK, A6-D11 MOSI, D10-D12 MISO, D3-A3 CS"

// Output on the ST-LINK virtual COM port, USART2 (DevNo 2), RX PA15 AF3 and
// TX PA2 AF7, the BOARD_VCP_PINS of stm32_boards.h. The Release build sends
// printf there, the Debug build uses semihosting.
#define UART_DEVNO					BOARD_VCP_DEVNO
#define UART_RX_PORT				IOPORTA
#define UART_RX_PIN					15
#define UART_RX_PINOP				IOPINOP_FUNC3
#define UART_TX_PORT				IOPORTA
#define UART_TX_PIN					2
#define UART_TX_PINOP				IOPINOP_FUNC7
#define UART_CTS_PORT				-1
#define UART_CTS_PIN				-1
#define UART_CTS_PINOP				IOPINOP_GPIO
#define UART_RTS_PORT				-1
#define UART_RTS_PIN				-1
#define UART_RTS_PINOP				IOPINOP_GPIO

// Master: SPI1 (DevNo 0), AF5: SCK A1 PA1, MISO D10 PA11, MOSI A6 PA7. The
// chip select D3 PB0 is a GPIO. A6 and D12 are also the I2C3 lines of the
// I2CMasterSlave example: a breadboard row can hold D1, A6 and D11, another
// D0, D12 and D10.
#define SPI_MASTER_SOFTWARE			false
#define SPI_MASTER_DEVNO			0
#define SPI_MASTER_SCK_PORT			IOPORTA
#define SPI_MASTER_SCK_PIN			1
#define SPI_MASTER_SCK_PINOP		IOPINOP_FUNC5
#define SPI_MASTER_MISO_PORT		IOPORTA
#define SPI_MASTER_MISO_PIN			11
#define SPI_MASTER_MISO_PINOP		IOPINOP_FUNC5
#define SPI_MASTER_MOSI_PORT		IOPORTA
#define SPI_MASTER_MOSI_PIN			7
#define SPI_MASTER_MOSI_PINOP		IOPINOP_FUNC5
#define SPI_MASTER_CS_PORT			IOPORTB
#define SPI_MASTER_CS_PIN			0
#define SPI_MASTER_CS_PINOP			IOPINOP_GPIO
#define SPI_MASTER_RATE				500000

// The slave takes each byte in its interrupt, so it runs above the master
#define SPI_SLAVE_INT_PRIO			5
#define SPI_MASTER_INT_PRIO			6

// Slave: SPI3 (DevNo 2), AF6: SCK D13 PB3 (also LD3), MISO D12 PB4, MOSI D11
// PB5, the fourth pin is the hardware NSS A3 PA4
#define SPI_SLAVE_DEVNO				2
#define SPI_SLAVE_SCK_PORT			IOPORTB
#define SPI_SLAVE_SCK_PIN			3
#define SPI_SLAVE_SCK_PINOP			IOPINOP_FUNC6
#define SPI_SLAVE_MISO_PORT			IOPORTB
#define SPI_SLAVE_MISO_PIN			4
#define SPI_SLAVE_MISO_PINOP		IOPINOP_FUNC6
#define SPI_SLAVE_MOSI_PORT			IOPORTB
#define SPI_SLAVE_MOSI_PIN			5
#define SPI_SLAVE_MOSI_PINOP		IOPINOP_FUNC6
#define SPI_SLAVE_CS_PORT			IOPORTA
#define SPI_SLAVE_CS_PIN			4
#define SPI_SLAVE_CS_PINOP			IOPINOP_FUNC6

// The L4 SPI has no DMA yet: master polling and interrupt, slave interrupt
#define SPI_TEST_MASTER_MASK		0x03
#define SPI_TEST_SLAVE_MASK			0x01

#endif
