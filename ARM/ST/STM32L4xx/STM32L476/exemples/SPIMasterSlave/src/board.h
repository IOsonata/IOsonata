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

// Board, one of the names in stm32_boards.h. This STM32L476 project builds for
// B_L475E_IOT01A (B-L475E-IOT01A and B-L475E-IOT01A2, STM32L475VG).
#define BOARD				B_L475E_IOT01A

#include "stm32_boards.h"

#define SPI_LOOPBACK_WIRING			"Wire D13-PMOD4 SCK, D11-PMOD2 MOSI, D12-PMOD3 MISO, D10-PMOD8 CS"

// Output on the ST-LINK virtual COM port, USART1 (DevNo 1), RX PB7 and TX
// PB6, AF7, the BOARD_VCP_PINS of stm32_boards.h. The Release build sends
// printf there, the Debug build uses semihosting.
#define UART_DEVNO					BOARD_VCP_DEVNO
#define UART_RX_PORT				IOPORTB
#define UART_RX_PIN					7
#define UART_RX_PINOP				IOPINOP_FUNC7
#define UART_TX_PORT				IOPORTB
#define UART_TX_PIN					6
#define UART_TX_PINOP				IOPINOP_FUNC7
#define UART_CTS_PORT				-1
#define UART_CTS_PIN				-1
#define UART_CTS_PINOP				IOPINOP_GPIO
#define UART_RTS_PORT				-1
#define UART_RTS_PIN				-1
#define UART_RTS_PINOP				IOPINOP_GPIO

// Master: SPI1 (DevNo 0) on the Arduino header, AF5: SCK D13 PA5 (also LD1),
// MISO D12 PA6, MOSI D11 PA7. The chip select D10 PA2 is a GPIO.
#define SPI_MASTER_SOFTWARE			false
#define SPI_MASTER_DEVNO			0
#define SPI_MASTER_SCK_PORT			IOPORTA
#define SPI_MASTER_SCK_PIN			5
#define SPI_MASTER_SCK_PINOP		IOPINOP_FUNC5
#define SPI_MASTER_MISO_PORT		IOPORTA
#define SPI_MASTER_MISO_PIN			6
#define SPI_MASTER_MISO_PINOP		IOPINOP_FUNC5
#define SPI_MASTER_MOSI_PORT		IOPORTA
#define SPI_MASTER_MOSI_PIN			7
#define SPI_MASTER_MOSI_PINOP		IOPINOP_FUNC5
#define SPI_MASTER_CS_PORT			IOPORTA
#define SPI_MASTER_CS_PIN			2
#define SPI_MASTER_CS_PINOP			IOPINOP_GPIO
#define SPI_MASTER_RATE				500000

// The slave takes each byte in its interrupt, so it runs above the master
#define SPI_SLAVE_INT_PRIO			5
#define SPI_MASTER_INT_PRIO			6

// Slave: SPI2 (DevNo 1) on the Pmod connector CN10, AF5: SCK pin 4 PD1, MISO
// pin 3 PD3, MOSI pin 2 PD4, the fourth pin is the hardware NSS, pin 8 PD0.
// The other SPI2 pins of the device are taken on this board: PB13 and PB12 by
// the WiFi module, PB14 by LD2, PB15 by the sub-GHz module.
#define SPI_SLAVE_DEVNO				1
#define SPI_SLAVE_SCK_PORT			IOPORTD
#define SPI_SLAVE_SCK_PIN			1
#define SPI_SLAVE_SCK_PINOP			IOPINOP_FUNC5
#define SPI_SLAVE_MISO_PORT			IOPORTD
#define SPI_SLAVE_MISO_PIN			3
#define SPI_SLAVE_MISO_PINOP		IOPINOP_FUNC5
#define SPI_SLAVE_MOSI_PORT			IOPORTD
#define SPI_SLAVE_MOSI_PIN			4
#define SPI_SLAVE_MOSI_PINOP		IOPINOP_FUNC5
#define SPI_SLAVE_CS_PORT			IOPORTD
#define SPI_SLAVE_CS_PIN			0
#define SPI_SLAVE_CS_PINOP			IOPINOP_FUNC5

#endif // __BOARD_H__
