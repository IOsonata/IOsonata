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

// I2C1 (DevNo 0) master on D1 PA9 SCL and D0 PA10 SDA, AF4. I2C3 (DevNo 2)
// slave on A6 PA7 SCL and D12 PB4 SDA, AF4. Wires: D1 to A6 and D0 to D12,
// the internal pull-ups of both ends hold the bus. The SPIMasterSlave
// example uses the same two lines: A6 PA7 with D11 PB5 and D12 PB4 with D10
// PA11, so a breadboard row can hold D1, A6 and D11, another D0, D12 and D10.
#define I2C_MASTER_DEVNO			0
#define I2C_MASTER_SDA_PORT			IOPORTA
#define I2C_MASTER_SDA_PIN			10
#define I2C_MASTER_SDA_PINOP		IOPINOP_FUNC4
#define I2C_MASTER_SCL_PORT			IOPORTA
#define I2C_MASTER_SCL_PIN			9
#define I2C_MASTER_SCL_PINOP		IOPINOP_FUNC4

#define I2C_SLAVE_DEVNO				2
#define I2C_SLAVE_SDA_PORT			IOPORTB
#define I2C_SLAVE_SDA_PIN			4
#define I2C_SLAVE_SDA_PINOP			IOPINOP_FUNC4
#define I2C_SLAVE_SCL_PORT			IOPORTA
#define I2C_SLAVE_SCL_PIN			7
#define I2C_SLAVE_SCL_PINOP			IOPINOP_FUNC4

// The L4 I2C has no DMA yet: master polling and interrupt, slave interrupt
#define I2C_TEST_MASTER_MASK		0x03
#define I2C_TEST_SLAVE_MASK			0x01

#endif
