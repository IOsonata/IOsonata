/**-------------------------------------------------------------------------
@example	board.h

@brief	Board definitions of the STM32WBA65 I2C master and slave example

		Two I2C controllers of the same chip on the Arduino header of the
		STM32WBA65I-DK1, AF4. Master I2C1 on D14 (SDA, PB1) and D15 (SCL, PB2).
		Slave I2C2 on D5 (SDA, PB14) and D3 (SCL, PB13).
		Wire D14 to D5 and D15 to D3. The pins use the internal pull-ups, which
		is enough for 100 kHz on short wires. Add 4.7k pull-ups to 3.3V for
		longer ones. The result is printed on the first ST-LINK virtual COM
		port, USART1.

		Not tested on hardware.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2026

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

#define I2C_MASTER_DEVNO			BOARD_I2C_DEVNO
#define I2C_MASTER_SDA_PORT			IOPORTB
#define I2C_MASTER_SDA_PIN			1
#define I2C_MASTER_SDA_PINOP		IOPINOP_FUNC4
#define I2C_MASTER_SCL_PORT			IOPORTB
#define I2C_MASTER_SCL_PIN			2
#define I2C_MASTER_SCL_PINOP		IOPINOP_FUNC4

#define I2C_SLAVE_DEVNO				1
#define I2C_SLAVE_SDA_PORT			IOPORTB
#define I2C_SLAVE_SDA_PIN			14
#define I2C_SLAVE_SDA_PINOP			IOPINOP_FUNC4
#define I2C_SLAVE_SCL_PORT			IOPORTB
#define I2C_SLAVE_SCL_PIN			13
#define I2C_SLAVE_SCL_PINOP			IOPINOP_FUNC4

// Start with the polling master and the interrupt slave. Set the master
// INT and DMA flags, then the slave DMA flag, to run the other modes.
#define I2C_MASTER_DMA_ENABLE		false
#define I2C_MASTER_INT_ENABLE		false
#define I2C_SLAVE_DMA_ENABLE		false
#define I2C_SLAVE_INT_ENABLE		true

#endif
