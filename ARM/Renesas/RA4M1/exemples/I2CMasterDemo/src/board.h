/**-------------------------------------------------------------------------
@file	board.h

@brief	RA4M1 I2C master example configuration.

		SCI0 is kept for the shared example's console. IIC0 uses P401/P400
		with PSEL 7. External I2C pull-ups are required for a production bus.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

#include "coredev/iopincfg.h"

#define UART_DEVNO				0
#define UART_RX_PORT			1
#define UART_RX_PIN				0
#define UART_RX_PINOP			IOPINOP_FUNC3
#define UART_TX_PORT			1
#define UART_TX_PIN				1
#define UART_TX_PINOP			IOPINOP_FUNC3

#define UART_PINS				{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define I2C_MASTER_DEVNO		0
#define I2C_MASTER_SDA_PORT		4
#define I2C_MASTER_SDA_PIN		1
#define I2C_MASTER_SDA_PINOP	IOPINOP_FUNC6
#define I2C_MASTER_SCL_PORT		4
#define I2C_MASTER_SCL_PIN		0
#define I2C_MASTER_SCL_PINOP	IOPINOP_FUNC6

#define I2C_MASTER_DMA_ENABLE	false
#define I2C_MASTER_INT_ENABLE	false

#endif // __BOARD_H__
