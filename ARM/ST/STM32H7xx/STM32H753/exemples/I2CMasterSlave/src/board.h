/**-------------------------------------------------------------------------
@file	board.h

@brief	NUCLEO-H753ZI board definitions for I2CMasterSlave.

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

// Both selected instances have DMA hardware; exercise all eight combinations.
#define I2C_MASTER_HAS_DMA 1
#define I2C_SLAVE_HAS_DMA 1
#define I2C_MASTER_DEVNO 0
#define I2C_SLAVE_DEVNO 1
#define I2C_MASTER_INT_PRIO 7
#define I2C_SLAVE_INT_PRIO 6

#define I2C_MASTER_SCL_PORT IOPORTB
#define I2C_MASTER_SCL_PIN 8
#define I2C_MASTER_SCL_PINOP IOPINOP_FUNC4

#define I2C_MASTER_SDA_PORT IOPORTB
#define I2C_MASTER_SDA_PIN 9
#define I2C_MASTER_SDA_PINOP IOPINOP_FUNC4

#define I2C_SLAVE_SCL_PORT IOPORTF
#define I2C_SLAVE_SCL_PIN 1
#define I2C_SLAVE_SCL_PINOP IOPINOP_FUNC4

#define I2C_SLAVE_SDA_PORT IOPORTF
#define I2C_SLAVE_SDA_PIN 0
#define I2C_SLAVE_SDA_PINOP IOPINOP_FUNC4

#endif
