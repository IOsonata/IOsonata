/**-------------------------------------------------------------------------
@file	board.h

@brief	Thingy:52 production PCA20020 pin map.

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

/** Thingy:52 production PCA20020 pin map. SPDX-License-Identifier: MIT */
#ifndef __BOARD_H__
#define __BOARD_H__
#include "thingy52_board.h"

// External UART adapter: P4.3 / ANA_DIG0 -> adapter RX; P4.4 -> adapter TX.
#define UART_DEVNO 0
#define UART_TX_PORT 0
#define UART_TX_PIN 2
#define UART_TX_PINOP 1
#define UART_RX_PORT 0
#define UART_RX_PIN 3
#define UART_RX_PINOP 1

#define I2C_DEVNO THINGY52_I2C_DEVNO
#define I2C_SDA_PORT THINGY52_I2C_PORT
#define I2C_SDA_PIN THINGY52_I2C_SDA_PIN
#define I2C_SDA_PINOP 1
#define I2C_SCL_PORT THINGY52_I2C_PORT
#define I2C_SCL_PIN THINGY52_I2C_SCL_PIN
#define I2C_SCL_PINOP 1
#endif
