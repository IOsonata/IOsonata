/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro I2C slave demo

I2C slave at address 0x22 on the EXT1 I2C pins (11 SDA, 12 SCL), with the
I2CMasterDemo on a second board: wire EXT1 11 to 11, 12 to 12 and GND, with
4.7 kohm pull-ups to VCC. Messages on the EDBG COM port at 1000000 baud
in Release, by semihosting in Debug (SAM D21).

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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

#include "coredev/iopincfg.h"

// EDBG virtual COM port on SERCOM3, function C: RX PA23 pad 1, TX PA22 pad 0.
// Configure RX before TX in the pin map.
#define UART_DEVNO			3
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			23
#define UART_RX_PINOP		IOPINOP_PERIPHC
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			22
#define UART_TX_PINOP		IOPINOP_PERIPHC
// I2C slave on SERCOM2, function D: SDA PA08 pad 0, SCL PA09 pad 1
#define I2C_SLAVE_DEVNO		2
#define I2C_SLAVE_SDA_PORT	IOPORTA
#define I2C_SLAVE_SDA_PIN		8
#define I2C_SLAVE_SDA_PINOP	IOPINOP_PERIPHD
#define I2C_SLAVE_SCL_PORT	IOPORTA
#define I2C_SLAVE_SCL_PIN		9
#define I2C_SLAVE_SCL_PINOP	IOPINOP_PERIPHD
// No DMA path on SAM D
#define I2C_SLAVE_DMA_ENABLE	false

#endif // __BOARD_H__
