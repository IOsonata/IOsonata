/**-------------------------------------------------------------------------
@file	board.h

@brief	I2C bus scan on the FPB-R9A02G021

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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

// The IICA needs PCLKB at 32 MHz or lower (notes of table 8.2), the HOCO is
// set to 32 MHz. SystemInit runs PCLKB at the core clock.
#define MCUOSC	{ {OSC_TYPE_RC, 32000000, 0, 0}, {OSC_TYPE_RC, 32768, 0, 0}, false }

// SAU UART0 on the on-board J-Link OB virtual COM port, R31 and R33 fitted
// and E56 closed (board defaults):
// TxD0_B P302, RxD0_B P303. PSEL 0x11 (SAU) is IOPINOP_FUNC16.
// Configure RX before TX in the pin map.
#define UART_DEVNO			0
#define UART_RX_PORT		3
#define UART_RX_PIN			3
#define UART_RX_PINOP		IOPINOP_FUNC16

#define UART_TX_PORT		3
#define UART_TX_PIN			2
#define UART_TX_PINOP		IOPINOP_FUNC16

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define UART_RETARGET

// IICA1: SDAA1 P010, SCLA1 P011, PSEL 0x07 (IICA) is IOPINOP_FUNC6. Both are
// on the Arduino header. The bus needs external pull-ups, the internal ones
// are too weak above a few devices or 100 kHz.
#define I2C_DEVNO			1
#define I2C_PINS			{ \
	{0, 10, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
	{0, 11, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
}

// The simplified I2C of the SAU works the same way, for example IIC20,
// I2C_DEVNO 6: SDA20 P102 as {1, 2, IOPINOP_FUNC16, IOPINDIR_BI,
// IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, SCL20 P101 as {1, 1, IOPINOP_FUNC16,
// IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}. PSEL 0x11 (SAU) is
// IOPINOP_FUNC16. Its slaves must not stretch SCL during a byte.

// true for the data of the register read in the I2C interrupt
#define I2C_SCAN_INT_ENABLE	false

#endif
