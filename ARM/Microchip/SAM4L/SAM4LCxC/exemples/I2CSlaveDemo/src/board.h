/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM4LCxC I2C slave demo configuration

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
#include "coredev/system_core_clock.h"

#define MCUOSC { \
	{ OSC_TYPE_XTAL, 12000000, 20, 180 }, \
	{ OSC_TYPE_XTAL, 32768, 20, 125 }, true }

#define UART_DEVNO			1
#define UART_RX_PORT		IOPORTC
#define UART_RX_PIN			26
#define UART_RX_PINOP		IOPINOP_PERIPHA
#define UART_TX_PORT		IOPORTC
#define UART_TX_PIN			27
#define UART_TX_PINOP		IOPINOP_PERIPHA


// SAM4L8 Xplained Pro EXT1 TWI: TWIS0 on PA23/PA24 peripheral B.
#define I2C_SLAVE_DEVNO		0
#define I2C_SLAVE_SDA_PORT	IOPORTA
#define I2C_SLAVE_SDA_PIN	23
#define I2C_SLAVE_SDA_PINOP	IOPINOP_PERIPHB
#define I2C_SLAVE_SCL_PORT	IOPORTA
#define I2C_SLAVE_SCL_PIN	24
#define I2C_SLAVE_SCL_PINOP	IOPINOP_PERIPHB
#define I2C_SLAVE_DMA_ENABLE	false

// Set to 1 for the paired SAM4L WAIT address-match wake test.
// Setup and expected results: exemples/i2c/sleepwalking.md.
#ifndef I2C_SLEEP_TEST
#define I2C_SLEEP_TEST 0
#endif

#endif
