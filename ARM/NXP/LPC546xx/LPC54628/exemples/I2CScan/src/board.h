/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPCXpresso54628

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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

// Output on the LPC-Link2 VCOM, Flexcomm 0, P0_29 RXD and P0_30 TXD
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			29
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			30
#define UART_TX_PINOP		IOPINOP_FUNC1

// On board I2C bus, Flexcomm 2, P3_23 SDA and P3_24 SCL on function 1. These
// are open drain I2C pins with pull-ups on the board. The audio codec and
// the accelerometer are on this bus.
#define I2C_DEVNO			2
#define I2C_PINS			{ \
	{3, 23, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
	{3, 24, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
}

// MMA8652FC accelerometer WHO_AM_I register, reads 0x4A
#define I2C_SCAN_REG_DEVADDR	0x1D
#define I2C_SCAN_REG			0x0D

#endif
