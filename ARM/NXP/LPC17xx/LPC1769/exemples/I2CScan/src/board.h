/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPC1769 I2CScan

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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

// LPC1769 MCU pin assignments; locate these signals on your board.

#define I2C_DEVNO			0
#ifndef I2C_SCAN_RATE
#define I2C_SCAN_RATE		100000
#endif

// I2C0: SDA P0.27, SCL P0.28.
// Dedicated open-drain I2C pins; external pull-ups are required.
// The generic I2C pin order is SDA, then SCL.
#define I2C_PINS { \
	{0, 27, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
	{0, 28, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
}

// Optional register read after scanning: define both I2C_SCAN_REG_DEVADDR
// (7-bit address) and I2C_SCAN_REG (8-bit register address) for your device.

// Same UART pins as TimerDemo; output at 115200 baud, no flow control.
// UART0: RX P0.3, TX P0.2.
#ifndef DEBUG
#define UART_RETARGET
#endif

#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			3
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			2
#define UART_TX_PINOP		IOPINOP_FUNC1

#endif
