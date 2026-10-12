/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions for the LPC11U35 I2CScan

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

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

// IBK-IDAP-M revision 0, schematic 03-2021-04 (2021-04-25).

#define I2C_DEVNO			0
#ifndef I2C_SCAN_RATE
#define I2C_SCAN_RATE		100000
#endif

// P2.11 = SDA (P0.5), P2.10 = SCL (P0.4).
// Dedicated open-drain I2C pins; external pull-ups are required.
// The generic I2C pin order is SDA, then SCL.
#define I2C_PINS { \
	{0, 5, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
	{0, 4, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
}

// Optional register read after scanning: define both I2C_SCAN_REG_DEVADDR
// (7-bit address) and I2C_SCAN_REG (8-bit register address) for your device.

// Same UART pins as UartPrbsTxTest; output at 115200 baud, no flow control.
// UART0: RX P2.3 (P0.18), TX P2.4 (P0.19).
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			18
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			19
#define UART_TX_PINOP		IOPINOP_FUNC1

#endif
