/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro I2C bus scan

Probes the 7-bit addresses 0x08 to 0x77 on the EXT1 I2C pins (11 SDA,
12 SCL) and lists the devices that answer. Results by semihosting in Debug
(SAM D21), on the EDBG COM port at 1000000 baud in Release.

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
#ifndef DEBUG
#define UART_RETARGET
#endif
// I2C master on SERCOM2, function D: SDA PA08 pad 0, SCL PA09 pad 1
#define I2C_DEVNO			2
#define I2C_PINS			{ \
	{IOPORTA, 8, IOPINOP_PERIPHD, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
	{IOPORTA, 9, IOPINOP_PERIPHD, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN}, \
}
// The SAMD StartTx only selects the address, START goes out with the first
// data byte. A one-byte read probes the address.
#define I2C_SCAN_READ_PROBE	true
// Optional register read after the scan: define I2C_SCAN_REG_DEVADDR (7-bit
// address) and I2C_SCAN_REG (register) for the device.

#endif // __BOARD_H__
