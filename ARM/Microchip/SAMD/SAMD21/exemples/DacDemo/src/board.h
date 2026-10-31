/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D21 Xplained Pro DAC demo

This file contains the I/O definitions for this application firmware.
Keep it in the application project and modify it for the hardware in use.
The pins are those of the SAM D21 Xplained Pro. The DAC output is on PA02
(EXT3 pin 3), the voltages are printed on the EDBG COM port.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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

// EDBG virtual COM port on SERCOM3, function C: RX PA23 pad 1, TX PA22 pad 0,
// 1000000 baud 8N1. The Release build prints there, the Debug build uses
// semihosting.
#define UART_RETARGET
#define UART_DEVNO			3
#define UART_RX_PORT		IOPORTA
#define UART_RX_PIN			23
#define UART_RX_PINOP		IOPINOP_PERIPHC
#define UART_TX_PORT		IOPORTA
#define UART_TX_PIN			22
#define UART_TX_PINOP		IOPINOP_PERIPHC

// VDDANA, reference of the DAC and of the ADC, 3.3 V on the board
#define DAC_REF_VOLTAGE		3.3f

#endif // __BOARD_H__
