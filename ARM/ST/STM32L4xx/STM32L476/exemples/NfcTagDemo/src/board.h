/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Oct. 21, 2026

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

// Board, one of the names in stm32_boards.h. This STM32L476 project builds for
// B_L475E_IOT01A (B-L475E-IOT01A and B-L475E-IOT01A2, STM32L475VG).
#define BOARD				B_L475E_IOT01A

#include "stm32_boards.h"

// Output on USART1, the ST-LINK virtual COM port, 115200 8N1. The Release
// build prints there, the Debug build uses semihosting.
#define UART_RETARGET
#define UART_DEVNO			BOARD_VCP_DEVNO
#define UART_RX_PORT		IOPORTB
#define UART_RX_PIN			7
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			6
#define UART_TX_PINOP		IOPINOP_FUNC7

// I2C2 (DevNo 1), the onboard sensor and NFC tag bus: SDA PB11, SCL PB10, AF4.
// The board has the bus pull-up resistors.
#define I2C_DEVNO			1
#define I2C_PINS			STM32_BOARD_I2C(IOPORTB, 11, IOPINOP_FUNC4, IOPORTB, 10, IOPINOP_FUNC4)

// Onboard M24SR64-Y dynamic NFC tag, its RF disable pin on PE2, high turns the
// RF side off. The GPO output of the tag is on PE4, the demo does not use it.
#define M24SR
#define NFC_RFDIS_PORT		IOPORTE
#define NFC_RFDIS_PIN		2
#define NFC_RFDIS_PINOP		IOPINOP_GPIO

#endif // __BOARD_H__
