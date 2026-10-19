/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

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

// I2C2 (DevNo 1), the onboard sensor bus: SDA PB11, SCL PB10, AF4. The board
// has the bus pull-up resistors. Devices on it include the HTS221, LPS22HB,
// LSM6DSL, LIS3MDL, VL53L0X and the NFC tag.
#define I2C_DEVNO			1
#define I2C_PINS			STM32_BOARD_I2C(IOPORTB, 11, IOPINOP_FUNC4, IOPORTB, 10, IOPINOP_FUNC4)

// After the scan, read the LSM6DSL WHO_AM_I register, 0x6A expected
#define I2C_SCAN_REG_DEVADDR	0x6A
#define I2C_SCAN_REG			0x0F

#endif // __BOARD_H__
