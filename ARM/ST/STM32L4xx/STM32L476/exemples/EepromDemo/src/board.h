/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	July 26, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

// The board has no serial EEPROM: connect an I2C EEPROM to the Arduino
// header, I2C1 (DevNo 0), AF4, SDA D14 PB9 and SCL D15 PB8, open drain. The
// eeprom_demo.cpp defaults are a 24xx32 class part at 0x50: 2 address bytes,
// 32 byte pages, 4 KB. Fit pull-up resistors to 3.3 V if the EEPROM module
// has none.
#define I2C_DEVNO			0
#define I2C_PINS			STM32_BOARD_I2C(IOPORTB, 9, IOPINOP_FUNC4, IOPORTB, 8, IOPINOP_FUNC4)

// The Debug build prints with semihosting. With EEPROM_DEMO_UART 1, a Release
// build prints on USART1, the ST-LINK virtual COM port, at 1 Mbaud.
#define EEPROM_UART_DEVNO	BOARD_VCP_DEVNO
#define EEPROM_UART_PINS	BOARD_VCP_PINS

#endif // __BOARD_H__
