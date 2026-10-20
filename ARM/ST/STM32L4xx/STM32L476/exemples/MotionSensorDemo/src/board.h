/**-------------------------------------------------------------------------
@example	board.h

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

// The demo reads an ICM456x connected to the Arduino header. The Debug build
// prints with semihosting.
//
// I2C1 (DevNo 0, the I2C instance the demo uses) on the Arduino header,
// AF4: SDA D14 PB9, SCL D15 PB8. ICM456x AP_AD0 low selects address 0x68.
#define I2C_PINS			STM32_BOARD_I2C(IOPORTB, 9, IOPINOP_FUNC4, IOPORTB, 8, IOPINOP_FUNC4)
#define ACC_DEV_ADDR		ICM456X_I2C_7BITS_DEVADDR0

// ICM456x INT1 on Arduino D2 PD14
#define IMU_INT_PORT		IOPORTD
#define IMU_INT_PIN			14
#define IMU_INT_PINOP		IOPINOP_GPIO

#endif // __BOARD_H__
