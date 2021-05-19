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

#include "stm32l4xx.h"

#define UART_NO				1
#define UART_RX_PORT		IOPORTD
#define UART_RX_PIN			6
#define UART_RX_PINOP		(0x72)

#define UART_TX_PORT		IOPORTD
#define UART_TX_PIN			5
#define UART_TX_PINOP		(0x72)

#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		0

#define I2C_DEVNO	1

#define I2C_SCL_PORT		IOPORTF
#define I2C_SCL_PIN			1
#define I2C_SCL_PINOP		0x42		// AF4
#define I2C_SDA_PORT		IOPORTF
#define I2C_SDA_PIN			0
#define I2C_SDA_PINOP		0x42		// AF4

#define I2C_PINS_MAP	{ \
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL},\
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},}


#endif // __BOARD_H__

