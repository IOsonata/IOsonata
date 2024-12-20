/**-------------------------------------------------------------------------
@file	board.h

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

#include "blueio_board.h"

#define NORDIC_DK

#ifdef NORDIC_DK

#define LED1_PORT		0
#define LED1_PIN		2
#define LED1_PINOP		0

#define LED2_PORT		0
#define LED2_PIN		3
#define LED2_PINOP		0

#define LED3_PORT		0
#define LED3_PIN		4
#define LED3_PINOP		0

#define LED4_PORT		0
#define LED4_PIN		5
#define LED4_PINOP		0

#define UART_DEVNO			0

#define UART_RX_PORT		0
#define UART_RX_PIN			1
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			0
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		26
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		27
#define UART_RTS_PINOP		1

#else

#define LED1_PORT		2
#define LED1_PIN		1
#define LED1_PINOP		0

#define LED2_PORT		2
#define LED2_PIN		2
#define LED2_PINOP		0

#define LED3_PORT		2
#define LED3_PIN		3
#define LED3_PINOP		0

#define UART_DEVNO			0

#define UART_RX_PORT		0
#define UART_RX_PIN			0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			1
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		2
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		4
#define UART_RTS_PINOP		1
#endif

#define LED_PINS_MAP	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}
	//{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	//{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}


#define TIMER_DEVNO			2

#endif // __BOARD_H__

