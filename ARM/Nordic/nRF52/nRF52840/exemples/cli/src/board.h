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

//#define NORDIC_DK
//#define NEBLINA

#define UART_DEVNO				0

#ifdef NORDIC_DK
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			6
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		7
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		5
#define UART_RTS_PINOP		1
#elif defined(NEBLINA)
#define UART_RX_PORT		0
#define UART_RX_PIN			6
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			7
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		29
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		28
#define UART_RTS_PINOP		1
#else
#define UART_RX_PORT		0
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			9
#define UART_TX_PINOP		1
#endif

#define UART_NO		0

#define UART_PINS	{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
}

#endif // __BOARD_H__

