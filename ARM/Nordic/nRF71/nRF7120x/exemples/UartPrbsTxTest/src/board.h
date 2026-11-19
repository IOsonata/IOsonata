/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	Nov. 17, 2026

@license

Copyright (c) 2026, I-SYST inc., all rights reserved

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

// nRF7120 DK

// UART device number
// 0 : UARTE30, 1 : UARTE20, 2 : UARTE21, 3 : UARTE22, 4 : UARTE00,
// 5 : UARTE23, 6 : UARTE24
#define UART_DEVNO			1

#if UART_DEVNO == 1

// UARTE20, first virtual COM port of the DK
#define UART_RX_PORT		1
#define UART_RX_PIN			1
#define UART_RX_PINOP		1

#define UART_TX_PORT		1
#define UART_TX_PIN			0
#define UART_TX_PINOP		1

#define UART_CTS_PORT		1
#define UART_CTS_PIN		3
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		1
#define UART_RTS_PIN		2
#define UART_RTS_PINOP		1

#elif UART_DEVNO == 2

// UARTE21, second virtual COM port of the DK
#define UART_RX_PORT		1
#define UART_RX_PIN			9
#define UART_RX_PINOP		1

#define UART_TX_PORT		1
#define UART_TX_PIN			8
#define UART_TX_PINOP		1

#define UART_CTS_PORT		1
#define UART_CTS_PIN		11
#define UART_CTS_PINOP		1

#define UART_RTS_PORT		1
#define UART_RTS_PIN		10
#define UART_RTS_PINOP		1

#else
#error "No pin assignment for this UART_DEVNO"
#endif

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif // __BOARD_H__
