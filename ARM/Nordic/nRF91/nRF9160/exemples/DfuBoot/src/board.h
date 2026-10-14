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

// Board selection, define one:
//	NRF9160_DK		nRF9160 DK, UART0 on the interface MCU VCOM0
//	NORDIC_THINGY91	Nordic Thingy:91, UART0 on the first USB serial port of
//					its nRF52840 running the Connectivity Bridge firmware
#define NRF9160_DK
//#define NORDIC_THINGY91

// UART0, the DFU transport
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PINOP		1

#if defined(NORDIC_THINGY91)
// P0.19, P0.18, P0.21 and P0.20, to UART0 of the nRF52840
#define UART_RX_PIN			19
#define UART_TX_PIN			18
#define UART_CTS_PIN		21
#define UART_RTS_PIN		20
#elif defined(NRF9160_DK)
#define UART_RX_PIN			28
#define UART_TX_PIN			29
#define UART_CTS_PIN		26
#define UART_RTS_PIN		27
#else
#error "Select the board: NRF9160_DK or NORDIC_THINGY91"
#endif

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif
