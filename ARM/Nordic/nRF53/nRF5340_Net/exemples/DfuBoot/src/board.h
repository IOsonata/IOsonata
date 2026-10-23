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

#define UART_DEVNO			0

#ifdef NORDIC_DK
// Default is the nRF5340 Audio DK, the dev board of the nRF5340 work.
// Define NRF5340_DK_PCA10095 in the project symbols for the standard
// nRF5340 DK. Pins from the Zephyr board files of both boards.
// The application core must hand these pins to the network core (GPIO
// MCUSEL) before it releases it.
#if defined(NRF5340_DK_PCA10095)
// nRF5340 DK: LED1 to LED4 P0.28 to P0.31 active low, buttons 1 and 2
// P0.23, P0.24; DK network core virtual COM port P1.01 TXD, P1.00 RXD, P1.11 RTS, P1.10 CTS
#define LED_ACTIVE_LOW	1
#define UART_RX_PORT		1
#define UART_RX_PIN			0
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			1
#define UART_TX_PINOP		1
#define UART_CTS_PORT		1
#define UART_CTS_PIN		10
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		1
#define UART_RTS_PIN		11
#define UART_RTS_PINOP		1
#else
// nRF5340 Audio DK: LED1 (blue) P0.31, LED2 P1.00, LED3 P1.01, RGB2 green
// P0.29 as LED4, active high; buttons VOL- P0.02, VOL+ P0.03; network core virtual COM port P1.09 TXD, P1.08 RXD, P1.10 RTS, P1.11 CTS (the second port the board enumerates)
#define LED_ACTIVE_LOW	0
#define UART_RX_PORT		1
#define UART_RX_PIN			8
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			9
#define UART_TX_PINOP		1
#define UART_CTS_PORT		1
#define UART_CTS_PIN		11
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		1
#define UART_RTS_PIN		10
#define UART_RTS_PINOP		1
#endif
#else
#define UART_RX_PORT		BLUEIO_UART_RX_PORT
#define UART_RX_PIN			BLUEIO_UART_RX_PIN
#define UART_RX_PINOP		BLUEIO_UART_RX_PINOP
#define UART_TX_PORT		BLUEIO_UART_TX_PORT
#define UART_TX_PIN			BLUEIO_UART_TX_PIN
#define UART_TX_PINOP		BLUEIO_UART_TX_PINOP
#define UART_CTS_PORT		BLUEIO_UART_CTS_PORT
#define UART_CTS_PIN		BLUEIO_UART_CTS_PIN
#define UART_CTS_PINOP		BLUEIO_UART_CTS_PINOP
#define UART_RTS_PORT		BLUEIO_UART_RTS_PORT
#define UART_RTS_PIN		BLUEIO_UART_RTS_PIN
#define UART_RTS_PINOP		BLUEIO_UART_RTS_PINOP
#endif

#define UART_PINS			{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif
