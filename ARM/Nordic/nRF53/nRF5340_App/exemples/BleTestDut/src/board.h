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

// Default is the nRF5340 Audio DK, the dev board of the nRF5340 work.
// Define NRF5340_DK_PCA10095 in the project symbols for the standard
// nRF5340 DK. Pins verified against the Zephyr board files of both boards.
#define NORDIC_DK

// Console UARTE0 on the virtual COM port of the board. The HCI link to the
// network core is the IPC rings, see bt_hci_ctlr_ipc.cpp; nothing here
// configures it.
#define UART_DEVNO			0

// The timer of the examples that update their data on a timer
#define TIMER_DEVNO			1

#ifdef NORDIC_DK
#if defined(NRF5340_DK_PCA10095)

// nRF5340 DK LED1 to LED4, active low
#define LED1_PORT		0
#define LED1_PIN		28
#define LED1_PINOP		0

#define LED2_PORT		0
#define LED2_PIN		29
#define LED2_PINOP		0

#define LED3_PORT		0
#define LED3_PIN		30
#define LED3_PINOP		0

#define LED4_PORT		0
#define LED4_PIN		31
#define LED4_PINOP		0

#define LED_ACTIVE_LOW	1

// nRF5340 DK buttons 1 and 2, active low
#define BUT1_PORT		0
#define BUT1_PIN		23
#define BUT1_PINOP		0
#define BUT2_PORT		0
#define BUT2_PIN		24
#define BUT2_PINOP		0

// Connection LED of the Bluetooth examples, LED4, active low
#define CONNECT_LED_PORT	LED4_PORT
#define CONNECT_LED_PIN		LED4_PIN
#define CONNECT_LED_LOGIC	0

// DK virtual COM port: P0.20 TXD, P0.22 RXD, P0.19 RTS, P0.21 CTS
#define UART_RX_PORT		0
#define UART_RX_PIN			22
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			20
#define UART_TX_PINOP		1
#define UART_CTS_PORT		0
#define UART_CTS_PIN		21
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		0
#define UART_RTS_PIN		19
#define UART_RTS_PINOP		1

#else

// nRF5340 Audio DK: LED1 (blue) P0.31, LED2 (green) P1.00, LED3 (green)
// P1.01, and RGB2 green P0.29 as the fourth; all active high
#define LED1_PORT		0
#define LED1_PIN		31
#define LED1_PINOP		0

#define LED2_PORT		1
#define LED2_PIN		0
#define LED2_PINOP		0

#define LED3_PORT		1
#define LED3_PIN		1
#define LED3_PINOP		0

#define LED4_PORT		0
#define LED4_PIN		29
#define LED4_PINOP		0

#define LED_ACTIVE_LOW	0

// Audio DK buttons: VOL- P0.02, VOL+ P0.03, active low
#define BUT1_PORT		0
#define BUT1_PIN		2
#define BUT1_PINOP		0
#define BUT2_PORT		0
#define BUT2_PIN		3
#define BUT2_PINOP		0

// Connection LED of the Bluetooth examples, RGB2 green, active high
#define CONNECT_LED_PORT	LED4_PORT
#define CONNECT_LED_PIN		LED4_PIN
#define CONNECT_LED_LOGIC	1

// Audio DK virtual COM port of the application core: P1.05 TXD,
// P1.04 RXD, P1.07 RTS, P1.06 CTS. The network core's own port is
// P1.09 TXD, P1.08 RXD.
#define UART_RX_PORT		1
#define UART_RX_PIN			4
#define UART_RX_PINOP		1
#define UART_TX_PORT		1
#define UART_TX_PIN			5
#define UART_TX_PINOP		1
#define UART_CTS_PORT		1
#define UART_CTS_PIN		6
#define UART_CTS_PINOP		1
#define UART_RTS_PORT		1
#define UART_RTS_PIN		7
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

#define BUT1_INT		0
#define BUT1_INT_PRIO	6
#define BUT2_INT		1
#define BUT2_INT_PRIO	6

#define BUTTON_PINS		{ \
	{BUT1_PORT, BUT1_PIN, BUT1_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{BUT2_PORT, BUT2_PIN, BUT2_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#define UART_PINS		{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define LED_PINS	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define LED_PINS_MAP	{ \
	{LED1_PORT, LED1_PIN, LED1_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED2_PORT, LED2_PIN, LED2_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED3_PORT, LED3_PIN, LED3_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{LED4_PORT, LED4_PIN, LED4_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// Periodic advertising with responses sets the HciController network core
// build reserves for its host. Zero asks for none and the DUT answers the
// pawr commands with a refusal.
#define DUT_PAWR_ADV_COUNT		0

#endif // __BOARD_H__

