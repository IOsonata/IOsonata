/**-------------------------------------------------------------------------
@file	board.h

@brief	Board definitions of the IpcIntrf test, nRF5340 Audio DK or DK application core (peer)

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#ifndef __BOARD_H__
#define __BOARD_H__

// This image is the peer: it starts the network core and checks the echo
#define IPCTEST_OWNER			0

// Shared memory block of the IPC link. Same values on both cores.
// The block is the top of the application core RAM, the IPC0 region of
// nrf5340_xxaa_application.ld, which keeps it out of the RAM the image uses.
#define IPCTEST_MEM_ADDR		0x20070000UL
#define IPCTEST_MEM_SIZE		0x2000U

// IPC channels. The owner's Tx channel is the peer's Rx channel.
#define IPCTEST_OWNER_TX_CHAN	0
#define IPCTEST_OWNER_RX_CHAN	1
#define IPCTEST_PEER_TX_CHAN	1
#define IPCTEST_PEER_RX_CHAN	0

// 1 packet slots, 0 byte stream
#define IPCTEST_PACKET			1
// Messages in flight. 1 is lock step, larger values fill the rings.
#define IPCTEST_WINDOW			6U

// Default is the nRF5340 Audio DK, the dev board of the nRF5340 work.
// Define NRF5340_DK_PCA10095 in the project symbols for the standard
// nRF5340 DK. Pins verified against the Zephyr board files of both boards.
#if defined(NRF5340_DK_PCA10095)

// LEDs of the nRF5340 DK, active low
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

// UART of the DK virtual COM port: P0.20 TXD, P0.22 RXD. No flow control.
#define UART_DEVNO		0
#define UART_RX_PORT	0
#define UART_RX_PIN		22
#define UART_RX_PINOP	0
#define UART_TX_PORT	0
#define UART_TX_PIN		20
#define UART_TX_PINOP	0

#else

// LEDs of the nRF5340 Audio DK, active high: LED1 (blue) P0.31, LED2
// (green) P1.00, LED3 (green) P1.01, RGB2 green P0.29 as the fourth
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

// UART of the Audio DK virtual COM port of the application core:
// P1.05 TXD, P1.04 RXD. No flow control.
#define UART_DEVNO		0
#define UART_RX_PORT	1
#define UART_RX_PIN		4
#define UART_RX_PINOP	0
#define UART_TX_PORT	1
#define UART_TX_PIN		5
#define UART_TX_PINOP	0

#endif

#define UART_BAUDRATE	1000000

// Flash of the network core, readable from the application core; the
// peer checks that an image is there before it waits for the link.
#define IPCTEST_NET_FLASH	0x01000000UL

// GPIOs the network core image uses, its trace UART on the network core's
// own virtual COM port. The application core owns every pin after reset
// and hands these to the network core (PIN_CNF MCUSEL) before releasing it.
#if defined(NRF5340_DK_PCA10095)
#define IPCTEST_NET_PINS	{ {1, 0}, {1, 1} }
#else
#define IPCTEST_NET_PINS	{ {1, 8}, {1, 9} }
#endif

#define UART_PINS	{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

#endif // __BOARD_H__
