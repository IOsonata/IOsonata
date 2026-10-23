/**-------------------------------------------------------------------------
@file	board.h

@brief	Board definitions of the IpcIntrf test, nRF5340 DK network core (owner)

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

// This image is the owner: it formats the block and echoes
#define IPCTEST_OWNER			1

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

// Trace UART of the owner, on the network core's own virtual COM port.
// Default is the nRF5340 Audio DK: P1.09 TXD, P1.08 RXD, the second port
// the board enumerates. Define NRF5340_DK_PCA10095 for the nRF5340 DK:
// P1.01 TXD, P1.00 RXD. The application core hands these pins to the
// network core (GPIO MCUSEL) before it releases it, see the peer's board.h.
#define UART_DEVNO		0
#if defined(NRF5340_DK_PCA10095)
#define UART_RX_PORT	1
#define UART_RX_PIN		0
#define UART_TX_PORT	1
#define UART_TX_PIN		1
#else
#define UART_RX_PORT	1
#define UART_RX_PIN		8
#define UART_TX_PORT	1
#define UART_TX_PIN		9
#endif
#define UART_RX_PINOP	0
#define UART_TX_PINOP	0
#define UART_BAUDRATE	1000000

#define UART_PINS	{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},\
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},}

// 1 packet slots, 0 byte stream
#define IPCTEST_PACKET			1
// Messages in flight. 1 is lock step, larger values fill the rings.
#define IPCTEST_WINDOW			6U

#endif // __BOARD_H__
