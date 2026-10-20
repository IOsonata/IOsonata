/**-------------------------------------------------------------------------
@file	board.h

@brief	NUCLEO-H753ZI Ethernet example board configuration.

@author	Hoang Nguyen Hoan
@date	Oct. 20, 2026

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
#define BOARD NUCLEO_H753ZI
#include "stm32_boards.h"
#define UART_DEVNO BOARD_VCP_DEVNO
#define UART_PINS BOARD_VCP_PINS
#define ETHERNET_TIMER_DEVNO 8
#define ETHERNET_PHY_ADDR 0
// Locally administered demonstration address; assign a distinct MAC per board.
#define ETHERNET_MAC_ADDR {0x02,0x00,0x00,0x75,0x30,0x01}
// MB1364 H753ZI: RMII AF11. TXD1 uses PB13, not PG14.
#define ETHERNET_PINS { \
	{IOPORTA, 1, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 2, IOPINOP_FUNC11, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 1, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 7, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 4, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 5, IOPINOP_FUNC11, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTG, 11, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTG, 13, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTB, 13, IOPINOP_FUNC11, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}
#define NET_DEMO_HOST "192.168.1.10"
#define NET_DEMO_PROTO SOCKINTRF_PROTO_TCP

#endif
