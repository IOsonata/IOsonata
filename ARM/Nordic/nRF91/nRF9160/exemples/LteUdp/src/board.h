/**-------------------------------------------------------------------------
@example	board.h

@brief	Board specific definitions, LTE UDP on the nRF9160 DK

The console is the nRF9160 UART0 on the interface MCU VCOM port.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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

#include "coredev/iopincfg.h"

// UDP echo server, see lte_udp.cpp. No release assistance: the echo comes
// back after the send, and modem firmware 1.2.x has no AT%RAI.
#define LTE_UDP_HOST		"echo.u-blox.com"
#define LTE_UDP_PORT		7
#define LTE_UDP_RAI			0

// LTE-M only: modem firmware 1.2.x runs one of LTE-M and NB-IoT at a time,
// both at once needs 1.3.0 or later
#define LTE_UDP_RAT			LTE_RAT_LTEM
#define LTE_UDP_RAT_PREF	LTE_RAT_NONE

// Console, UART0 to the interface MCU (VCOM0)
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			28
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PIN			29
#define UART_TX_PINOP		1

// Timer of the modem waits and of the send interval: RTC0, RTC1 is the
// timer of the Bluetooth port (bt_app_nrf91.cpp)
#define LTE_TIMER_DEVNO		0

#endif
