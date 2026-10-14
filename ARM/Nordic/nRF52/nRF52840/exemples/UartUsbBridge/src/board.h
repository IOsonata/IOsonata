/**-------------------------------------------------------------------------
@file	board.h

@brief	UART USB bridge on the nRF52840 DK or the Nordic Thingy:91

nRF52840 DK: the bridge connects the nRF52840 USB port to UART0 on the lines
of the interface MCU VCOM0, so the two serial ports of the DK talk to each
other.

Nordic Thingy:91: the bridge runs on the nRF52840 in place of the Connectivity
Bridge firmware and connects its USB port to the nRF9160 UART0, the console of
the nRF9160 examples. Program the nRF52840 through the debug connector with
the SWD select switch on nRF52. This erases the Connectivity Bridge firmware.

Select the board below.

@author	Hoang Nguyen Hoan
@date	Oct. 9, 2026

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

// Board selection, define one:
//	NRF52840_DK		nRF52840 DK, bridge to the interface MCU VCOM0
//	NORDIC_THINGY91	Nordic Thingy:91, bridge to the nRF9160 UART0
//#define NRF52840_DK
#define NORDIC_THINGY91

// Start rate. The host sets the rate it wants when it opens the port.
#define UART_DEVNO			0
#define UART_RATE			115200

#define UART_RX_PORT		0
#define UART_RX_PINOP		1
#define UART_TX_PORT		0
#define UART_TX_PINOP		1

#if defined(NORDIC_THINGY91)
// P0.11 and P0.15 of the nRF52840 to the nRF9160 UART0 TX P0.18 and RX P0.19,
// no flow control
#define UART_RX_PIN			11
#define UART_TX_PIN			15
#elif defined(NRF52840_DK)
#define UART_RX_PIN			8
#define UART_TX_PIN			6
#else
#error "Select the board: NRF52840_DK or NORDIC_THINGY91"
#endif

// RX pulled up so a peer in reset or powered off does not read as data
#define UART_PINS	{ \
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#endif
