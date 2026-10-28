/**-------------------------------------------------------------------------
@example	board.h

@brief	LPCXpresso54628 board definitions for SdCardTest

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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

#include "LPC546xx.h"

// Output on the LPC-Link2 VCOM, Flexcomm 0, P0_29 RXD and P0_30 TXD, 1000000
// baud 8N1. The Release build prints there, the Debug build uses semihosting.
#define UART_RETARGET
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			29
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			30
#define UART_TX_PINOP		IOPINOP_FUNC1

// SD card slot (full size) on the SDIF, IOCON function 2 of the SD pins (NXP
// SDK board files): SD_CLK P2_3, SD_CMD P2_4, SD_D0 to SD_D3 P2_6 to P2_9,
// SD_CARD_DET_N P2_10, SD_POW_EN P2_5, then SD_D4 to SD_D7 (P4_29, P4_30,
// P4_31, P5_0), not wired to the slot: the SDIF needs their SD function (NXP
// SDK, errata). Pull ups on the command and data lines and on the card
// detect. The write protect switch (P3_15) is not read.
#define SDMMC_PINS			{ \
	{2, 3, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{2, 4, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{2, 6, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{2, 7, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{2, 8, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{2, 9, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{2, 10, IOPINOP_FUNC2, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{2, 5, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{4, 29, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{4, 30, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{4, 31, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{5, 0, IOPINOP_FUNC2, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}
#define SDMMC_DEVNO			0
#define SDMMC_BUS_WIDTH		4
#define SDMMC_RATE			25000000	// 24.4 MHz, the main clock / 9
#define SDMMC_CARD_DETECT	true
#define SDMMC_POWER_CTRL	true

#endif // __BOARD_H__
