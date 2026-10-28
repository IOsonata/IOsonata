/**-------------------------------------------------------------------------
@example	board.h

@brief	LPCXpresso54628 board definitions for SpifiFlashDemo

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

#include "coredev/iopincfg.h"
#include "spifi_lpc546xx.h"

// Output on the LPC-Link2 VCOM, Flexcomm 0, P0_29 RXD and P0_30 TXD, 1000000
// baud 8N1, in Debug and Release
#define NVM_DEMO_UART_DEVNO	0
#define UART_PINS			{ \
	{0, 29, IOPINOP_FUNC1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 30, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

// The quad SPI flash of the board on the SPIFI, polling, every operation
// finished in its call. The checks run once.
#define NVM_DEMO_MEDIUM		1
#define NVM_DEMO_BLE		0
#define NVM_DEMO_ASYNC		0

// SPIFI (SPI DevNo LPC546XX_SPIFI_DEVNO), IOCON function 6: SCK P0_26, IO0
// P0_24, IO1 P0_25, IO2 P0_28, IO3 P0_27, CS P0_23. The pin map order is SCK,
// IO0 to IO3, then the chip select (QSPI_CS_IOPIN_IDX).
#define SPI_DEVNO			LPC546XX_SPIFI_DEVNO
#define SPI_PHY				SPIPHY_QUAD_SDR
#define SPI_PINS_CFG	{ \
	{0, 26, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 24, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 25, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 28, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 27, IOPINOP_FUNC6, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 23, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// SPIFI clock: the main clock (220 MHz) divided, the closest to the request:
// 73.3 MHz. The NXP SDK example runs this flash at 96 MHz.
#define NVM_DEMO_SPI_RATE	80000000

// The flash of the board. Recent boards have the Winbond W25Q128JV, the
// default of the NXP SDK SPIFI example, earlier ones the Micron MT25QL128.
// Define SPIFI_FLASH_MT25Q for the Micron part: Init fails on the id when
// the part does not match.
//#define SPIFI_FLASH_MT25Q

#ifdef SPIFI_FLASH_MT25Q
// MT25QL128 : 128 Mbits, 4K subsector, 256 byte page, JEDEC id 20 BA 18.
// Quad: 0x6B (1-1-4, 8 dummy cycles) and 0x32 (1-1-4). No Quad Enable bit,
// the quad commands use DQ2 and DQ3 directly.
#define NVM_DEMO_CHIP_CFG	{ \
	.DevNo = 0, \
	.TotalSize = 128 * 1024 * 1024 / 8, \
	.EraseSize = 4 * 1024, \
	.PageSize = 256, \
	.AddrSize = 3, \
	.DevId = 0x18BA20, \
	.DevIdSize = 3, \
	.RdCmd = { 0x6B, 8 }, \
	.WrCmd = { 0x32, 0 }, \
	.WrProtMask = 0x7C, \
	.WrProtPin = { -1, -1, }, \
}
#else
// W25Q128JV : 128 Mbits, 4K sector, 256 byte page, JEDEC id EF 40 18. Quad:
// 0x6B (1-1-4, 8 dummy cycles) and 0x32 (1-1-4), with the Quad Enable bit,
// status register 2 bit 1, set by Nvm.
#define NVM_DEMO_CHIP_CFG	{ \
	.DevNo = 0, \
	.TotalSize = 128 * 1024 * 1024 / 8, \
	.EraseSize = 4 * 1024, \
	.PageSize = 256, \
	.AddrSize = 3, \
	.DevId = 0x1840EF, \
	.DevIdSize = 3, \
	.RdCmd = { 0x6B, 8 }, \
	.WrCmd = { 0x32, 0 }, \
	.QuadEnable = { 2, 1 }, \
	.WrProtMask = 0x7C, \
	.WrProtPin = { -1, -1, }, \
}
#endif

#endif // __BOARD_H__
