/**-------------------------------------------------------------------------
@example	board.h

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

// Board, one of the names in stm32_boards.h. This STM32L476 project builds for
// B_L475E_IOT01A (B-L475E-IOT01A and B-L475E-IOT01A2, STM32L475VG).
#define BOARD				B_L475E_IOT01A

#include "stm32_boards.h"

// Output on USART1, the ST-LINK virtual COM port, 1000000 baud 8N1
#define NVM_DEMO_UART_DEVNO	BOARD_VCP_DEVNO
#define UART_PINS			BOARD_VCP_PINS

// The onboard MX25R6435F through the Nvm driver, the checks run once
#define NVM_DEMO_MEDIUM		1
#define NVM_DEMO_BLE		0
#define NVM_DEMO_ASYNC		0

// QUADSPI (SPI DevNo 3), AF10: CLK PE10, IO0 PE12, IO1 PE13, IO2 PE14, IO3
// PE15, NCS PE11. The pin map order is SCK, D0 to D3, then the chip select
// (QSPI_CS_IOPIN_IDX).
#define SPI_DEVNO			3
#define SPI_PHY				SPIPHY_QUAD_SDR
#define SPI_PINS_CFG	{ \
	{IOPORTE, 10, IOPINOP_FUNC10, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 12, IOPINOP_FUNC10, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 13, IOPINOP_FUNC10, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 14, IOPINOP_FUNC10, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 15, IOPINOP_FUNC10, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 11, IOPINOP_FUNC10, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// MX25R6435F : 64 Mbits NOR flash, 4K sector, 256 byte page, JEDEC id
// C2 28 17. Quad: QREAD 0x6B (1-1-4, 8 dummy cycles) and 4PP 0x38 (1-4-4),
// with the Quad Enable bit, status register 1 bit 6, set by Nvm.
#define NVM_DEMO_CHIP_CFG	{ \
	.DevNo = 0, \
	.TotalSize = 64 * 1024 * 1024 / 8, \
	.EraseSize = 4 * 1024, \
	.PageSize = 256, \
	.AddrSize = 3, \
	.DevId = 0x1728C2, \
	.DevIdSize = 3, \
	.RdCmd = { 0x6B, 8 }, \
	.WrCmd = { 0x38, 0 }, \
	.QuadEnable = { 1, 6 }, \
	.WrProtMask = 0x3C, \
	.WrProtPin = { -1, -1, }, \
}

#endif // __BOARD_H__
