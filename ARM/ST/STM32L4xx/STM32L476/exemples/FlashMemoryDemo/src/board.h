/**-------------------------------------------------------------------------
@file	board.h

@brief	Board specific definitions

This file contains all I/O definitions for a specific board for the
application firmware.  This files should be located in each project and
modified to suit the need for the application use case.

@author	Hoang Nguyen Hoan
@date	July 26, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

// UART pins, not used by the demo: the Debug build prints with semihosting
#define UART_RX_PORT		IOPORTB
#define UART_RX_PIN			7
#define UART_RX_PINOP		IOPINOP_FUNC7
#define UART_TX_PORT		IOPORTB
#define UART_TX_PIN			6
#define UART_TX_PINOP		IOPINOP_FUNC7
#define UART_CTS_PORT		-1
#define UART_CTS_PIN		-1
#define UART_CTS_PINOP		0
#define UART_RTS_PORT		-1
#define UART_RTS_PIN		-1
#define UART_RTS_PINOP		0

// Onboard MX25R6435F 64 Mbit flash on QUADSPI (SPI DevNo 3), AF10: CLK PE10,
// IO0 PE12, IO1 PE13, IO2 PE14, IO3 PE15, NCS PE11. The pin map order is
// SCK, D0 to D3, then the chip select (QSPI_CS_IOPIN_IDX).
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

// QUADSPI is polling only
#define SPI_DMA_ENABLE		false

// The legacy Flash driver drives QUADSPI with QuadSPISendCmd
#define NVM_MODE			0

// MX25R6435F with the single line read and page program commands: the quad
// commands need the QE bit of the flash status register, which this demo
// does not set
#define FLASH_CFG(InitCB, WaitCB)		{ \
	.DevNo = 0, \
	.TotalSize = 64 * 1024 / 8, \
	.BlkSize = 64 * 1024, \
	.SectSize = FLASH_MX25R6435F_SECTSIZE, \
	.PageSize = 256, \
	.AddrSize = 3, \
	.DevId = 0x1728c2, \
	.DevIdSize = 3, \
	.pInitCB = InitCB, \
	.pWaitCB = WaitCB, \
	.RdCmd = { FLASH_CMD_READ, 0 }, \
	.WrCmd = { FLASH_CMD_WRITE, 0 }, }

#endif // __BOARD_H__
