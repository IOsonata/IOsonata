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

// Thingy:53 expansion connector P1. Only TX and ground are needed for the log.
#define UART_PINS { \
	{0, 5, 1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 4, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define NVM_DEMO_MEDIUM		1
#define SPI_DEVNO			5
#define SPI_PHY				SPIPHY_DUAL
#define NVM_DEMO_SPI_RATE	8000000

// Dedicated QSPI pins in SCK, IO0, IO1, IO2, IO3, CS order.
#define SPI_PINS_CFG { \
	{0, 17, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 13, 1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 14, 1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 18, 1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// MX25R6435F: READ2IO over two data lines and single-line page program.
#define NVM_DEMO_CHIP_CFG { \
	.TotalSize = 8U * 1024U * 1024U, \
	.EraseSize = 4096, \
	.PageSize = 256, \
	.AddrSize = 3, \
	.DevId = 0x1728C2, \
	.DevIdSize = 3, \
	.RdCmd = { FLASH_CMD_2READ, 4 }, \
	.WrCmd = { FLASH_CMD_WRITE, 0 }, \
	.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
}

#endif // __BOARD_H__
