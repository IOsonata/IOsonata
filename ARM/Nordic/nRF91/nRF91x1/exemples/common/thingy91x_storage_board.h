/**-------------------------------------------------------------------------
@file	thingy91x_storage_board.h

@brief	Thingy:91 X external SPI flash configuration for the shared storage demos.

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

/** Thingy:91 X external SPI flash configuration for the shared storage demos. */
#ifndef THINGY91X_STORAGE_BOARD_H
#define THINGY91X_STORAGE_BOARD_H

#define UART_DEVNO 0
#define UART_RX_PORT 0
#define UART_RX_PIN 0
#define UART_RX_PINOP 1
#define UART_TX_PORT 0
#define UART_TX_PIN 1
#define UART_TX_PINOP 1
#define UART_CTS_PORT -1
#define UART_CTS_PIN -1
#define UART_CTS_PINOP 0
#define UART_RTS_PORT -1
#define UART_RTS_PIN -1
#define UART_RTS_PINOP 0

#define SPI_DEVNO 3
#define SPI_PHY SPIPHY_NORMAL
#define SPI_DMA_ENABLE false
#define FLASH_SECTOR_SIZE 4096

// SPI3: SCK, MISO, MOSI, then flash CS0, Wi-Fi CS1 and BMI270 CS2.
// Configuring every chip select holds the other devices inactive.
#define SPI_PINS_CFG { \
	{0, 13, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 15, 1, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 14, 1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{0, 12, 1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 17, 1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{0, 10, 1, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

// GD25LE255E: 256 Mbit, 4 KiB sectors, 256-byte pages, four-byte addresses.
// FlashCfg_t uses KiB; the Nvm path in flash_memory_demo converts to bytes.
#define FLASH_CFG(InitCB, WaitCB) { \
	.DevNo = 0, \
	.TotalSize = 32 * 1024, \
	.BlkSize = 64 * 1024, \
	.SectSize = FLASH_SECTOR_SIZE, \
	.PageSize = 256, \
	.AddrSize = 4, \
	.DevId = 0x1960C8, \
	.DevIdSize = 3, \
	.pInitCB = InitCB, \
	.pWaitCB = WaitCB, \
	.RdCmd = {0, 0}, \
	.WrCmd = {0, 0}, \
}

#endif
