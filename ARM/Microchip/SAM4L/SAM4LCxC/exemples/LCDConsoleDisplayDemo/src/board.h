/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM4L8 Xplained Pro with OLED1 on EXT2


@author	Hoang Nguyen Hoan
@date	Nov. 17, 2026

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
#define DISPLAY_DRIVER LcdSSD1306
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 32
#define DISPLAY_PIXEL_SIZE 1
#define DISPLAY_STRIDE 16
#define DISPLAY_SPI_CLKPOL SPICLKPOL_HIGH
#ifndef DISPLAY_SPI_DMA_ENABLE
#define DISPLAY_SPI_DMA_ENABLE false
#endif
#ifndef DISPLAY_SPI_INT_ENABLE
#define DISPLAY_SPI_INT_ENABLE false
#endif
#define DISPLAY_SYSLOG_ENABLE 1
#define CONSOLE_LOG_UART 1
#define CONSOLE_LOG_DISPLAY 2
#ifndef CONSOLE_LOG_OUTPUT
#define CONSOLE_LOG_OUTPUT CONSOLE_LOG_DISPLAY
#endif
#if CONSOLE_LOG_OUTPUT == CONSOLE_LOG_UART
#define CONSOLE_SYSLOG_UART 1
#elif CONSOLE_LOG_OUTPUT != CONSOLE_LOG_DISPLAY
#error "SAM4LC: select UART or display for syslog, not both"
#endif
// EDBG virtual COM port, USART1: RX PC26/A, TX PC27/A.
#define UART_DEVNO 1
#define UART_BAUDRATE 1000000
#define UART_PINS { \
	{IOPORTC, 26, IOPINOP_PERIPHA, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{IOPORTC, 27, IOPINOP_PERIPHA, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
#define SPI_DEVNO 0
#define SPI_PHY SPIPHY_NORMAL
#define SPI_RATE 1000000
// SPI0: SCK PC30/B, MISO PA21/A (unconnected), MOSI PA22/A.
#define SPI_PINS { \
	{IOPORTC, 30, IOPINOP_PERIPHB, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 21, IOPINOP_PERIPHA, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{IOPORTA, 22, IOPINOP_PERIPHA, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTB, 11, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
// D/C PC08, no backlight, reset PC09.
#define TFT_PINS { \
	{IOPORTC, 8, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 9, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
#endif
