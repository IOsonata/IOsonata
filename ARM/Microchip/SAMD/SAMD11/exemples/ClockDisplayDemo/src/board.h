/**-------------------------------------------------------------------------
@file	board.h

@brief	SAM D11 Xplained Pro with OLED1 on EXT1


@author	Hoang Nguyen Hoan
@date	Nov. 15, 2026

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
#include "display/display_ssd1306.h"
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
// Select the Cortex-M SysTick implementation from the IOsonata library.
// Initial wall time is restored after reset.
#define CLOCK_TIMER_DEVNO 4
#define CLOCK_TIMER_PRIORITY 1
#define CLOCK_START_HOUR 12
#define CLOCK_START_MINUTE 0
#define CLOCK_START_SECOND 0
// SW0 on PA14, active low. Short press: minute; hold: hour.
#define CLOCK_BUTTON_PORT IOPORTA
#define CLOCK_BUTTON_PIN 14

#define SPI_DEVNO 0
#define SPI_PHY SPIPHY_NORMAL
#define SPI_RATE 1000000
// SCK PA07/C pad1, MISO PA09/D pad3 (OLED1 not connected), MOSI PA06/C pad0.
#define SPI_PINS { \
	{IOPORTA, 7, IOPINOP_PERIPHC, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 9, IOPINOP_PERIPHD, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{IOPORTA, 6, IOPINOP_PERIPHC, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 8, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
// D/C PA04, no backlight, reset PA15.
#define TFT_PINS { \
	{IOPORTA, 4, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTA, 15, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
#endif
