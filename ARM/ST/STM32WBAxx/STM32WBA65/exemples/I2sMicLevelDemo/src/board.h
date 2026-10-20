/**-------------------------------------------------------------------------
@file	board.h

@brief	STM32WBA65I-DK1 WM8904, SAI1 and SSD1315 pin mapping (UM3462).

@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#define BOARD STM32WBA65I_DK1
#include "stm32_boards.h"

#define OLED_SPI_DEVNO 2
#define OLED_SCK_PORT IOPORTA
#define OLED_SCK_PIN 0
#define OLED_SCK_PINOP IOPINOP_FUNC6
#define OLED_MOSI_PORT IOPORTB
#define OLED_MOSI_PIN 8
#define OLED_MOSI_PINOP IOPINOP_FUNC6
#define OLED_CS_PORT IOPORTE
#define OLED_CS_PIN 1
#define OLED_DC_PORT IOPORTE
#define OLED_DC_PIN 0
#define OLED_RST_PORT IOPORTE
#define OLED_RST_PIN 3

#define DISPLAY_DRIVER LcdSSD1315
#define DISPLAY_ORIENTATION DISPL_ORIENT_LANDSCAPE
#define DISPLAY_WIDTH 128
#define DISPLAY_HEIGHT 64
#define DISPLAY_STRIDE 16
#define DISPLAY_PIXEL_SIZE 1
// IOsonata names the active polarity: HIGH means CPOL=0, idle low.
#define DISPLAY_SPI_CLKPOL SPICLKPOL_HIGH
#define DISPLAY_SPI_DMA_ENABLE false

#define SPI_DEVNO OLED_SPI_DEVNO
#define SPI_PHY SPIPHY_NORMAL
#define SPI_RATE 4000000
#define SPI_PINS { \
	{OLED_SCK_PORT, OLED_SCK_PIN, OLED_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{OLED_MOSI_PORT, OLED_MOSI_PIN, OLED_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{OLED_CS_PORT, OLED_CS_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
#define TFT_PINS { \
	{OLED_DC_PORT, OLED_DC_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{OLED_RST_PORT, OLED_RST_PIN, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}

// Onboard codec control uses the shared I2C1 bus (7-bit codec address).
#define AUDIO_I2C_DEVNO BOARD_I2C_DEVNO
#define AUDIO_I2C_PINS BOARD_I2C_PINS
#define AUDIO_CODEC_ADDR 0x1A
#define AUDIO_MIC_INPUT 2
#define AUDIO_I2S_DEVNO 0
// SCK_A, FS_A, SD_B in, unused SD_A out, MCLK_A.
#define AUDIO_I2S_PINS { \
	{IOPORTA, 7, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 4, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTC, 5, IOPINOP_FUNC13, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{IOPORTE, 2, IOPINOP_FUNC13, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL} \
}
#define TIMER_DEVNO 2

#endif
