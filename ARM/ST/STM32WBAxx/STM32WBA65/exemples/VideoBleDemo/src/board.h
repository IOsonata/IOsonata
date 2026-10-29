/**-------------------------------------------------------------------------
@file	board.h

@brief	VideoBleDemo pin configuration

ArduCAM Mega on SPI, frames over BLE.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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

// Board, one of the names in stm32_boards.h. This STM32WBA65 project builds for
// STM32WBA65I_DK1.
#define BOARD				STM32WBA65I_DK1

#include "stm32_boards.h"

// LD6 green PD8, connection state
#define BLUEIO_CONNECT_LED_PORT	IOPORTD
#define BLUEIO_CONNECT_LED_PIN	8

// ArduCAM Mega SPI camera module
#define ARDUCAM_MEGA

// SPI2 on the Arduino pins: SCK PB10 (D13), MISO PA9 (D12), MOSI PC3 (D11),
// chip select PB9 (D10)
#define SPI_DEVNO			BOARD_SPI_DEVNO
#define SPI_PINS	{ \
	{BOARD_SPI_SCK_PORT, BOARD_SPI_SCK_PIN, BOARD_SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BOARD_SPI_MISO_PORT, BOARD_SPI_MISO_PIN, BOARD_SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BOARD_SPI_MOSI_PORT, BOARD_SPI_MOSI_PIN, BOARD_SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{BOARD_SPI_CS_PORT, BOARD_SPI_CS_PIN, BOARD_SPI_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
