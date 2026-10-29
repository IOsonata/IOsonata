/**-------------------------------------------------------------------------
@file	board.h

@brief	VideoBleDemo pin configuration

ArduCAM Mega on SPI, frames over BLE. BlueIO breakout board by default,
define NORDIC_DK for the Nordic DK PCA10040.

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

//#define NORDIC_DK

#ifdef NORDIC_DK
// LED1, connection state
#define BLUEIO_CONNECT_LED_PORT		0
#define BLUEIO_CONNECT_LED_PIN		17
#else
#include "blueio_board.h"
#endif

// ArduCAM Mega SPI camera module
#define ARDUCAM_MEGA

#ifdef NORDIC_DK
// Nordic DK PCA10040, Arduino header D13 SCK, D12 MISO, D11 MOSI, D10 CS
#define SPI_DEVNO			1
#define SPI_SCK_PORT		0
#define SPI_SCK_PIN			25
#define SPI_SCK_PINOP		1
#define SPI_MISO_PORT		0
#define SPI_MISO_PIN		24
#define SPI_MISO_PINOP		1
#define SPI_MOSI_PORT		0
#define SPI_MOSI_PIN		23
#define SPI_MOSI_PINOP		1
#define SPI_CS_PORT			0
#define SPI_CS_PIN			22
#define SPI_CS_PINOP		1
#else
// BlueIO breakout board SPI1
#define SPI_DEVNO			BLUEIO_SPI1_DEVNO
#define SPI_SCK_PORT		BLUEIO_SPI1_SCK_PORT
#define SPI_SCK_PIN			BLUEIO_SPI1_SCK_PIN
#define SPI_SCK_PINOP		BLUEIO_SPI1_SCK_PINOP
#define SPI_MISO_PORT		BLUEIO_SPI1_MISO_PORT
#define SPI_MISO_PIN		BLUEIO_SPI1_MISO_PIN
#define SPI_MISO_PINOP		BLUEIO_SPI1_MISO_PINOP
#define SPI_MOSI_PORT		BLUEIO_SPI1_MOSI_PORT
#define SPI_MOSI_PIN		BLUEIO_SPI1_MOSI_PIN
#define SPI_MOSI_PINOP		BLUEIO_SPI1_MOSI_PINOP
#define SPI_CS_PORT			0
#define SPI_CS_PIN			26
#define SPI_CS_PINOP		1
#endif

#define SPI_PINS	{ \
	{SPI_SCK_PORT, SPI_SCK_PIN, SPI_SCK_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_MISO_PORT, SPI_MISO_PIN, SPI_MISO_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_MOSI_PORT, SPI_MOSI_PIN, SPI_MOSI_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{SPI_CS_PORT, SPI_CS_PIN, SPI_CS_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
}

#endif // __BOARD_H__
