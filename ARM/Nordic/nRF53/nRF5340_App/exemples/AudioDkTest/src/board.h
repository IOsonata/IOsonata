/**-------------------------------------------------------------------------
@file	board.h

@brief	nRF5340 Audio DK board configuration

@author	Nguyen Hoan Hoang
@date	Oct. 24, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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

// Nordic PCA10121 nRF5340 Audio DK, application core.
#define UART_RETARGET
#define UART_DEVNO 0
#define UART_RX_PORT 1
#define UART_RX_PIN 4
#define UART_RX_PINOP 0
#define UART_TX_PORT 1
#define UART_TX_PIN 5
#define UART_TX_PINOP 0

#define AUDIO_I2C_DEVNO 1
#define AUDIO_SPI_DEVNO 4
#define AUDIO_CODEC_CS 1
#define AUDIO_CARD_CS 0
#define AUDIO_CODEC_IF_PORT 0
#define AUDIO_CODEC_IF_PIN 21
#define AUDIO_NPM_ISET_PORT 0
#define AUDIO_NPM_ISET_PIN 23

#define AUDIO_TEST_I2S_RESTART 0
#define AUDIO_I2S_DEVNO 0
#define AUDIO_CODEC_RESET_PORT 0
#define AUDIO_CODEC_RESET_PIN 18
#define AUDIO_CODEC_IRQ_PORT 0
#define AUDIO_CODEC_IRQ_PIN 19
#define AUDIO_UART_PINS { \
	{ UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL } \
}
#define AUDIO_I2C_PINS { \
	{ 1, 2, 0, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN }, \
	{ 1, 3, 0, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN } \
}
#define AUDIO_SPI_PINS { \
	{ 0, 8, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 10, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 9, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 11, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 17, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL } \
}
#define AUDIO_I2S_PINS { \
	{ 0, 14, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 16, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 15, 0, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 13, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL }, \
	{ 0, 12, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL } \
}
#define AUDIO_MONITOR_ADDRESSES { 0x44, 0x45, 0x41, 0x40 }
#define AUDIO_MONITOR_SHUNTS { 510000, 2200000, 2200000, 1000000 }

#endif
