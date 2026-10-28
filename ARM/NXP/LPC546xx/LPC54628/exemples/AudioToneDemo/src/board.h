/**-------------------------------------------------------------------------
@example	board.h

@brief	LPCXpresso54628 board definitions for AudioToneDemo

@author	Hoang Nguyen Hoan
@date	Oct. 27, 2026

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

#include "LPC546xx.h"
#include "i2s_lpc546xx.h"

// Output on the LPC-Link2 VCOM, Flexcomm 0, P0_29 RXD and P0_30 TXD, 1000000
// baud 8N1. The Release build prints there, the Debug build uses semihosting.
#define UART_RETARGET
#define UART_DEVNO			0
#define UART_RX_PORT		0
#define UART_RX_PIN			29
#define UART_RX_PINOP		IOPINOP_FUNC1
#define UART_TX_PORT		0
#define UART_TX_PIN			30
#define UART_TX_PINOP		IOPINOP_FUNC1

// WM8904 codec control: on board I2C bus, Flexcomm 2, P3_23 SDA and P3_24
// SCL on function 1, open drain with pull-ups on the board. Address 0x1A.
#define AUDIO_I2C_DEVNO		2
#define AUDIO_I2C_PINS		{ \
	{3, 23, IOPINOP_FUNC1, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
	{3, 24, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}, \
}
#define AUDIO_CODEC_ADDR	0x1A

// I2S0 on Flexcomm 6 sends to the codec and makes SCK (BCLK) and WS
// (LRCLK). MCLK on P3_11 is the audio PLL / K, 256 x the frame rate. Pins of
// the NXP SDK I2S examples (pin_mux.c). Pin map: FC6_SCK P4_1, FC6 WS P4_3,
// no SDI, FC6 data to DACDAT P4_2, MCLK P3_11.
#define AUDIO_I2S_DEVNO		6
#define AUDIO_I2S_PINS		{ \
	{4, 1, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{4, 3, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{-1, -1, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
	{4, 2, IOPINOP_FUNC2, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL}, \
	{3, 11, IOPINOP_FUNC1, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}, \
}

#define AUDIO_RATE			48000

#endif // __BOARD_H__
