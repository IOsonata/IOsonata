/**-------------------------------------------------------------------------
@file	display_hx8357.h

@brief	HX8357 LCD display controller implementation


@author	Hoang Nguyen Hoan
@date	Mar. 18, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

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
#ifndef __DISPLAY_HX8357_H__
#define __DISPLAY_HX8357_H__

#include "display/display_lcdmtrx.h"

#define HX8357_WIDTH_MAX					320
#define HX8357_HEIGHT_MAX					480

#define HX8357_CMD_SET_OSC					0xB0	// Set internal oscillator

#define HX8357_CMD_SET_PWR_CTRL				0xB1	// Set power control
#define HX8357_CMD_SET_DISPL_CTRL			0xB2	// Set display control
#define HX8357_CMD_SET_RGB					0xB3	// Set RGB interface
#define HX8357_CMD_SET_CYC					0xB4	// Set display cycle
#define HX8357_CMD_SET_BGP					0xB5	// Set BGP voltage
#define HX8357_CMD_SET_VCOM					0xB6	// Set VCOM voltage
#define HX8357_CMD_SET_OTP					0xB7	// Set one time programmable
#define HX8357_CMD_SET_EXTC					0xB9	// Enter extension command

#define HX8357_CMD_SET_STBA					0xC0	// Set source option
#define HX8357_CMD_SET_DGC					0xC1	// Set digital gamma correction
#define HX8357_CMD_SET_ID					0xC3	// Set ID
#define HX8357_CMD_SET_DDB					0xC4	// Set DDB ???
#define HX8357_CMD_SET_CABC					0xC9	// Set CABC	???
#define HX8357_CMD_SET_PANEL				0xCC	// Set panel characteristics

#define HX8357_CMD_SET_GAMMA				0xE0	// Set gamma
#define HX8357_CMD_SET_IMAGE_TYPE			0xE9	// Set image type
#define HX8357_CMD_SET_MESSI				0xEA	// Set command type
#define HX8357_CMD_SET_COLOR				0xEB	// Set color
#define HX8357_CMD_SET_READ_INDEX			0xFE	// Set SPI read command address
#define HX8357_CMD_GET_SPI_READ				0xFF	// Read SPI command data

#define HX8357_LINE_BUFFLEN					(HX8357_HEIGHT_MAX << 2)

class LcdHX8357 : public LCDMatrix {
public:
	bool Init(const DisplayCfg_t &, DeviceIntrf * const pIntrf);


private:
	uint8_t vLineBuff[HX8357_LINE_BUFFLEN];
};


#endif // __DISPLAY_HX8357_H__
