/**-------------------------------------------------------------------------
@file	lcd_display.h

@brief	Generic LCD display controller definitions


@author	Hoang Nguyen Hoan
@date	Mar. 9, 2022

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
#ifndef __DISPLAY_H__
#define __DISPLAY_H__

#include "device.h"

#define LCD_CTRL_DCX_PINIDX			0		// Cmd/Data mode pin index
#define LCD_CTRL_RST_PINIDX			1		// Reset pin index

typedef enum __Display_Orientation {
	DISPL_ORIENT_PORTRAIT,
	DISPL_ORIENT_LANDSCAPE
} DISPL_ORIENT;

typedef struct __Lcd_Display_Cfg {
	uint32_t DevAddr;
	IOPinCfg_t const *pPins;
	int NbPins;
	uint16_t Stride;		//!< Memory stride in bytes for one line
	uint16_t HLen;			//!< Horizontal length in pixels
	uint16_t VLen;			//!< Vertical length in pixels
	uint8_t PixelSize;		//!< Pixel size in bits/pixel
	DISPL_ORIENT Orient;	//!< Display orientation
} LcdDisplayCfg_t;

class LcdDisplay : public Device {
public:
	virtual bool Init(LcdDisplayCfg_t &, DeviceIntrf *pIntrf) = 0;
	virtual void Backlight(bool bOn) = 0;

protected:
	LcdDisplayCfg_t vCfg;
};


#endif // __DISPLAY_H__
