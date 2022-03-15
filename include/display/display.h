/**-------------------------------------------------------------------------
@file	display.h

@brief	Generic display controller definitions


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
#include "iopinctrl.h"

#define DISPL_CTRL_DCX_PINIDX			0		// Cmd/Data mode pin index
#define DISPL_CTRL_BKLIGHT_PINIDX		1		// External back light pin index
#define DISPL_CTRL_RST_PINIDX			2		// Reset pin index

typedef enum __Display_Orientation {
	DISPL_ORIENT_PORTRAIT,
	DISPL_ORIENT_LANDSCAPE
} DISPL_ORIENT;

typedef struct __Display_Cfg {
	uint32_t DevAddr;		//!< Device address (or SPI device CS index)
	IOPinCfg_t const *pPins;
	int NbPins;
	uint16_t Stride;		//!< Memory stride in bytes for one line
	uint16_t HLen;			//!< Horizontal length in pixels
	uint16_t VLen;			//!< Vertical length in pixels
	uint8_t PixelSize;		//!< Pixel size in bits/pixel
	DISPL_ORIENT Orient;	//!< Display orientation
} DisplayCfg_t;

class Display : public Device {
public:
	virtual bool Init(DisplayCfg_t &, DeviceIntrf *pIntrf) = 0;
	virtual void Clear() = 0;
	virtual void Fill(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint32_t Color) = 0;
	virtual void SetPixel(uint16_t X, uint16_t Y, uint32_t Color) = 0;
	virtual void BitBlt(uint16_t X, uint16_t Y, uint16_t Width, uint16_t Height, uint8_t *pBuffer) = 0;
	virtual void Backlight(bool bOn);

protected:
	DisplayCfg_t vCfg;	//!< Internal copy of config data
};


#endif // __DISPLAY_H__
