/**-------------------------------------------------------------------------
@file	display_ili9341.h

@brief	ILI9341 LCD display controller implementation


@author	Hoang Nguyen Hoan
@date	Mar. 28, 2022

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
#ifndef __DISPLAY_ILI9341_H__
#define __DISPLAY_ILI9341_H__

#include "display/display_lcdmtrx.h"

// ST7789 default portrait
#define ILI9341_WIDTH_MAX					240
#define ILI9341_HEIGHT_MAX					320

#define ILI9341_CMD_RGB_IFCTRL				0xB0	// RGB interface signal control
#define ILI9341_CMD_FRAMCTRL_NORMAL			0xB1	// Frame control in normal mode
#define ILI9341_CMD_FRAMCTRL_IDLE			0xB2	// Frame control in idle mode
#define ILI9341_CMD_FRAMCTRL_PARTIAL		0xB3	// Frame control1 in partial mode
#define ILI9341_CMD_DISPLINV_CTRL			0xB4	// Display inversion control
#define ILI9341_CMD_BLANK_PORCH_CTRL		0xB5	// Blanking porch control
#define ILI9341_CMD_DISPLFCT_CTRL			0xB6	// Display function control
#define ILI9341_CMD_ENTRY_MODE_SET			0xB7	// Entry mode set
#define ILI9341_CMD_BACKLIGHT_CTRL1			0xB8	// Backlight control 1
#define ILI9341_CMD_BACKLIGHT_CTRL2			0xB9	// Backlight control 2
#define ILI9341_CMD_BACKLIGHT_CTRL3			0xBA	// Backlight control 3
#define ILI9341_CMD_BACKLIGHT_CTRL4			0xBB	// Backlight control 4
#define ILI9341_CMD_BACKLIGHT_CTRL5			0xBC	// Backlight control 5
#define ILI9341_CMD_BACKLIGHT_CTRL6			0xBD	// Backlight control 6
#define ILI9341_CMD_BACKLIGHT_CTRL7			0xBE	// Backlight control 7
#define ILI9341_CMD_BACKLIGHT_CTRL8			0xBF	// Backlight control 8
#define ILI9341_CMD_PWR_CTRL1				0xC0	// Power control 1
#define ILI9341_CMD_PWR_CTRL2				0xC1	// Power control 1
#define ILI9341_CMD_VCOM_CTRL1				0xC5	// VCOM control 1
#define ILI9341_CMD_VCOM_CTRL2				0xC7	// VCOM control 2
#define ILI9341_CMD_NVMEM_WRITE				0xD0	// NV memory write
#define ILI9341_CMD_NVMEM_PROTKEY			0xD1	// NV memory protection key
#define ILI9341_CMD_NVMEM_STATUS			0xD2	// Read NV memory status
#define ILI9341_CMD_READ_ID4				0xD3	// Read ID4
#define ILI9341_CMD_POS_GAMMA_CORR			0xE0	// Positive gamma correction
#define ILI9341_CMD_NEG_GAMMA_CORR			0xE1	// Negative gamma correction
#define ILI9341_CMD_DIGI_GAMMA_CTRL1		0xE2	// Digital gamma control 1
#define ILI9341_CMD_DIGI_GAMMA_CTRL2		0xE3	// Digital gamma control 2
#define ILI9341_CMD_INTRF_CTRL				0xF6	// Interface control
#define ILI9341_CMD_INTRF_CTRL_WEMODE				(1<<0)	// Memory write wrap
#define ILI9341_CMD_INTRF_CTRL_EPF_POS				(12)
#define ILI9341_CMD_INTRF_CTRL_EPF_MASK				(3<<12)
#define ILI9341_CMD_INTRF_CTRL_LITTLE_ENDIAN		(1<<21)	// Little endian

#define ILI9341_LINE_BUFFLEN					(ILI9341_HEIGHT_MAX << 2)

class LcdILI9341 : public LCDMatrix {
public:
	bool Init(const DisplayCfg_t &, DeviceIntrf * const pIntrf);

private:
	uint8_t vLineBuff[ILI9341_LINE_BUFFLEN];
};


#endif // __DISPLAY_ILI9341_H__
