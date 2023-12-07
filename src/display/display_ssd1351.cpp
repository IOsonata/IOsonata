/**-------------------------------------------------------------------------
@file	display_ssd1351.cpp

@brief	SSD1351 LCD display controller implementation


@author	Hoang Nguyen Hoan
@date	Jul. 5, 2023

@license

MIT License

Copyright (c) 2023, I-SYST inc., all rights reserved

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
#include "idelay.h"
#include "convutil.h"
#include "iopinctrl.h"
#include "display/display_ssd1351.h"

bool LcdSSD1351::Init(DisplayCfg_t &Cfg, DeviceIntrf *pIntrf)
{
	LCDMatrix::vpLineBuff = vLineBuff;
	SetDevRes(SSD1351_WIDTH_MAX, SSD1351_HEIGHT_MAX);

	if (LCDMatrix::Init(Cfg, pIntrf) == false)
	{
		return false;
	}

	uint8_t cmd = 0xA6;
	Write(&cmd, 1, NULL, 0);

	cmd = 0xAF;
	uint32_t d = 0;
	Write(&cmd, 1, (uint8_t*)&d, 1);

	return true;
}

