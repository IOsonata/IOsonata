/**-------------------------------------------------------------------------
@file	display_ili9341.cpp

@brief	ILI9341 LCD display controller implementation


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
#include "idelay.h"
#include "convutil.h"
#include "iopinctrl.h"
#include "display/display_ili9341.h"

bool LcdILI9341::Init(DisplayCfg_t &Cfg, DeviceIntrf *pIntrf)
{
	LCDMatrix::vpLineBuff = vLineBuff;
	SetDevRes(ILI9341_WIDTH_MAX, ILI9341_HEIGHT_MAX);

	if (LCDMatrix::Init(Cfg, pIntrf) == false)
	{
		return false;
	}

//	uint8_t cmd = ILI9341_CMD_INTRF_CTRL;
//	uint32_t d = ILI9341_CMD_INTRF_CTRL_LITTLE_ENDIAN;//ILI9341_CMD_INTRF_CTRL_WEMODE | (2<<ILI9341_CMD_INTRF_CTRL_EPF_POS);
//	Write(&cmd, 1, (uint8_t*)&d, 3);

	return true;
}


