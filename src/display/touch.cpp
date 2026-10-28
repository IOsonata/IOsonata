/**-------------------------------------------------------------------------
@file	touch.cpp

@brief	Generic touch panel controller: panel, pins and coordinates

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
#include <string.h>

#include "iopinctrl.h"
#include "display/touch.h"

#define TOUCH_FLAG_ALL		(TOUCH_FLAG_XY_SWAP | TOUCH_FLAG_X_INV | TOUCH_FLAG_Y_INV)

Touch::Touch() : vXRange(0), vYRange(0), vOrient(DISPL_ORIENT_LANDSCAPE)
{
	memset(&vCfg, 0, sizeof(vCfg));
	vCfg.RstPin.PortNo = -1;
	vCfg.IntPin.PortNo = -1;
}

void Touch::Orientation(DISPL_ORIENT Orient)
{
	vOrient = Orient;
}

uint16_t Touch::Width() const
{
	return vOrient == DISPL_ORIENT_PORTRAIT || vOrient == DISPL_ORIENT_PORTRAIT_INV ? vCfg.Height : vCfg.Width;
}

uint16_t Touch::Height() const
{
	return vOrient == DISPL_ORIENT_PORTRAIT || vOrient == DISPL_ORIENT_PORTRAIT_INV ? vCfg.Width : vCfg.Height;
}

bool Touch::SetPanel(const TouchCfg_t &Cfg, uint8_t DefaultAddr)
{
	if (Cfg.Width == 0 || Cfg.Height == 0 || Cfg.DevAddr > 0x7F || (Cfg.Flags & ~TOUCH_FLAG_ALL) != 0)
	{
		return false;
	}

	vCfg = Cfg;
	if (vCfg.DevAddr == 0)
	{
		vCfg.DevAddr = DefaultAddr;
	}
	DeviceAddress(vCfg.DevAddr);
	if (vCfg.Flags & TOUCH_FLAG_XY_SWAP)
	{
		Range(vCfg.Height, vCfg.Width);
	}
	else
	{
		Range(vCfg.Width, vCfg.Height);
	}
	IntPinIn();

	return true;
}

void Touch::Range(uint16_t XRange, uint16_t YRange)
{
	vXRange = XRange;
	vYRange = YRange;
}

bool Touch::Convert(uint16_t X, uint16_t Y, uint8_t Id, TouchPoint_t &Pt) const
{
	if (X >= vXRange || Y >= vYRange)
	{
		return false;
	}

	bool swap = (vCfg.Flags & TOUCH_FLAG_XY_SWAP) != 0;
	uint32_t px = (uint32_t)(swap ? Y : X) * vCfg.Width / (swap ? vYRange : vXRange);
	uint32_t py = (uint32_t)(swap ? X : Y) * vCfg.Height / (swap ? vXRange : vYRange);

	if (vCfg.Flags & TOUCH_FLAG_X_INV)
	{
		px = vCfg.Width - 1U - px;
	}
	if (vCfg.Flags & TOUCH_FLAG_Y_INV)
	{
		py = vCfg.Height - 1U - py;
	}

	// The inverse of the frame buffer mapping of the display
	switch (vOrient)
	{
		case DISPL_ORIENT_LANDSCAPE_INV:
			Pt.X = (uint16_t)(vCfg.Width - 1U - px);
			Pt.Y = (uint16_t)(vCfg.Height - 1U - py);
			break;
		case DISPL_ORIENT_PORTRAIT:
			Pt.X = (uint16_t)py;
			Pt.Y = (uint16_t)(vCfg.Width - 1U - px);
			break;
		case DISPL_ORIENT_PORTRAIT_INV:
			Pt.X = (uint16_t)(vCfg.Height - 1U - py);
			Pt.Y = (uint16_t)px;
			break;
		default:
			Pt.X = (uint16_t)px;
			Pt.Y = (uint16_t)py;
			break;
	}
	Pt.Id = Id;

	return true;
}

void Touch::ResetPin(bool bHigh)
{
	if (bHigh)
	{
		IOPinSet(vCfg.RstPin.PortNo, vCfg.RstPin.PinNo);
	}
	else
	{
		IOPinClear(vCfg.RstPin.PortNo, vCfg.RstPin.PinNo);
	}
	IOPinConfig(vCfg.RstPin.PortNo, vCfg.RstPin.PinNo, vCfg.RstPin.PinOp, IOPINDIR_OUTPUT, vCfg.RstPin.Res,
				vCfg.RstPin.Type);
}

void Touch::IntPinOut(bool bHigh)
{
	if (HasIntPin() == false)
	{
		return;
	}
	if (bHigh)
	{
		IOPinSet(vCfg.IntPin.PortNo, vCfg.IntPin.PinNo);
	}
	else
	{
		IOPinClear(vCfg.IntPin.PortNo, vCfg.IntPin.PinNo);
	}
	IOPinConfig(vCfg.IntPin.PortNo, vCfg.IntPin.PinNo, vCfg.IntPin.PinOp, IOPINDIR_OUTPUT, vCfg.IntPin.Res,
				vCfg.IntPin.Type);
}

void Touch::IntPinIn()
{
	if (HasIntPin())
	{
		IOPinConfig(vCfg.IntPin.PortNo, vCfg.IntPin.PinNo, vCfg.IntPin.PinOp, IOPINDIR_INPUT, vCfg.IntPin.Res,
					vCfg.IntPin.Type);
	}
}
