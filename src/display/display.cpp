/**-------------------------------------------------------------------------
@file	display.cpp

@brief	Generic display controller


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
#include "display/display.h"
#include "iopinctrl.h"

void Display::Backlight(bool bOn)
{
	if (vCfg.NbPins > DISPL_CTRL_DCX_PINIDX)
	{
		if (bOn)
		{
			IOPinSet(vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
		}
		else
		{
			IOPinClear(vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_BKLIGHT_PINIDX].PinNo);
		}
	}
}

void Display::Reset()
{
	if (vCfg.NbPins > DISPL_CTRL_BKLIGHT_PINIDX)
	{
		IOPinClear(vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
		usDelay(100);
		IOPinSet(vCfg.pPins[DISPL_CTRL_RST_PINIDX].PortNo, vCfg.pPins[DISPL_CTRL_RST_PINIDX].PinNo);
	}
}
