/**-------------------------------------------------------------------------
@file	direct_link.cpp

@brief	Implementation of Excelitas Direct Link single wire interface


@author	Hoang Nguyen Hoan
@date	Nov. 16, 2019

@license

MIT License

Copyright (c) 2019 I-SYST inc. All rights reserved.

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

#include "direct_link.h"

bool DirectLinkInit(DIRECTLINK_DEV *pDev, int PortNo, int PinNo, int PinOp)
{
	if (pDev == NULL)
	{
		return false;
	}

	pDev->Pin.PortNo = PortNo;
	pDev->Pin.PinNo = PinNo;
	pDev->Pin.PinOp = PinOp;
	pDev->Pin.PinDir = IOPINDIR_INPUT;
	pDev->Pin.Res = IOPINRES_NONE;
	pDev->Pin.Type = IOPINTYPE_NORMAL;

	IOPinCfg(&pDev->Pin, 1);

	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(1);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);
}

uint32_t DirectLinkRead(DIRECTLINK_DEV *pDev, int NbBits)
{
	uint32_t val = 0;

	IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(110);

	for (int i = 0; i < NbBits; i++)
	{
		IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
		IOPinSet(pDev->Pin.PortNo, pDev->Pin.PinNo);
		usDelay(1);
		IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);
		usDelay(10);
		val <<= 1;
		val |= IOPinRead(pDev->Pin.PortNo, pDev->Pin.PinNo);
	}

	IOPinClear(pDev->Pin.PortNo, pDev->Pin.PinNo);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_OUTPUT);
	usDelay(1);
	IOPinSetDir(pDev->Pin.PortNo, pDev->Pin.PinNo, IOPINDIR_INPUT);

	return val;
}




