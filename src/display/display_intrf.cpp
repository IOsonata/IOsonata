/**-------------------------------------------------------------------------
@file	display_intrf.cpp

@brief	Display console output interface.

@author	Hoang Nguyen Hoan
@date	Oct. 18, 2026

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

#include "display/display_intrf.h"

DisplayIntrf::DisplayIntrf() : vIntrf{}, vpDisplay(nullptr), vbEnabled(false)
{
	vIntrf.pDevData = this;
	vIntrf.Type = DEVINTRF_TYPE_UNKOWN;
	vIntrf.Disable = DisableOutput;
	vIntrf.Enable = EnableOutput;
	vIntrf.GetRate = GetRate;
	vIntrf.SetRate = SetRate;
	vIntrf.StartRx = StartRxOutput;
	vIntrf.RxData = Receive;
	vIntrf.StopRx = Stop;
	vIntrf.StartTx = StartTxOutput;
	vIntrf.TxData = Transmit;
	vIntrf.TxSrData = Transmit;
	vIntrf.StopTx = Stop;
	vIntrf.Reset = ResetOutput;
	vIntrf.PowerOff = PowerOffOutput;
	vIntrf.GetHandle = Handle;
	atomic_flag_clear(&vIntrf.bBusy);
}

bool DisplayIntrf::Init(Display *pDisplay)
{
	vpDisplay = pDisplay;
	vbEnabled = pDisplay != nullptr && pDisplay->Valid();
	vIntrf.EnCnt = vbEnabled ? 1 : 0;
	vIntrf.bTxReady = vbEnabled;
	return vbEnabled;
}

void DisplayIntrf::DisableOutput(DevIntrf_t *pDev)
{
	static_cast<DisplayIntrf *>(pDev->pDevData)->vbEnabled = false;
}

void DisplayIntrf::EnableOutput(DevIntrf_t *pDev)
{
	static_cast<DisplayIntrf *>(pDev->pDevData)->vbEnabled = true;
}

uint32_t DisplayIntrf::GetRate(DevIntrf_t *pDev)
{
	(void)pDev;
	return 0;
}

uint32_t DisplayIntrf::SetRate(DevIntrf_t *pDev, uint32_t Value)
{
	(void)pDev;
	(void)Value;
	return 0;
}

bool DisplayIntrf::StartRxOutput(DevIntrf_t *pDev, uint32_t Address)
{
	(void)pDev;
	(void)Address;
	return false;
}

int DisplayIntrf::Receive(DevIntrf_t *pDev, uint8_t *pData, int Len)
{
	(void)pDev;
	(void)pData;
	(void)Len;
	return 0;
}

bool DisplayIntrf::StartTxOutput(DevIntrf_t *pDev, uint32_t Address)
{
	DisplayIntrf *dev = static_cast<DisplayIntrf *>(pDev->pDevData);
	(void)Address;
	return dev->vbEnabled && dev->vpDisplay != nullptr && dev->vpDisplay->Valid();
}

int DisplayIntrf::Transmit(DevIntrf_t *pDev, const uint8_t *pData, int Len)
{
	DisplayIntrf *dev = static_cast<DisplayIntrf *>(pDev->pDevData);
	if (pData == nullptr || Len <= 0 || !StartTxOutput(pDev, 0))
	{
		return 0;
	}
	// Bound each printf below Display's default 80-byte formatting buffer.
	// Copy by length: DeviceIntrf input need not have a terminating NUL.
	char text[33];
	int pos = 0;
	while (pos < Len)
	{
		int count = 0;
		while (pos < Len && count < (int)sizeof(text) - 1)
		{
			char ch = (char)pData[pos++];
			if (ch != '\0')
			{
				text[count++] = ch;
			}
		}
		text[count] = '\0';
		dev->vpDisplay->printf("%s", text);
	}
	return Len;
}

void DisplayIntrf::Stop(DevIntrf_t *pDev)
{
	(void)pDev;
}

void DisplayIntrf::ResetOutput(DevIntrf_t *pDev)
{
	(void)pDev;
}

void DisplayIntrf::PowerOffOutput(DevIntrf_t *pDev)
{
	DisplayIntrf *dev = static_cast<DisplayIntrf *>(pDev->pDevData);
	dev->vpDisplay = nullptr;
	dev->vbEnabled = false;
	pDev->EnCnt = 0;
}

void *DisplayIntrf::Handle(DevIntrf_t *pDev)
{
	return pDev->pDevData;
}
