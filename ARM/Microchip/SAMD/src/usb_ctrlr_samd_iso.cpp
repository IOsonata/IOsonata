/**-------------------------------------------------------------------------
@file	usb_ctrlr_samd_iso.cpp

@brief	SAM D11/D21 USB isochronous endpoints.

Separate archive member: an application referencing UsbIsoIntrf links it, the
others do not. Isochronous endpoints use the same banks as the other data
endpoints, EPTYPE isochronous, up to 1023 bytes.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include <stddef.h>

#include "usb_ctrlr.h"

bool UsbCtrlrIsoInit(int DevNo)
{
	return DevNo == 0;
}

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn, uint16_t MaxPacketSize)
{
	if (DevNo != 0 || EpNo == 0U || EpNo >= USB_EPIN_CNT_0 ||
		MaxPacketSize == 0U || MaxPacketSize > USB_CTRLR0_ISO_PKT_LEN_MAX)
	{
		return false;
	}

	return UsbCtrlrEpOpenData(DevNo, EpNo, bIn, ISO, MaxPacketSize);
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer, uint16_t Length)
{
	if (DevNo != 0 || EpNum == 0U || EpNum >= USB_EPIN_CNT_0 ||
		Length > USB_CTRLR0_ISO_PKT_LEN_MAX)
	{
		return false;
	}

	// No frame queued: the IN bank stays empty for the IN token, which sets
	// TRFAIL1. A queued zero length frame has a FIFO entry, it is sent so
	// that its completion releases the entry.
	if (pBuffer == nullptr)
	{
		return Length == 0U;
	}

	return UsbCtrlrEpSend(DevNo, EpNum, pBuffer, Length);
}

// No trace ring on this port
uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;

	if (ppData != nullptr)
	{
		*ppData = nullptr;
	}

	return 0U;
}
