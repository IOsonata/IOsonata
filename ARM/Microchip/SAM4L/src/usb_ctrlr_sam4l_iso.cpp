/**-------------------------------------------------------------------------
@file	usb_ctrlr_sam4l_iso.cpp

@brief	Optional SAM4L USBC isochronous endpoint entry points.

SAM4L has independent DMA-backed endpoint banks. Unlike nRF52 USBD there is no
shared EasyDMA scheduler to arbitrate: ISO uses the same per-endpoint bank
ownership path as the other SAM4L data endpoints. Keeping these public ISO
entry points in a separate archive member means applications that do not use
UsbIsoIntrf do not pull them in.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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

bool UsbCtrlrIsoOpen(int DevNo, uint8_t EpNo, bool bIn,
	uint16_t MaxPacketSize)
{
	if (DevNo != 0 || EpNo == 0U || EpNo >= USB_EPIN_CNT_0 ||
		MaxPacketSize == 0U ||
		MaxPacketSize > USB_CTRLR0_ISO_PKT_LEN_MAX)
	{
		return false;
	}

	return UsbCtrlrEpOpenData(DevNo, EpNo, bIn, ISO, MaxPacketSize);
}

bool UsbCtrlrIsoSend(int DevNo, uint8_t EpNum, uint8_t *pBuffer,
	uint16_t Length)
{
	if (DevNo != 0 || EpNum == 0U || EpNum >= USB_EPIN_CNT_0 ||
		Length > USB_CTRLR0_ISO_PKT_LEN_MAX)
	{
		return false;
	}

	// No queued frame: leave the independent IN bank empty. The SAM4L USBC
	// answers an ISO IN token from an empty bank with a zero-length packet.
	// A queued zero-length frame has a non-null FIFO payload pointer and must
	// still be submitted so its completion releases that FIFO entry.
	if (pBuffer == nullptr)
	{
		return Length == 0U;
	}

	return UsbCtrlrEpSend(DevNo, EpNum, pBuffer, Length);
}

uint16_t UsbCtrlrIsoTraceSnapshot(int DevNo, uint8_t **ppData)
{
	(void)DevNo;
	if (ppData != nullptr)
		*ppData = nullptr;
	return 0U;
}
