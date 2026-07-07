/**-------------------------------------------------------------------------
@file	rftag_ndef_demo.cpp

@brief	Generic RFTag NDEF demo

Shows the RFTag facet writing a NDEF message into a tag, independent of the
transport. The target project provides RFTagDemoGetIntrf, which can return any
DeviceIntrf: an I2C dynamic tag such as ST25DV, a SPI or UART reader adapter,
or an MCU internal NFC target. The facet only sees DeviceIntrf.

@author	Hoang Nguyen Hoan
@date	Jul. 7, 2026

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

#include "istddef.h"
#include "device_intrf.h"
#include "rftag/rftag.h"
#include "rftag/rftag_ndef.h"

// The target project provides the transport. It can return any DeviceIntrf
// implementation: an I2C dynamic tag, a SPI or UART reader adapter, or an
// MCU internal NFC target. The facet only sees DeviceIntrf.
extern DeviceIntrf *RFTagDemoGetIntrf(void);

// A tag with an event sink. Override EvtHandler to observe RF side activity.
class DemoTag : public RFTag {
public:
	virtual void EvtHandler(RFTAG_EVT Evt, uint32_t P0, uint32_t P1)
	{
		(void)P0; (void)P1;

		switch (Evt)
		{
			case RFTAG_EVT_MEM_CHANGED:
				// RF side changed the tag memory. Call GetNdef here if needed.
				break;

			case RFTAG_EVT_SELECTED:
			case RFTAG_EVT_DESELECTED:
			default:
				break;
		}
	}
};

static DemoTag g_RFTag;
static uint8_t s_NdefMem[256];

static const RFTagCfg_t s_RFTagCfg = {
	.NfcId = { 0 },
	.IdLen = 0,
	.bReadOnly = false,
	.pMem = s_NdefMem,
	.MemSize = sizeof(s_NdefMem),
	.NdefAddr = 0,
	.NdefMaxLen = sizeof(s_NdefMem),
	.NdefFmt = RFTAG_NDEF_FMT_TLV,
	.WrProtCB = nullptr,
	.pWrProtCtx = nullptr,
};

int main()
{
	RFNdefMsg_t msg;
	DeviceIntrf *pIntrf = RFTagDemoGetIntrf();

	if (pIntrf == nullptr)
	{
		while (true)
		{
		}
	}

	g_RFTag.Init(s_RFTagCfg, pIntrf);

	RFNdefInit(&msg, s_NdefMem, sizeof(s_NdefMem));
	RFNdefAddText(&msg, "en", "IOsonata RFTag");
	RFNdefAddUri(&msg, "https://i-syst.com");

	g_RFTag.SetNdef(msg.pBuf, msg.Len);

	while (true)
	{
	}

	return 0;
}
