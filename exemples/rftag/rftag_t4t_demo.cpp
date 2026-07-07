/**-------------------------------------------------------------------------
@file	rftag_t4t_demo.cpp

@brief	RFTag NFC Type 4 Tag demo

Shows the RFTag facet with a Type 4 protocol engine attached. The design has
three steps:

  1. The target project provides the frame transport as a DeviceIntrf. For a
     local tag this wraps the Nordic NFCT peripheral, whose RX path calls
     g_Tag.ProcessFrame on each reader frame and whose TX path sends the
     response the call returns.
  2. Attach the protocol engine. RFTagProtoT4 gives the tag Type 4 behavior.
  3. Fill the NDEF file. The engine serves it to the reader.

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
#include "rftag/rftag_proto_t4.h"
#include "rftag/rftag_ndef.h"

// The target project provides the frame transport. For a local tag this wraps
// the Nordic NFCT peripheral. Its RX path calls g_Tag.ProcessFrame on each
// reader frame, and its TX path sends the response frame that returns.
extern DeviceIntrf *RFTagDemoGetTransport(void);

// A tag with an event sink. Override EvtHandler to observe RF side activity.
class DemoTag : public RFTag {
public:
	virtual void EvtHandler(RFTAG_EVT Evt, uint32_t P0, uint32_t P1)
	{
		(void)P0; (void)P1;

		switch (Evt)
		{
			case RFTAG_EVT_MEM_CHANGED:
				// A reader updated the NDEF file. Re-read with GetNdef if needed.
				break;

			case RFTAG_EVT_SELECTED:
			case RFTAG_EVT_DESELECTED:
			default:
				break;
		}
	}
};

// External linkage, the target port file references these objects. The
// transport RX path feeds frames to g_Tag.ProcessFrame.
DemoTag g_Tag;
RFTagProtoT4 g_T4;

// The tag NDEF file. Type 4 layout starts with a 2 byte NLEN field.
static uint8_t s_NdefFile[512];
static uint8_t s_NdefMsg[256];

static const RFTagCfg_t s_TagCfg = {
	.NfcId = { 0 },
	.IdLen = 0,
	.bReadOnly = false,
	.pMem = s_NdefFile,
	.MemSize = sizeof(s_NdefFile),
	.NdefAddr = 0,
	.NdefMaxLen = sizeof(s_NdefFile),
	.NdefFmt = RFTAG_NDEF_FMT_NLEN16,
	.WrProtCB = nullptr,
	.pWrProtCtx = nullptr,
};

int main()
{
	RFNdefMsg_t msg;
	DeviceIntrf *pTransport = RFTagDemoGetTransport();

	if (pTransport == nullptr)
	{
		while (true)
		{
		}
	}

	g_Tag.Init(s_TagCfg, pTransport);

	// Attach the Type 4 engine. Instantiating and attaching the engine is the
	// protocol selection, the linker pulls exactly this engine object.
	g_Tag.Attach(&g_T4);

	RFNdefInit(&msg, s_NdefMsg, sizeof(s_NdefMsg));
	RFNdefAddText(&msg, "en", "IOsonata NFC Type 4 Tag");
	RFNdefAddUri(&msg, "https://i-syst.com");

	g_Tag.SetNdef(msg.pBuf, msg.Len);

	while (true)
	{
	}

	return 0;
}
