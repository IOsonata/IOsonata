/**-------------------------------------------------------------------------
@file	rftag_t4t_demo.cpp

@brief	RFTag NFC Type 4 Tag demo

App side flow:
	1. The target port builds and configures the frame transport. Any DeviceIntrf
	   frame transport works: Nordic NFCT for a local tag, RFTagController for a
	   remote tag, or a bus interface for a chip tag.
	2. Configure the tag. Proto selects the tag behavior. NFC Type 4 here.
	   XCap states which layers the transport performs. The values below match
	   the Nordic NFCT: anticollision, CRC and timing in hardware, ISO-DEP in
	   the protocol module. An NCI class transport adds RFTAG_XCAP_ISODEP and
	   the module then receives bare APDUs.
	3. Drop the transport into the tag Init.
	4. Read, Write, SetNdef, GetNdef and the event callback.

Build this example with RFTAG_PROTO_T4T_ENABLE defined and link rftag_proto_t4t so the
Type 4 protocol object is pulled in.

@author	Hoang Nguyen Hoan
@date	Jul. 5, 2026

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
#include "istddef.h"
#include "device_intrf.h"
#include "rftag/rftag.h"
#include "rftag/rftag_ndef.h"

// Frame transport provided by the target port. For a local tag this wraps the
// Nordic NFCT peripheral. Its RX path calls g_Tag.ProcessFrame on each reader
// frame, and its TX path sends the response frame that ProcessFrame returns.
extern DeviceIntrf *RFTagDemoGetTransport(void);

// External linkage, the target port file references this tag object.
RFTag g_Tag;

// The tag NDEF file. Type 4 layout starts with a 2 byte NLEN field.
static uint8_t s_NdefFile[512];
static uint8_t s_NdefMsg[256];

static void TagEvent(void *pCtx, const RFTagEvt_t *pEvt);

static const RFTagCfg_t s_TagCfg = {
	.Proto = RFTAG_PROTO_NFC_T4,
	.XCap = RFTAG_XCAP_ANTICOL | RFTAG_XCAP_CRC | RFTAG_XCAP_FDT,
	.pMem = s_NdefFile,
	.MemSize = sizeof(s_NdefFile),
	.DevAddr = 0,
	.AddrLen = 2,
	.PageSize = 0,
	.Size = sizeof(s_NdefFile),
	.WrDelay = 0,
	.NdefAddr = 0,
	.NdefMaxLen = sizeof(s_NdefFile),
	.NdefFmt = RFTAG_NDEF_FMT_NLEN16,
	.FdPin = {-1, -1},
	.WrProtPin = {-1, -1},
	.pInitCB = nullptr,
	.pWaitCB = nullptr,
	.pEvtCB = TagEvent,
	.pCtx = nullptr,
};

static void TagEvent(void *pCtx, const RFTagEvt_t *pEvt)
{
	(void)pCtx;

	switch (pEvt->Evt)
	{
		case RFTAG_EVT_MEM_CHANGED:
			// A reader updated the NDEF file. Re-read with g_Tag.GetNdef if needed.
			break;

		case RFTAG_EVT_SELECTED:
		case RFTAG_EVT_DESELECTED:
		default:
			break;
	}
}

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

	RFNdefInit(&msg, s_NdefMsg, sizeof(s_NdefMsg));
	RFNdefAddText(&msg, "en", "IOsonata NFC Type 4 Tag");
	RFNdefAddUri(&msg, "https://i-syst.com");

	g_Tag.SetNdef(msg.pBuf, msg.Len);

	while (true)
	{
	}

	return 0;
}
