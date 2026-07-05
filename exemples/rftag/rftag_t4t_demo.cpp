/*--------------------------------------------------------------------------
File   : rftag_t4t_demo.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : RFTag NFC Type 4 Tag demo

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

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST, I-SYST inc. or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------
Modified by          Date              Description

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
