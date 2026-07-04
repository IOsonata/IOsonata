/*--------------------------------------------------------------------------
File   : rftag_ndef_demo.cpp

Author : Hoang Nguyen Hoan

Desc   : Generic RFTag NDEF demo

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

/*
 * This example is intentionally not tied to I2C, SPI, UART, or Nordic NFCT.
 *
 * The target project provides RFTagDemoGetIntrf(). It can return any
 * DeviceIntrf implementation:
 *
 *      I2C dynamic tag       ST25DV-style memory tag
 *      SPI controller        external NFC/RFID controller
 *      UART controller       PN532-style adapter
 *      API adapter           vendor NFC stack wrapper
 *      Nordic NFCT adapter   MCU internal NFC target
 *
 * RFTag only sees DeviceIntrf.
 */
extern DeviceIntrf *RFTagDemoGetIntrf(void);

static RFTag g_RFTag;
static uint8_t s_NdefMem[256];

static void RFTagEvent(void *pCtx, const RFTagEvt_t *pEvt)
{
	(void)pCtx;

	switch (pEvt->Evt)
	{
		case RFTAG_EVT_WRITE:
		case RFTAG_EVT_MEM_CHANGED:
			/*
			 * RF side changed the tag memory.
			 * Application can call g_RFTag.GetNdef() here if needed.
			 */
			break;

		case RFTAG_EVT_FIELD_ON:
		case RFTAG_EVT_FIELD_OFF:
		case RFTAG_EVT_SELECTED:
		case RFTAG_EVT_DESELECTED:
		default:
			break;
	}
}

static const RFTagCfg_t s_RFTagCfg = {
	.DevAddr = 0,
	.AddrLen = 2,
	.PageSize = 4,
	.Size = 512,
	.WrDelay = 5,
	.NdefAddr = 0,
	.NdefMaxLen = sizeof(s_NdefMem),
	.NdefFmt = RFTAG_NDEF_FMT_TLV,
	.FdPin = {-1, -1},
	.WrProtPin = {-1, -1},
	.pInitCB = nullptr,
	.pWaitCB = nullptr,
	.pEvtCB = RFTagEvent,
	.pCtx = nullptr,
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
