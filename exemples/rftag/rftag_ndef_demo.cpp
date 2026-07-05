/**-------------------------------------------------------------------------
@file	rftag_ndef_demo.cpp

@brief	Generic RFTag NDEF demo

@author	Hoang Nguyen Hoan
@date	Desc   : Generic RFTag NDEF demo

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
