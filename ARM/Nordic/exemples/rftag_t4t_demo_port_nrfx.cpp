/**-------------------------------------------------------------------------
@file	rftag_t4t_demo_port_nrfx.cpp

@brief	Nordic NFCT target port for the RFTag Type 4 demo

Builds the NFCT frame transport and hands it to the demo. Each reader frame is
fed to RFTag::ProcessFrame, which runs the Type 4 protocol and writes the
response into a caller buffer. The port sends that buffer back through the
transport. SEL_RES advertises ISO-DEP so a phone sends RATS after selection.

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
#include <string.h>

#include "device_intrf.h"
#include "coredev/system_core_clock.h"
#include "rftag/rftag.h"
#include "nfct_nrfx.h"

// The tag object lives in the demo main file.
extern RFTag g_Tag;

static NfctIntrf s_NfctIntrf;

// SEL_RES protocol field 0x20 announces ISO-DEP compliance.
#define DEMO_SEL_RES_ISODEP		0x20

static void NfctFrameHandler(NfctIntrfDev_t * const pDev, const uint8_t *pFrame, int FrameLen)
{
	(void)pDev;

	// Runs in the NFCT interrupt. ProcessFrame runs the protocol and writes
	// the response into the local buffer. Send it back through this transport
	// within the reply window. A zero length result needs no answer.
	uint8_t tx[NFCT_INTRF_FRAME_MAX];

	int n = g_Tag.ProcessFrame(pFrame, FrameLen, tx, sizeof(tx));

	if (n > 0)
	{
		int bits = g_Tag.ResponseBits(n);
		if (bits == n * 8)
		{
			DeviceIntrfTx((DevIntrf_t *)s_NfctIntrf, 0, tx, n);
		}
		else
		{
			s_NfctIntrf.BitsTx(tx, bits);
		}
	}
}

DeviceIntrf *RFTagDemoGetTransport(void)
{
	NfctIntrfCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.IdLen = 0;		// keep the hardware default NFCID1
	cfg.SelRes = DEMO_SEL_RES_ISODEP;
	cfg.pFrameCB = NfctFrameHandler;
	cfg.pEvtCB = nullptr;
	cfg.pCtx = nullptr;

	if (s_NfctIntrf.Init(cfg) == false)
	{
		return nullptr;
	}

	s_NfctIntrf.Enable();

	return &s_NfctIntrf;
}
