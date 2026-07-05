/*--------------------------------------------------------------------------
File   : rftag_t4t_demo_port_nrfx.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : Nordic NFCT target port for the RFTag Type 4 demo

Builds the NFCT frame transport and hands it to the demo. Each reader frame
is fed to RFTagProcessFrame, which runs the Type 4 protocol and sends the
response back through the transport. SEL_RES advertises ISO-DEP so a phone
sends RATS after selection.

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

	// Runs in the NFCT interrupt. The protocol builds the response and the
	// tag sends it back through this transport within the reply window.
	g_Tag.ProcessFrame(pFrame, FrameLen);
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
