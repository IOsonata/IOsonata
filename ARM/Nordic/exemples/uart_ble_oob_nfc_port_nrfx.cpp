/*--------------------------------------------------------------------------
File   : uart_ble_oob_nfc_port_nrfx.cpp

Author : Hoang Nguyen Hoan          Jul. 5, 2026

Desc   : Nordic NFCT port for the uart_ble LESC OOB NFC channel

Provides the NFC frame transport the uart_ble example publishes its LESC OOB
data on when built with BLE_SC_METHOD == BLE_SC_OOB and BLE_SC_OOB_NFC. Each
reader frame goes to the tag object in the example, which runs the Type 4
protocol and answers through this transport.

Build with RFTAG_PROTO_T4T_ENABLE and link nfct_nrfx.cpp, rftag.cpp,
rftag_ndef.cpp, bt_oob_rftag.cpp and rftag_proto_t4t.cpp. The NFCT interrupt
vector must reach nrfx_nfct_irq_handler and the high frequency crystal clock
must be running, which the BLE controller already requires.

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
#include "rftag/rftag.h"
#include "nfct_nrfx.h"

// The tag object lives in uart_ble.cpp under BLE_SC_OOB_NFC.
extern RFTag g_BleOobTag;

static NfctIntrf s_BleOobNfct;

// SEL_RES protocol field 0x20 announces ISO-DEP compliance.
#define OOB_SEL_RES_ISODEP		0x20

static void BleOobNfcFrameHandler(NfctIntrfDev_t * const pDev, const uint8_t *pFrame, int FrameLen)
{
	(void)pDev;

	// Runs in the NFCT interrupt. Keep it short, the reply window is timed.
	g_BleOobTag.ProcessFrame(pFrame, FrameLen);
}

DeviceIntrf *BleOobNfcGetTransport(void)
{
	NfctIntrfCfg_t cfg;

	memset(&cfg, 0, sizeof(cfg));
	cfg.IdLen = 0;		// keep the hardware default NFCID1
	cfg.SelRes = OOB_SEL_RES_ISODEP;
	cfg.pFrameCB = BleOobNfcFrameHandler;
	cfg.pEvtCB = nullptr;
	cfg.pCtx = nullptr;

	if (s_BleOobNfct.Init(cfg) == false)
	{
		return nullptr;
	}

	s_BleOobNfct.Enable();

	return &s_BleOobNfct;
}
