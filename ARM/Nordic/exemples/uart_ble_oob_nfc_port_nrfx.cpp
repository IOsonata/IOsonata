/**-------------------------------------------------------------------------
@file	uart_ble_oob_nfc_port_nrfx.cpp

@brief	Nordic NFCT port for the uart_ble LESC OOB NFC channel

Provides the NFC frame transport the uart_ble example publishes its LESC OOB
data on when built with BLE_SC_METHOD == BLE_SC_OOB and BLE_SC_OOB_NFC. Each
reader frame goes to the tag object in the example, which runs the Type 4
protocol and answers through this transport.

Link nfct_nrfx.cpp, rftag.cpp, rftag_ndef.cpp, bt_oob_rftag.cpp and
rftag_proto_t4.cpp. Attaching the RFTagProtoT4 engine selects the protocol,
no enable macro is needed. The NFCT interrupt vector must reach
nrfx_nfct_irq_handler and the high frequency crystal clock must be running,
which the BLE controller already requires.

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
	// ProcessFrame runs the protocol and writes the response into the local
	// buffer. Send it back through this transport. A zero length result, such
	// as a read only OOB tag ignoring a write, needs no answer.
	uint8_t tx[NFCT_INTRF_FRAME_MAX];

	int n = g_BleOobTag.ProcessFrame(pFrame, FrameLen, tx, sizeof(tx));

	if (n > 0)
	{
		DeviceIntrfTx((DevIntrf_t *)s_BleOobNfct, 0, tx, n);
	}
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

	// Not enabled here. The example enables the transport after the tag is
	// initialized and the NDEF record is in place, so a reader can never see
	// an uninitialized tag or an empty record.
	return &s_BleOobNfct;
}
