/**-------------------------------------------------------------------------
@file	bt_app.cpp

@brief	Generic Bluetooth application code shared across all ports.
		Holds cross-arch state and helpers that have no SDK dependency.


@author	Hoang Nguyen Hoan
@date	Nov. 30, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_peer.h"

// Cross-arch app state. Port-specific state lives in port-private structs
// inside each ARM/<vendor>/<chip>/src/bt_app_<port>.cpp.
BtAppData_t g_BtAppData = {
	.State           = BTAPP_STATE_UNKNOWN,
	.AdvHdl          = 0xFF,		// port overrides during init
	.ConnLedPort     = -1,
	.ConnLedPin      = -1,
	.ConnLedActLevel = 0,
	.PeriphDevCnt    = 0,
	.CoexMode        = BTAPP_COEXMODE_NONE,
	.bExtAdv         = false,
	.bScan           = false,
	.bInitialized    = false,
	.AppDevice = {
		// Local device identity. Filled in by BtAppInit from BtAppCfg_t.
		// Every member is listed. The omitted ones would be zeroed anyway, but
		// leaving them out makes the compiler report an incomplete initialiser
		// on every build that includes this file.
		.Conn = {
			.Hdl            = BT_CONN_HDL_INVALID,	// unused for local
			.Role           = BTAPP_ROLE_PERIPHERAL,
			.PeerAddrType   = 0,
			.PeerAddr       = {0,},
			.OwnAddrType    = 0,
			.OwnAddr        = {0,},
			.MaxMtu         = 0,
			.pLongWrBuff    = NULL,
			.LongWrBuffSize = 0,
			.LongWrLen      = 0,
			.bIndCfmPending = false,
			.IndCfmTime     = 0,
			.NbCccd         = 0,
			.Cccd           = {},
			.Sec            = {},
		},
		.Name       = {0,},
		.Appearance = 0,
		.VendorId   = 0,
		.ProductId  = 0,
		.ProductVer = 0,
		.bIsLocal   = true,
		.bSecure    = false,
		.pHciDev    = NULL,
		.NbSrvc     = 0,
		.Services   = {},
		.Discovery  = {},
		.pIndChar = NULL,
		.bAttReqPending = false,
		.bAttTimedOut = false,
		.AttReqOpcode = 0,
		.AttRspOpcode = 0,
		.AttReqTime = 0,
		.TxPend     = {},
		.TxPendHead = 0,
		.TxPendCount = 0,
	},
};

bool BtInitialized(void)
{
	return g_BtAppData.State != BTAPP_STATE_UNKNOWN;
}

bool BtConnected(void)
{
	return BtPeerGetActive() != NULL;
}

bool isConnected(void)
{
	return BtPeerGetActive() != NULL;
}

// Largest number of links one broadcast walks in a single call. The peer pool
// is smaller than this on every port today, so the bound never truncates; it
// exists so the handle array stays a fixed local.
#ifndef BT_APP_LINK_MAX
#define BT_APP_LINK_MAX				8
#endif

// Store the value once for a broadcast, then let each link send from it. Doing
// it per link would rewrite the same bytes for every subscriber.
static bool BtAppStoreValue(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (pChar == nullptr)
	{
		return false;
	}

	if (DataLen > 0 && pData == nullptr)
	{
		return false;
	}

	// A zero-length value is a value. Skipping the store left the previous
	// contents readable after a zero-length notification, so a Read returned
	// something the client had just been told was gone.
	return BtGattCharSetValue(pChar, pData, DataLen);
}

// Resolve the one connected link. Returns false when none is up and when more
// than one is: there is no single link to mean then, and picking one is what
// the old active connection handle did.
static bool BtAppSoleLink(uint16_t *pHdl)
{
	// Ask for two. One means exactly one link; two means there is no sole one.
	uint16_t hdl[2];
	size_t n = BtPeerGetConnectedHandles(hdl, 2);

	if (n != 1)
	{
		return false;
	}

	*pHdl = hdl[0];

	return true;
}

// One implementation for every port. The SoftDevice and ST ports used to carry
// a copy each, all four the same apart from which handle they read.
__attribute__((weak)) bool BtAppNotifyConn(uint16_t ConnHdl, BtGattChar_t *pChar,
										   uint8_t *pData, uint16_t DataLen)
{
	if (BtAppStoreValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	// Through BtGattCharNotify so the packet is tracked in the TX pending ring
	// and TxCompleteCB fires on completion.
	return BtGattCharNotify(ConnHdl, pChar, pData, DataLen);
}

__attribute__((weak)) bool BtAppIndicateConn(uint16_t ConnHdl, BtGattChar_t *pChar,
											 uint8_t *pData, uint16_t DataLen)
{
	if (BtAppStoreValue(pChar, pData, DataLen) == false)
	{
		return false;
	}

	// Through BtGattCharIndicate so the indication is tracked: pending flag,
	// transaction timeout, and the TX pending ring.
	return BtGattCharIndicate(ConnHdl, pChar, pData, DataLen);
}

__attribute__((weak)) bool BtAppNotify(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	uint16_t hdl;

	if (BtAppSoleLink(&hdl) == false)
	{
		return false;
	}

	return BtAppNotifyConn(hdl, pChar, pData, DataLen);
}

__attribute__((weak)) bool BtAppIndicate(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	uint16_t hdl;

	if (BtAppSoleLink(&hdl) == false)
	{
		return false;
	}

	return BtAppIndicateConn(hdl, pChar, pData, DataLen);
}

__attribute__((weak)) int BtAppNotifyAll(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (BtAppStoreValue(pChar, pData, DataLen) == false)
	{
		return 0;
	}

	// Walk the peer pool itself. A fixed local array silently skipped every
	// link past its size, and the pool is sized by the caller's memory, not by
	// a constant here.
	uint16_t n = BtPeerCount();
	int sent = 0;

	for (uint16_t i = 0; i < n; i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr || pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}

		// A link whose client did not subscribe is skipped by
		// BtGattCharNotify, so it is not counted.
		if (BtGattCharNotify(pPeer->Conn.Hdl, pChar, pData, DataLen))
		{
			sent++;
		}
	}

	return sent;
}

__attribute__((weak)) int BtAppIndicateAll(BtGattChar_t *pChar, uint8_t *pData, uint16_t DataLen)
{
	if (BtAppStoreValue(pChar, pData, DataLen) == false)
	{
		return 0;
	}

	// Walk the peer pool itself. A fixed local array silently skipped every
	// link past its size, and the pool is sized by the caller's memory, not by
	// a constant here.
	uint16_t n = BtPeerCount();
	int sent = 0;

	for (uint16_t i = 0; i < n; i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr || pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}

		// A link that already has an indication outstanding is refused until
		// its confirmation arrives, so the count can be below the number of
		// subscribed links.
		if (BtGattCharIndicate(pPeer->Conn.Hdl, pChar, pData, DataLen))
		{
			sent++;
		}
	}

	return sent;
}

__attribute__((weak)) void BtAppDisconnect(void)
{
	uint16_t hdl;

	if (BtAppSoleLink(&hdl) == false)
	{
		return;
	}

	BtAppDisconnectConn(hdl);
}

__attribute__((weak)) int BtAppDisconnectAll(void)
{
	// Walk the peer pool itself. A fixed local array silently skipped every
	// link past its size, and the pool is sized by the caller's memory, not by
	// a constant here.
	uint16_t n = BtPeerCount();
	int dropped = 0;

	for (uint16_t i = 0; i < n; i++)
	{
		BtDevice_t *pPeer = BtPeerSlot(i);
		if (pPeer == nullptr || pPeer->Conn.Hdl == BT_CONN_HDL_INVALID)
		{
			continue;
		}

		BtAppDisconnectConn(pPeer->Conn.Hdl);
		dropped++;
	}

	return dropped;
}

// --- BtDevice queries (work on any BtDevice_t, local or remote) ---

int BtDeviceFindService(BtDevice_t * const pDev, uint16_t Uuid)
{
	if (pDev == NULL)
	{
		return -1;
	}

	for (int i = 0; i < pDev->NbSrvc; i++)
	{
		if (pDev->Services[i].srv_uuid.Uuid == Uuid)
		{
			return i;
		}
	}
	return -1;
}

int BtDeviceFindCharacteristic(BtDevice_t * const pDev, int SrvcIdx, uint16_t Uuid)
{
	if (pDev == NULL || SrvcIdx < 0 || SrvcIdx >= pDev->NbSrvc)
	{
		return -1;
	}

	BtGattDBSrvc_t *pSrvc = &pDev->Services[SrvcIdx];
	for (int i = 0; i < pSrvc->char_count; i++)
	{
		if (pSrvc->characteristics[i].characteristic.uuid.Uuid == Uuid)
		{
			return i;
		}
	}
	return -1;
}

// Weak default for the discovery-complete callback. App overrides.
__attribute__((weak)) void BtDeviceDiscovered(BtDevice_t *pDev)
{
	(void)pDev;
}

void BtAppConnLedOff(void)
{
	if (g_BtAppData.ConnLedPort < 0 || g_BtAppData.ConnLedPin < 0)
		return;

	if (g_BtAppData.ConnLedActLevel)
	{
		//IOPinClear(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
	else
	{
		//IOPinSet(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
}

void BtAppConnLedOn(void)
{
	if (g_BtAppData.ConnLedPort < 0 || g_BtAppData.ConnLedPin < 0)
		return;

	if (g_BtAppData.ConnLedActLevel)
	{
		//IOPinSet(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
	else
	{
		//IOPinClear(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
}
