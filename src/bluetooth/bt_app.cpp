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

#include "iopinctrl.h"
#include "bluetooth/bt_app.h"

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
		.Name       = {0,},
		.Addr       = {0,},
		.AddrType   = 0,
		.Role       = BTAPP_ROLE_PERIPHERAL,
		.Appearance = 0,
		.ConnHdl    = BT_CONN_HDL_INVALID,	// unused for local
		.MaxMtu     = 0,
		.VendorId   = 0,
		.ProductId  = 0,
		.ProductVer = 0,
		.bIsLocal   = true,
		.bSecure    = false,
		.pHciDev    = NULL,
		.NbSrvc     = 0,
	},
};

// Peer device pool. Slots are free when ConnHdl == BT_CONN_HDL_INVALID.
// Initialized lazily by BtAppPeerAlloc / BtAppGetActivePeer below; ports may
// also call BtAppPeerPoolInit explicitly from BtAppInit for clarity.
BtDevice_t g_BtPeerDevice[CFG_BT_PEER_MAX];
static bool s_PeerPoolReady = false;

void BtAppPeerPoolInit(void)
{
	for (int i = 0; i < CFG_BT_PEER_MAX; i++)
	{
		memset(&g_BtPeerDevice[i], 0, sizeof(g_BtPeerDevice[i]));
		g_BtPeerDevice[i].ConnHdl  = BT_CONN_HDL_INVALID;
		g_BtPeerDevice[i].bIsLocal = false;
	}
	s_PeerPoolReady = true;
}

static inline void EnsurePeerPool(void)
{
	if (!s_PeerPoolReady)
	{
		BtAppPeerPoolInit();
	}
}

bool BtInitialized(void)
{
	return g_BtAppData.State != BTAPP_STATE_UNKNOWN;
}

bool BtConnected(void)
{
	return BtAppGetActivePeer() != NULL;
}

bool isConnected(void)
{
	return BtAppGetActivePeer() != NULL;
}

uint16_t BtAppGetConnHandle(void)
{
	BtDevice_t *p = BtAppGetActivePeer();
	return p ? p->ConnHdl : BT_CONN_HDL_INVALID;
}

// --- Peer pool helpers ---

BtDevice_t * BtAppPeerAlloc(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID)
	{
		return NULL;
	}
	EnsurePeerPool();

	for (int i = 0; i < CFG_BT_PEER_MAX; i++)
	{
		if (g_BtPeerDevice[i].ConnHdl == BT_CONN_HDL_INVALID)
		{
			BtDevice_t *p = &g_BtPeerDevice[i];
			memset(p, 0, sizeof(*p));
			p->ConnHdl  = ConnHdl;
			p->bIsLocal = false;
			return p;
		}
	}
	return NULL;
}

BtDevice_t * BtAppPeerFindByHdl(uint16_t ConnHdl)
{
	if (ConnHdl == BT_CONN_HDL_INVALID)
	{
		return NULL;
	}
	EnsurePeerPool();

	for (int i = 0; i < CFG_BT_PEER_MAX; i++)
	{
		if (g_BtPeerDevice[i].ConnHdl == ConnHdl)
		{
			return &g_BtPeerDevice[i];
		}
	}
	return NULL;
}

void BtAppPeerFree(BtDevice_t *pPeer)
{
	if (pPeer == NULL)
	{
		return;
	}
	pPeer->ConnHdl = BT_CONN_HDL_INVALID;
}

BtDevice_t * BtAppGetActivePeer(void)
{
	EnsurePeerPool();

	for (int i = 0; i < CFG_BT_PEER_MAX; i++)
	{
		if (g_BtPeerDevice[i].ConnHdl != BT_CONN_HDL_INVALID)
		{
			return &g_BtPeerDevice[i];
		}
	}
	return NULL;
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
		IOPinClear(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
	else
	{
		IOPinSet(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
}

void BtAppConnLedOn(void)
{
	if (g_BtAppData.ConnLedPort < 0 || g_BtAppData.ConnLedPin < 0)
		return;

	if (g_BtAppData.ConnLedActLevel)
	{
		IOPinSet(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
	else
	{
		IOPinClear(g_BtAppData.ConnLedPort, g_BtAppData.ConnLedPin);
	}
}
