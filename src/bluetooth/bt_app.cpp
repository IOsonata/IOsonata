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
		.Conn = {
			.Hdl          = BT_CONN_HDL_INVALID,	// unused for local
			.Role         = BTAPP_ROLE_PERIPHERAL,
			.PeerAddrType = 0,
			.PeerAddr     = {0,},
			.MaxMtu       = 0,
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

uint16_t BtAppGetConnHandle(void)
{
	BtDevice_t *p = BtPeerGetActive();
	return p ? p->Conn.Hdl : BT_CONN_HDL_INVALID;
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
