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

#include "iopinctrl.h"
#include "bluetooth/bt_app.h"

// Cross-arch app state. Port-specific state lives in port-private structs
// inside each ARM/<vendor>/<chip>/src/bt_app_<port>.cpp.
BtAppData_t g_BtAppData = {
	BTAPP_STATE_UNKNOWN,			// State
	BTAPP_ROLE_PERIPHERAL,			// Role
	0xFF,							// AdvHdl - port overrides during init
	BT_CONN_HDL_INVALID,			// ConnHdl
	-1,								// ConnLedPort
	-1,								// ConnLedPin
	0,								// ConnLedActLevel
};

bool BtInitialized(void)
{
	return g_BtAppData.State != BTAPP_STATE_UNKNOWN;
}

bool BtConnected(void)
{
	return g_BtAppData.ConnHdl != BT_CONN_HDL_INVALID;
}

bool isConnected(void)
{
	return g_BtAppData.ConnHdl != BT_CONN_HDL_INVALID;
}

uint16_t BtAppGetConnHandle(void)
{
	return g_BtAppData.ConnHdl;
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
