/**-------------------------------------------------------------------------
@file	bt_app.cpp

@brief	Generic Bluetooth application firmware type.


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

#include "bluetooth/bt_host.h"
#include "bluetooth/bt_app.h"

#pragma pack(push, 4)

typedef struct __Bt_App_Data {
	BTDEV_ROLE Role;
	uint16_t ConnHdl;	// BLE connection handle
	int ConnLedPort;
	int ConnLedPin;
	uint8_t ConnLedActLevel;
	int PeriphDevCnt;
	uint32_t (*SDEvtHandler)(void) ;
	int MaxMtu;
	bool bSecure;
	bool bScan;
    bool bInitialized;
	BTDEV_COEXMODE CoexMode;
} BtAppData_t;

#pragma pack(pop)

static BtAppData_t s_BtAppData = {
	BTDEV_ROLE_PERIPHERAL, 0xFFFF, -1, -1, 0,
};

static BtHostDev_t s_BtHostDev;

bool BtAppInit(const BtHostCfg_t *pCfg)
{
	BtHostInit(&s_BtHostDev, pCfg);

	return true;
}

void BtAppRun()
{
	if (s_BtAppData.bInitialized == false)
	{
		return;
	}

	if (s_BtAppData.Role & (BTDEV_ROLE_PERIPHERAL | BTDEV_ROLE_BROADCASTER))
	{
		BtDevAdvStart();
	}

}
