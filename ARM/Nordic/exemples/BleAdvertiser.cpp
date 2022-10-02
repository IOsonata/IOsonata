/**-------------------------------------------------------------------------
@example	BleAdvertiser.cpp

@brief	BLE non-connectable advertiser

This demo show how to advertise an incremental counter in the manufacturer
specific data.  The counter increments every second.

@author	Hoang Nguyen Hoan
@date	Dec. 19, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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


#include "istddef.h"

#include "bluetooth/ble_app.h"
#ifndef NRFXLIB_SDC
#include "ble_app_nrf5.h"
#endif
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"

//#define MCUOSC	{{OSC_TYPE_XTAL, 32000000, 20}, {OSC_TYPE_RC, }, false}

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#define DEVICE_NAME                     "Advertiser"

#define APP_ADV_INTERVAL_MSEC       100//MSEC_TO_UNITS(100, UNIT_0_625_MS)
#define APP_ADV_TIMEOUT_MSEC      	1000//MSEC_TO_UNITS(1000, UNIT_10_MS)

uint32_t g_AdvCnt = 0;

const BleAppCfg_t s_BleAppCfg = {
	.Role = BLEAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0,		// Number of central link
	.PeriLinkCount = 1,		// Number of peripheral link
	.pDevName = DEVICE_NAME,			// Device name
	.VendorID = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,                      // PnP Product ID
	.ProductVer = 0,						// Pnp prod version
//	.bEnDevInfoService = false,					// Enable device information service (DIS)
	.pDevDesc = NULL,					// Pointer device info descriptor
	.AdvType = BLEADV_TYPE_ADV_NONCONN_IND,
	.pAdvManData  = (uint8_t*)&g_AdvCnt,   	// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_AdvCnt),      	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BLEAPP_SECEXCHG_NONE,   // Security key exchange
	.pAdvUuids = NULL,      				// Service uuids to advertise
	.NbAdvUuid = 0, 						// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL_MSEC,       	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT_MSEC,	// Advertising timeout in sec
	.AdvSlowInterval = 0,							// Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = 0,		// Min. connection interval
	.ConnIntervalMax = 0,		// Max. connection interval
	.ConnLedPort = -1,		// Led port nuber
	.ConnLedPin = -1,		// Led pin number
	.ConnLedActLevel = 0,
	.TxPower = 0,		// Tx power
	.SDEvtHandler = NULL,	// RTOS Softdevice handler
	.MaxMtu = 0,						//!< Max MTU size or 0 for default
};


void BleAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BleAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
	BleAppAdvStart(BLEAPP_ADVMODE_FAST);
}

int main()
{
    BleAppInit((const BleAppCfg_t *)&s_BleAppCfg);//, true);

    BleAppRun();

	return 0;
}

