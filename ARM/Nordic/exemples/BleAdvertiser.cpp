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

#include "app_util.h"

#include "istddef.h"
#include "ble_app.h"
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

const BLEAPP_CFG s_BleAppCfg = {
#if 0
	{
		// Clock config nrf_clock_lf_cfg_t
#ifdef IMM_NRF51822
		NRF_CLOCK_LF_SRC_RC,	// Source RC
		1, 1, 0
#else
		NRF_CLOCK_LF_SRC_XTAL,	// Source 32KHz XTAL
		//NRF_CLOCK_LF_SRC_RC,
#ifdef NRF51
		0, 0, NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM
#else
		0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM
#endif
#endif
	},
#endif
	.Role = BLEAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0,		// Number of central link
	.PeriLinkCount = 1,		// Number of peripheral link
	//BLEAPP_MODE_NOCONNECT,	// Connectionless beacon type
	DEVICE_NAME,			// Device name
	ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	1,                      // PnP Product ID
	0,						// Pnp prod version
	false,					// Enable device information service (DIS)
	NULL,					// Pointer device info descriptor
	BLEADV_TYPE_ADV_NONCONN_IND,
	(uint8_t*)&g_AdvCnt,   	// Manufacture specific data to advertise
	sizeof(g_AdvCnt),      	// Length of manufacture specific data
	NULL,
	0,
	BLEAPP_SECTYPE_NONE,    // Secure connection type
	BLEAPP_SECEXCHG_NONE,   // Security key exchange
	NULL,      				// Service uuids to advertise
	0, 						// Total number of uuids
	APP_ADV_INTERVAL_MSEC,       	// Advertising interval in msec
	APP_ADV_TIMEOUT_MSEC,	// Advertising timeout in sec
	0,							// Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	0,		// Min. connection interval
	0,		// Max. connection interval
	-1,		// Led port nuber
	-1,		// Led pin number
	0,
	0,		// Tx power
	NULL	// RTOS Softdevice handler
};

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
}

void BleAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BleAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
	BleAppAdvStart(BLEAPP_ADVMODE_FAST);
}

int main()
{
	unsigned x, y, z;
	int8_t a, b, c;

	z = min(x, y);
	c = min(a, b);

    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);

    BleAppRun();

	return 0;
}

