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
#include "coredev/timer.h"

#include "bluetooth/bt_app.h"
//#ifndef NRFXLIB_SDC
//#include "ble_app_nrf5.h"
//#endif
#include "bluetooth/bt_appearance.h"
#include "iopinctrl.h"
#include "coredev/system_core_clock.h"

// Uncomment this to set custom board oscillator
// Default library setting is :
// #define MCUOSC	{{OSC_TYPE_XTAL, 32000000, 20}, {OSC_TYPE_XTAL, 32768, 20}, false}

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#define EXTADV		// Uncomment to enable extended advertisement

#ifdef EXTADV
#define DEVICE_NAME                     "Advertising extra long long name"
#else
#define DEVICE_NAME                     "Advertising"
#endif

#define APP_ADV_INTERVAL_MSEC       50
#define APP_ADV_TIMEOUT_MSEC      	0

uint32_t g_AdvCnt = 0;
uint8_t g_AdvLong[] = "1234567890abcdefghijklmnopqrstuvwxyz`!@#$%^&*()_+";

const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_BROADCASTER,
	.CentLinkCount = 0,						// Number of central link
	.PeriLinkCount = 1,						// Number of peripheral link
	.pDevName = (char*)DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,			// PnP Bluetooth/USB vendor id
	.Appearance = BT_APPEAR_COMPUTER_WEARABLE,
#ifdef EXTADV
	.bExtAdv = true,						// Enable extended advertising
	.pAdvManData  = (uint8_t*)&g_AdvLong,   // Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_AdvLong),     // Length of manufacture specific data
#else
	.bExtAdv = false,						// Legacy advertising
	.pAdvManData  = (uint8_t*)&g_AdvCnt,   	// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_AdvCnt),      // Length of manufacture specific data
#endif
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.AdvInterval = APP_ADV_INTERVAL_MSEC,   // Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT_MSEC,		// Advertising timeout in msec (not available yet with SDC implementation
	.TxPower = 0,							// Tx power in dbm
};

static const TimerCfg_t s_TimerCfg = {
	.DevNo = 2,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = 6,
};

#if 1
Timer g_Timer;

void TimerTrigEvtHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	g_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
}

void BtAppInitUserData()
{
	g_Timer.Init(s_TimerCfg);

	g_Timer.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS, TimerTrigEvtHandler);
}

#else
void BleAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BleAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
	BleAppAdvStart();
}
#endif

int main()
{
    BtAppInit(&s_BtAppCfg);

    BtAppRun();

	return 0;
}

