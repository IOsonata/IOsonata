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
#include "coredev/uart.h"
#include "board.h"

// Uncomment this to set custom board oscillator
// Default library setting is :
// #define MCU_OSC	{{OSC_TYPE_XTAL, 32000000, 20}, {OSC_TYPE_XTAL, 32768, 20}, false}

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

#ifdef UART_PINS
static const IOPinCfg_t s_UartPins[] = UART_PINS;

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,//UARTFIFOSIZE,
	.pRxMem = NULL,//s_UartRxFifo,
	.TxMemSize = 0,//UARTFIFOSIZE,//FIFOSIZE,
	.pTxMem = NULL,//s_UartTxFifo,//g_TxBuff,
	.bDMAMode = true,
};

UART g_Uart;
#endif
//#define EXTADV		// Uncomment to enable extended advertisement

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
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = 6,
};

#if 0
Timer g_Timer;

void TimerTrigEvtHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	IOPinToggle(s_Leds[3].PortNo, s_Leds[3].PinNo);

	g_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
}

void BtAppInitUserData()
{
	IOPinSet(s_Leds[2].PortNo, s_Leds[2].PinNo);
	g_Timer.Init(s_TimerCfg);

	g_Timer.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS, TimerTrigEvtHandler);
}

#else
void BtAppAdvTimeoutHandler()
{
	g_AdvCnt++;

	BtAppAdvManDataSet((uint8_t*)&g_AdvCnt, sizeof(g_AdvCnt), NULL, 0);
}
#endif

/* NOTE 1: Build DFU zip file
 * SoftDevice v7.2.0
 * 		nrfutil pkg generate --hw-version 52 --sd-req 0x0101 --application-version 0x0 --application ./BleAdvertiser.hex --key-file ../../../../../../src/iosonata_dfukey.pem BleAdvertiser_package.zip
 *
 * SoftDevice v7.3.0
 * 		nrfutil pkg generate --hw-version 52 --sd-req 0x0124 --application-version 0x0 --application ./BleAdvertiser.hex --key-file ../../../../../../src/iosonata_dfukey.pem BleAdvertiser_package.zip
 */

int main()
{
	// Configure Leds
	IOPinCfg(s_Leds, s_NbLeds);

	IOPinToggle(0,4);

#ifdef UART_PINS
	bool res = g_Uart.Init(s_UartCfg);

	g_Uart.printf("BleAdvertiser\n\r");
#endif

	// Clear all leds
	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinClear(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	IOPinSet(s_Leds[0].PortNo, s_Leds[0].PinNo);

	BtAppInit(&s_BtAppCfg);

	IOPinSet(s_Leds[1].PortNo, s_Leds[1].PinNo);

	BtAppRun();

	return 0;
}

