/**-------------------------------------------------------------------------
@example	ble_periodic_advertiser.cpp

@brief	BLE periodic advertising broadcaster

This demo brings up a non-connectable extended advertising set and attaches a
periodic advertising train to it. The train carries a counter that increments
once a second, so a scanner that synchronises to the train sees the value move
and can prove the payload arrives intact.

Periodic advertising rides on an extended set. The set is made extended by
giving it more advertising data than a legacy PDU holds, and it is made
non-connectable, non-scannable by the broadcaster role. Those are the
properties a controller requires before it accepts a periodic train on the
set, so the ordinary advertising path is configured first and the periodic
layer is added on top.

Periodic advertising is an IOsonata generic HCI host feature and needs the
SoftDevice Controller (SDC) build, which is the link layer plus HCI with the
host stack on top. No Nordic SoftDevice exposes periodic advertising: the s132
and s140 on nRF52 and the s145 on nRF54 all carry extended advertising but no
periodic advertising API, so there is nothing to wrap a SoftDevice port on.
Every target here therefore ships SDC configurations only. Build Debug_SDC or
Release_SDC.

Set PADV_ENABLE_PAWR to 1 to configure a Periodic Advertising with Responses
train instead of a plain one. PAwR needs a controller that has the feature and
a response set reserved for it, which is the nRF54L15 build here; the nRF52
controllers reserve none and the plain periodic path is the one to use on
them.

@author	Hoang Nguyen Hoan
@date	Aug. 18, 2026

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

#include "istddef.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "syslog.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_padv.h"
#include "bluetooth/bt_hci_ctlr.h"

#include "board.h"

// Route the demo trace to SysLog, the same way the stack sources do. SysLog is
// bound to the UART in main when UART_PINS is defined; without it these calls
// go nowhere, which is fine for a headless board.
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)

// Uncomment this to set custom board oscillator
// Default library setting is :
// #define MCU_OSC	{{OSC_TYPE_XTAL, 32000000, 20}, {OSC_TYPE_XTAL, 32768, 20}, false}

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

// Configure a Periodic Advertising with Responses train instead of a plain
// periodic train. Off by default: the plain path runs on every SDC target,
// the PAwR path needs a controller with the feature and a reserved response
// set, which is BOARD_PAWR_ADV_COUNT below.
#ifndef PADV_ENABLE_PAWR
#define PADV_ENABLE_PAWR		0
#endif

// Response sets the controller reserves for PAwR, from the board. A controller
// without the feature refuses the reservation, so this is 0 on the nRF52
// targets and the plain periodic path is the one to build there.
#ifndef BOARD_PAWR_ADV_COUNT
#define BOARD_PAWR_ADV_COUNT	0
#endif

#define DEVICE_NAME				"IOsonata PADV"

// The advertising set the stack builds. bt_adv_hci.cpp builds handle 0 and no
// other, and the periodic train attaches to that set, so this is the handle
// every periodic command names.
#define APP_ADV_HDL				0

#define APP_ADV_INTERVAL_MSEC	100

// Periodic advertising interval, 1.25 ms units. 0x50 to 0x60 is 100 ms to
// 120 ms, slow enough to watch on a scanner and fast enough to lock quickly.
#define APP_PADV_INTERVAL_MIN	0x0050
#define APP_PADV_INTERVAL_MAX	0x0060

static const IOPinCfg_t s_Leds[] = LED_PINS_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

#ifdef UART_PINS
#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

static const IOPinCfg_t s_UartPins[] = UART_PINS;

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
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

UART g_Uart;
#endif

// Advertising manufacturer data for the extended set. It is padded past the
// legacy 31 octet limit on purpose: a legacy set cannot carry a periodic
// train, so the size forces BtAdvEncode to build an extended set. The content
// is only a marker a scanner reads before it synchronises; the moving value
// is in the periodic data below.
static const uint8_t s_ExtManData[] =
	"IOsonata periodic advertising demo marker payload";

#if PADV_ENABLE_PAWR == 0
// Periodic advertising payload the train carries. A four byte counter a timer
// increments once a second, so a synchronised scanner sees it move. It is
// wrapped in a manufacturer specific AD structure so a scanner parses it as
// manufacturer data rather than four loose octets: length, type 0xFF, the
// two byte company id, then the counter.
static uint32_t g_PeriodicCnt = 0;

#define PERIODIC_AD_SIZE		8		// 1 len + 1 type + 2 company id + 4 counter
static uint8_t s_PeriodicAd[PERIODIC_AD_SIZE];

static void PeriodicAdBuild(void)
{
	uint16_t vid = ISYST_BLUETOOTH_ID;

	s_PeriodicAd[0] = PERIODIC_AD_SIZE - 1;					// Octets after the length byte
	s_PeriodicAd[1] = BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA;	// 0xFF
	memcpy(&s_PeriodicAd[2], &vid, sizeof(vid));
	memcpy(&s_PeriodicAd[4], &g_PeriodicCnt, sizeof(g_PeriodicCnt));
}
#endif

static const TimerCfg_t s_TimerCfg = {
	.DevNo = TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = 6,
};

Timer g_Timer;

const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_BROADCASTER,			// Non-connectable, non-scannable set
	.PeriphDevMax = 0,
	.CentralDevMax = 0,
	.pDevName = (char*)DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 1,
	.Appearance = BT_APPEAR_COMPUTER_WEARABLE,
	.pDevInfo = nullptr,
	.pAdvManData = (uint8_t*)s_ExtManData,	// Sized to force an extended set
	.AdvManDataLen = sizeof(s_ExtManData),
	.pSrManData = nullptr,
	.SrManDataLen = 0,
	.AdvInterval = APP_ADV_INTERVAL_MSEC,
	.AdvTimeout = 0,						// Advertise continuously
	.TxPower = 0,
	// Reserve one periodic advertising set. Without it every periodic command
	// is refused whatever the train looks like. The response set count comes
	// from the board because PAwR is not on every controller.
	.PeriodicAdvCount = 1,
	// Reserve PAwR response resources only for the PAwR build. A plain periodic
	// train needs none, and reserving them makes the controller size the
	// periodic advertising response buffers: the nRF54 SDC refuses that
	// configuration (SDC_CFG_TYPE_PERIODIC_ADV_RSP_BUFFER_CFG) when nothing asks
	// to answer subevents, which fails BtAppInit. nRF52 never hit this because
	// its board sets BOARD_PAWR_ADV_COUNT to 0.
	.PawrAdvCount = PADV_ENABLE_PAWR ? BOARD_PAWR_ADV_COUNT : 0,
};

#if PADV_ENABLE_PAWR

// PAwR shape, small enough to fit the RAM budget of the smallest target that
// has the feature while still giving a scanner more than one subevent.
#define APP_PAWR_SUBEVENT_CNT		4
#define APP_PAWR_SUBEVENT_DATA		32
#define APP_PAWR_RSP_SLOT_CNT		4
#define APP_PAWR_SUBEVT_INTERVAL	0x20
#define APP_PAWR_RSP_SLOT_DELAY		0x08
#define APP_PAWR_RSP_SLOT_SPACING	0x10

static uint8_t s_PawrData[APP_PAWR_SUBEVENT_CNT][APP_PAWR_SUBEVENT_DATA];
static uint8_t s_PawrLen[APP_PAWR_SUBEVENT_CNT];
static bool s_PawrReady = false;

// The controller asks for subevent data every periodic event. Answer every
// requested subevent from the retained buffers so the train keeps advertising
// the same payload each rotation. The requested set wraps at the subevent
// count, so BtPadvSubeventOfRequest resolves the indices.
void BtPadvSubeventDataRequest(uint8_t AdvHdl, uint8_t Start, uint8_t Count)
{
	if (s_PawrReady == false || Count == 0)
	{
		return;
	}

	if (Count > APP_PAWR_SUBEVENT_CNT)
	{
		Count = APP_PAWR_SUBEVENT_CNT;
	}

	BtPadvSubeventData_t sub[APP_PAWR_SUBEVENT_CNT];

	for (uint8_t i = 0; i < Count; i++)
	{
		uint8_t subevent = BtPadvSubeventOfRequest(Start, i,
			APP_PAWR_SUBEVENT_CNT);

		sub[i].Subevent = subevent;
		sub[i].RspSlotStart = 0;
		sub[i].RspSlotCount = APP_PAWR_RSP_SLOT_CNT;
		sub[i].DataLen = s_PawrLen[subevent];
		sub[i].pData = s_PawrData[subevent];
	}

	BtPadvSubeventDataSet(AdvHdl, sub, Count);
}

static bool PeriodicTrainInit(void)
{
	BtPadvCfg_t cfg = {};
	cfg.AdvHdl = APP_ADV_HDL;
	cfg.IntervalMin = APP_PADV_INTERVAL_MIN;
	cfg.IntervalMax = APP_PADV_INTERVAL_MAX;
	cfg.Properties = 0;						// No Include TxPower, not every controller has it
	cfg.NbSubevents = APP_PAWR_SUBEVENT_CNT;
	cfg.SubeventInterval = APP_PAWR_SUBEVT_INTERVAL;
	cfg.RspSlotDelay = APP_PAWR_RSP_SLOT_DELAY;
	cfg.RspSlotSpacing = APP_PAWR_RSP_SLOT_SPACING;
	cfg.NbRspSlots = APP_PAWR_RSP_SLOT_CNT;

	if (BtPadvInit(&cfg) == false)
	{
		DEBUG_PRINTF("PAwR init failed\r\n");
		return false;
	}

	// Give every subevent a payload that names itself, so a scanner reading
	// one subevent can tell which it landed in.
	for (uint8_t i = 0; i < APP_PAWR_SUBEVENT_CNT; i++)
	{
		s_PawrData[i][0] = 'S';
		s_PawrData[i][1] = i;
		s_PawrLen[i] = 2;
	}

	s_PawrReady = true;

	return BtPadvStart(APP_ADV_HDL);
}

#else

static bool PeriodicTrainInit(void)
{
	BtPadvCfg_t cfg = {};
	cfg.AdvHdl = APP_ADV_HDL;
	cfg.IntervalMin = APP_PADV_INTERVAL_MIN;
	cfg.IntervalMax = APP_PADV_INTERVAL_MAX;
	cfg.Properties = 0;						// No Include TxPower, not every controller has it
	cfg.NbSubevents = 0;					// Plain periodic train, no responses

	if (BtPadvInit(&cfg) == false)
	{
		DEBUG_PRINTF("Periodic init failed\r\n");
		return false;
	}

	PeriodicAdBuild();

	if (BtPadvDataSet(APP_ADV_HDL, s_PeriodicAd, sizeof(s_PeriodicAd)) == false)
	{
		DEBUG_PRINTF("Periodic data set failed\r\n");
		return false;
	}

	return BtPadvStart(APP_ADV_HDL);
}

#endif

// One second tick. Increment the counter and push it into the periodic data.
// A complete single command update is accepted while the train is enabled, and
// the payload is four octets, so it always fits one command.
void BtAppTimerHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	if (TrigNo != 0)
	{
		return;
	}

	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);

#if PADV_ENABLE_PAWR == 0
	g_PeriodicCnt++;
	PeriodicAdBuild();
	bool ok = BtPadvDataSet(APP_ADV_HDL, s_PeriodicAd, sizeof(s_PeriodicAd));
#ifdef UART_PINS
	// Diagnostic: confirm the tick fires each second and the data update is
	// accepted. A counter that stops advancing at the receiver means either
	// this stops printing (timer fired once) or ok goes 0 (update refused).
	g_Uart.printf("tick cnt=%lu set=%d\r\n", (unsigned long)g_PeriodicCnt, ok ? 1 : 0);
#endif
#endif

	// Re-arm the tick. On nRF54L the GRTC based timer's continuous mode relies
	// on the CCADD hardware auto-reload, which does not repeat here (the GRTC is
	// shared with MPSL), so a continuous trigger fires once. Re-arming from the
	// handler makes the tick repeat regardless. Harmless where continuous
	// already works: it just re-sets the same one second period.
	g_Timer.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS,
		BtAppTimerHandler);
}

int main()
{
	IOPinCfg(s_Leds, s_NbLeds);

	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinClear(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

#ifdef UART_PINS
	g_Uart.Init(s_UartCfg);
	g_Uart.printf("BlePeriodicAdvertiser\r\n");

	// Enable SysLog output through the UART so library DEBUG_PRINTF appears.
	SysLogGetInstance()->Init(g_Uart);
#endif

	// Build the advertising set. This configures an extended, non-connectable
	// set on handle 0 and leaves it disabled; BtAppRun enables it.
	if (BtAppInit(&s_BtAppCfg) == false)
	{
#ifdef UART_PINS
		// On the SDC build the controller bring-up records where it stopped.
		// stage names the step (1 MPSL init, 2 controller init, 3 cfg set,
		// 4 mem pool, 5 arbiter, 6 controller enable), value is its code or the
		// resource tag / size that step reported.
		g_Uart.printf("BtAppInit failed: stage=%d value=%d\r\n",
			(int)BtHciCtlrErrorGet(), (int)BtHciCtlrErrorValueGet());
#endif
		while (1);
	}

	// Attach the periodic train to the set. The set exists now, so the
	// parameters and data commands are accepted; the train reaches the air
	// once BtAppRun enables the set.
	if (PeriodicTrainInit() == false)
	{
#ifdef UART_PINS
		g_Uart.printf("Periodic train init failed\r\n");
#endif
	}
	else
	{
#ifdef UART_PINS
		g_Uart.printf("Periodic train running\r\n");
#endif
	}

	g_Timer.Init(s_TimerCfg);
	g_Timer.EnableTimerTrigger(0, 1000UL, TIMER_TRIG_TYPE_CONTINUOUS,
		BtAppTimerHandler);

	BtAppRun();

	return 0;
}
