/**-------------------------------------------------------------------------
@example	ble_periodic_sync.cpp

@brief	BLE periodic advertising synchronization, receiving side

The receiving half of ble_periodic_advertiser.cpp. It scans, finds the
advertiser by name, synchronizes to its periodic advertising train, and prints
the four octet counter the train carries, so a second board proves the train is
on air and the payload is moving. This is the interop test a phone cannot do,
since a phone stack exposes no periodic advertising synchronization.

Periodic advertising synchronization is an IOsonata generic HCI host feature
and needs the SoftDevice Controller (SDC) build. No Nordic SoftDevice exposes
it, so every project here ships SDC configurations only, the same as the
advertiser. Build Debug_SDC or Release_SDC.

Flow: scan extended and non-connectable, match the advertiser by its device
name in the extended advertising report, then BtPsyncCreate names the
advertiser by address with the SID the advertiser uses (0). The controller
synchronizes while scanning is enabled and reports it through BtPsyncEstablished.
BtPsyncReport then delivers each periodic advertisement, from which the counter
is read.

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
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "syslog.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_gap.h"
#include "bluetooth/bt_appearance.h"
#include "bluetooth/bt_psync.h"

#include "board.h"

// Uncomment this to set custom board oscillator
// #define MCU_OSC	{{OSC_TYPE_XTAL, 32000000, 20}, {OSC_TYPE_XTAL, 32768, 20}, false}
#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

// The advertiser this demo synchronizes to. Its name is what the scan matches
// and its SID is what BtPsyncCreate names, both from ble_periodic_advertiser.
#define TARGET_NAME				"IOsonata PADV"
#define TARGET_SID				0

#define DEVICE_NAME				"IOsonata PSYNC"

#define SCAN_INTERVAL_MSEC		100
#define SCAN_WINDOW_MSEC		50

// Sync timeout in 10 ms units. 1000 is 10 s, long enough that a missed event
// or two does not drop the train.
#define SYNC_TIMEOUT			1000

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
#define OUT(...)				g_Uart.printf(__VA_ARGS__)
#else
#define OUT(...)
#endif

const BtAppCfg_t s_BtAppCfg = {
	.Role = BTAPP_ROLE_OBSERVER,			// Scan only, no connection
	.PeriphDevMax = 0,
	.CentralDevMax = 0,
	.pDevName = (char*)DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 1,
	.Appearance = BT_APPEAR_UNKNOWN_GENERIC,
	.pDevInfo = nullptr,
	.pAdvManData = nullptr,
	.AdvManDataLen = 0,
	.pSrManData = nullptr,
	.SrManDataLen = 0,
	.AdvInterval = 0,
	.AdvTimeout = 0,
	.TxPower = 0,
	// One train synchronized to at a time. Without it the controller reserves
	// no sync context and BtPsyncCreate is refused for want of one.
	.PeriodicSyncCount = 1,
};

// Extended passive scan. Passive because the advertiser is non-connectable
// non-scannable, so an active scan gains nothing, and extended because the
// periodic train rides an extended set whose AUX_ADV_IND carries the SyncInfo
// the controller needs to synchronize. No service filter: the broadcaster
// advertises no service UUID, so it is matched by name in the report.
static const BtGapScanCfg_t s_ScanCfg = {
	.Type = BTSCAN_TYPE_PASSIVE_EXT,
	.Param = {
		.Phy = BT_GAP_PHY_1MBITS,
		.OwnAddrType = BTADDR_TYPE_RAND,
		.Interval = SCAN_INTERVAL_MSEC,
		.Duration = SCAN_WINDOW_MSEC,
		.Timeout = 0,						// Never stop scanning
	},
	.BaseUid = {0},
	.ServUid = 0,
};

// One create-sync attempt at a time. BtAppScanReport fires for every report,
// and Create Sync is refused while one is pending, so the attempt is guarded.
static volatile bool s_SyncRequested = false;
static volatile bool s_Synced = false;
static uint32_t g_LastCnt = 0;
static bool g_bFirst = true;

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6],
					 size_t AdvLen, uint8_t *pAdvData)
{
	if (s_SyncRequested || s_Synced || AdvLen == 0)
	{
		return true;
	}

	char name[32];
	size_t l = BtAdvDataGetDevName(pAdvData, AdvLen, name, sizeof(name));

	if (l == 0)
	{
		return true;
	}

	if (l >= sizeof(name))
	{
		l = sizeof(name) - 1;
	}
	name[l] = 0;

	if (strstr(name, TARGET_NAME) == nullptr)
	{
		return true;
	}

	OUT("Found %s %02x:%02x:%02x:%02x:%02x:%02x rssi=%d, create sync\r\n",
		name, Addr[0], Addr[1], Addr[2], Addr[3], Addr[4], Addr[5], Rssi);

	BtPsyncCfg_t cfg = {};
	cfg.Options = 0;						// Name the advertiser, reporting on
	cfg.AdvSid = TARGET_SID;
	cfg.AdvAddrType = (AddrType == BTADDR_TYPE_PUBLIC) ?
		BTPSYNC_ADDR_PUBLIC : BTPSYNC_ADDR_RANDOM;
	memcpy(cfg.AdvAddr, Addr, sizeof(cfg.AdvAddr));
	cfg.Skip = 0;
	cfg.SyncTimeout = SYNC_TIMEOUT;
	cfg.SyncCteType = 0;

	if (BtPsyncCreate(&cfg))
	{
		s_SyncRequested = true;
	}
	else
	{
		OUT("BtPsyncCreate refused\r\n");
	}

	// Keep scanning: synchronization only completes while scanning is enabled.
	return true;
}

void BtPsyncEstablished(const BtPsyncInfo_t * const pInfo)
{
	if (pInfo->Status != 0)
	{
		OUT("Sync failed status 0x%02x, will retry\r\n", pInfo->Status);
		s_SyncRequested = false;
		return;
	}

	s_Synced = true;
	g_bFirst = true;
	OUT("Synced hdl=%u sid=%u interval=%u (x1.25ms)\r\n",
		pInfo->SyncHdl, pInfo->AdvSid, pInfo->Interval);
}

void BtPsyncReport(const BtPsyncReportInfo_t * const pRep)
{
	if (pRep->pData == nullptr || pRep->Len == 0)
	{
		return;
	}

	// Walk the AD structures and read the counter out of the manufacturer
	// specific record the advertiser fills: type 0xFF, two byte company id,
	// then the four byte counter.
	uint16_t idx = 0;
	while (idx + 1 < pRep->Len)
	{
		uint8_t adLen = pRep->pData[idx];
		if (adLen == 0 || idx + 1 + adLen > pRep->Len)
		{
			break;
		}

		uint8_t adType = pRep->pData[idx + 1];
		if (adType == BT_GAP_DATA_TYPE_MANUF_SPECIFIC_DATA && adLen >= 7)
		{
			uint32_t cnt;
			memcpy(&cnt, &pRep->pData[idx + 4], sizeof(cnt));

			IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);

			if (g_bFirst)
			{
				OUT("Counter %lu rssi=%d\r\n", (unsigned long)cnt, pRep->Rssi);
				g_bFirst = false;
			}
			else
			{
				uint32_t d = cnt - g_LastCnt;
				OUT("Counter %lu (+%lu) rssi=%d\r\n",
					(unsigned long)cnt, (unsigned long)d, pRep->Rssi);
			}
			g_LastCnt = cnt;
			return;
		}

		idx += adLen + 1;
	}
}

void BtPsyncLost(uint16_t SyncHdl)
{
	OUT("Sync lost hdl=%u, rescanning\r\n", SyncHdl);
	s_Synced = false;
	s_SyncRequested = false;
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
	g_Uart.printf("BlePeriodicSync\r\n");

	// Route the stack DEBUG_PRINTF to the same UART.
	SysLogGetInstance()->Init(g_Uart);
#endif

	if (BtAppInit(&s_BtAppCfg) == false)
	{
		OUT("BtAppInit failed\r\n");
		while (1);
	}

	BtAppScanInit((BtGapScanCfg_t*)&s_ScanCfg);
	BtAppScan();

	OUT("Scanning for %s ...\r\n", TARGET_NAME);

	BtAppRun();

	return 0;
}
