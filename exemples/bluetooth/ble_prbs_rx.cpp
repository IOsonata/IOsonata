/**-------------------------------------------------------------------------
@example	ble_prbs_rx.cpp

@brief	BLE PRBS receiver

Receive and check BLE PRBS in firmware. UART carries result text only.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017
@author Thinh Tran
@date 	May 9, 2022

@license

Copyright (c) 2017-2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#include <string.h>

#include "istddef.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_hci_ctlr.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "app_evt_handler.h"
#include "board.h"

#if defined(S132) || defined(S140)
#include "ble.h"
#endif

#define PRBS_RX_DURATION_MS 60000U
#define PRBS_RX_REPORT_MS 1000U
#define DEVICE_NAME "BlePrbsRx"
#define TARGET_NAME "BlePrbs_SDC"
#define BLE_MTU_SIZE 247
#define MIN_CONN_INTERVAL 15
#define MAX_CONN_INTERVAL 15
#define APP_ADV_INTERVAL 64
#define APP_ADV_TIMEOUT 0
#define SCAN_INTERVAL 1000
#define SCAN_WINDOW 100
#define SCAN_TIMEOUT 0
#define UARTFIFOSIZE CFIFO_MEMSIZE(32)
#ifndef UART_DEVNO
#define UART_DEVNO 0
#endif

#if defined(NRFXLIB_SDC)
alignas(8) static uint8_t s_CentralCtlrMem[16384];
extern "C" const BtHciCtlrMemPool_t g_BtHciCtlrMemPool = {
	s_CentralCtlrMem, sizeof(s_CentralCtlrMem)
};
#endif

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,				// Device role, ref: BTAPP_ROLE
	.PeriphDevMax = 1, 					// Max peripheral devices we connect to as central
	.CentralDevMax = 0, 					// Max central devices we serve as peripheral
	.pDevName = DEVICE_NAME,				// Device name
	.VendorId = ISYST_BLUETOOTH_ID,			// PnP Bluetooth/USB vendor id
	.ProductId = 1,							// PnP Product ID
	.ProductVer = 0,						// Pnp prod version
	.Appearance = 0,						// 16 bits Bluetooth appearance value
	.pDevInfo = NULL,//&s_BleDevDesc,		// App description
	.pAdvManData = NULL,//g_ManData,		// Manufacture specific data to advertise
	.AdvManDataLen = 0,//sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,						// Addition Manufacture specific data to advertise in scan response
	.SrManDataLen = 0,						// Length of manufacture specific data in scan response
	.SecType = BTGAP_SECTYPE_NONE,					// Open link
	.SecExchg = BTAPP_SECEXCHG_NONE,					// No pairing
	.bCompleteUuidList = false,				// true - Follow is a complete uuid list. false - incomplete list (more uuid than listed here)
	.pAdvUuid = NULL,      					// Service uuids to advertise
	.AdvInterval = APP_ADV_INTERVAL,		// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,			// Advertising timeout in sec
	.AdvSlowInterval = 0,					// Slow advertising interval, if > 0, fallback to
											// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,			// Led port nuber
	.ConnLedPin = CONNECT_LED_PIN,				// Led pin number
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,							// Tx power
	.MaxMtu = BLE_MTU_SIZE,
};

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration
alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];//internal Uart_Rx buffer
// UART carries short result lines only.
alignas(4) static uint8_t s_UartTxFifo[CFIFO_MEMSIZE(1024)];

UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,									// Device number zero based
	.pIOPinMap = s_UartPins,					// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),					// Total number of UART pins used
	.Rate = 1000000,								// Baudrate
	.DataBits = 8,								// Data bits
	.Parity = UART_PARITY_NONE,					// Parity
	.StopBits = 1,								// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,			// Flow control
	.bIntMode = true,							// Interrupt mode
	.IntPrio = 6,//APP_IRQ_PRIORITY_LOW,		// Interrupt priority
	.EvtCallback = nullptr,			// UART event handler
	.bFifoBlocking = true,						// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = sizeof(s_UartTxFifo),
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

// UART object instance
UART g_Uart;

static const BtGapScanCfg_t s_ScanParams = {
	.Type = BTSCAN_TYPE_ACTIVE,
	.Param = {
		.OwnAddrType = BTADDR_TYPE_RAND,
		.Interval = SCAN_INTERVAL,
		.Duration = SCAN_WINDOW,
		.Timeout = SCAN_TIMEOUT,
	},
	.BaseUid = BLUEIO_UUID_BASE,
	.ServUid = BLUEIO_UUID_UART_SERVICE,
};

static BtGapConnParams_t s_ConnParams = {
	.IntervalMin = MIN_CONN_INTERVAL,
	.IntervalMax = MAX_CONN_INTERVAL,
	.Latency = 0,
	.Timeout = 4000,
};


typedef struct {
	uint32_t Bytes;
	uint32_t Notifications;
	uint32_t Breaks;
	uint32_t OutOfFrame;
	uint32_t FirstMs;
	uint16_t MinLen;
	uint16_t MaxLen;
	uint8_t Previous;
	uint8_t ValidRun;
	bool Synced;
	bool Started;
} PrbsRxStats_t;

static volatile PrbsRxStats_t s_Rx;
static uint8_t s_TestPhy = BT_GAP_PHY_1MBITS;
static volatile uint16_t s_ConnHdl = BT_CONN_HDL_INVALID;
static volatile uint16_t s_ValueHdl = BT_ATT_HANDLE_INVALID;
static volatile uint16_t s_CccdHdl = BT_ATT_HANDLE_INVALID;
static volatile uint8_t s_TxPhy;
static volatile uint8_t s_RxPhy;
static volatile bool s_DiscoveryPending;
static volatile bool s_Subscribed;
static volatile bool s_Failed;
static volatile bool s_Finished;
static volatile bool s_Disconnected;
static uint32_t s_ConnectMs;
static uint32_t s_LastPollMs;
static uint32_t s_LastReportMs;
static uint32_t s_LastReportBytes;
static bool s_FinalPrinted;

static void PrbsReset(void)
{
	s_Rx.Bytes = 0;
	s_Rx.Notifications = 0;
	s_Rx.Breaks = 0;
	s_Rx.OutOfFrame = 0;
	s_Rx.FirstMs = 0;
	s_Rx.MinLen = UINT16_MAX;
	s_Rx.MaxLen = 0;
	s_Rx.Previous = 0;
	s_Rx.ValidRun = 0;
	s_Rx.Synced = false;
	s_Rx.Started = false;
	s_LastReportMs = 0;
	s_LastReportBytes = 0;
	s_FinalPrinted = false;
}

static void PrbsReceive(const uint8_t *pData, uint16_t Len, uint32_t Now)
{
	uint16_t idx;
	uint8_t expected;

	if (s_Finished || Len == 0 || pData == nullptr)
	{
		return;
	}
	if (s_Rx.Started && (uint32_t)(Now - s_Rx.FirstMs) >= PRBS_RX_DURATION_MS)
	{
		s_Finished = true;
		return;
	}
	if (s_Rx.Started == false)
	{
		s_Rx.FirstMs = Now;
	}
	for (idx = 0; idx < Len; idx++)
	{
		expected = ((s_Rx.Previous << 1) |
			(((s_Rx.Previous >> 6) ^ (s_Rx.Previous >> 5)) & 1)) & 0x7f;
		if (s_Rx.Started && pData[idx] != expected)
		{
			s_Rx.Breaks++;
		}
		if (pData[idx] > 0x7f && pData[idx] != 0xff)
		{
			s_Rx.OutOfFrame++;
		}
		if (pData[idx] == 0 || (pData[idx] > 0x7f && pData[idx] != 0xff))
		{
			s_Rx.ValidRun = 0;
		}
		else if (!s_Rx.Started || pData[idx] != expected)
		{
			s_Rx.ValidRun = 1;
		}
		else if (s_Rx.ValidRun < 8)
		{
			s_Rx.ValidRun++;
		}
		if (s_Rx.ValidRun >= 8)
		{
			s_Rx.Synced = true;
		}
		s_Rx.Previous = pData[idx];
		s_Rx.Started = true;
	}
	s_Rx.Bytes += Len;
	s_Rx.Notifications++;
	if (Len < s_Rx.MinLen)
	{
		s_Rx.MinLen = Len;
	}
	if (Len > s_Rx.MaxLen)
	{
		s_Rx.MaxLen = Len;
	}
}

void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl, uint8_t *pData, uint16_t Len)
{
	if (ConnHdl == s_ConnHdl && ValHdl == s_ValueHdl)
	{
		PrbsReceive(pData, Len, BtGattMsTick());
	}
}

static void StartDiscovery(uint16_t ConnHdl)
{
	BtDevice_t *peer;

	peer = BtPeerFindByHdl(ConnHdl);
	if (peer != nullptr)
	{
		s_DiscoveryPending = !BtAppDiscoverDevice(peer);
	}
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	PrbsReset();
	s_ConnHdl = ConnHdl;
	s_ValueHdl = BT_ATT_HANDLE_INVALID;
	s_CccdHdl = BT_ATT_HANDLE_INVALID;
	s_TxPhy = BT_GAP_PHY_1MBITS;
	s_RxPhy = BT_GAP_PHY_1MBITS;
	s_Subscribed = false;
	s_Failed = false;
	s_Finished = false;
	s_Disconnected = false;
	s_ConnectMs = BtGattMsTick();
	s_LastPollMs = s_ConnectMs;
	g_Uart.printf("Connected handle=%u; requesting PHY=%u\r\n", ConnHdl, s_TestPhy);
	if (!BtGapSetPhy(ConnHdl, s_TestPhy, s_TestPhy, 0))
	{
		s_Failed = true;
	}
#if defined(NRFXLIB_SDC)
	if (!BtGapSetDataLength(ConnHdl, 251, 2120))
	{
		s_Failed = true;
	}
#endif
	StartDiscovery(ConnHdl);
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	(void)ConnHdl;
	s_ConnHdl = BT_CONN_HDL_INVALID;
	s_Disconnected = true;
	if (!s_Finished)
	{
		s_Failed = true;
		s_Finished = true;
	}
}

void BtAppEvtConnectFailed(void)
{
	s_Failed = true;
	s_Finished = true;
	s_Disconnected = true;
}

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData)
{
	char name[32] = {0};
	size_t length;
	BtGapPeerAddr_t address = {};

	length = BtAdvDataGetDevName(pAdvData, AdvLen, name, sizeof(name));
	if (length >= sizeof(name))
	{
		length = sizeof(name) - 1;
	}
	name[length] = 0;
	if (strcmp(name, TARGET_NAME) != 0)
	{
		return true;
	}
	g_Uart.printf("Found %s RSSI=%d\r\n", name, Rssi);
	BtGapScanStop();
	address.Type = AddrType;
	memcpy(address.Addr, Addr, sizeof(address.Addr));
	if (!BtGapConnect(&address, &s_ConnParams))
	{
		s_Failed = true;
		s_Finished = true;
		s_Disconnected = true;
	}
	return false;
}

void BtDeviceDiscovered(BtDevice_t *pDev)
{
	int service;
	int characteristic;

	if (pDev == nullptr || pDev->Conn.Hdl != s_ConnHdl)
	{
		return;
	}
	service = BtDeviceFindService(pDev, BLUEIO_UUID_UART_SERVICE);
	if (service < 0)
	{
		s_Failed = true;
		return;
	}
	characteristic = BtDeviceFindCharacteristic(pDev, service, BLUEIO_UUID_UART_RX_CHAR);
	if (characteristic < 0)
	{
		s_Failed = true;
		return;
	}
	s_ValueHdl = pDev->pServices[service].characteristics[characteristic].characteristic.handle_value;
	s_CccdHdl = pDev->pServices[service].characteristics[characteristic].cccd_handle;
	if (s_CccdHdl == BT_ATT_HANDLE_INVALID)
	{
		s_Failed = true;
	}
}

void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx)
{
#if defined(S132) || defined(S140)
	const ble_evt_t *evt = (const ble_evt_t *)pCtx;

	if (evt == nullptr)
	{
		return;
	}
	switch (Evt)
	{
		case BLE_GAP_EVT_CONNECTED:
			if (evt->evt.gap_evt.params.connected.conn_params.max_conn_interval != 12 ||
				evt->evt.gap_evt.params.connected.conn_params.slave_latency != 0)
			{
				s_Failed = true;
			}
			g_Uart.printf("Interval units=%u latency=%u\r\n",
				evt->evt.gap_evt.params.connected.conn_params.max_conn_interval,
				evt->evt.gap_evt.params.connected.conn_params.slave_latency);
			break;
		case BLE_GAP_EVT_CONN_PARAM_UPDATE:
			if (evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval != 12 ||
				evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency != 0)
			{
				s_Failed = true;
			}
			break;
		case BLE_GAP_EVT_PHY_UPDATE:
			s_TxPhy = evt->evt.gap_evt.params.phy_update.tx_phy;
			s_RxPhy = evt->evt.gap_evt.params.phy_update.rx_phy;
			if (evt->evt.gap_evt.params.phy_update.status != 0)
			{
				s_Failed = true;
			}
			break;
		case BLE_GAP_EVT_DATA_LENGTH_UPDATE:
			g_Uart.printf("LL octets tx=%u rx=%u\r\n",
				evt->evt.gap_evt.params.data_length_update.effective_params.max_tx_octets,
				evt->evt.gap_evt.params.data_length_update.effective_params.max_rx_octets);
			break;
		case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
			if (s_DiscoveryPending)
			{
				StartDiscovery(evt->evt.gattc_evt.conn_handle);
			}
			break;
		default:
			break;
	}
#else
	(void)Evt;
	(void)pCtx;
#endif
}

void BtAppInitUserServices(void)
{
}

void BtAppInitUserData(void)
{
}

static void PrbsReport(uint32_t Now)
{
	PrbsRxStats_t stats;
	uint32_t state;
	uint32_t elapsed;
	uint32_t interval;
	uint32_t rate;
	uint32_t average;
	uint32_t notificationsPerInterval;
	uint32_t target;
	bool final;
	bool valid;

	if (s_FinalPrinted)
	{
		return;
	}
	state = DisableInterrupt();
	Now = BtGattMsTick();
	stats.Bytes = s_Rx.Bytes;
	stats.Notifications = s_Rx.Notifications;
	stats.Breaks = s_Rx.Breaks;
	stats.OutOfFrame = s_Rx.OutOfFrame;
	stats.FirstMs = s_Rx.FirstMs;
	stats.MinLen = s_Rx.MinLen;
	stats.MaxLen = s_Rx.MaxLen;
	stats.Started = s_Rx.Started;
	stats.Synced = s_Rx.Synced;
	elapsed = stats.Started ? (uint32_t)(Now - stats.FirstMs) : 0;
	if (elapsed >= PRBS_RX_DURATION_MS || s_Failed)
	{
		s_Finished = true;
	}
	final = s_Finished;
	EnableInterrupt(state);
	if (!stats.Started && !final)
	{
		return;
	}
	if (s_LastReportMs == 0)
	{
		s_LastReportMs = stats.FirstMs;
	}
	interval = (uint32_t)(Now - s_LastReportMs);
	if (interval < PRBS_RX_REPORT_MS && !final)
	{
		return;
	}
	rate = interval ? (uint32_t)((uint64_t)(stats.Bytes - s_LastReportBytes) * 1000 / interval) : 0;
	average = elapsed ? (uint32_t)((uint64_t)stats.Bytes * 1000 / elapsed) : 0;
	notificationsPerInterval = elapsed ? (uint32_t)((uint64_t)stats.Notifications * 15000 / elapsed) : 0;
	target = s_TestPhy == BT_GAP_PHY_1MBITS ? 4 : 8;
	g_Uart.printf("%sPHY=%u Bytes/sec=%lu average=%lu ",
		final ? "Final " : "", s_TestPhy, (unsigned long)rate, (unsigned long)average);
	g_Uart.printf("elapsed_ms=%lu\r\n", (unsigned long)elapsed);
	g_Uart.printf("bytes=%lu notifications=%lu ",
		(unsigned long)stats.Bytes, (unsigned long)stats.Notifications);
	g_Uart.printf("drop=%lu out_of_frame=%lu\r\n",
		(unsigned long)stats.Breaks, (unsigned long)stats.OutOfFrame);
	g_Uart.printf("len_min=%u len_max=%u notifications/15ms=%lu.%03lu\r\n",
		stats.Notifications ? stats.MinLen : 0, stats.MaxLen,
		(unsigned long)(notificationsPerInterval / 1000), (unsigned long)(notificationsPerInterval % 1000));
	s_LastReportMs = Now;
	s_LastReportBytes = stats.Bytes;
	if (final)
	{
		valid = stats.Synced && stats.Breaks == 0 && stats.OutOfFrame == 0 && !s_Failed;
		g_Uart.printf("PRBS integrity: %s\r\n", valid ? "PASS" : "FAIL");
		g_Uart.printf("Notification target=%lu: %s (average only)\r\n",
			(unsigned long)target, notificationsPerInterval >= target * 1000 ? "MET" : "NOT MET");
		g_Uart.printf("PHY=%u RESULT: %s\r\n", s_TestPhy,
			valid && elapsed >= PRBS_RX_DURATION_MS && notificationsPerInterval >= target * 1000 ? "PASS" : "FAIL");
		g_Uart.printf("PHY=%u trial complete.\r\n", s_TestPhy);
		s_FinalPrinted = true;
		if (s_ConnHdl != BT_CONN_HDL_INVALID)
		{
			BtAppDisconnectConn(s_ConnHdl);
		}
	}
}

bool AppCheckStatus(void)
{
	uint32_t now;
	BtDevice_t *peer;
#if defined(NRFXLIB_SDC)
	uint8_t txPhy;
	uint8_t rxPhy;
#endif

	BtAppCheckStatus();
	if (s_FinalPrinted && s_Disconnected && s_TestPhy == BT_GAP_PHY_1MBITS)
	{
		s_TestPhy = BT_GAP_PHY_2MBITS;
		PrbsReset();
		s_Failed = false;
		s_Finished = false;
		s_Disconnected = false;
		g_Uart.printf("Starting 60-second 2M trial\r\n");
		BtAppScan();
	}
	now = BtGattMsTick();
	if (s_ConnHdl != BT_CONN_HDL_INVALID && !s_Finished &&
		(uint32_t)(now - s_LastPollMs) >= PRBS_RX_REPORT_MS)
	{
		s_LastPollMs = now;
#if defined(NRFXLIB_SDC)
		if (BtGapReadPhy(s_ConnHdl, &txPhy, &rxPhy))
		{
			s_TxPhy = txPhy;
			s_RxPhy = rxPhy;
		}
		else
		{
			s_Failed = true;
		}
#endif
		if (!s_Failed && !s_Subscribed && s_CccdHdl != BT_ATT_HANDLE_INVALID &&
			s_TxPhy == s_TestPhy && s_RxPhy == s_TestPhy)
		{
			peer = BtPeerFindByHdl(s_ConnHdl);
			if (peer != nullptr && peer->Conn.MaxMtu == BLE_MTU_SIZE)
			{
				g_Uart.printf("Verified PHY tx=%u rx=%u MTU=%u\r\n",
					s_TxPhy, s_RxPhy, peer->Conn.MaxMtu);
				s_Subscribed = BtAppEnableNotify(s_ConnHdl, s_CccdHdl);
			}
		}
		if (s_Subscribed && (s_TxPhy != s_TestPhy || s_RxPhy != s_TestPhy))
		{
			s_Failed = true;
		}
		if (!s_Rx.Started && (uint32_t)(now - s_ConnectMs) >= 10000)
		{
			s_Failed = true;
		}
	}
	PrbsReport(now);
	return true;
}

int main(void)
{
	g_Uart.Init(g_UartCfg);
	g_Uart.printf("BlePrbsRx: target=%s requested_PHY=%u\r\n", TARGET_NAME, s_TestPhy);
	g_Uart.printf("Interval=15 ms MTU=247; 60 s at 1M then 60 s at 2M\r\n");
	if (!BtAppInit(&s_BleAppCfg))
	{
		g_Uart.printf("BtAppInit failed\r\n");
#if defined(NRFXLIB_SDC)
		g_Uart.printf("SDC error=%d value=%d needed=%d pool=%u\r\n",
			(int)BtHciCtlrErrorGet(), (int)BtHciCtlrErrorValueGet(),
			(int)BtHciCtlrMemPoolSizeNeeded(), (unsigned)g_BtHciCtlrMemPool.Size);
#endif
		return 1;
	}
	BtAppScanInit((BtGapScanCfg_t *)&s_ScanParams);
	BtAppScan();
	AppRun();
	return 0;
}
