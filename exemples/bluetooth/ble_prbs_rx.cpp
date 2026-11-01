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

#include "ble_prbs_benchmark.h"

#include "istddef.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/bt_dev.h"
#include "coredev/uart.h"
#include "coredev/interrupt.h"
#include "app_evt_handler.h"
#include "board.h"

#ifdef MCUOSC
#include "coredev/system_core_clock.h"
McuOsc_t g_McuOsc = MCUOSC;
#endif

#define PRBS_RX_SETUP_TIMEOUT_MS 10000U
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

static const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,
	.PeriphDevMax = 1,
	.CentralDevMax = 0,
	.pDevName = DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 0,
	.Appearance = 0,
	.pDevInfo = NULL,
	.pAdvManData = NULL,
	.AdvManDataLen = 0,
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,
	.SecExchg = BTAPP_SECEXCHG_NONE,
	.bCompleteUuidList = false,
	.pAdvUuid = NULL,
	.AdvInterval = APP_ADV_INTERVAL,
	.AdvTimeout = APP_ADV_TIMEOUT,
	.AdvSlowInterval = 0,

	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,
	.ConnLedPin = CONNECT_LED_PIN,
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,
	.MaxMtu = BLE_MTU_SIZE,
};

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];

alignas(4) static uint8_t s_UartTxFifo[CFIFO_MEMSIZE(1024)];

UARTCfg_t g_UartCfg = {
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
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = sizeof(s_UartTxFifo),
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

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

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(PRBS_BENCH_QUEUE_COUNT)];
static PrbsBenchmark_t s_Benchmark;
static uint8_t s_TestPhy = BT_GAP_PHY_1MBITS;
static volatile uint16_t s_ConnHdl = BT_CONN_HDL_INVALID;
static volatile uint16_t s_ValueHdl = BT_ATT_HANDLE_INVALID;
static volatile bool s_Disconnected;
static volatile uint32_t s_ConnectMs;
// Set while PrbsReceive is in the AppEvt queue. The Bluetooth stack queues its
// own work in the same queue, so the consumer takes one entry however many
// notifications wait, as a sequencer task bit does in the ST client.
static volatile bool s_RxQueued;

static void PrbsReceive(uint32_t Evt, void *pCtx);

// Called from the notification callback, which runs in interrupt context on
// some ports, and from the consumer itself.
static bool PrbsSchedule(void)
{
	uint32_t state = DisableInterrupt();
	bool queue = s_RxQueued == false;

	s_RxQueued = true;
	EnableInterrupt(state);
	if (queue && AppEvtHandlerQue(0, nullptr, PrbsReceive) == false)
	{
		// The packets stay in the CFIFO; AppCheckStatus schedules again.
		s_RxQueued = false;
		return false;
	}
	return true;
}

static bool PrbsWrite(const uint8_t *pData, uint16_t Len)
{
	return g_Uart.Tx((uint8_t *)pData, Len) == Len;
}

static void PrbsDone(void)
{
	if (s_ConnHdl != BT_CONN_HDL_INVALID)
	{
		BtAppDisconnectConn(s_ConnHdl);
	}
}

static const PrbsBenchAdapters_t s_BenchAdapters = {
	BtGattMsTick,
	PrbsSchedule,
	PrbsWrite,
	PrbsDone
};

static void PrbsReceive(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	// Clear first: a notification during the check queues the next pass.
	s_RxQueued = false;
	PrbsBenchReceive(&s_Benchmark);
}

void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl, uint8_t *pData, uint16_t Len)
{
	if (ConnHdl == s_ConnHdl && ValHdl == s_ValueHdl)
	{
		PrbsBenchNotify(&s_Benchmark, pData, Len);
	}
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	BtDevice_t *pPeer;

	// Some ports deliver this callback from an interrupt. Reset measurement
	// in the consumer before checking the first queued notification.
	s_Benchmark.Phy = s_TestPhy;
	s_Benchmark.ResetPending = true;
	s_Benchmark.Dropped = 0;
	s_ConnHdl = ConnHdl;
	s_ValueHdl = BT_ATT_HANDLE_INVALID;
	s_Benchmark.Failed = false;
	s_Benchmark.Finished = false;
	s_Disconnected = false;
	s_ConnectMs = BtGattMsTick();
	g_Uart.printf("Connected handle=%u; requesting PHY=%u\r\n", ConnHdl, s_TestPhy);
	pPeer = BtPeerFindByHdl(ConnHdl);
	if (!BtGapSetPhy(ConnHdl, s_TestPhy, s_TestPhy, 0) ||
		!BtGapSetDataLength(ConnHdl, 251, 2120) ||
		pPeer == nullptr || !BtAppDiscoverDevice(pPeer))
	{
		s_Benchmark.Failed = true;
	}
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	if (ConnHdl != s_ConnHdl)
	{
		return;
	}
	s_ConnHdl = BT_CONN_HDL_INVALID;
	s_Disconnected = true;
	if (!s_Benchmark.Finished)
	{
		g_Uart.printf("Disconnected before trial completion\r\n");
		s_Benchmark.Failed = true;
		s_Benchmark.Finished = true;
	}
}

void BtAppEvtConnectFailed(void)
{
	s_Benchmark.Failed = true;
	s_Benchmark.Finished = true;
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
		s_Benchmark.Failed = true;
		s_Benchmark.Finished = true;
		s_Disconnected = true;
	}
	return false;
}

void BtDeviceDiscovered(BtDevice_t *pDev)
{
	int service;
	int characteristic;
	uint16_t cccd;
	uint8_t txPhy;
	uint8_t rxPhy;

	if (pDev == nullptr || pDev->Conn.Hdl != s_ConnHdl)
	{
		return;
	}
	service = BtDeviceFindService(pDev, BLUEIO_UUID_UART_SERVICE);
	if (service < 0)
	{
		s_Benchmark.Failed = true;
		return;
	}
	characteristic = BtDeviceFindCharacteristic(pDev, service, BLUEIO_UUID_UART_RX_CHAR);
	if (characteristic < 0)
	{
		s_Benchmark.Failed = true;
		return;
	}
	s_ValueHdl = pDev->pServices[service].characteristics[characteristic].characteristic.handle_value;
	cccd = pDev->pServices[service].characteristics[characteristic].cccd_handle;
	if (cccd == BT_ATT_HANDLE_INVALID || pDev->Conn.MaxMtu != BLE_MTU_SIZE ||
		!BtGapReadPhy(s_ConnHdl, &txPhy, &rxPhy) ||
		txPhy != s_TestPhy || rxPhy != s_TestPhy ||
		!BtAppEnableNotify(s_ConnHdl, cccd))
	{
		s_Benchmark.Failed = true;
	}
}

void BtAppInitUserServices(void)
{
}

void BtAppInitUserData(void)
{
}

bool AppCheckStatus(void)
{
	uint32_t now;

	BtAppCheckStatus();
	if (s_Benchmark.FinalPrinted && s_Benchmark.Complete && s_Disconnected && s_TestPhy == BT_GAP_PHY_1MBITS)
	{
		s_TestPhy = BT_GAP_PHY_2MBITS;
		PrbsBenchReset(&s_Benchmark, s_TestPhy);
		s_Disconnected = false;
		g_Uart.printf("Starting 60-second 2M trial\r\n");
		BtAppScan();
	}
	now = BtGattMsTick();
	if (s_ConnHdl != BT_CONN_HDL_INVALID && !s_Benchmark.Finished &&
		!s_Benchmark.Stats.Started &&
		(uint32_t)(now - s_ConnectMs) >= PRBS_RX_SETUP_TIMEOUT_MS)
	{
		s_Benchmark.Failed = true;
	}
	PrbsBenchPoll(&s_Benchmark, now);
	return true;
}

int main(void)
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)))
	{
		return 1;
	}
	if (!PrbsBenchInit(&s_Benchmark, &s_BenchAdapters, s_TestPhy))
	{
		return 1;
	}
	if (!g_Uart.Init(g_UartCfg))
	{
		return 1;
	}
	g_Uart.printf("BlePrbsRx: target=%s requested_PHY=%u\r\n", TARGET_NAME, s_TestPhy);
	g_Uart.printf("Interval=15 ms MTU=247; 60 s at 1M then 60 s at 2M\r\n");
#if defined(__OPTIMIZE__)
	g_Uart.printf("Compiler optimization: enabled\r\n");
#else
	g_Uart.printf("Compiler optimization: disabled; use Release for throughput\r\n");
#endif
	if (!BtAppInit(&s_BleAppCfg))
	{
		g_Uart.printf("BtAppInit failed\r\n");
		return 1;
	}
	BtAppScanInit((BtGapScanCfg_t *)&s_ScanParams);
	BtAppScan();
	AppRun();
	return 0;
}
