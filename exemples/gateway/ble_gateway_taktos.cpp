/**-------------------------------------------------------------------------
@example	ble_gateway_taktos.cpp

@brief	BLE advertisement observer with LTE-M UDP telemetry.

Starts the LTE subsystem (lte.h) with PSM requested, opens a UDP socket
(net/sock_intrf.h) once registered and sends a short message every
LTE_UDP_INTERVAL seconds, with a release assistance hint so the network
lets the modem sleep right after it. Network events and anything the
server sends back are logged with SysLog on the console UART. Between
packets the modem is in PSM and the application waits in AppRun. For an
average current measurement, leave the console out: its receiver draws far
more than the modem in PSM. After a modem fault the example starts LTE again.

Everything runs from the application event queue: the LTE events, the
received data and the send timer.

Configure the server here or from board.h:

	LTE_UDP_HOST		server name or address
	LTE_UDP_PORT		server port
	LTE_UDP_APN			APN, NULL for the network default
	LTE_UDP_INTERVAL	seconds between packets
	LTE_UDP_RAI			1 to use release assistance (modem support needed)
	LTE_UDP_RAT			radio access technologies allowed, LTE_RAT_*
	LTE_UDP_RAT_PREF	preferred one when both are allowed, LTE_RAT_NONE for none

The board.h of the project gives the console UART pins and the timer used
by the modem waits and the send interval.

@author	Hoang Nguyen Hoan
@date	Oct. 6, 2026

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
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "syslog.h"
#include "coredev/timer.h"
#include "lte/lte.h"
#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_adv.h"
#include "bluetooth/bt_adv_mandata.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/bt_gatt.h"
#include "net/sock_intrf.h"

#include "board.h"
#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSQueue.h"

#ifndef LTE_UDP_HOST
#define LTE_UDP_HOST			"udp.example.com"
#endif
#ifndef LTE_UDP_PORT
#define LTE_UDP_PORT			2469
#endif
#ifndef LTE_UDP_APN
#define LTE_UDP_APN				nullptr
#endif
#ifndef LTE_UDP_INTERVAL
#define LTE_UDP_INTERVAL		60
#endif
#ifndef LTE_UDP_RAI
#define LTE_UDP_RAI				1
#endif
#ifndef LTE_UDP_RAT
#define LTE_UDP_RAT				LTE_RAT_LTEM_NBIOT
#endif
#ifndef LTE_UDP_RAT_PREF
#define LTE_UDP_RAT_PREF		LTE_RAT_LTEM
#endif

// Timer triggers: one kept for the modem waits, one for the send interval
#define LTE_UDP_TRIG_MODEM		0
#define LTE_UDP_TRIG_SEND		1

// Longest message and longest received datagram printed
#define LTE_UDP_MSG_LEN			512
#define LTE_UDP_RX_LEN			512

// Application events
#define LTE_UDP_EVT_SEND		1
#define LTE_UDP_EVT_RX			2

// Keep scan reports bounded: the application queue processes UDP outside
// the Bluetooth callback. Each report supersedes the previous one.
#define GATEWAY_SCAN_INTERVAL 1000U
#define GATEWAY_SCAN_WINDOW 100U

#define GATEWAY_TES_SERVICE 0x0200U
#define GATEWAY_TES_TEMPERATURE 0x0201U
#define GATEWAY_TES_PRESSURE 0x0202U
#define GATEWAY_TES_HUMIDITY 0x0203U
#define GATEWAY_MAX_TAGS 4U

static BtGapConnParams_t s_GattConnParams = {
	.IntervalMin = 30.0f,
	.IntervalMax = 75.0f,
	.Latency = 0,
	.Timeout = 4000,
};

typedef struct {
    uint8_t Addr[6];
    uint8_t AddrType;
    uint16_t ConnHdl;
    uint16_t TempHdl;
    uint16_t PressureHdl;
    uint16_t HumidityHdl;
    uint16_t CccdHdl[3];
    uint8_t NotifyStage;
    bool Active;
    bool Connecting;
    bool Pending;
    bool TempValid;
    bool PressureValid;
    bool HumidityValid;
    int16_t TempCenti;
    uint32_t PressurePa;
    uint8_t HumidityPct;
} GatewayTag_t;

static GatewayTag_t s_Tags[GATEWAY_MAX_TAGS] = {};
static GatewayTag_t *s_pInitiating = nullptr;
static uint64_t s_ConnectStartTicks = 0;
static bool s_ConnectCancelPending = false;
static volatile bool s_GattScanRestartPending = false;
static BtDevSrvcCache_t s_GattCaches[GATEWAY_MAX_TAGS] = {};
const BtDevSrvcCacheCfg_t g_BtDevSrvcCacheCfg = {
    s_GattCaches, GATEWAY_MAX_TAGS
};

static GatewayTag_t *GatewayFindAddr(const uint8_t Addr[6])
{
    for (unsigned i = 0; i < GATEWAY_MAX_TAGS; ++i)
    {
        if (s_Tags[i].Active && memcmp(s_Tags[i].Addr, Addr, 6) == 0)
        {
            return &s_Tags[i];
        }
    }
    return nullptr;
}

static GatewayTag_t *GatewayFindHandle(uint16_t ConnHdl)
{
    for (unsigned i = 0; i < GATEWAY_MAX_TAGS; ++i)
    {
        if (s_Tags[i].Active && s_Tags[i].ConnHdl == ConnHdl)
        {
            return &s_Tags[i];
        }
    }
    return nullptr;
}

static GatewayTag_t *GatewayFindOrCreate(const uint8_t Addr[6], uint8_t AddrType)
{
    GatewayTag_t *pTag = GatewayFindAddr(Addr);
    if (pTag != nullptr)
    {
        return pTag;
    }
    GatewayTag_t *pAvailable = nullptr;
    for (unsigned i = 0; i < GATEWAY_MAX_TAGS; ++i)
    {
        if (!s_Tags[i].Active)
        {
            pAvailable = &s_Tags[i];
            break;
        }
        if (pAvailable == nullptr &&
            s_Tags[i].ConnHdl == BT_CONN_HDL_INVALID &&
            !s_Tags[i].Connecting && !s_Tags[i].Pending &&
            s_pInitiating != &s_Tags[i])
        {
            pAvailable = &s_Tags[i];
        }
    }
    if (pAvailable == nullptr)
    {
        return nullptr;
    }

    // Reuse a disconnected slot only. Never replace an active link or
    // a peer for which a connection initiation is already pending.
    memset(pAvailable, 0, sizeof(*pAvailable));
    memcpy(pAvailable->Addr, Addr, 6);
    pAvailable->AddrType = AddrType;
    pAvailable->ConnHdl = BT_CONN_HDL_INVALID;
    pAvailable->TempHdl = BT_ATT_HANDLE_INVALID;
    pAvailable->PressureHdl = BT_ATT_HANDLE_INVALID;
    pAvailable->HumidityHdl = BT_ATT_HANDLE_INVALID;
    pAvailable->Active = true;
    return pAvailable;
}

static const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,
	.PeriphDevMax = GATEWAY_MAX_TAGS,
	.CentralDevMax = 0,
	.pDevName = "IOsonataGateway",
};

static BtGapScanCfg_t s_BleScanCfg = {
	.Type = BTSCAN_TYPE_ACTIVE,
	.Param = {
		.Interval = GATEWAY_SCAN_INTERVAL,
		.Duration = GATEWAY_SCAN_WINDOW,
		.Timeout = 0,
	},
};

static uint8_t s_ScanAddress[6];
static int8_t s_ScanRssi = 0;
static uint32_t s_ScanReports = 0;
static bool s_ScanValid = false;
static uint8_t s_SensorType = 0U;
static uint8_t s_SensorData[8] = {};
static uint8_t s_SensorAddress[6] = {};
static bool s_SensorValid = false;

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6],
					 size_t AdvLen, uint8_t *pAdvData)
{
	uint8_t manufacturer[32] = {};
	const size_t manufacturerLen = BtAdvDataGetManData(
		pAdvData, AdvLen, manufacturer, sizeof(manufacturer));
	// Manufacturer record: two-byte company ID, BlueIO type, sensor bytes.
	// Require a complete TPH payload before decoding its fixed-point fields.
	const bool isBlueIo = manufacturerLen >= 9U &&
		manufacturer[0] == (uint8_t)(ISYST_BLUETOOTH_ID & 0xFFU) &&
		manufacturer[1] == (uint8_t)(ISYST_BLUETOOTH_ID >> 8);
	bool isTph = isBlueIo && manufacturerLen >= 11U &&
		manufacturer[2] == BT_ADV_MANDATA_TYPE_TPH;
	const bool isGas = isBlueIo && manufacturerLen >= 11U &&
		manufacturer[2] == BT_ADV_MANDATA_TYPE_GAS;
	if (isTph)
	{
		const uint32_t pressure = (uint32_t)manufacturer[3] |
			((uint32_t)manufacturer[4] << 8) |
			((uint32_t)manufacturer[5] << 16) |
			((uint32_t)manufacturer[6] << 24);
		const int16_t temperature = (int16_t)((uint16_t)manufacturer[7] |
			((uint16_t)manufacturer[8] << 8));
		const uint16_t humidity = (uint16_t)manufacturer[9] |
			((uint16_t)manufacturer[10] << 8);
		// Reject uninitialized or malformed sensor values before caching
		// them or initiating a GATT connection.
		isTph = pressure >= 30000U && pressure <= 120000U &&
			temperature >= -4000 && temperature <= 8500 &&
			humidity <= 10000U;
	}

	const uint32_t irq = DisableInterrupt();
	for (unsigned i = 0; i < 6U; ++i)
	{
		s_ScanAddress[i] = Addr[i];
	}
	s_ScanRssi = Rssi;
	++s_ScanReports;
	s_ScanValid = true;
	if (isTph || isGas)
	{
		s_SensorType = manufacturer[2];
		for (unsigned i = 0; i < sizeof(s_SensorAddress); ++i)
		{
			s_SensorAddress[i] = Addr[i];
		}
		for (unsigned i = 0; i < sizeof(s_SensorData); ++i)
		{
			s_SensorData[i] = manufacturer[i + 3U];
		}
		s_SensorValid = true;
	}
	EnableInterrupt(irq);
    if (isTph)
    {
        GatewayTag_t *pTag = GatewayFindOrCreate(Addr, AddrType);
        if (pTag != nullptr && pTag->ConnHdl == BT_CONN_HDL_INVALID &&
            !pTag->Connecting && !pTag->Pending)
        {
            pTag->Pending = true;
        }
    }
	return true;
}

static int GatewayLog(const char *pFormat, ...);

void BtAppEvtConnected(uint16_t ConnHdl)
{
    GatewayTag_t *pTag = s_pInitiating;
    s_pInitiating = nullptr;
    s_ConnectCancelPending = false;
    if (pTag == nullptr)
    {
        BtAppDisconnectConn(ConnHdl);
        return;
    }
    pTag->ConnHdl = ConnHdl;
    pTag->Connecting = false;
    GatewayLog("GATT connected, handle=%u tag=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
               (unsigned)ConnHdl, pTag->Addr[5], pTag->Addr[4],
               pTag->Addr[3], pTag->Addr[2], pTag->Addr[1], pTag->Addr[0]);
    BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
    if (pPeer == nullptr || !BtAppDiscoverDevice(pPeer))
    {
        GatewayLog("GATT discovery start failed\r\n");
        BtAppDisconnectConn(ConnHdl);
    }
    s_GattScanRestartPending = true;
}

void BtAppEvtConnectFailed(void)
{
    if (s_pInitiating != nullptr)
    {
        s_pInitiating->Connecting = false;
        s_pInitiating = nullptr;
    }
    s_ConnectCancelPending = false;
    GatewayLog("GATT connection attempt failed\r\n");
    s_GattScanRestartPending = true;
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
    GatewayTag_t *pTag = GatewayFindHandle(ConnHdl);
    if (pTag != nullptr)
    {
        GatewayLog("GATT disconnected, handle=%u tag=%02x:%02x:%02x:%02x:%02x:%02x\r\n",
                   (unsigned)ConnHdl, pTag->Addr[5], pTag->Addr[4],
                   pTag->Addr[3], pTag->Addr[2], pTag->Addr[1], pTag->Addr[0]);
    }
    else
    {
        GatewayLog("GATT disconnected, handle=%u\r\n", (unsigned)ConnHdl);
    }
    if (pTag != nullptr)
    {
        pTag->ConnHdl = BT_CONN_HDL_INVALID;
        pTag->TempHdl = BT_ATT_HANDLE_INVALID;
        pTag->PressureHdl = BT_ATT_HANDLE_INVALID;
        pTag->HumidityHdl = BT_ATT_HANDLE_INVALID;
        pTag->NotifyStage = 0;
        pTag->TempValid = false;
        pTag->PressureValid = false;
        pTag->HumidityValid = false;
        pTag->Connecting = false;
        pTag->Pending = false;
    }
    s_GattScanRestartPending = true;
}

void BtDeviceDiscovered(BtDevice_t *pDev)
{
    if (pDev == nullptr)
    {
        return;
    }
    GatewayTag_t *pTag = GatewayFindHandle(pDev->Conn.Hdl);
    if (pTag == nullptr)
    {
        return;
    }
    const int service = BtDeviceFindService(pDev, GATEWAY_TES_SERVICE);
    if (service < 0)
    {
        GatewayLog("GATT environmental service not found\r\n");
        BtAppDisconnectConn(pDev->Conn.Hdl);
        return;
    }
    const uint16_t uuids[] = {
        GATEWAY_TES_TEMPERATURE,
        GATEWAY_TES_PRESSURE,
        GATEWAY_TES_HUMIDITY,
    };
    uint16_t *handles[] = {
        &pTag->TempHdl,
        &pTag->PressureHdl,
        &pTag->HumidityHdl,
    };
    for (unsigned i = 0; i < 3U; ++i)
    {
        const int index = BtDeviceFindCharacteristic(pDev, service, uuids[i]);
        if (index < 0)
        {
            GatewayLog("GATT characteristic 0x%04x missing\r\n", (unsigned)uuids[i]);
            BtAppDisconnectConn(pDev->Conn.Hdl);
            return;
        }
        const uint16_t cccd = pDev->pServices[service].characteristics[index].cccd_handle;
        if (cccd == BT_ATT_HANDLE_INVALID)
        {
            GatewayLog("GATT CCCD missing for 0x%04x\r\n", (unsigned)uuids[i]);
            BtAppDisconnectConn(pDev->Conn.Hdl);
            return;
        }
        pTag->CccdHdl[i] = cccd;
        *handles[i] = pDev->pServices[service].characteristics[index].characteristic.handle_value;
    }
    pTag->NotifyStage = 0;
    if (!BtAppEnableNotify(pDev->Conn.Hdl, pTag->CccdHdl[0]))
    {
        GatewayLog("GATT temperature subscription failed\r\n");
        BtAppDisconnectConn(pDev->Conn.Hdl);
    }
}

void BtAppEvtWriteCompleted(uint16_t ConnHdl, bool Success)
{
    GatewayTag_t *pTag = GatewayFindHandle(ConnHdl);
    if (pTag == nullptr || pTag->NotifyStage >= 3U)
    {
        return;
    }
    if (!Success)
    {
        GatewayLog("GATT subscription rejected at stage %u\r\n",
                   (unsigned)pTag->NotifyStage);
        BtAppDisconnectConn(ConnHdl);
        return;
    }
    ++pTag->NotifyStage;
    if (pTag->NotifyStage == 3U)
    {
        GatewayLog("GATT environmental notifications enabled, handle=%u\r\n",
                   (unsigned)ConnHdl);
    }
    else if (!BtAppEnableNotify(ConnHdl, pTag->CccdHdl[pTag->NotifyStage]))
    {
        GatewayLog("GATT subscription failed at stage %u\r\n",
                   (unsigned)pTag->NotifyStage);
        BtAppDisconnectConn(ConnHdl);
    }
}

void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl,
                          uint8_t *pData, uint16_t Len)
{
    GatewayTag_t *pTag = GatewayFindHandle(ConnHdl);
    if (pTag == nullptr || pData == nullptr)
    {
        return;
    }
    if (ValHdl == pTag->TempHdl && Len == 2)
    {
        const int8_t integer = (int8_t)pData[0];
        const uint8_t decimal = pData[1];
        if (decimal < 100)
        {
            pTag->TempCenti = (int16_t)(integer * 100 +
                (integer < 0 ? -(int16_t)decimal : (int16_t)decimal));
            pTag->TempValid = true;
        }
    }
    else if (ValHdl == pTag->PressureHdl && Len == 5)
    {
        const int32_t integer = (int32_t)((uint32_t)pData[0] |
            ((uint32_t)pData[1] << 8) | ((uint32_t)pData[2] << 16) |
            ((uint32_t)pData[3] << 24));
        if (integer >= 0 && pData[4] < 100)
        {
            pTag->PressurePa = (uint32_t)integer * 100U + pData[4];
            pTag->PressureValid = true;
        }
    }
    else if (ValHdl == pTag->HumidityHdl && Len == 1 && pData[0] <= 100)
    {
        pTag->HumidityPct = pData[0];
        pTag->HumidityValid = true;
    }
}

// Application event queue: LTE work, received data and the send timer
// One TaktOS worker serializes BLE, LTE and gateway work.
typedef struct {
    uint32_t Evt;
    void *pCtx;
    AppEvtHandler_t Handler;
} GatewayWork_t;

#define GATEWAY_WORK_QUE_COUNT 32U
static uint8_t s_GatewayWorkMem[GATEWAY_WORK_QUE_COUNT * sizeof(GatewayWork_t)] TAKT_ALIGNED(4);
static TaktOSQueue_t s_GatewayWorkQue;

static bool GatewayWorkQue(uint32_t Evt, void *pCtx, AppEvtHandler_t Handler)
{
    if (Handler == nullptr)
    {
        return false;
    }
    const GatewayWork_t work = { Evt, pCtx, Handler };
    return TaktOSQueueSend(&s_GatewayWorkQue, &work, false, 0) == TAKTOS_OK;
}

bool BtEvtQue(uint32_t Evt, void *pCtx, BtEvtQueHandler_t Handler)
{
    return GatewayWorkQue(Evt, pCtx, Handler);
}

bool LteEvtQue(uint32_t Evt, void *pCtx, LteEvtQueHandler_t Handler)
{
    return GatewayWorkQue(Evt, pCtx, Handler);
}

static void LteUdpEvtHandler(LTE_EVT Evt, const LteStatus_t * const pStatus);
static void LteUdpUrcHandler(const char *pUrc);
static int LteUdpSockEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len);
static void LteUdpRecv(uint32_t Evt, void *pCtx);

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

// The UART callback runs in interrupt context. Only signal that more console
// TX space is available; AppCheckStatus drains SysLog from the main context.
static volatile bool s_bLogTxReady = false;

static int GatewayUartEvt(UARTDev_t * const pDev, UART_EVT EvtId,
                          uint8_t *pBuffer, int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	if (EvtId == UART_EVT_TXREADY)
	{
		s_bLogTxReady = true;
	}

	return 0;
}

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtCallback = GatewayUartEvt,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

// Low frequency timer: modem waits and the send interval
static const TimerCfg_t s_TimerCfg = {
	.DevNo = LTE_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0,
	.IntPrio = IRQ_PRIO_LOW,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static TimerDev_t s_TimerDev;

static const LteCfg_t s_LteCfg = {
	.Rat = LTE_UDP_RAT,
	.RatPref = LTE_UDP_RAT_PREF,
	.bGnss = false,
	.pBand = nullptr,			// Band setting of the modem kept
	.NbBand = 0,
	.pApn = LTE_UDP_APN,
	.PdnType = LTE_PDN_IPV4V6,
	.bPsm = true,
	.PsmTau = 3600,				// Periodic TAU requested: 1 h
	.PsmActive = 10,			// Active time requested: 10 s
	.EdrxCycle = 0,
	.bRai = LTE_UDP_RAI != 0,
	.IntPrio = IRQ_PRIO_LOW,
	.pTimer = &s_TimerDev,
	.TimerTrigNo = LTE_UDP_TRIG_MODEM,
	.UrcMemSize = 0,
	.EvtHandler = LteUdpEvtHandler,
	.UrcHandler = LteUdpUrcHandler,
};

static const SockIntrfCfg_t s_SockCfg = {
	.Proto = SOCKINTRF_PROTO_UDP,
	.pHost = LTE_UDP_HOST,
	.Port = LTE_UDP_PORT,
	.LocalPort = 0,
	.SecTag = -1,
	.bPeerVerify = false,
	.Timeout = 0,
	.EvtCB = LteUdpSockEvt,
};

static UART s_Uart;

// SysLog on the console UART, a record goes out as soon as it is logged.
// The record holds the longest received datagram plus the timestamp and
// formatting. A record the UART does not take in full stays in the store and
// the rest goes out at the next log call. bBlocking keeps the oldest records
// when the store is full.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, LTE_UDP_RX_LEN + 64)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = LTE_UDP_RX_LEN + 64,
	.bBlocking = true,
};

static SockIntrf s_Sock;
static uint32_t s_TxCount = 0;

// RTC0 is the existing application timebase. Its counter measures elapsed
// monotonic time, not UTC, and must not be read until TimerInit succeeds.
static bool s_bTimerReady = false;
static volatile uint32_t s_UdpTimerEvents = 0U;
static volatile bool s_bSendPending = false;
static volatile bool s_bSendQueued = false;

// Prefix every application log line without changing the shared SysLog ABI.
static int GatewayLog(const char *pFormat, ...)
{
    char detail[LTE_UDP_RX_LEN + 32];
    va_list args;
    va_start(args, pFormat);
    const int len = vsnprintf(detail, sizeof(detail), pFormat, args);
    va_end(args);
    if (len < 0)
    {
        return len;
    }

    if (!s_bTimerReady)
    {
        return SysLogPrintf(SysLogGet(), "[boot] %s", detail);
    }

    const uint64_t ticks = TimerGetTickCount(&s_TimerDev);
    const uint32_t freq = TimerGetFrequency(&s_TimerDev);
    if (freq == 0U)
    {
        return SysLogPrintf(SysLogGet(), "[timer unavailable] %s", detail);
    }

    const uint32_t seconds = (uint32_t)(ticks / freq);
    const uint32_t millis = (uint32_t)(((ticks % freq) * 1000ULL) / freq);
    return SysLogPrintf(SysLogGet(), "[%lu.%03lu] %s",
                        (unsigned long)seconds, (unsigned long)millis, detail);
}


// Data arrived and not read yet: queued once, kept when the queue refused it
static volatile bool s_bRxPending = false;
static volatile bool s_bRxQueued = false;

static void LteUdpSend(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

    s_bSendPending = false;
    s_bSendQueued = false;

	// A read the queue refused
	if (s_bRxPending)
	{
		LteUdpRecv(LTE_UDP_EVT_RX, nullptr);
	}

	if (LteRegistered() == false)
	{
		return;
	}

	if (s_Sock.Connected() == false && s_Sock.Init(s_SockCfg) == false)
	{
		GatewayLog( "Socket to %s:%d failed\r\n", LTE_UDP_HOST, LTE_UDP_PORT);
		return;
	}

	char msg[LTE_UDP_MSG_LEN];
	int rsrp = 0;

	if (LteGetSignal(&rsrp, nullptr) == false)
	{
		rsrp = 0;
	}

	uint8_t address[6] = {};
    int8_t rssi = 0;
    uint32_t reports = 0;
    bool valid = false;
    bool sensorValid = false;
    uint8_t sensorType = 0U;
    uint8_t tph[8] = {};
    uint8_t sensorAddress[6] = {};
    const uint32_t irq = DisableInterrupt();
    valid = s_ScanValid;
    for (unsigned i = 0; i < 6U; ++i)
    {
        address[i] = s_ScanAddress[i];
    }
    rssi = s_ScanRssi;
    reports = s_ScanReports;
    sensorValid = s_SensorValid;
    sensorType = s_SensorType;
    for (unsigned i = 0; i < sizeof(sensorAddress); ++i)
    {
        sensorAddress[i] = s_SensorAddress[i];
    }
    for (unsigned i = 0; i < sizeof(tph); ++i)
    {
        tph[i] = s_SensorData[i];
    }
    EnableInterrupt(irq);

    int len = snprintf(msg, sizeof(msg),
                       "gateway seq=%lu rsrp=%d reports=%lu ble=%s%02x:%02x:%02x:%02x:%02x:%02x rssi=%d",
                       (unsigned long)s_TxCount, rsrp, (unsigned long)reports,
                       valid ? "" : "none/",
                       address[5], address[4], address[3],
                       address[2], address[1], address[0], (int)rssi);
    if (sensorValid && sensorType == BT_ADV_MANDATA_TYPE_TPH &&
        len > 0 && len < (int)sizeof(msg))
    {
        const uint32_t pressure = (uint32_t)tph[0] |
            ((uint32_t)tph[1] << 8) | ((uint32_t)tph[2] << 16) |
            ((uint32_t)tph[3] << 24);
        const int16_t temperature = (int16_t)((uint16_t)tph[4] |
            ((uint16_t)tph[5] << 8));
        const uint16_t humidity = (uint16_t)tph[6] |
            ((uint16_t)tph[7] << 8);
        const int extra = snprintf(msg + len, sizeof(msg) - (size_t)len,
                                   " tag=%02x:%02x:%02x:%02x:%02x:%02x tph_pa=%lu tph_centi_c=%d tph_centi_pct=%u",
                                   sensorAddress[5], sensorAddress[4],
                                   sensorAddress[3], sensorAddress[2],
                                   sensorAddress[1], sensorAddress[0],
                                   (unsigned long)pressure, (int)temperature,
                                   (unsigned)humidity);
        if (extra < 0 || extra >= (int)(sizeof(msg) - (size_t)len))
        {
            return;
        }
        len += extra;
    }
    if (sensorValid && sensorType == BT_ADV_MANDATA_TYPE_GAS &&
        len > 0 && len < (int)sizeof(msg))
    {
        const uint32_t resistance = (uint32_t)tph[0] |
            ((uint32_t)tph[1] << 8) | ((uint32_t)tph[2] << 16) |
            ((uint32_t)tph[3] << 24);
        const uint16_t airQuality = (uint16_t)tph[4] |
            ((uint16_t)tph[5] << 8);
        const int extra = snprintf(msg + len, sizeof(msg) - (size_t)len,
                                   " tag=%02x:%02x:%02x:%02x:%02x:%02x gas_res=%lu aqi=%u",
                                   sensorAddress[5], sensorAddress[4],
                                   sensorAddress[3], sensorAddress[2],
                                   sensorAddress[1], sensorAddress[0],
                                   (unsigned long)resistance, (unsigned)airQuality);
        if (extra < 0 || extra >= (int)(sizeof(msg) - (size_t)len))
        {
            return;
        }
        len += extra;
    }
    for (unsigned i = 0; i < GATEWAY_MAX_TAGS && len > 0; ++i)
    {
        const GatewayTag_t *pTag = &s_Tags[i];
        if (!pTag->Active || pTag->ConnHdl == BT_CONN_HDL_INVALID ||
            (!pTag->TempValid && !pTag->PressureValid && !pTag->HumidityValid))
        {
            continue;
        }
        const int extra = snprintf(msg + len, sizeof(msg) - (size_t)len,
            " g%u=%02x:%02x:%02x:%02x:%02x:%02x,%d,%lu,%u",
            i, pTag->Addr[5], pTag->Addr[4], pTag->Addr[3],
            pTag->Addr[2], pTag->Addr[1], pTag->Addr[0],
            pTag->TempValid ? (int)pTag->TempCenti : 0,
            pTag->PressureValid ? (unsigned long)pTag->PressurePa : 0UL,
            pTag->HumidityValid ? (unsigned)pTag->HumidityPct : 0U);
        if (extra < 0 || extra >= (int)(sizeof(msg) - (size_t)len))
        {
            return;
        }
        len += extra;
    }
    if (len <= 0 || len >= (int)sizeof(msg))
    {
        return;
    }

	// The send is the last of the exchange: the network releases the radio
	// connection right after it
	if (LTE_UDP_RAI)
	{
		s_Sock.Rai(SOCKINTRF_RAI_LAST);
	}

	if (s_Sock.Tx(0, (uint8_t *)msg, len) == len)
	{
		GatewayLog( "Sent %d (rtc_events=%lu): %s\r\n", len, (unsigned long)s_UdpTimerEvents, msg);
		s_TxCount++;
	}
	else
	{
		GatewayLog( "Send failed\r\n");
	}
}

static void LteUdpRecv(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	// Allow the next socket callback to queue a read.
	s_bRxPending = false;
	s_bRxQueued = false;

	uint8_t buf[LTE_UDP_RX_LEN];
	int n;

	while ((n = s_Sock.Rx(0, buf, sizeof(buf) - 1)) > 0)
	{
		buf[n] = 0;
		GatewayLog( "Received %d: %s\r\n", n, (char *)buf);
	}
}

// Interrupt context: queue the read
static int LteUdpSockEvt(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;
	(void)Len;

	if (EvtId == DEVINTRF_EVT_RX_DATA && s_bRxPending == false)
	{
		s_bRxPending = true;
        if (GatewayWorkQue(LTE_UDP_EVT_RX, nullptr, LteUdpRecv))
        {
            s_bRxQueued = true;
        }
	}

	return 0;
}

// Interrupt context: queue the send
static void LteUdpTimerTrig(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pContext;

    s_UdpTimerEvents = s_UdpTimerEvents + 1U;
    s_bSendPending = true;
    if (!s_bSendQueued && GatewayWorkQue(LTE_UDP_EVT_SEND, nullptr, LteUdpSend))
    {
        s_bSendQueued = true;
    }
}

// Retry timer events rejected by the shared BLE/LTE application queue.
bool AppCheckStatus(void)
{
	if (s_bLogTxReady)
	{
		s_bLogTxReady = false;
		(void)SysLogFlush(SysLogGet());
	}

	// Issue scan and connection commands only outside HCI event callbacks.
	if (s_GattScanRestartPending)
	{
		s_GattScanRestartPending = false;
		BtAppScan();
	}

    if (s_pInitiating == nullptr)
    {
        for (unsigned i = 0; i < GATEWAY_MAX_TAGS; ++i)
        {
            GatewayTag_t *pTag = &s_Tags[i];
            if (!pTag->Active || !pTag->Pending ||
                pTag->ConnHdl != BT_CONN_HDL_INVALID)
            {
                continue;
            }
            pTag->Pending = false;
            pTag->Connecting = true;
            s_pInitiating = pTag;
            BtGapPeerAddr_t peer = {};
            peer.Type = pTag->AddrType;
            memcpy(peer.Addr, pTag->Addr, sizeof(peer.Addr));
            BtGapScanStop();
            s_ConnectCancelPending = false;
            s_ConnectStartTicks = TimerGetTickCount(&s_TimerDev);
            if (!BtGapConnect(&peer, &s_GattConnParams))
            {
                pTag->Connecting = false;
                s_pInitiating = nullptr;
                s_GattScanRestartPending = true;
            }
            break;
        }
    }

    if (s_pInitiating != nullptr && !s_ConnectCancelPending &&
        s_bTimerReady)
    {
        const uint32_t freq = TimerGetFrequency(&s_TimerDev);
        if (freq != 0U &&
            (TimerGetTickCount(&s_TimerDev) - s_ConnectStartTicks) / freq >= 30U)
        {
            if (BtGapConnectCancel())
            {
                s_ConnectCancelPending = true;
                GatewayLog("GATT initiation timed out; cancel requested\r\n");
            }
        }
    }

    BtAppCheckStatus();
    LteCheckStatus();

    if (s_bRxPending && !s_bRxQueued &&
        GatewayWorkQue(LTE_UDP_EVT_RX, nullptr, LteUdpRecv))
    {
        s_bRxQueued = true;
    }

    if (s_bSendPending && !s_bSendQueued &&
        GatewayWorkQue(LTE_UDP_EVT_SEND, nullptr, LteUdpSend))
    {
        s_bSendQueued = true;
    }

    return !s_bLogTxReady && !s_bSendPending && !s_bRxPending;
}

static void LteUdpEvtHandler(LTE_EVT Evt, const LteStatus_t * const pStatus)
{
	switch (Evt)
	{
		case LTE_EVT_REGISTERED:
			GatewayLog( "Registered %s, %s, TAC %04X, cell %08X\r\n",
									  pStatus->RegStat == LTE_REG_ROAMING ? "roaming" : "home",
									  pStatus->Rat == LTE_RAT_NBIOT ? "NB-IoT" : "LTE-M",
									  (unsigned)pStatus->Tac, (unsigned)pStatus->CellId);
			if (pStatus->bPsm)
			{
				GatewayLog( "PSM TAU %u s, active %u s\r\n",
										  (unsigned)pStatus->PsmTau, (unsigned)pStatus->PsmActive);
			}
			LteUdpSend(0, nullptr);
			break;

		case LTE_EVT_UNREGISTERED:
			GatewayLog( "Unregistered, state %d\r\n", (int)pStatus->RegStat);
			break;

		case LTE_EVT_REG_STATE:
			GatewayLog( "Registration state %d\r\n", (int)pStatus->RegStat);
			break;

		case LTE_EVT_CELL:
			GatewayLog( "Cell TAC %04X, cell %08X\r\n", (unsigned)pStatus->Tac, (unsigned)pStatus->CellId);
			break;

		case LTE_EVT_RRC_CONNECTED:
			GatewayLog( "RRC connected\r\n");
			break;

		case LTE_EVT_RRC_IDLE:
			GatewayLog( "RRC idle\r\n");
			break;

		case LTE_EVT_PSM:
			GatewayLog( "PSM %s, TAU %u s, active %u s\r\n", pStatus->bPsm ? "on" : "off",
									  (unsigned)pStatus->PsmTau, (unsigned)pStatus->PsmActive);
			break;

		case LTE_EVT_EDRX:
			GatewayLog( "eDRX %u ms, PTW %u ms\r\n", (unsigned)pStatus->EdrxCycle, (unsigned)pStatus->EdrxPtw);
			break;

		case LTE_EVT_MODEM_FAULT:
			GatewayLog( "Modem fault, restarting\r\n");
			s_Sock.Close();
			if (LteInit(&s_LteCfg) == false)
			{
				GatewayLog( "LTE init failed\r\n");
			}
			break;
	}
}

static void LteUdpUrcHandler(const char *pUrc)
{
	GatewayLog( "URC %s\r\n", pUrc);
}

// A single TaktOS worker owns the shared BLE/LTE event queue and all
// synchronous GAP commands. The modem's TaktOS OS adapter is active here.
static void GatewayTask(void *pCtx)
{
    (void)pCtx;
    s_Uart.Init(s_UartCfg);
    SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
    GatewayLog( "BleGatewayTaktOS UDP starting\r\n");

    // BLE and LTE share the worker's application event dispatcher.
    // The scan callback never calls the socket.
    bool bleReady = BtAppInit(&s_BleAppCfg);
    if (bleReady)
    {
        bleReady = BtAppConnInit();
        GatewayLog("BLE connection support %s\r\n",
                   bleReady ? "ready" : "failed");
    }
    if (bleReady)
    {
        bleReady = BtAppScanInit(&s_BleScanCfg);
        if (bleReady)
        {
            BtAppScan();
        }
    }
    GatewayLog( "BLE scan %s\r\n",
                 bleReady ? "started" : "failed");

	char info[48];

	if (TimerInit(&s_TimerDev, &s_TimerCfg) == false)
	{
		GatewayLog( "Timer init failed\r\n");
	}
    else
    {
        s_bTimerReady = true;
        if (LteInit(&s_LteCfg) == false)
        {
            GatewayLog("LTE init failed\r\n");
        }
        else
        {
		if (LteGetInfo(LTE_INFO_FWVER, info, sizeof(info)))
		{
			GatewayLog( "Modem firmware %s\r\n", info);
		}
		if (LteGetInfo(LTE_INFO_IMEI, info, sizeof(info)))
		{
			GatewayLog( "IMEI %s\r\n", info);
		}
		msTimerEnableTrigger(&s_TimerDev, LTE_UDP_TRIG_SEND, LTE_UDP_INTERVAL * 1000U,
							 TIMER_TRIG_TYPE_CONTINUOUS, LteUdpTimerTrig, nullptr);
		GatewayLog( "Attaching\r\n");
        }
    }

    while (true)
    {
        GatewayWork_t work;
        if (TaktOSQueueReceive(&s_GatewayWorkQue, &work, false, 0) != TAKTOS_OK)
        {
            AppCheckStatus();
            // A bounded wait also services rejected work and asynchronous
            // console TX progress when the queue is otherwise idle.
            if (TaktOSQueueReceive(&s_GatewayWorkQue, &work, true, 50U) != TAKTOS_OK)
            {
                continue;
            }
        }
        work.Handler(work.Evt, work.pCtx);
    }
}

#define GATEWAY_TAKTOS_STACK 4096U
#define GATEWAY_TAKTOS_TICK_HZ 1000U

static uint8_t s_GatewayTaskMem[TAKTOS_THREAD_MEM_SIZE(GATEWAY_TAKTOS_STACK)] TAKT_ALIGNED(4);

int main()
{
    if (TaktOSQueueInit(&s_GatewayWorkQue, s_GatewayWorkMem,
                        sizeof(GatewayWork_t), GATEWAY_WORK_QUE_COUNT) != TAKTOS_OK)
    {
        while (true)
        {
            __WFE();
        }
    }

    TaktOSCfg_t cfg = {
        .KernClockHz = SystemCoreClockGet(),
        .TickHz = GATEWAY_TAKTOS_TICK_HZ,
    };
    if (TaktOSInit(&cfg) != TAKTOS_OK)
    {
        while (true)
        {
            __WFE();
        }
    }

    hTaktOSThread_t task = TaktOSThreadCreate(
        s_GatewayTaskMem, sizeof(s_GatewayTaskMem),
        GatewayTask, nullptr, TAKTOS_PRIORITY_NORMAL);
    if (task == nullptr)
    {
        while (true)
        {
            __WFE();
        }
    }

    TaktOSStart();
    return 0;
}
