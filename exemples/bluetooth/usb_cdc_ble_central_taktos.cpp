/**-------------------------------------------------------------------------
@example	usb_cdc_ble_central_taktos.cpp

@brief	USB CDC to BLE central bridge with TaktOS

TaktOS version of usb_cdc_ble_central.cpp. The board enumerates as a USB CDC
serial port, scans for a BLE peripheral running the BlueIO UART service
(uart_ble.cpp, advertising as "UARTDemo") and bridges the two: bytes written
by the host go to the peer UART Tx characteristic, notifications from the peer
UART Rx characteristic come back on the serial port, together with the status
lines of the bridge (scan, connection, discovery).

Each subsystem has its own thread and its own work queue. The library
defaults of UsbEvtQue and BtEvtQue are overridden to send the work as a
message to that thread; the application event queue is not used.

	UsbThread : USB work, CDC Rx -> s_ToBleFifo, s_ToHostFifo -> CDC Tx
	BleThread : Bluetooth work, s_ToBleFifo -> BtAppWrite, 20 bytes at a time

The bridge steps are queued the same way, at most once at a time (pending
flag). A peer notification arrives in the stack interrupt: it is copied to
s_ToHostFifo and the USB thread is told.

The link is open (no pairing). Build the peer with BLE_SC_METHOD set to
BLE_SC_NONE: a peer asking for security disconnects this central.

board.h gives LED_PINS and the CONNECT_LED_PORT / CONNECT_LED_PIN /
CONNECT_LED_LOGIC of the link LED.

@author	Hoang Nguyen Hoan
@date	Oct. 3, 2026

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

#include "istddef.h"
#include "cfifo.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/blueio_blesrvc.h"
#include "TaktOS.h"
#include "TaktOSThread.h"
#include "TaktOSQueue.h"

#include "board.h"

#define DEVICE_NAME				"UsbBleCentral"

// Peripheral to connect to, by its advertised name
#define TARGET_DEV_NAME			"UARTDemo"

#define MIN_CONN_INTERVAL		7.5		// msec
#define MAX_CONN_INTERVAL		40		// msec

#define SCAN_INTERVAL			1000	// msec
#define SCAN_WINDOW				100		// msec
#define SCAN_TIMEOUT			0		// 0 : no timeout

#define BLE_MTU_SIZE			247

// The peer UART characteristics hold 20 octets (PACKET_SIZE in uart_ble.cpp).
// A longer write is refused, not truncated.
#define BLE_WRITE_MAX			20

#define USB_DEVNO				0

// One full speed bulk packet
#define USB_PKT_SIZE			USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

//
// Threads and their work queues
//

// One message per queued piece of work. Same shape for USB and Bluetooth.
typedef struct {
	uint32_t EvtId;
	void *pCtx;
	void (*Handler)(uint32_t EvtId, void *pCtx);
} Work_t;

#define WORK_QUE_SIZE			16U

alignas(4) static uint8_t s_UsbWorkQueMem[WORK_QUE_SIZE * sizeof(Work_t)];
alignas(4) static uint8_t s_BleWorkQueMem[WORK_QUE_SIZE * sizeof(Work_t)];
static TaktOSQueue_t s_UsbWorkQue;
static TaktOSQueue_t s_BleWorkQue;

alignas(8) static uint8_t s_UsbThreadMem[TAKTOS_THREAD_MEM_SIZE(2048)];
alignas(8) static uint8_t s_BleThreadMem[TAKTOS_THREAD_MEM_SIZE(1024)];

static void BridgeCheckStatus(TaktOSQueue_t *pQue);

//
// USB CDC
//

#define CDC_RXFIFO_PKTCNT		4
#define CDC_RXFIFO_MEMSIZE		USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, USB_PKT_SIZE)
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len);

static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = CdcEvtHandler,
};

// 0x1209 is the pid.codes vendor id, which exists for open hardware. Put your
// own vendor and product id here before shipping anything.
static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x000A,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB BLE Central TaktOS",
	.pSerial = nullptr,			// Taken from the MCU unique id
	.pFuncName = "IOsonata CDC",
	.IntPrio = 6,				// Below the priorities a radio stack keeps
	.DeviceClass = USB_DEVCLASS_MISC,
	.DeviceSubClass = 2U,
	.DeviceProtocol = 1U,
	.bSelfPowered = false,
	.bRemoteWakeup = false,
	.bLowPowerSuspend = false,
	.MaxPower = 100,
	.EvtHandler = nullptr,
};

UsbdCdc g_Cdc;

static const IOPinCfg_t s_Leds[] = LED_PINS;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

//
// Bluetooth central
//

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,
	.PeriphDevMax = 1,					// Max peripheral devices we connect to as central
	.CentralDevMax = 0,					// Max central devices we serve as peripheral
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
	.AdvInterval = 0,
	.AdvTimeout = 0,
	.AdvSlowInterval = 0,
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,
	.ConnLedPin = CONNECT_LED_PIN,
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,
	.MaxMtu = BLE_MTU_SIZE,
};

static const BtGapScanCfg_t s_ScanCfg = {
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

static volatile uint16_t s_ConnHdl = BT_CONN_HDL_INVALID;
static volatile uint16_t s_BleTxCharHdl = BT_ATT_HANDLE_INVALID;	// write target on the peer
static volatile uint16_t s_BleRxCharHdl = BT_ATT_HANDLE_INVALID;	// notify source on the peer

//
// Bridge
//

// Host to peer. The USB thread is the only producer, the BLE thread the only
// consumer.
alignas(4) static uint8_t s_ToBleFifoMem[CFIFO_MEMSIZE(512)];
static hCFifo_t s_hToBleFifo;

// Peer to host: notifications (stack interrupt) and status lines (stack
// interrupt and threads), put with the interrupts masked. The USB thread is
// the only consumer. Kept while the port is closed, up to the FIFO size.
alignas(4) static uint8_t s_ToHostFifoMem[CFIFO_MEMSIZE(1024)];
static hCFifo_t s_hToHostFifo;
static volatile uint32_t s_ToHostDropCnt = 0;

static volatile bool s_bUsbReadQueued = false;
static volatile bool s_bUsbReadOwed = false;
static volatile bool s_bBleWriteQueued = false;
static volatile bool s_bBleWriteOwed = false;
static volatile bool s_bHostTxQueued = false;
static volatile bool s_bHostTxOwed = false;

static void UsbReadEvt(uint32_t Evt, void *pCtx);
static void BleWriteEvt(uint32_t Evt, void *pCtx);
static void HostTxEvt(uint32_t Evt, void *pCtx);

// BLE thread. Data taken from s_ToBleFifo, not yet written to the peer.
static uint8_t s_BleTxBuf[BLE_WRITE_MAX];
static int s_BleTxLen = 0;

// USB thread. Data taken from s_ToHostFifo, not yet accepted by the CDC.
static uint8_t s_HostTxBuf[64];
static int s_HostTxLen = 0;
static int s_HostTxOff = 0;

// Interrupt or thread context. Never blocks.
static bool WorkSend(TaktOSQueue_t *pQue, uint32_t EvtId, void *pCtx,
					 void (*Handler)(uint32_t, void *))
{
	const Work_t work = { EvtId, pCtx, Handler };

	return TaktOSQueueSend(pQue, &work, false, 0) == TAKTOS_OK;
}

// Link-time overrides of the library defaults, see usb.h and bt_app.h. The
// stacks call them from their interrupts and from the thread running the work.
bool UsbEvtQue(uint32_t EvtId, void *pCtx, UsbEvtQueHandler_t Handler)
{
	return WorkSend(&s_UsbWorkQue, EvtId, pCtx, Handler);
}

bool BtEvtQue(uint32_t EvtId, void *pCtx, BtEvtQueHandler_t Handler)
{
	return WorkSend(&s_BleWorkQue, EvtId, pCtx, Handler);
}

// Run the work of one queue as its messages arrive, forever
static void WorkThread(void *pArg)
{
	TaktOSQueue_t *pQue = (TaktOSQueue_t *)pArg;

	while (true)
	{
		Work_t work;
		if (TaktOSQueueReceive(pQue, &work, false, 0) != TAKTOS_OK)
		{
			if (pQue == &s_UsbWorkQue)
			{
				UsbCheckStatus();
			}
			else
			{
				BtAppCheckStatus();
			}
			BridgeCheckStatus(pQue);
			if (TaktOSQueueReceive(pQue, &work, true, TAKTOS_WAIT_FOREVER) != TAKTOS_OK)
			{
				continue;
			}
		}
		work.Handler(work.EvtId, work.pCtx);
	}
}

static void Que(volatile bool *pbQueued, volatile bool *pbOwed,
				TaktOSQueue_t *pQue, void (*Handler)(uint32_t, void *))
{
	// Finish recording admission before a woken worker can run the callback.
	uint32_t state = DisableInterrupt();
	if (*pbQueued == false)
	{
		*pbQueued = true;
		*pbOwed = WorkSend(pQue, 0, nullptr, Handler) == false;
		if (*pbOwed)
		{
			*pbQueued = false;
		}
	}
	EnableInterrupt(state);
}

static void UsbReadQue(void)
{
	Que(&s_bUsbReadQueued, &s_bUsbReadOwed, &s_UsbWorkQue, UsbReadEvt);
}

static void BleWriteQue(void)
{
	Que(&s_bBleWriteQueued, &s_bBleWriteOwed, &s_BleWorkQue, BleWriteEvt);
}

static void HostTxQue(void)
{
	Que(&s_bHostTxQueued, &s_bHostTxOwed, &s_UsbWorkQue, HostTxEvt);
}

// Retry only work refused by this worker's queue.
static void BridgeCheckStatus(TaktOSQueue_t *pQue)
{
	if (pQue == &s_UsbWorkQue)
	{
		if (s_bUsbReadOwed)
		{
			UsbReadQue();
		}
		if (s_bHostTxOwed)
		{
			HostTxQue();
		}
	}
	else if (s_bBleWriteOwed)
	{
		BleWriteQue();
	}
}

// Copy to the host FIFO, from any context
static void ToHostPut(const uint8_t *pData, int Len)
{
	uint32_t state = DisableInterrupt();

	// Two passes when the data wraps the end of the FIFO memory
	for (int i = 0; i < 2 && Len > 0; i++)
	{
		int l = Len;
		uint8_t *p = CFifoPutMultiple(s_hToHostFifo, &l);
		if (p == nullptr)
		{
			break;
		}
		memcpy(p, pData, l);
		pData += l;
		Len -= l;
	}

	EnableInterrupt(state);

	if (Len > 0)
	{
		s_ToHostDropCnt += Len;
	}

	HostTxQue();
}

// Status line to the USB port
static void Status(const char *pFormat, ...)
{
	char line[96];
	va_list args;

	va_start(args, pFormat);
	int l = vsnprintf(line, sizeof(line), pFormat, args);
	va_end(args);

	if (l <= 0)
	{
		return;
	}
	if (l >= (int)sizeof(line))
	{
		l = sizeof(line) - 1;
	}

	ToHostPut((const uint8_t *)line, l);
}

static bool BridgeReady(void)
{
	return s_ConnHdl != BT_CONN_HDL_INVALID && s_BleTxCharHdl != BT_ATT_HANDLE_INVALID;
}

// USB thread. Move host data from the CDC to s_ToBleFifo while it has room
// for a packet. Data the FIFO cannot take stays in the CDC, so the host is
// held back by the USB flow control instead of losing bytes.
static void UsbReadEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bUsbReadQueued = false;

	while (CFifoAvail(s_hToBleFifo) >= USB_PKT_SIZE)
	{
		uint8_t buf[USB_PKT_SIZE];
		int n = g_Cdc.Rx(0, buf, sizeof(buf));
		if (n <= 0)
		{
			return;
		}

		int l = n;
		uint8_t *p = CFifoResvMultiple(s_hToBleFifo, &l);
		if (p != nullptr)
		{
			memcpy(p, buf, l);
			(void)CFifoPutMultiple(s_hToBleFifo, &l);
			if (l < n)
			{
				int r = n - l;
				uint8_t *q = CFifoResvMultiple(s_hToBleFifo, &r);
				if (q != nullptr)
				{
					memcpy(q, &buf[l], r);
					(void)CFifoPutMultiple(s_hToBleFifo, &r);
				}
			}
		}

		BleWriteQue();
	}
}

static void BleWriteEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bBleWriteQueued = false;

	while (BridgeReady())
	{
		if (s_BleTxLen == 0)
		{
			int l = BLE_WRITE_MAX;
			uint8_t *p = CFifoPeekMultiple(s_hToBleFifo, &l);
			if (p == nullptr)
			{
				break;
			}
			memcpy(s_BleTxBuf, p, l);
			(void)CFifoGetMultiple(s_hToBleFifo, &l);
			s_BleTxLen = l;

			// Room again for the host data waiting in the CDC
			UsbReadQue();
		}

		if (BtAppWrite(s_ConnHdl, s_BleTxCharHdl, s_BleTxBuf, (uint16_t)s_BleTxLen) == false)
		{
			// Stack Tx queue full. The port has no generic write done event:
			// give it one tick, then try again.
			(void)TaktOSThreadSleepTicks(TaktOSCurrentThread(), 1);
			BleWriteQue();
			return;
		}

		s_BleTxLen = 0;
	}
}

static void HostTxEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bHostTxQueued = false;

	while (true)
	{
		if (s_HostTxOff >= s_HostTxLen)
		{
			s_HostTxOff = 0;
			s_HostTxLen = sizeof(s_HostTxBuf);

			uint32_t state = DisableInterrupt();
			uint8_t *p = CFifoGetMultiple(s_hToHostFifo, &s_HostTxLen);
			if (p != nullptr)
			{
				memcpy(s_HostTxBuf, p, s_HostTxLen);
			}
			EnableInterrupt(state);

			if (p == nullptr)
			{
				s_HostTxLen = 0;
				return;
			}
		}

		if (g_Cdc.IsPortOpen() == false)
		{
			// Kept for the port open, which queues this again
			return;
		}

		int n = g_Cdc.Tx(0, &s_HostTxBuf[s_HostTxOff], s_HostTxLen - s_HostTxOff);
		if (n <= 0)
		{
			// CDC Tx FIFO full, TX_FIFO_EMPTY queues this again
			return;
		}
		s_HostTxOff += n;
	}
}

// Data callbacks may run in the controller interrupt or the USB worker.
static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;

	switch (EvtId)
	{
		case DEVINTRF_EVT_RX_DATA:
		case DEVINTRF_EVT_RX_FIFO_FULL:
			UsbReadQue();
			break;

		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			if (s_HostTxOff < s_HostTxLen || CFifoUsed(s_hToHostFifo) > 0)
			{
				HostTxQue();
			}
			break;

		case DEVINTRF_EVT_STATECHG:
			if (Len)
			{
				Status("\r\nIOsonata USB BLE Central TaktOS, target %s, %s\r\n",
					   TARGET_DEV_NAME, BridgeReady() ? "bridge ready" :
					   s_ConnHdl != BT_CONN_HDL_INVALID ? "connected" : "scanning");
			}
			break;

		default:
			break;
	}

	return 0;
}

//
// Bluetooth events, stack interrupt context
//

void BtAppEvtConnected(uint16_t ConnHdl)
{
	s_ConnHdl = ConnHdl;
	Status("Connected, ConnHdl %d\r\n", ConnHdl);

	// Open link: the peer GATT server can be read now
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer == nullptr || BtAppDiscoverDevice(pPeer) == false)
	{
		Status("Discovery not started\r\n");
	}
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	Status("Disconnected, ConnHdl %d\r\n", ConnHdl);

	s_ConnHdl = BT_CONN_HDL_INVALID;
	s_BleTxCharHdl = BT_ATT_HANDLE_INVALID;
	s_BleRxCharHdl = BT_ATT_HANDLE_INVALID;

	// Scanning stopped when the target was found. Look for it again.
	BtAppScan();
}

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData)
{
	char name[32];
	size_t l = BtAdvDataGetDevName(pAdvData, AdvLen, name, sizeof(name) - 1);

	if (l == 0)
	{
		return true;	// keep scanning
	}
	if (l >= sizeof(name))
	{
		l = sizeof(name) - 1;
	}
	name[l] = 0;

	if (strcmp(name, TARGET_DEV_NAME) != 0)
	{
		Status("Seen %s, RSSI %d\r\n", name, Rssi);
		return true;
	}

	Status("Found %s, RSSI %d\r\n", name, Rssi);
	BtGapScanStop();

	BtGapPeerAddr_t addr = { .Type = AddrType };
	memcpy(addr.Addr, Addr, 6);
	BtGapConnect(&addr, &s_ConnParams);

	return false;		// stop scan reporting
}

void BtDeviceDiscovered(BtDevice_t *pDev)
{
	if (pDev == nullptr)
	{
		return;
	}

	int sidx = BtDeviceFindService(pDev, BLUEIO_UUID_UART_SERVICE);
	if (sidx < 0)
	{
		Status("UART service not found\r\n");
		return;
	}

	int rxidx = BtDeviceFindCharacteristic(pDev, sidx, BLUEIO_UUID_UART_RX_CHAR);
	int txidx = BtDeviceFindCharacteristic(pDev, sidx, BLUEIO_UUID_UART_TX_CHAR);
	if (rxidx < 0 || txidx < 0)
	{
		Status("UART characteristics not found (rx %d, tx %d)\r\n", rxidx, txidx);
		return;
	}

	uint16_t cccd = pDev->pServices[sidx].characteristics[rxidx].cccd_handle;
	if (cccd == BT_ATT_HANDLE_INVALID || BtAppEnableNotify(pDev->Conn.Hdl, cccd) == false)
	{
		Status("Notify not enabled\r\n");
		return;
	}

	s_BleRxCharHdl = pDev->pServices[sidx].characteristics[rxidx].characteristic.handle_value;
	s_BleTxCharHdl = pDev->pServices[sidx].characteristics[txidx].characteristic.handle_value;

	Status("Bridge ready, Rx 0x%04X, Tx 0x%04X\r\n", s_BleRxCharHdl, s_BleTxCharHdl);

	// Host data may have been waiting for the peer
	BleWriteQue();
	UsbReadQue();
}

void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl, uint8_t *pData, uint16_t Len)
{
	(void)ConnHdl;

	if (ValHdl != s_BleRxCharHdl || pData == nullptr || Len == 0)
	{
		return;
	}

	ToHostPut(pData, Len);
}

void BtAppInitUserServices(void)
{
	// A central with no local service
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
}

void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
}

//
// Main
//

int main()
{
	IOPinCfg(s_Leds, s_NbLeds);
	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	s_hToBleFifo = CFifoInit(s_ToBleFifoMem, sizeof(s_ToBleFifoMem), 1, true);
	s_hToHostFifo = CFifoInit(s_ToHostFifoMem, sizeof(s_ToHostFifoMem), 1, true);

	const TaktOSCfg_t cfg = {
		.KernClockHz = SystemCoreClockGet(),
		.TickHz = 1000,
		.TickClockSrc = TAKTOS_TICK_CLOCK_PROCESSOR,
	};

	// The work queues exist before either stack can queue its first work
	if (TaktOSInit(&cfg) != TAKTOS_OK ||
		TaktOSQueueInit(&s_UsbWorkQue, s_UsbWorkQueMem, sizeof(Work_t), WORK_QUE_SIZE) != TAKTOS_OK ||
		TaktOSQueueInit(&s_BleWorkQue, s_BleWorkQueMem, sizeof(Work_t), WORK_QUE_SIZE) != TAKTOS_OK ||
		UsbInit(&s_UsbCfg) == false || g_Cdc.Init(s_CdcCfg) == false)
	{
		// Nowhere to report it, stop here for the debugger
		while (true) {}
	}

	// A Bluetooth failure is reported on the USB port, which still runs
	if (BtAppInit(&s_BleAppCfg) == false)
	{
		Status("BtAppInit failed\r\n");
	}
	else
	{
		if (BtAppScanInit((BtGapScanCfg_t *)&s_ScanCfg) == false)
		{
			Status("Scan start failed\r\n");
		}
		BtAppScan();
	}

	// Without a cable this does nothing; the cable interrupt comes back to it
	(void)UsbEnable(USB_DEVNO);

	if (TaktOSThreadCreate(s_UsbThreadMem, sizeof(s_UsbThreadMem),
			WorkThread, &s_UsbWorkQue, TAKTOS_PRIORITY_NORMAL) == nullptr ||
		TaktOSThreadCreate(s_BleThreadMem, sizeof(s_BleThreadMem),
			WorkThread, &s_BleWorkQue, TAKTOS_PRIORITY_NORMAL) == nullptr)
	{
		while (true) {}
	}

	TaktOSStart();

	while (true) {}
}
