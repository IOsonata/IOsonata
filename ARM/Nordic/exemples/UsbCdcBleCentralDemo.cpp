/**-------------------------------------------------------------------------
@example	UsbCdcBleCentralDemo.cpp

@brief	USB CDC to BLE central bridge

The board enumerates as a USB CDC serial port and scans for a BLE peripheral
running the BlueIO UART service (uart_ble.cpp, advertising as "UARTDemo").
Once connected, bytes written by the host to the serial port go to the peer
UART Tx characteristic, and notifications from the peer UART Rx characteristic
go back to the host on the same serial port. Status lines go to the board
UART, which is the debug console.

USB and Bluetooth share the one application event queue, run by AppRun. Each
side does what needs immediate service in its interrupt and queues the rest:

	USB OUT data  -> UsbToBleEvt -> BtAppWrite, 20 bytes at a time
	BLE notify    -> s_BleRxFifo -> BleToUsbEvt -> CDC Tx

Each queued event is queued at most once at a time (pending flag), so a burst
of interrupts cannot fill the queue.

The link is open (no pairing). Build the peer with BLE_SC_METHOD set to
BLE_SC_NONE: a peer asking for security disconnects this central.

@author	Thinh Tran
@date	June 30, 2022
@author	Hoang Nguyen Hoan
@date	Oct. 3, 2026

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
#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

#include "istddef.h"
#include "idelay.h"
#include "cfifo.h"
#include "app_evt_handler.h"
#include "blueio_board.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"
#include "coredev/uart.h"
#include "iopinctrl.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_dev.h"
#include "bluetooth/blueio_blesrvc.h"

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

// Application event queue memory, replaces the 4 event library default. USB
// endpoint events, Bluetooth work and the bridge events all queue here.
alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

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
	.Pid = 0x0009,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB BLE Central",
	.pSerial = nullptr,			// Taken from the MCU unique id
	.pFuncName = "IOsonata CDC",
	.IntPrio = 6,				// Allowed with the SoftDevice (it keeps 0, 1, 4)
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

//
// Debug console UART
//

static const IOPinCfg_t s_UartPins[] = UART_PIN_MAP;

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
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
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

UART g_Uart;

static const IOPinCfg_t s_Leds[] = LED_PIN_MAP;
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
	.ConnLedPort = LED_BLUE_PORT,
	.ConnLedPin = LED_BLUE_PIN,
	.ConnLedActLevel = 0,
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

// To the host: peer notifications (stack interrupt) and status lines (stack
// interrupt and main), put with the interrupts masked. BleToUsbEvt is the
// only consumer. Kept while the port is closed, up to the FIFO size.
#define BLE_RXFIFO_MEMSIZE		CFIFO_MEMSIZE(1024)

alignas(4) static uint8_t s_BleRxFifoMem[BLE_RXFIFO_MEMSIZE];
static hCFifo_t s_hBleRxFifo;
static volatile uint32_t s_BleRxDropCnt = 0;

// Data taken from s_BleRxFifo, not yet accepted by the CDC Tx FIFO
static uint8_t s_UsbTxBuf[64];
static int s_UsbTxLen = 0;
static int s_UsbTxOff = 0;

// USB to BLE. Data read from the CDC, not yet written to the peer.
static uint8_t s_BleTxBuf[USB_PKT_SIZE];
static int s_BleTxLen = 0;
static int s_BleTxOff = 0;

static volatile bool s_bUsbToBleQueued = false;
static volatile bool s_bBleToUsbQueued = false;

static void UsbToBleEvt(uint32_t Evt, void *pCtx);
static void BleToUsbEvt(uint32_t Evt, void *pCtx);

static void UsbToBleQue(void)
{
	if (s_bUsbToBleQueued == false)
	{
		s_bUsbToBleQueued = true;
		if (AppEvtHandlerQue(0, nullptr, UsbToBleEvt) == false)
		{
			s_bUsbToBleQueued = false;
		}
	}
}

static void BleToUsbQue(void)
{
	if (s_bBleToUsbQueued == false)
	{
		s_bBleToUsbQueued = true;
		if (AppEvtHandlerQue(0, nullptr, BleToUsbEvt) == false)
		{
			s_bBleToUsbQueued = false;
		}
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
		uint8_t *p = CFifoPutMultiple(s_hBleRxFifo, &l);
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
		s_BleRxDropCnt += Len;
	}

	BleToUsbQue();
}

// Status line to the USB port and the debug UART
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
	g_Uart.Tx((uint8_t *)line, l);
}

static bool BridgeReady(void)
{
	return s_ConnHdl != BT_CONN_HDL_INVALID && s_BleTxCharHdl != BT_ATT_HANDLE_INVALID;
}

// Host to peer. Data stays in the CDC Rx FIFO while there is no peer, so the
// host is held back by the USB flow control instead of losing bytes.
static void UsbToBleEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bUsbToBleQueued = false;

	while (BridgeReady())
	{
		if (s_BleTxOff >= s_BleTxLen)
		{
			s_BleTxOff = 0;
			s_BleTxLen = g_Cdc.Rx(0, s_BleTxBuf, sizeof(s_BleTxBuf));
			if (s_BleTxLen <= 0)
			{
				s_BleTxLen = 0;
				return;
			}
		}

		int l = s_BleTxLen - s_BleTxOff;
		if (l > BLE_WRITE_MAX)
		{
			l = BLE_WRITE_MAX;
		}

		if (BtAppWrite(s_ConnHdl, s_BleTxCharHdl, &s_BleTxBuf[s_BleTxOff], (uint16_t)l) == false)
		{
			// Stack Tx queue full. The port has no generic write done event,
			// so try again from the back of the event queue.
			UsbToBleQue();
			return;
		}

		s_BleTxOff += l;
	}
}

// Peer and status to host
static void BleToUsbEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	s_bBleToUsbQueued = false;

	while (true)
	{
		if (s_UsbTxOff >= s_UsbTxLen)
		{
			s_UsbTxOff = 0;
			s_UsbTxLen = sizeof(s_UsbTxBuf);
			uint8_t *p = CFifoGetMultiple(s_hBleRxFifo, &s_UsbTxLen);
			if (p == nullptr)
			{
				s_UsbTxLen = 0;
				return;
			}
			memcpy(s_UsbTxBuf, p, s_UsbTxLen);
		}

		if (g_Cdc.IsPortOpen() == false)
		{
			// Kept for the port open, which queues this again
			return;
		}

		int n = g_Cdc.Tx(0, &s_UsbTxBuf[s_UsbTxOff], s_UsbTxLen - s_UsbTxOff);
		if (n <= 0)
		{
			// CDC Tx FIFO full, TX_FIFO_EMPTY queues this again
			return;
		}
		s_UsbTxOff += n;
	}
}

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len)
{
	(void)pDev;
	(void)pBuffer;

	switch (EvtId)
	{
		case DEVINTRF_EVT_RX_DATA:
		case DEVINTRF_EVT_RX_FIFO_FULL:
			UsbToBleQue();
			break;

		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			if (s_UsbTxOff < s_UsbTxLen || CFifoUsed(s_hBleRxFifo) > 0)
			{
				BleToUsbQue();
			}
			break;

		case DEVINTRF_EVT_STATECHG:
			if (Len)
			{
				Status("\r\nIOsonata USB BLE Central, target %s, %s\r\n",
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
// Bluetooth events
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
	UsbToBleQue();
}

// Stack interrupt context. Copy and hand over to the queue.
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

static void HardwareInit(void)
{
	g_Uart.Init(s_UartCfg);

	IOPinCfg(s_Leds, s_NbLeds);
	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	s_hBleRxFifo = CFifoInit(s_BleRxFifoMem, sizeof(s_BleRxFifoMem), 1, true);

	g_Uart.printf("USB CDC BLE Central Demo, target %s\r\n", TARGET_DEV_NAME);
}

int main()
{
	// The queue exists before any interrupt can queue an event
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));

	HardwareInit();

	if (UsbInit(&s_UsbCfg) == false || g_Cdc.Init(s_CdcCfg) == false)
	{
		g_Uart.printf("USB init failed\r\n");
		while (true)
		{
			__WFE();
		}
	}

	if (BtAppInit(&s_BleAppCfg) == false)
	{
		g_Uart.printf("BtAppInit failed\r\n");
		while (true)
		{
			__WFE();
		}
	}

	if (BtAppScanInit((BtGapScanCfg_t *)&s_ScanCfg) == false)
	{
		Status("Scan start failed\r\n");
	}
	BtAppScan();

	// Without a cable this does nothing; the cable interrupt comes back to it
	UsbEnable(USB_DEVNO);

	AppRun();

	return 0;
}
