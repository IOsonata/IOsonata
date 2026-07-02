/**-------------------------------------------------------------------------
@example	uart_ble_central.cpp


@brief	Uart BLE Central demo

This application demo shows UART Rx/Tx over BLE central using EHAL library.

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

#include "istddef.h"
#include "stddev.h"
#include "idelay.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/bt_dev.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "app_evt_handler.h"
#include "cfifo.h"
#include "iopinctrl.h"

#include "board.h"

#define DEBUG_PRINT		// Enable printing debug info over UART interface

// BLE
#define DEVICE_NAME             "UARTCentral"          /**< Name of device. Will be included in the advertising data. */

// LE Secure Connections method selector. This board has no screen or keypad;
// the UART console is the display and the keyboard. Pick a method and build the
// peer (uart_ble) with the compatible row:
//
//   this side             peer side             resulting method
//   BLE_SC_NONE           BLE_SC_NONE           open link, no pairing
//   BLE_SC_JUSTWORKS      BLE_SC_JUSTWORKS      Just Works (bonded, no MITM)
//   BLE_SC_NUMCOMP        BLE_SC_NUMCOMP        Numeric Comparison
//   BLE_SC_PASSKEY_INPUT  BLE_SC_PASSKEY_DISP   Passkey Entry (this side types)
//   BLE_SC_PASSKEY_DISP   BLE_SC_PASSKEY_INPUT  Passkey Entry (this side shows)
#define BLE_SC_NONE				0
#define BLE_SC_JUSTWORKS		1
#define BLE_SC_NUMCOMP			2
#define BLE_SC_PASSKEY_DISP		3
#define BLE_SC_PASSKEY_INPUT	4

#ifndef BLE_SC_METHOD
#define BLE_SC_METHOD			BLE_SC_NUMCOMP
#endif

#if BLE_SC_METHOD != BLE_SC_NONE
#include "bluetooth/bt_smp.h"		// SMP IO caps and console pairing callbacks
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_KEYBOARD
#endif

#if BLE_SC_METHOD == BLE_SC_JUSTWORKS
#define BLE_SEC_TYPE			BTGAP_SECTYPE_STATICKEY_NO_MITM
#define BLE_SC_IOCAPS			BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#define BLE_SC_AUTHREQ			BT_SMP_AUTHREQ_BONDING_FLAG_BONDING
#define BLE_SC_NAME				"Just Works (bonded, no MITM)"
#elif BLE_SC_METHOD == BLE_SC_NUMCOMP
#define BLE_SEC_TYPE			BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS			BT_SMP_IOCAPS_DISPLAY_YESNO
#define BLE_SC_AUTHREQ			(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME				"Numeric Comparison"
#elif BLE_SC_METHOD == BLE_SC_PASSKEY_DISP
#define BLE_SEC_TYPE			BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS			BT_SMP_IOCAPS_DISPLAY_ONLY
#define BLE_SC_AUTHREQ			(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME				"Passkey Entry (display)"
#elif BLE_SC_METHOD == BLE_SC_PASSKEY_INPUT
#define BLE_SEC_TYPE			BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS			BT_SMP_IOCAPS_KEYBOARD_ONLY
#define BLE_SC_AUTHREQ			(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME				"Passkey Entry (keyboard)"
#else
#define BLE_SEC_TYPE			BTGAP_SECTYPE_NONE
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_NONE
#define BLE_SC_NAME				"NONE (open link)"
#endif

#define MANUFACTURER_NAME       "I-SYST inc."          /**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME              "IMM-NRF5x"            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID         ISYST_BLUETOOTH_ID     /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID           ISYST_BLUETOOTH_ID     /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define MIN_CONN_INTERVAL       7.5	/**< Minimum acceptable connection interval (ms), Will be converted to units of 1.25ms */
#define MAX_CONN_INTERVAL       40	/**< Maximum acceptable connection interval (ms), Will be converted to units of 1.25ms */

#define SCAN_INTERVAL           1000 /**< Determines scan interval (ms). Will be converted to units of 0.625ms */
#define SCAN_WINDOW             100  /**< Determines scan interval (ms). Will be converted to units of 0.625ms */
#define SCAN_TIMEOUT            0 	 /**< Timout when scanning. 0x0000 disables timeout. */

#define APP_ADV_INTERVAL        64 	/**< The advertising interval (ms), Will be converted to units of 0.625ms */
#define APP_ADV_TIMEOUT			0	/**< The advertising timeout (ms), Will be converted to units of 10ms */

// UART
#define BLE_MTU_SIZE			247//byte
#define PACKET_SIZE				128
#define UART_MAX_DATA_LEN  		(PACKET_SIZE)
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN)

// BLE
#define BLEFIFOSIZE				CFIFO_MEMSIZE(PACKET_SIZE)

/** Target BLE peripheral device, which requires 2 pieces of info
 *  - Name : the advertising name of the device
 *  - MAC address of the device
 */
#define TARGET_BRIDGE_DEV_NAME	"UARTDemo"	/**< Name of BLE peripheral device to be scanned */

#define Nordic_PCA10040_DK		{0x25, 0xD3, 0x83, 0x6A, 0xEA, 0xDE}
#define BlueIO832_01			{0x7C, 0x75, 0x96, 0x65, 0x28, 0xF1}
#define BlueIO832_02			{0xC3, 0x84, 0xC0, 0x8C, 0x2A, 0xC2}
#define UART_Demo				{0x12, 0xB1, 0x6A, 0xE7, 0x68, 0xE4}

uint8_t g_clientMacAddr[6] = UART_Demo;
uint8_t g_searchCnt = 0;

// CFIFO for BleAppWrite
alignas(4) uint8_t g_UartRx2BleBuff[BLEFIFOSIZE];//FIFO buffer for UART-to-BLE
std::atomic<int> g_UartRx2BleBuffLen(0);//buffer counter for g_UartRx2BleBuff
hCFifo_t g_UartRx2BleFifo;

alignas(4) uint8_t g_UartRxExtBuff[PACKET_SIZE * 2];//external buffer for Uart Rx data
volatile int g_UartRxExtBuffLen = 0;

uint32_t g_DropCnt = 0;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void BleTxSchedHandler(uint32_t Evt, void *pCtx);

static const IOPinCfg_t s_Leds[] = LED_PINS;
static int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,				// Device role, ref: BTAPP_ROLE
	.CentLinkCount = 1, 					// Number of central link
	.PeriLinkCount = 0, 					// Number of peripheral link
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
	.SecType = BLE_SEC_TYPE,					// see BLE_SC_METHOD selector
	.SecExchg = BLE_SEC_EXCHG,					// key distribution for the method
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
alignas(4) uint8_t s_UartRxFifo[UARTFIFOSIZE];//internal Uart_Rx buffer
alignas(4) uint8_t s_UartTxFifo[UARTFIFOSIZE];//internal Uart_Tx buffer

UARTCfg_t g_UartCfg = {
	.DevNo = 0,									// Device number zero based
	.pIOPinMap = s_UartPins,					// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),					// Total number of UART pins used
	.Rate = 1000000,								// Baudrate
	.DataBits = 8,								// Data bits
	.Parity = UART_PARITY_NONE,					// Parity
	.StopBits = 1,								// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,			// Flow control
	.bIntMode = true,							// Interrupt mode
	.IntPrio = 6,//APP_IRQ_PRIORITY_LOW,		// Interrupt priority
	.EvtCallback = nRFUartEvthandler,			// UART event handler
	.bFifoBlocking = true,						// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

// UART object instance
UART g_Uart;

static const BtGapScanCfg_t g_ScanParams = {
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

BtDevice_t g_ConnectedDev = {
	.Conn = { .Hdl = BT_CONN_HDL_INVALID },
};

// BLE characteristic handles used by central bridge path.
uint16_t g_BleTxCharHdl = BT_ATT_HANDLE_INVALID;   // BlueIO UART TX characteristic (write target)
uint16_t g_BleRxCharHdl = BT_ATT_HANDLE_INVALID;   // BlueIO UART RX characteristic (notify source)

// Kick off GATT discovery on the connected peer. Called from connect on an
// open link, or from the secured event on an encrypted link, selected by
// BLE_SC_METHOD.
static void StartPeerDiscovery(uint16_t ConnHdl)
{
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer)
	{
		BtAppDiscoverDevice(pPeer);
	}
	else
	{
		g_Uart.printf("BtPeerFindByHdl failed for ConnHdl=%u\r\n", ConnHdl);
	}
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	g_ConnectedDev.Conn.Hdl = ConnHdl;
	g_Uart.printf("BtAppEvtConnected ConnHdl = %d (0x%x)\r\n", ConnHdl, ConnHdl);

#if BLE_SC_METHOD != BLE_SC_NONE
	// Secure mode: the stack starts pairing now. Do not touch the peer GATT
	// server yet; its characteristics may be encryption gated. Discovery runs
	// from BtAppEvtSecured() once the link is encrypted.
	g_Uart.printf("Securing link...\r\n");
#else
	// Open mode: no pairing. Discover the peer GATT server immediately.
	StartPeerDiscovery(ConnHdl);
#endif
}

#if BLE_SC_METHOD != BLE_SC_NONE
void BtAppEvtSecured(uint16_t ConnHdl)
{
	// Link is now encrypted: freshly paired, or re-encrypted from a stored
	// bond on reconnect. Protected characteristics are accessible now, so
	// discovery and the later notify-enable run from here.
	g_Uart.printf("Link secured. Starting discovery\r\n");
	StartPeerDiscovery(ConnHdl);
}
#endif

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	g_Uart.printf("BtAppEvtDisconnected ConnHdl = %d (0x%x) \r\n", ConnHdl, ConnHdl);

	g_ConnectedDev.Conn.Hdl = BT_CONN_HDL_INVALID;
	g_BleTxCharHdl = BT_ATT_HANDLE_INVALID;
	g_BleRxCharHdl = BT_ATT_HANDLE_INVALID;
}

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData)
{
	char name[32] = {0};
	size_t l = BtAdvDataGetDevName(pAdvData, AdvLen, name, sizeof(name));

	if (l > 0)
	{
		if (l >= sizeof(name))
		{
			l = sizeof(name) - 1;
		}
		name[l] = 0;

		g_Uart.printf("%s, RSSI=%d\r\n", name, Rssi);

		if (strcmp(name, TARGET_BRIDGE_DEV_NAME) == 0)
		{
			g_Uart.printf("Found target '%s'. Stop Scan\r\n", TARGET_BRIDGE_DEV_NAME);
			BtGapScanStop();

			BtGapPeerAddr_t add = {.Type = AddrType};
			memcpy(add.Addr, Addr, 6);

			BtGapConnect(&add, &s_ConnParams);
			return false; // stop scan reporting
		}
	}

	return true; // keep scanning
}


void BleAppInitUserData()
{
//	// Add passkey pairing
//	ble_opt_t opt;
//	opt.gap_opt.passkey.p_passkey = (uint8_t*)"123456";
//	uint32_t err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt);
//	APP_ERROR_CHECK(err_code);
}

void BtAppInitUserServices(void)
{
	// Central bridge app does not host local custom GATT services.
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	// Not used in central-only demo.
}

void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
}

// Peripheral -> central data stream. Forward the notified/indicated value from
// the BlueIO UART RX characteristic out the local UART.
void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl, uint8_t *pData, uint16_t Len)
{
	(void)ConnHdl;
	if (ValHdl == g_BleRxCharHdl && pData != nullptr && Len > 0)
	{
		g_Uart.Tx(pData, Len);
	}
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
		g_Uart.printf("BlueIO UART service not found\r\n");
		return;
	}

	int rxidx = BtDeviceFindCharacteristic(pDev, sidx, BLUEIO_UUID_UART_RX_CHAR);
	int txidx = BtDeviceFindCharacteristic(pDev, sidx, BLUEIO_UUID_UART_TX_CHAR);
	if (rxidx < 0 || txidx < 0)
	{
		g_Uart.printf("BlueIO UART chars not found (rx=%d tx=%d)\r\n", rxidx, txidx);
		return;
	}

	g_BleRxCharHdl = pDev->Services[sidx].characteristics[rxidx].characteristic.handle_value;
	g_BleTxCharHdl = pDev->Services[sidx].characteristics[txidx].characteristic.handle_value;

	g_Uart.printf("UART service discovered: RX=0x%04X TX=0x%04X\r\n", g_BleRxCharHdl, g_BleTxCharHdl);

	// Enable notify on the RX characteristic (peripheral -> central stream).
	// Notifications are turned on by writing to the CCCD, not the value handle.
	// The descriptor discovery phase fills cccd_handle.
	uint16_t rxCccd = pDev->Services[sidx].characteristics[rxidx].cccd_handle;
	if (rxCccd == BT_ATT_HANDLE_INVALID)
	{
		g_Uart.printf("RX CCCD not found\r\n");
		return;
	}

	if (!BtAppEnableNotify(pDev->Conn.Hdl, rxCccd))
	{
		g_Uart.printf("BtAppEnableNotify failed\r\n");
	}
}

void BleTxSchedHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);

	int len = PACKET_SIZE;
	uint8_t *p = CFifoGetMultiple(g_UartRx2BleFifo, &len);

	if (p != NULL)
	{
		if (g_ConnectedDev.Conn.Hdl != BT_CONN_HDL_INVALID && g_BleTxCharHdl != BT_ATT_HANDLE_INVALID)
		{
			BtAppWrite(g_ConnectedDev.Conn.Hdl, g_BleTxCharHdl, p, len);
		}

		// Schedule this func again if g_UartRx2BleFifo is not empty
		AppEvtHandlerQue(0, NULL, BleTxSchedHandler);
	}

	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
}

#if BLE_SC_METHOD != BLE_SC_NONE
// Console pairing IO: the SMP core prints through these callbacks and reads the
// y/n or the typed passkey back via PairInputPoll on the UART RX path.
enum { PAIR_INPUT_NONE = 0, PAIR_INPUT_NUMERIC, PAIR_INPUT_PASSKEY };
static volatile int s_PairInput = PAIR_INPUT_NONE;
static uint16_t s_PairConnHdl = 0;
static uint8_t  s_PairDigits = 0;
static uint32_t s_PairPasskey = 0;

void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value)
{
	g_Uart.printf("\r\nSMP numeric comparison: %06u\r\n", (unsigned)Value);
	g_Uart.printf("Do both devices show this value? type y or n\r\n");
	s_PairConnHdl = ConnHdl;
	s_PairInput = PAIR_INPUT_NUMERIC;
}

void BtSmpPasskeyDisplay(uint16_t ConnHdl, uint32_t Passkey)
{
	(void)ConnHdl;
	g_Uart.printf("\r\nSMP passkey (enter this on the peer): %06u\r\n", (unsigned)Passkey);
}

void BtSmpPasskeyRequest(uint16_t ConnHdl)
{
	g_Uart.printf("\r\nSMP passkey entry: type the 6 digits shown on the peer\r\n");
	s_PairConnHdl = ConnHdl;
	s_PairDigits = 0;
	s_PairPasskey = 0;
	s_PairInput = PAIR_INPUT_PASSKEY;
}

static bool PairInputPoll(void)
{
	if (s_PairInput == PAIR_INPUT_NONE)
	{
		return false;
	}
	uint8_t c;
	while (g_Uart.Rx(&c, 1) == 1)
	{
		if (s_PairInput == PAIR_INPUT_NUMERIC)
		{
			if (c == 'y' || c == 'Y')
			{
				s_PairInput = PAIR_INPUT_NONE;
				g_Uart.printf("match\r\n");
				BtSmpNumericComparisonReply(s_PairConnHdl, true);
				return true;
			}
			if (c == 'n' || c == 'N')
			{
				s_PairInput = PAIR_INPUT_NONE;
				g_Uart.printf("no match\r\n");
				BtSmpNumericComparisonReply(s_PairConnHdl, false);
				return true;
			}
		}
		else
		{
			if (c >= '0' && c <= '9' && s_PairDigits < 6)
			{
				s_PairPasskey = s_PairPasskey * 10 + (uint32_t)(c - '0');
				s_PairDigits++;
				g_Uart.Tx(&c, 1);
				if (s_PairDigits == 6)
				{
					s_PairInput = PAIR_INPUT_NONE;
					g_Uart.printf("\r\n");
					BtSmpPasskeyReply(s_PairConnHdl, s_PairPasskey);
					return true;
				}
			}
			else if (c == 0x1b)
			{
				s_PairInput = PAIR_INPUT_NONE;
				g_Uart.printf("\r\ncancelled\r\n");
				BtSmpPasskeyReply(s_PairConnHdl, BT_SMP_PASSKEY_INVALID);
				return true;
			}
		}
	}
	return true;
}
#endif

void UartRxSchedHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;

#if BLE_SC_METHOD != BLE_SC_NONE
	if (PairInputPoll())
	{
		return;		// console bytes consumed by the pairing step
	}
#endif

	bool flush = false;
	uint8_t *p = NULL;

	// Internal UartRxBuffer -> External Uart Rx Buffer
	int l1 = g_Uart.Rx(&g_UartRxExtBuff[g_UartRxExtBuffLen], PACKET_SIZE - g_UartRxExtBuffLen);

	if (l1 > 0)
	{
		g_UartRxExtBuffLen += l1;
		if (g_UartRxExtBuffLen >= PACKET_SIZE)
		{
			flush = true;
		}
	}
	else
	{
		if (g_UartRxExtBuffLen > 0)
		{
			flush = true;
		}
	}

	// Flush data ExternalUartRxBuffer -> g_UartRx2BleFifo
	if (flush)
	{
		int l2 = g_UartRxExtBuffLen;
		p = CFifoPutMultiple(g_UartRx2BleFifo, &l2);
		if (p == NULL)
		{
			g_DropCnt++;
		}
		else
		{
			memcpy(p, g_UartRxExtBuff, l2);
			if (l2 < g_UartRxExtBuffLen)
			{
				memcpy(&g_UartRxExtBuff[0], &g_UartRxExtBuff[l2], g_UartRxExtBuffLen - l2);
			}
			else
			{
				g_UartRxExtBuffLen = 0;
			}
		}

		// Schedule the BleTxSchedHandler for sending data via BLE
		AppEvtHandlerQue(0, NULL, BleTxSchedHandler);
	}

	// Schedule the UartRxSchedHandler if ExternalUartRxBuffer still has data
	if (g_UartRxExtBuffLen > 0)
	{
		AppEvtHandlerQue(0, NULL, UartRxSchedHandler);
	}
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	int cnt = 0;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			if (g_UartRxExtBuffLen <= 0)
			{
				AppEvtHandlerQue(0, NULL, UartRxSchedHandler);
			}
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);

	IOPinCfg(s_Leds, s_NbLeds);

	for (int i = 0; i < s_NbLeds; i++)
	{
		IOPinSet(s_Leds[i].PortNo, s_Leds[i].PinNo);
	}

	// Retarget printf to uart if semihosting is not used
	//UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#ifdef DEBUG_PRINT
	g_Uart.printf("UART BLE Central Demo\r\n");
	g_Uart.printf("security    : %s\r\n", BLE_SC_NAME);
	msDelay(10);
	g_Uart.printf("UART Configuration: Baudrate %d, FLow Control (%s), Parity (%s)\r\n",
			g_UartCfg.Rate,
			(g_UartCfg.FlowControl == UART_FLWCTRL_NONE) ? "No" : "Yes",
			(g_UartCfg.Parity == UART_PARITY_NONE) ? "No" : "Yes");
#endif

	// Init CFIFO instance
	g_UartRx2BleFifo = CFifoInit(g_UartRx2BleBuff, BLEFIFOSIZE, 1, true);
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	HardwareInit();

	BtAppInit(&s_BleAppCfg);

#if BLE_SC_METHOD != BLE_SC_NONE
	// Local IO capability for the selected method; the console fills the role.
	BtSmpAuthConfig(BLE_SC_IOCAPS, BLE_SC_AUTHREQ);
#endif

	// Register the non-GATT service and its characteristics
	BtAppScanInit((BtGapScanCfg_t*)&g_ScanParams);

	BtAppScan();

	BtAppRun();

	return 0;
}
