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

#include <string.h>

#include "istddef.h"
#include "stddev.h"
#include "idelay.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/bt_dev.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "cfifo.h"
#include "iopinctrl.h"

#include "board.h"

#ifdef MCUOSC
#include "coredev/system_core_clock.h"
McuOsc_t g_McuOsc = MCUOSC;
#endif

#if defined(NRFXLIB_SDC)
#include "bluetooth/bt_hci_ctlr.h"

// Central streaming reserves ten RX/TX packets and scanning buffers.
// Keep this application allocation separate from the smaller default pool.
alignas(8) static uint8_t s_CentralCtlrMem[16384];
extern "C" const BtHciCtlrMemPool_t g_BtHciCtlrMemPool = {
	s_CentralCtlrMem, sizeof(s_CentralCtlrMem)
};
#endif

#if defined(S132) || defined(S140)
#include "ble.h"
#endif

#ifndef UART_DEVNO
#define UART_DEVNO 0
#endif

#define DEBUG_PRINT		// Enable printing debug info over UART interface

// BLE
#define DEVICE_NAME             "UARTCentral"          //!< Name of device. Will be included in the advertising data.

// LE Secure Connections method selector. This board has no screen or keypad;
// the UART console is the display and the keyboard. Pick a method and build the
// peer (uart_ble) with the compatible row:
//
//   this side             peer side             resulting method
//   BLE_SC_NONE           BLE_SC_NONE           open link, no pairing
//   BLE_SC_JUSTWORKS      BLE_SC_JUSTWORKS      Just Works (bonded, no MITM)
//   BLE_SC_NUMCOMP        BLE_SC_NUMCOMP        Numeric Comparison
//   BLE_SC_PASSKEY_INPUT  BLE_SC_PASSKEY_DISP   Passkey Entry (this side types)
//   BLE_SC_OOB            BLE_SC_OOB            LESC OOB via UART copy/paste
//   BLE_SC_PASSKEY_DISP   BLE_SC_PASSKEY_INPUT  Passkey Entry (this side shows)
#define BLE_SC_NONE				0
#define BLE_SC_JUSTWORKS		1
#define BLE_SC_NUMCOMP			2
#define BLE_SC_PASSKEY_DISP		3
#define BLE_SC_PASSKEY_INPUT	4
#define BLE_SC_OOB				5

// Default to an open link for streaming. Override BLE_SC_METHOD to test
// security, and select the compatible method on the peripheral as listed above.
#ifndef BLE_SC_METHOD
#define BLE_SC_METHOD			BLE_SC_NONE
#endif

#if BLE_SC_METHOD != BLE_SC_NONE
#include "bluetooth/bt_smp.h"		// SMP IO caps and console pairing callbacks
#endif

// One row per method, the same table uart_ble.cpp uses. The previous form gave
// every non-OOB method BTAPP_SECEXCHG_KEYBOARD, which asks the port for a
// Keyboard Only IO capability and disagrees with BLE_SC_IOCAPS below.
#if BLE_SC_METHOD == BLE_SC_JUSTWORKS
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_NONE
#elif BLE_SC_METHOD == BLE_SC_NUMCOMP
#define BLE_SEC_EXCHG			(BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO)
#elif BLE_SC_METHOD == BLE_SC_PASSKEY_DISP
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_DISPLAY
#elif BLE_SC_METHOD == BLE_SC_PASSKEY_INPUT
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_KEYBOARD
#elif BLE_SC_METHOD == BLE_SC_OOB
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_OOB
#else
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_NONE
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
#elif BLE_SC_METHOD == BLE_SC_OOB
#define BLE_SEC_TYPE			BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS			BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#define BLE_SC_AUTHREQ			(BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME				"LESC OOB"
#else
#define BLE_SEC_TYPE			BTGAP_SECTYPE_NONE
#define BLE_SC_NAME				"NONE (open link)"
#endif

#define MANUFACTURER_NAME       "I-SYST inc."          //!< Manufacturer. Will be passed to Device Information Service.
#define MODEL_NAME              "IMM-NRF5x"            //!< Model number. Will be passed to Device Information Service.
#define MANUFACTURER_ID         ISYST_BLUETOOTH_ID     //!< Manufacturer ID, part of System ID. Will be passed to Device Information Service.
#define ORG_UNIQUE_ID           ISYST_BLUETOOTH_ID     //!< Organizational Unique ID, part of System ID. Will be passed to Device Information Service.

// Match the PRBS HCI receiver's interval. Keep both bounds equal so the
// controller cannot select a different interval within a wide range.
#ifndef CENTRAL_CONN_INTERVAL
#define CENTRAL_CONN_INTERVAL   15
#endif
#ifndef CENTRAL_PHY
// Establish the 1M baseline first. Override CENTRAL_PHY in the application
// project with BT_GAP_PHY_2MBITS for the separate 2M test.
#define CENTRAL_PHY             BT_GAP_PHY_1MBITS
#endif
#define MIN_CONN_INTERVAL       CENTRAL_CONN_INTERVAL
#define MAX_CONN_INTERVAL       CENTRAL_CONN_INTERVAL

#define SCAN_INTERVAL           1000 //!< Determines scan interval (ms). Will be converted to units of 0.625ms
#define SCAN_WINDOW             100  //!< Determines scan interval (ms). Will be converted to units of 0.625ms
#define SCAN_TIMEOUT            0 	 //!< Timout when scanning. 0x0000 disables timeout.

#define APP_ADV_INTERVAL        64 	//!< The advertising interval (ms), Will be converted to units of 0.625ms
#define APP_ADV_TIMEOUT			0	//!< The advertising timeout (ms), Will be converted to units of 10ms

// UART
#ifndef CENTRAL_UART_BAUD
#define CENTRAL_UART_BAUD 1000000
#endif
#define BLE_MTU_SIZE			247//byte
#define PACKET_SIZE				128

// The peer's UART characteristics hold 20 octets (PACKET_SIZE in uart_ble.cpp).
// A write longer than the attribute can hold is refused, not truncated, so the
// UART side buffers in PACKET_SIZE chunks but each BLE write is capped here.
#define BLE_WRITE_MAX			20
#define UART_MAX_DATA_LEN  		(PACKET_SIZE)
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN)

// BLE
#define BLEFIFOSIZE				CFIFO_MEMSIZE(PACKET_SIZE)

// Target BLE peripheral device, which requires 2 pieces of info
//  - Name : the advertising name of the device
//  - MAC address of the device
#define TARGET_BRIDGE_DEV_NAME	"BlePrbs_SDC"	//!< Name of BLE peripheral device to be scanned

// Diagnostic only; the bridge forwards without checking by default.
// Enable only when receiving the PRBS test stream. Ordinary UART payloads
// do not follow this sequence.
#ifndef CENTRAL_PRBS_CHECK
#define CENTRAL_PRBS_CHECK 0
#endif

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

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(12)];
static volatile bool s_StreamActive = false;
static volatile uint32_t s_BleToUartReceived = 0;
static volatile uint32_t s_BleNotifications = 0;
static volatile uint32_t s_BleFirstRxMs = 0;
static volatile uint16_t s_BleNotifyMinLen = UINT16_MAX;
static volatile uint16_t s_BleNotifyMaxLen = 0;
static volatile uint32_t s_BlePrbsBreaks = 0;
#if CENTRAL_PRBS_CHECK
static bool s_BlePrbsStarted = false;
static uint8_t s_BlePrbsPrevious = 0;
#endif
static volatile uint32_t s_BleToUartAccepted = 0;
static volatile uint32_t s_BleToUartDropped = 0;
alignas(4) uint8_t g_UartRxExtBuff[PACKET_SIZE * 2];//external buffer for Uart Rx data
volatile int g_UartRxExtBuffLen = 0;

uint32_t g_DropCnt = 0;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void BleTxSchedHandler(uint32_t Evt, void *pCtx);

static const IOPinCfg_t s_Leds[] = LED_PINS;
static int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

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
alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];//internal Uart_Rx buffer
// Ten notification payloads, excluding the three-byte ATT header.
alignas(4) static uint8_t s_UartTxFifo[CFIFO_MEMSIZE(10 * (BLE_MTU_SIZE - 3))];

UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,									// Device number zero based
	.pIOPinMap = s_UartPins,					// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),					// Total number of UART pins used
	.Rate = CENTRAL_UART_BAUD,								// Baudrate
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

BtDevice_t g_ConnectedDev = {
	.Conn = { .Hdl = BT_CONN_HDL_INVALID },
};

// BLE characteristic handles used by central bridge path.
uint16_t g_BleTxCharHdl = BT_ATT_HANDLE_INVALID;   // BlueIO UART TX characteristic (write target)
uint16_t g_BleRxCharHdl = BT_ATT_HANDLE_INVALID;   // BlueIO UART RX characteristic (notify source)

// Kick off GATT discovery on the connected peer. Called from connect on an
// open link, or from the secured event on an encrypted link, selected by
// BLE_SC_METHOD.
// Pairing IO state and OOB validity, grouped ahead of the first function.
#if BLE_SC_METHOD != BLE_SC_NONE
// Console pairing IO: the SMP core prints through these callbacks and reads the
// y/n or the typed passkey back via PairInputPoll on the UART RX path.
enum { PAIR_INPUT_NONE = 0, PAIR_INPUT_NUMERIC, PAIR_INPUT_PASSKEY };
static volatile int s_PairInput = PAIR_INPUT_NONE;
static uint16_t s_PairConnHdl = 0;
static uint8_t  s_PairDigits = 0;
static uint32_t s_PairPasskey = 0;
#endif
#if BLE_SC_METHOD != BLE_SC_NONE
static bool s_UartBlePeerOobValid = false;
#endif

// Pending stays set through queue refusal until the callback runs.
static volatile bool s_bUartRxPending = false;
static volatile bool s_bBleTxPending = false;
#if defined(S132) || defined(S140)
static bool s_bDiscoveryPending = false;
#endif

void UartRxSchedHandler(uint32_t Evt, void *pCtx);
void BleTxSchedHandler(uint32_t Evt, void *pCtx);

#if BLE_SC_METHOD != BLE_SC_NONE
// Runtime association-model selection.
//
// BLE_SC_METHOD is the boot default; the console "sec" command switches the
// method for the next pairing without a rebuild, so one binary covers the whole
// matrix instead of one build per cell. BtSmpAuthConfig only assigns the local
// IO capability and the authentication requirements, and those two plus whether
// OOB data is present are what choose the model at pairing time, so calling it
// between connections is enough. SecType in the app config stays as built: it
// arms security and bonding, it does not pick the model.
typedef struct {
	const char *pName;			// console keyword
	uint8_t     IoCaps;			// BT_SMP_IOCAPS_*
	uint8_t     AuthReq;		// bonding / MITM, SC is forced by BtSmpAuthConfig
	const char *pDesc;
} UartBleSecMethod_t;

static const UartBleSecMethod_t s_SecMethods[] = {
	{ "justworks",		BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING,
	  "Just Works (bonded, no MITM)" },
	{ "numcomp",		BT_SMP_IOCAPS_DISPLAY_YESNO,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "Numeric Comparison" },
	{ "passkey-disp",	BT_SMP_IOCAPS_DISPLAY_ONLY,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "Passkey Entry (display)" },
	{ "passkey-input",	BT_SMP_IOCAPS_KEYBOARD_ONLY,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "Passkey Entry (keyboard)" },
	{ "oob",			BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "LESC OOB" },
};

#define UART_BLE_SEC_METHOD_CNT		(int)(sizeof(s_SecMethods) / sizeof(s_SecMethods[0]))

static int s_SecMethodIdx = 0;
#endif

static void UartRxQue(void)
{
	uint32_t state = DisableInterrupt();
	if (s_bUartRxPending == false)
	{
		s_bUartRxPending = true;
		(void)AppEvtHandlerQue(0, nullptr, UartRxSchedHandler);
	}
	EnableInterrupt(state);
}

static void BleTxQue(void)
{
	uint32_t state = DisableInterrupt();
	if (s_bBleTxPending == false)
	{
		s_bBleTxPending = true;
		(void)AppEvtHandlerQue(0, nullptr, BleTxSchedHandler);
	}
	EnableInterrupt(state);
}

static void StartPeerDiscovery(uint16_t ConnHdl)
{
#if defined(S132) || defined(S140)
	s_bDiscoveryPending = false;
#endif
	BtDevice_t *pPeer = BtPeerFindByHdl(ConnHdl);
	if (pPeer)
	{
		g_Uart.printf("Starting discovery: handle=%u\r\n", ConnHdl);
		if (BtAppDiscoverDevice(pPeer) == false)
		{
			g_Uart.printf("Discovery start failed: handle=%u\r\n", ConnHdl);
#if defined(S132) || defined(S140)
			// Nordic's GATT handler can have an MTU exchange in progress.
			s_bDiscoveryPending = true;
#endif
		}
		else
		{
			g_Uart.printf("Discovery request accepted\r\n");
		}
	}
	else
	{
		g_Uart.printf("BtPeerFindByHdl failed for ConnHdl=%u\r\n", ConnHdl);
	}
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	s_StreamActive = false;
#if CENTRAL_PRBS_CHECK
	s_BlePrbsStarted = false;
#endif
	g_ConnectedDev.Conn.Hdl = ConnHdl;
	g_Uart.printf("BtAppEvtConnected ConnHdl = %d (0x%x)\r\n", ConnHdl, ConnHdl);

	// The streamer need not initiate a PHY procedure. An accepted request
	// does not mean negotiation has completed; SoftDevice reports its result
	// through BtAppCentralEvtHandler, and SDC can be queried with stats.
	if (!BtGapSetPhy(ConnHdl, CENTRAL_PHY, CENTRAL_PHY, 0))
	{
		g_Uart.printf("Central PHY request refused; retaining current PHY\r\n");
	}

#if defined(NRFXLIB_SDC)
	if (!BtGapSetDataLength(ConnHdl, 251, 2120))
	{
		g_Uart.printf("Central data-length request refused\r\n");
	}
#endif

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
	s_StreamActive = false;
	g_Uart.printf("BtAppEvtDisconnected ConnHdl = %d (0x%x) \r\n", ConnHdl, ConnHdl);

	g_ConnectedDev.Conn.Hdl = BT_CONN_HDL_INVALID;
#if defined(S132) || defined(S140)
	s_bDiscoveryPending = false;
#endif
	g_BleTxCharHdl = BT_ATT_HANDLE_INVALID;
	g_BleRxCharHdl = BT_ATT_HANDLE_INVALID;

	// Scanning stopped when the target was found, so without this the central
	// sits idle after any disconnect, including a failed pairing, and never
	// looks for the peer again.
	BtAppScan();
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
#if defined(S132) || defined(S140)
	const ble_evt_t *pEvt = (const ble_evt_t *)pCtx;

	if (pEvt == nullptr || s_StreamActive)
	{
		return;
	}

	switch (Evt)
	{
		case BLE_GAP_EVT_CONNECTED:
			g_Uart.printf("BLE interval_units=%u latency=%u\r\n",
				pEvt->evt.gap_evt.params.connected.conn_params.max_conn_interval,
				pEvt->evt.gap_evt.params.connected.conn_params.slave_latency);
			break;
		case BLE_GAP_EVT_PHY_UPDATE:
			g_Uart.printf("BLE PHY status=%u tx=%u rx=%u\r\n",
				pEvt->evt.gap_evt.params.phy_update.status,
				pEvt->evt.gap_evt.params.phy_update.tx_phy,
				pEvt->evt.gap_evt.params.phy_update.rx_phy);
			break;
		case BLE_GATTC_EVT_EXCHANGE_MTU_RSP:
			g_Uart.printf("MTU exchange response: status=0x%04X\r\n",
				pEvt->evt.gattc_evt.gatt_status);
			if (s_bDiscoveryPending &&
				pEvt->evt.gattc_evt.conn_handle == g_ConnectedDev.Conn.Hdl)
			{
				StartPeerDiscovery(pEvt->evt.gattc_evt.conn_handle);
			}
			break;
		case BLE_GATTC_EVT_PRIM_SRVC_DISC_RSP:
			g_Uart.printf("Discovery services: status=0x%04X count=%u\r\n",
				pEvt->evt.gattc_evt.gatt_status,
				pEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS ?
				pEvt->evt.gattc_evt.params.prim_srvc_disc_rsp.count : 0);
			break;
		case BLE_GATTC_EVT_CHAR_DISC_RSP:
			g_Uart.printf("Discovery characteristics: status=0x%04X count=%u\r\n",
				pEvt->evt.gattc_evt.gatt_status,
				pEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS ?
				pEvt->evt.gattc_evt.params.char_disc_rsp.count : 0);
			break;
		case BLE_GATTC_EVT_DESC_DISC_RSP:
			g_Uart.printf("Discovery descriptors: status=0x%04X count=%u\r\n",
				pEvt->evt.gattc_evt.gatt_status,
				pEvt->evt.gattc_evt.gatt_status == BLE_GATT_STATUS_SUCCESS ?
				pEvt->evt.gattc_evt.params.desc_disc_rsp.count : 0);
			break;
		case BLE_GATTC_EVT_TIMEOUT:
			g_Uart.printf("Discovery/GATT timeout\r\n");
			break;
		case BLE_GATTC_EVT_WRITE_RSP:
			g_Uart.printf("GATT write response: status=0x%04X\r\n",
				pEvt->evt.gattc_evt.gatt_status);
			break;
		default:
			break;
	}
#else
	(void)Evt;
	(void)pCtx;
#endif
}

// Peripheral -> central data stream. Forward the notified/indicated value from
// the BlueIO UART RX characteristic out the local UART.
void BtGattClientNotified(uint16_t ConnHdl, uint16_t ValHdl, uint8_t *pData, uint16_t Len)
{
#if CENTRAL_PRBS_CHECK
	uint16_t idx;
	uint8_t expected;
	uint8_t previous;
	bool started;
	uint32_t breaks;
#endif
	int sent;

	if (ConnHdl == g_ConnectedDev.Conn.Hdl &&
		ValHdl == g_BleRxCharHdl && pData != nullptr && Len > 0)
	{
		s_StreamActive = true;
#if CENTRAL_PRBS_CHECK
		// Check before copying or rejecting a notification. This is the same
		// adjacent-byte check as uartprbs_rx.py, including packet boundaries.
		previous = s_BlePrbsPrevious;
		started = s_BlePrbsStarted;
		breaks = s_BlePrbsBreaks;
		for (idx = 0; idx < Len; idx++)
		{
			expected = ((previous << 1) |
				(((previous >> 6) ^ (previous >> 5)) & 1)) & 0x7f;
			if (started && pData[idx] != expected)
			{
				breaks++;
			}
			previous = pData[idx];
			started = true;
		}
		s_BlePrbsPrevious = previous;
		s_BlePrbsStarted = started;
		s_BlePrbsBreaks = breaks;
#endif
		// Count at BLE delivery, before any forwarding queue can refuse data.
		if (s_BleNotifications == 0)
		{
			s_BleFirstRxMs = BtGattMsTick();
		}
		s_BleNotifications++;
		if (Len < s_BleNotifyMinLen)
		{
			s_BleNotifyMinLen = Len;
		}
		if (Len > s_BleNotifyMaxLen)
		{
			s_BleNotifyMaxLen = Len;
		}
		s_BleToUartReceived += Len;
		sent = g_Uart.Tx(pData, Len);
		if (sent > 0)
		{
			s_BleToUartAccepted += sent;
		}
		else
		{
			sent = 0;
		}
		s_BleToUartDropped += Len - sent;
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

	g_BleRxCharHdl = pDev->pServices[sidx].characteristics[rxidx].characteristic.handle_value;
	g_BleTxCharHdl = pDev->pServices[sidx].characteristics[txidx].characteristic.handle_value;

	g_Uart.printf("UART service discovered: RX=0x%04X TX=0x%04X\r\n", g_BleRxCharHdl, g_BleTxCharHdl);

	// Enable notify on the RX characteristic (peripheral -> central stream).
	// Notifications are turned on by writing to the CCCD, not the value handle.
	// The descriptor discovery phase fills cccd_handle.
	uint16_t rxCccd = pDev->pServices[sidx].characteristics[rxidx].cccd_handle;
	if (rxCccd == BT_ATT_HANDLE_INVALID)
	{
		g_Uart.printf("RX CCCD not found\r\n");
		return;
	}

	if (!BtAppEnableNotify(pDev->Conn.Hdl, rxCccd))
	{
		g_Uart.printf("BtAppEnableNotify failed\r\n");
	}

	BleTxQue();
}

void BleTxSchedHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bBleTxPending = false;

	if (g_ConnectedDev.Conn.Hdl == BT_CONN_HDL_INVALID ||
		g_BleTxCharHdl == BT_ATT_HANDLE_INVALID)
	{
		// Discovery queues this again when the peer is ready.
		return;
	}

	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
	int len = BLE_WRITE_MAX;
	uint8_t *p = CFifoPeekMultiple(g_UartRx2BleFifo, &len);
	if (p != NULL)
	{
		if (BtAppWrite(g_ConnectedDev.Conn.Hdl, g_BleTxCharHdl, p, len))
		{
			(void)CFifoGetMultiple(g_UartRx2BleFifo, &len);
		}
		// A refused write retains the same bytes for the next pass.
		BleTxQue();
	}
	IOPinToggle(s_Leds[0].PortNo, s_Leds[0].PinNo);
}

#if BLE_SC_METHOD != BLE_SC_NONE
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

#if BLE_SC_METHOD != BLE_SC_NONE
static bool UartBleSecIsOob(void)
{
	return strcmp(s_SecMethods[s_SecMethodIdx].pName, "oob") == 0;
}

// One line, fixed field order, so a host script can parse it. Printed on every
// change and on a bare "sec".
static void UartBleSecPrint(void)
{
	const UartBleSecMethod_t *m = &s_SecMethods[s_SecMethodIdx];

	g_Uart.printf("SEC method=%s iocaps=%d authreq=0x%02x desc=%s\r\n",
				  m->pName, m->IoCaps, m->AuthReq, m->pDesc);
}

static void UartBleSecApply(int Idx)
{
	if (Idx < 0 || Idx >= UART_BLE_SEC_METHOD_CNT)
	{
		return;
	}

	s_SecMethodIdx = Idx;
	BtSmpAuthConfig(s_SecMethods[Idx].IoCaps, s_SecMethods[Idx].AuthReq);
	UartBleSecPrint();
}

// Select the entry whose IO capability and MITM setting match what this build
// was configured with, so the boot default and BLE_SC_METHOD agree.
static void UartBleSecInit(void)
{
	for (int i = 0; i < UART_BLE_SEC_METHOD_CNT; i++)
	{
		if (s_SecMethods[i].IoCaps == BLE_SC_IOCAPS &&
			s_SecMethods[i].AuthReq == (BLE_SC_AUTHREQ))
		{
#if BLE_SC_METHOD == BLE_SC_OOB
			// Just Works and OOB share NoInputNoOutput; take the OOB row.
			if (strcmp(s_SecMethods[i].pName, "oob") != 0)
			{
				continue;
			}
#endif
			s_SecMethodIdx = i;
			break;
		}
	}

	BtSmpAuthConfig(s_SecMethods[s_SecMethodIdx].IoCaps,
					s_SecMethods[s_SecMethodIdx].AuthReq);
}

static int UartBleHexVal(uint8_t c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static int UartBleHexDecode(const uint8_t *pText, int Len, uint8_t *pOut, int MaxOut)
{
	int high = -1;
	int out = 0;

	for (int i = 0; i < Len; i++)
	{
		int v = UartBleHexVal(pText[i]);
		if (v < 0)
		{
			if (pText[i] == ' ' || pText[i] == ':' || pText[i] == '-' ||
				pText[i] == '\r' || pText[i] == '\n' || pText[i] == '\t')
			{
				continue;
			}
			return -1;
		}

		if (high < 0)
		{
			high = v;
		}
		else
		{
			if (out >= MaxOut)
			{
				return -1;
			}
			pOut[out++] = (uint8_t)((high << 4) | v);
			high = -1;
		}
	}

	return (high < 0) ? out : -1;
}

static void UartBlePrintHex(const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++)
	{
		g_Uart.printf("%02X", pData[i]);
	}
}

static void UartBleOobPrintLocal(void)
{
	uint8_t r[16];
	uint8_t c[16];

	if (BtSmpOobLocalDataGen(nullptr, r, c) != 0)
	{
		g_Uart.printf("OOB local data generation failed\r\n");
		return;
	}

	g_Uart.printf("OOB local data. Paste this line on peer:\r\n");
	g_Uart.printf("oob peer ");
	UartBlePrintHex(r, sizeof(r));
	UartBlePrintHex(c, sizeof(c));
	g_Uart.printf("\r\n");
}

static bool UartBleOobSetPeer(const uint8_t *pText, int Len)
{
	uint8_t raw[1 + 6 + 16 + 16];
	int cnt = UartBleHexDecode(pText, Len, raw, sizeof(raw));

	if (cnt == 32)
	{
		BtSmpOobPeerDataSet(&raw[0], &raw[16]);
		s_UartBlePeerOobValid = true;
		g_Uart.printf("OOB peer data loaded\r\n");
		return true;
	}

	if (cnt == 39)
	{
		BtSmpOobPeerDataSet(&raw[7], &raw[23]);
		s_UartBlePeerOobValid = true;
		g_Uart.printf("OOB peer data loaded\r\n");
		return true;
	}

	g_Uart.printf("OOB peer format: oob peer <r+c hex> or <addrtype+addr+r+c hex>\r\n");
	return false;
}

static void UartBleOobInit(void)
{
	UartBleSecPrint();

	// The local set is only needed for the OOB model, and generating it costs a
	// P-256 key pair, so a build that boots into another method does not pay
	// for it. "sec oob" prints it when the method is selected.
	if (UartBleSecIsOob())
	{
		UartBleOobPrintLocal();
	}

	g_Uart.printf("Commands: sec [method], oob, oob peer <hex>\r\n");
}

// "sec" prints the current method, "sec <name>" selects one for the next
// pairing. Selecting OOB prints the local data set, since that is the point at
// which the operator needs it.
static bool UartBleSecTryCommand(const uint8_t *pData, int Len)
{
	if (Len < 3 || memcmp(pData, "sec", 3) != 0)
	{
		return false;
	}

	const uint8_t *p = pData + 3;
	int l = Len - 3;

	while (l > 0 && (*p == ' ' || *p == '\t'))
	{
		p++;
		l--;
	}

	if (l <= 0 || *p == '\r' || *p == '\n')
	{
		UartBleSecPrint();
		return true;
	}

	// Trim the line ending before the compare, the console sends CR or LF.
	while (l > 0 && (p[l - 1] == '\r' || p[l - 1] == '\n' || p[l - 1] == ' '))
	{
		l--;
	}

	for (int i = 0; i < UART_BLE_SEC_METHOD_CNT; i++)
	{
		int nl = (int)strlen(s_SecMethods[i].pName);

		if (nl == l && memcmp(p, s_SecMethods[i].pName, nl) == 0)
		{
			UartBleSecApply(i);

			if (UartBleSecIsOob())
			{
				UartBleOobPrintLocal();
			}

			return true;
		}
	}

	g_Uart.printf("SEC unknown, one of:");
	for (int i = 0; i < UART_BLE_SEC_METHOD_CNT; i++)
	{
		g_Uart.printf(" %s", s_SecMethods[i].pName);
	}
	g_Uart.printf("\r\n");

	return true;
}


static bool UartBleOobTryCommand(const uint8_t *pData, int Len)
{
	if (Len < 3 || memcmp(pData, "oob", 3) != 0)
	{
		return false;
	}

	const uint8_t *p = pData + 3;
	int l = Len - 3;

	while (l > 0 && (*p == ' ' || *p == '\t'))
	{
		p++;
		l--;
	}

	if (l <= 0 || *p == '\r' || *p == '\n')
	{
		UartBleOobPrintLocal();
		return true;
	}

	if (l >= 4 && memcmp(p, "peer", 4) == 0)
	{
		p += 4;
		l -= 4;
		while (l > 0 && (*p == ' ' || *p == '\t' || *p == ':'))
		{
			p++;
			l--;
		}
		(void)UartBleOobSetPeer(p, l);
		return true;
	}

	g_Uart.printf("Commands: oob, oob peer <hex>\r\n");
	return true;
}
#else
static void UartBleOobInit(void) {}
static bool UartBleOobTryCommand(const uint8_t *pData, int Len)
{
	(void)pData;
	(void)Len;
	return false;
}
static bool UartBleSecTryCommand(const uint8_t *pData, int Len)
{
	(void)pData;
	(void)Len;
	return false;
}
#endif

static void UartBlePrintStats(void)
{
	uint32_t elapsedMs;
	BtDevice_t *peer;
	uint16_t mtu;
#if defined(NRFXLIB_SDC)
	uint8_t txPhy;
	uint8_t rxPhy;
#endif
	uint32_t state;
	uint32_t received;
	uint32_t accepted;
	uint32_t dropped;
	uint32_t notifications;
	uint32_t prbsBreaks;
	uint32_t firstRxMs;
	uint16_t minLen;
	uint16_t maxLen;
	int queued;
	const UARTDev_t *uart;
	uint32_t uartDrop;

	state = DisableInterrupt();
	received = s_BleToUartReceived;
	accepted = s_BleToUartAccepted;
	dropped = s_BleToUartDropped;
	notifications = s_BleNotifications;
	prbsBreaks = s_BlePrbsBreaks;
	firstRxMs = s_BleFirstRxMs;
	minLen = notifications ? s_BleNotifyMinLen : 0;
	maxLen = s_BleNotifyMaxLen;
	uart = UARTGetInstance(UART_DEVNO);
	queued = uart != nullptr ? CFifoUsed(uart->hTxFifo) : 0;
	uartDrop = uart != nullptr ? uart->TxDropCnt : 0;

	EnableInterrupt(state);
	elapsedMs = notifications ? BtGattMsTick() - firstRxMs : 0;
	peer = BtPeerFindByHdl(g_ConnectedDev.Conn.Hdl);
	mtu = peer != nullptr ? peer->Conn.MaxMtu : 0;
	// UARTprintf formats at most 79 characters per call. Keep each part
	// within that limit, including maximum-width counter values.
	g_Uart.printf("\r\nBLE: notifications=%lu bytes=%lu ",
		(unsigned long)notifications, (unsigned long)received);
	g_Uart.printf("elapsed_ms=%lu len_min=%u len_max=%u mtu=%u ",
		(unsigned long)elapsedMs, minLen, maxLen, mtu);
	g_Uart.printf("prbs_check=%u prbs_breaks=%lu\r\n",
		(unsigned)CENTRAL_PRBS_CHECK, (unsigned long)prbsBreaks);
#if defined(NRFXLIB_SDC)
	if (BtGapReadPhy(g_ConnectedDev.Conn.Hdl, &txPhy, &rxPhy))
	{
		g_Uart.printf("BLE PHY tx=%u rx=%u\r\n", txPhy, rxPhy);
	}
#endif
	g_Uart.printf("\r\nStream: ble_rx=%lu uart_accepted=%lu ",
		(unsigned long)received, (unsigned long)accepted);
	g_Uart.printf("queued=%d ble_drop=%lu uart_refused=%lu\r\n", queued,
		(unsigned long)dropped, (unsigned long)uartDrop);
}

void UartRxSchedHandler(uint32_t Evt, void *pCtx)
{
	bool flush = false;
	uint8_t *p = NULL;
	int l1;
	int l2;
	int start;
	int i;

	s_bUartRxPending = false;

	(void)Evt;
	(void)pCtx;

#if BLE_SC_METHOD != BLE_SC_NONE
	if (PairInputPoll())
	{
		return;		// console bytes consumed by the pairing step
	}
#endif

	// Internal UartRxBuffer -> External Uart Rx Buffer
	l1 = g_Uart.Rx(&g_UartRxExtBuff[g_UartRxExtBuffLen], PACKET_SIZE - g_UartRxExtBuffLen);

	if (l1 > 0)
	{
		start = g_UartRxExtBuffLen;

		g_UartRxExtBuffLen += l1;
		if (g_UartRxExtBuffLen >= PACKET_SIZE)
		{
			flush = true;
		}

		// A console line ends at CR or LF. Flushing on it makes a short
		// command act at once instead of waiting for the buffer to fill.
		for (i = start; i < g_UartRxExtBuffLen; i++)
		{
			if (g_UartRxExtBuff[i] == '\r' || g_UartRxExtBuff[i] == '\n')
			{
				flush = true;
				break;
			}
		}
	}
	else
	{
		if (g_UartRxExtBuffLen > 0)
		{
			flush = true;
		}
	}

	// A short console prefix may span RX callbacks. Keep it until the line
	// arrives; an immediate empty read must not forward it to the streamer.
	if (g_UartRxExtBuffLen > 0 && g_UartRxExtBuffLen <= 5 &&
		memcmp(g_UartRxExtBuff, "stats", g_UartRxExtBuffLen) == 0)
	{
		return;
	}

	// Flush data ExternalUartRxBuffer -> g_UartRx2BleFifo
	if (flush)
	{
		if ((g_UartRxExtBuffLen == 6 &&
			(memcmp(g_UartRxExtBuff, "stats\n", 6) == 0 ||
			 memcmp(g_UartRxExtBuff, "stats\r", 6) == 0)) ||
			(g_UartRxExtBuffLen == 7 && memcmp(g_UartRxExtBuff, "stats\r\n", 7) == 0))
		{
			g_UartRxExtBuffLen = 0;
			UartBlePrintStats();
			return;
		}
		if (UartBleSecTryCommand(g_UartRxExtBuff, g_UartRxExtBuffLen))
		{
			g_UartRxExtBuffLen = 0;
			return;
		}
		if (UartBleOobTryCommand(g_UartRxExtBuff, g_UartRxExtBuffLen))
		{
			g_UartRxExtBuffLen = 0;
			return;
		}
		l2 = g_UartRxExtBuffLen;
		p = CFifoPutMultiple(g_UartRx2BleFifo, &l2);
		if (p == NULL)
		{
			g_DropCnt++;
		}
		else
		{
			memcpy(p, g_UartRxExtBuff, l2);
			g_UartRxExtBuffLen -= l2;
			if (g_UartRxExtBuffLen > 0)
			{
				memmove(g_UartRxExtBuff, &g_UartRxExtBuff[l2], g_UartRxExtBuffLen);
			}
		}

		// Schedule the BleTxSchedHandler for sending data via BLE
		BleTxQue();
	}

	// Schedule the UartRxSchedHandler if ExternalUartRxBuffer still has data
	if (g_UartRxExtBuffLen > 0)
	{
		UartRxQue();
	}
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	(void)pDev;
	(void)pBuffer;
	(void)BufferLen;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			UartRxQue();
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

void BtAppInitUserData()
{
#if BLE_SC_METHOD != BLE_SC_NONE
	// Start the security module. This call is what links it; BtAppInit
	// fails when SecType asks for security and it was not started.
	BtAppSecInit();
#endif
}

int main()
{
	if (!AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem)))
	{
		return 1;
	}
	HardwareInit();

	if (!BtAppInit(&s_BleAppCfg))
	{
		g_Uart.printf("BtAppInit failed\r\n");
#if defined(NRFXLIB_SDC)
		g_Uart.printf("SDC error=%d value=%d pool_needed=%d pool_size=%u\r\n",
			(int)BtHciCtlrErrorGet(), (int)BtHciCtlrErrorValueGet(),
			(int)BtHciCtlrMemPoolSizeNeeded(), (unsigned)g_BtHciCtlrMemPool.Size);
#endif
		while (true)
		{
			__NOP();
		}
	}

#if BLE_SC_METHOD != BLE_SC_NONE
	// Boot default from BLE_SC_METHOD, then the console "sec" command can move
	// to any other model without a rebuild.
	UartBleSecInit();
#endif
	UartBleOobInit();

	// Register the non-GATT service and its characteristics
	BtAppScanInit((BtGapScanCfg_t*)&s_ScanParams);

	BtAppScan();

	AppRun();

	return 0;
}

bool AppCheckStatus(void)
{
	uint32_t state;
	bool idle;

	BtAppCheckStatus();

	// A pending callback on an empty queue was refused. Keep the check and
	// retry together so an interrupt cannot queue the same callback between them.
	state = DisableInterrupt();
	if (AppEvtHandlerPending() == false)
	{
		if (s_bUartRxPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, UartRxSchedHandler);
		}
		if (s_bBleTxPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, BleTxSchedHandler);
		}
	}
	idle = s_bUartRxPending == false &&
		s_bBleTxPending == false &&
		AppEvtHandlerPending() == false;
	EnableInterrupt(state);
	return idle;
}
