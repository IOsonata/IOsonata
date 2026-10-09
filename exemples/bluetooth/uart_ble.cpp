/**-------------------------------------------------------------------------
@example	uart_ble.cpp

@brief	Uart BLE demo

This application demo shows UART Rx/Tx over BLE custom service using EHAL library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.
The Bluetooth library owns optional pairing, bonding and security.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

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
#include "bluetooth/bt_gatt.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "syslog.h"

#include "board.h"

#ifndef UART_DEVNO
#define UART_DEVNO					0
#endif

#ifndef BUT1_INT
#define BUT1_INT					0
#endif

#define DEVICE_NAME					"UARTDemo"

#define PACKET_SIZE					20

#define MANUFACTURER_NAME			"I-SYST inc."
#define MODEL_NAME					"Generic"

#define MANUFACTURER_ID				ISYST_BLUETOOTH_ID
#define ORG_UNIQUE_ID				ISYST_BLUETOOTH_ID

#define APP_ADV_INTERVAL			64	// in msec

#define APP_ADV_TIMEOUT				0	// in msec

#define MIN_CONN_INTERVAL			10	// in msec
#define MAX_CONN_INTERVAL			40	// in msec

#define BLE_UART_UUID_BASE			BLUEIO_UUID_BASE

#define BLE_UART_UUID_SERVICE		BLUEIO_UUID_UART_SERVICE		//!< BlueIO default service
#define BLE_UART_UUID_TX_CHAR		BLUEIO_UUID_UART_TX_CHAR		//!< Data characteristic
#define BLE_UART_UUID_RX_CHAR		BLUEIO_UUID_UART_RX_CHAR		//!< Command control characteristic

void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = {BLE_UART_UUID_SERVICE,}
};

static const char s_RxCharDescString[] = {
	"UART Rx characteristic",
};

static const char s_TxCharDescString[] = {
	"UART Tx characteristic",
};

uint8_t g_ManData[8];

/// Characteristic definitions
BtGattChar_t g_UartChars[] = {
	// Read + Notify (server-pushed value, peer can also subscribe)
	BT_CHAR(BLE_UART_UUID_RX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			s_RxCharDescString),
	// Write Without Response (command sink, callback consumes)
	BT_CHAR(BLE_UART_UUID_TX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_WRITE_WORESP,
			s_TxCharDescString,
			.WrCB = UartTxSrvcCallback),
};

uint8_t g_LWrBuffer[512];

/// Service definition
BtGattSrvc_t g_UartBleSrvc = BT_SRVC_CUSTOM(BLE_UART_UUID_BASE,
											BLE_UART_UUID_SERVICE,
											g_UartChars);

const BtAppDevInfo_t s_UartBleDevDesc = {
	MODEL_NAME,       		// Model name
	MANUFACTURER_NAME,		// Manufacturer name
	"123",					// Serial number string
	"0.0",					// Firmware version string
	"0.0",					// Hardware version string
};

uint8_t g_AdvLong[] = "1234567890abcdefghijklmnopqrstuvwxyz`!@#$%^&*()_+\0";

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0, 				// Max peripheral devices we connect to as central
	.CentralDevMax = 2, 				// Max central devices we serve as peripheral; keep advertising after the first
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.Appearance = 0,
	.pDevInfo = &s_UartBleDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,		// Open UART/BLE bridge
	.SecExchg = BTAPP_SECEXCHG_NONE,
	.bCompleteUuidList = false,
	.pAdvUuid = &s_AdvUuid,      			// Service uuids to advertise
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = CONNECT_LED_PIN,// Led pin number
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,						// Tx power
	// .SDEvtHandler removed for compatibility with older BtAppCfg_t definitions
	.pLongWrPoolMem = g_LWrBuffer,		// Long-write reassembly pool (split across peer slots)
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

// Board IO tables and UART state, grouped ahead of the first function.
static const IOPinCfg_t s_LedPins[] = LED_PINS;

static int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_ButPins[] = BUTTON_PINS;

static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPinCfg_t);

int g_DelayCnt = 0;
volatile bool g_bUartState = false;
int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define UARTFIFOSIZE				CFIFO_MEMSIZE(256)

static uint8_t s_UartRxFifo[UARTFIFOSIZE];
static uint8_t s_UartTxFifo[UARTFIFOSIZE];

/// UART pins definitions
static IOPinCfg_t s_UartPins[] = UART_PINS;

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,				// Device number zero based
	.pIOPinMap = s_UartPins,				// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),	// Total number of UART pins used
	.Rate = 115200,						// Baudrate
	.DataBits = 8,						// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,						// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,					// Interrupt mode
	.IntPrio = IRQ_PRIO_LOW,			// Interrupt priority
	.EvtCallback = UartEvthandler,		// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

/// UART object instance
UART g_Uart;

// SysLog store. With the UART attached at init, each record goes out as it
// is logged. 16 records of 128 bytes, non blocking, so a burst the UART
// cannot keep up with drops the oldest lines rather than the newest.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = false
};

// Pending stays set through queue refusal until the callback runs.
static volatile bool s_bUartRxPending = false;
static volatile bool s_bSysLogFlushPending = false;

static uint8_t s_UartRxBuff[PACKET_SIZE];
void UartRxHandler(uint32_t Evt, void *pCtx);
static void SysLogFlushEvt(uint32_t Evt, void *pCtx);
static void UartRxQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_bUartRxPending == false)
	{
		s_bUartRxPending = true;
		(void)AppEvtHandlerQue(0, nullptr, UartRxHandler);
	}
	EnableInterrupt(state);
}

static void SysLogFlushQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_bSysLogFlushPending == false)
	{
		s_bSysLogFlushPending = true;
		(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
	}
	EnableInterrupt(state);
}


void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = {BLE_UART_UUID_SERVICE,}
};

static const char s_RxCharDescString[] = {
	"UART Rx characteristic",
};

static const char s_TxCharDescString[] = {
	"UART Tx characteristic",
};

uint8_t g_ManData[8];

/// Characteristic definitions
BtGattChar_t g_UartChars[] = {
	// Read + Notify (server-pushed value, peer can also subscribe)
	BT_CHAR(BLE_UART_UUID_RX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			s_RxCharDescString),
	// Write Without Response (command sink, callback consumes)
	BT_CHAR(BLE_UART_UUID_TX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_WRITE_WORESP,
			s_TxCharDescString,
			.WrCB = UartTxSrvcCallback),
};

uint8_t g_LWrBuffer[512];

/// Service definition
BtGattSrvc_t g_UartBleSrvc = BT_SRVC_CUSTOM(BLE_UART_UUID_BASE,
											BLE_UART_UUID_SERVICE,
											g_UartChars);

const BtAppDevInfo_t s_UartBleDevDesc = {
	MODEL_NAME,       		// Model name
	MANUFACTURER_NAME,		// Manufacturer name
	"123",					// Serial number string
	"0.0",					// Firmware version string
	"0.0",					// Hardware version string
};

uint8_t g_AdvLong[] = "1234567890abcdefghijklmnopqrstuvwxyz`!@#$%^&*()_+\0";

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0, 				// Max peripheral devices we connect to as central
	.CentralDevMax = 2, 				// Max central devices we serve as peripheral; keep advertising after the first
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.Appearance = 0,
	.pDevInfo = &s_UartBleDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLE_SEC_TYPE,			// Secure connection type (see BLE_SC_METHOD selector)
	.SecExchg = BLE_SEC_EXCHG,			// Security key exchange
	.bCompleteUuidList = false,
	.pAdvUuid = &s_AdvUuid,      			// Service uuids to advertise
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = CONNECT_LED_PIN,// Led pin number
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,						// Tx power
	// .SDEvtHandler removed for compatibility with older BtAppCfg_t definitions
	.pLongWrPoolMem = g_LWrBuffer,		// Long-write reassembly pool (split across peer slots)
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

// Board IO tables and UART state, grouped ahead of the first function.
static const IOPinCfg_t s_LedPins[] = LED_PINS;

static int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_ButPins[] = BUTTON_PINS;

static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPinCfg_t);

int g_DelayCnt = 0;
volatile bool g_bUartState = false;
#if BLE_SC_METHOD != BLE_SC_NONE
static volatile bool s_OobRefreshPending = false;
#endif
#if BLE_SC_METHOD != BLE_SC_NONE
enum {
	PAIR_INPUT_NONE = 0,
	PAIR_INPUT_NUMERIC,
	PAIR_INPUT_PASSKEY
};
static volatile int s_PairInput = PAIR_INPUT_NONE;
static uint16_t s_PairConnHdl = 0;
static uint8_t  s_PairDigits = 0;
static uint32_t s_PairPasskey = 0;
#endif

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define UARTFIFOSIZE				CFIFO_MEMSIZE(256)

static uint8_t s_UartRxFifo[UARTFIFOSIZE];
static uint8_t s_UartTxFifo[UARTFIFOSIZE];

/// UART pins definitions
static IOPinCfg_t s_UartPins[] = UART_PINS;

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,				// Device number zero based
	.pIOPinMap = s_UartPins,				// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),	// Total number of UART pins used
	.Rate = 115200,						// Baudrate
	.DataBits = 8,						// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,						// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,					// Interrupt mode
	.IntPrio = IRQ_PRIO_LOW,			// Interrupt priority
	.EvtCallback = UartEvthandler,		// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

/// UART object instance
UART g_Uart;

#if BLE_SC_METHOD != BLE_SC_NONE
static bool s_UartBlePeerOobValid = false;

#ifdef BLE_SC_OOB_NFC
// NFC frame transport provided by the target port.
extern DeviceIntrf *BleOobNfcGetTransport(void);

// External linkage, the target port frame handler references this tag.
RFTag g_BleOobTag;

static uint8_t s_BleOobNdefFile[256];
static bool s_BleOobNfcReady = false;

static const RFTagCfg_t s_BleOobTagCfg = {
	.Proto = RFTAG_PROTO_NFC_T4,
	.XCap = RFTAG_XCAP_ANTICOL | RFTAG_XCAP_CRC | RFTAG_XCAP_FDT,
	.bReadOnly = true,				// pairing record, a reader must not overwrite it
	.pMem = s_BleOobNdefFile,
	.MemSize = sizeof(s_BleOobNdefFile),
	.DevAddr = 0,
	.AddrLen = 2,
	.PageSize = 0,
	.Size = sizeof(s_BleOobNdefFile),
	.WrDelay = 0,
	.NdefAddr = 0,
	.NdefMaxLen = sizeof(s_BleOobNdefFile),
	.NdefFmt = RFTAG_NDEF_FMT_NLEN16,
	.FdPin = {-1, -1},
	.WrProtPin = {-1, -1},
	.pInitCB = nullptr,
	.pWaitCB = nullptr,
	.pEvtCB = nullptr,
	.pCtx = nullptr,
};

#endif

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
	uint8_t IoCaps;				// BT_SMP_IOCAPS_*
	uint8_t AuthReq;				// bonding / MITM, SC is forced by BtSmpAuthConfig
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

// SysLog store. With the UART attached at init, each record goes out as it
// is logged. 16 records of 128 bytes, non blocking, so a burst the UART
// cannot keep up with drops the oldest lines rather than the newest.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = false
};

// Pending stays set through queue refusal until the callback runs.
static volatile bool s_bUartRxPending = false;
static volatile bool s_bUartRxTimeout = false;
static volatile bool s_bSysLogFlushPending = false;

void UartRxChedHandler(uint32_t Evt, void *pCtx);
static void SysLogFlushEvt(uint32_t Evt, void *pCtx);

#if BLE_SC_METHOD != BLE_SC_NONE
#ifdef BLE_SC_OOB_NFC
static DeviceIntrf *s_pOobTransport = nullptr;
#endif
#endif

static uint8_t s_UartRxBuff[PACKET_SIZE];
static int s_UartRxBuffLen = 0;

static void UartRxQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_bUartRxPending == false)
	{
		s_bUartRxPending = true;
		(void)AppEvtHandlerQue(0, nullptr, UartRxChedHandler);
	}
	EnableInterrupt(state);
}

static void SysLogFlushQue(void)
{
	uint32_t state = DisableInterrupt();

	if (s_bSysLogFlushPending == false)
	{
		s_bSysLogFlushPending = true;
		(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
	}
	EnableInterrupt(state);
}

#if BLE_SC_METHOD != BLE_SC_NONE
#ifdef BLE_SC_OOB_NFC
// Publish the local OOB data set on the NFC tag. Must be called with the
// same r and c as the UART printout, the generator makes a new key pair on
// every call so a second generation would invalidate the published confirm.
static void UartBleOobNfcPublish(const uint8_t *pRand, const uint8_t *pConf)
{

	if (s_BleOobNfcReady == false)
	{
		s_pOobTransport = BleOobNfcGetTransport();

		if (s_pOobTransport == nullptr ||
			g_BleOobTag.Init(s_BleOobTagCfg, s_pOobTransport) == false)
		{
			SysLogPrintf(SysLogGet(), "OOB NFC tag init failed\r\n");
			return;
		}
	}
	else if (s_pOobTransport)
	{
		// Republish. Take the field interface down so a reader cannot see a
		// half written record, the update is not atomic against RF reads.
		s_pOobTransport->Disable();
	}

	BtOobLe_t oob;
	RFNdefMsg_t msg;
	uint8_t msgbuf[160];

	memset(&oob, 0, sizeof(oob));
	BtSmpLocalAddrGet(&oob.AddrType, oob.Addr);
	oob.Role = BT_OOB_LEROLE_PERIPH;
	memcpy(oob.Confirm, pConf, 16);
	memcpy(oob.Rand, pRand, 16);
	oob.pName = DEVICE_NAME;

	RFNdefInit(&msg, msgbuf, sizeof(msgbuf));

	if (BtOobLeNdefAdd(&msg, &oob) == false ||
		g_BleOobTag.SetNdef(msg.pBuf, msg.Len) == false)
	{
		SysLogPrintf(SysLogGet(), "OOB NFC publish failed, NFC disabled\r\n");
		return;
	}

	// The record is in place, bring the field interface up.
	if (s_pOobTransport)
	{
		s_pOobTransport->Enable();
	}

	s_BleOobNfcReady = true;

	SysLogPrintf(SysLogGet(), "OOB data published on NFC tag, tap to pair\r\n");
}
#endif

static bool UartBleSecIsOob(void)
{
	return strcmp(s_SecMethods[s_SecMethodIdx].pName, "oob") == 0;
}

// One line, fixed field order, so a host script can parse it. Printed on every
// change and on a bare "sec".
static void UartBleSecPrint(void)
{
	const UartBleSecMethod_t *m = &s_SecMethods[s_SecMethodIdx];

	SysLogPrintf(SysLogGet(), "SEC method=%s iocaps=%d authreq=0x%02x desc=%s\r\n",
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
	if (c >= '0' && c <= '9')
	{
		return c - '0';
	}
	if (c >= 'a' && c <= 'f')
	{
		return c - 'a' + 10;
	}
	if (c >= 'A' && c <= 'F')
	{
		return c - 'A' + 10;
	}
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
		SysLogPrintf(SysLogGet(), "%02X", pData[i]);
	}
}

static void UartBleOobPrintLocal(void)
{
	uint8_t r[16];
	uint8_t c[16];

	if (BtSmpOobLocalDataGen(g_BtAppData.AppDevice.pHciDev, r, c) != 0)
	{
		SysLogPrintf(SysLogGet(), "OOB local data generation failed\r\n");
		return;
	}

	SysLogPrintf(SysLogGet(), "OOB local data. Paste this line on peer:\r\n");
	SysLogPrintf(SysLogGet(), "oob peer ");
	UartBlePrintHex(r, sizeof(r));
	UartBlePrintHex(c, sizeof(c));
	SysLogPrintf(SysLogGet(), "\r\n");

#ifdef BLE_SC_OOB_NFC
	// Same r and c as the printout, one generation feeds both channels.
	UartBleOobNfcPublish(r, c);
#endif
}

static bool UartBleOobSetPeer(const uint8_t *pText, int Len)
{
	uint8_t raw[1 + 6 + 16 + 16];
	int cnt = UartBleHexDecode(pText, Len, raw, sizeof(raw));

	if (cnt == 32)
	{
		BtSmpOobPeerDataSet(&raw[0], &raw[16]);
		s_UartBlePeerOobValid = true;
		SysLogPrintf(SysLogGet(), "OOB peer data loaded\r\n");
		return true;
	}

	if (cnt == 39)
	{
		BtSmpOobPeerDataSet(&raw[7], &raw[23]);
		s_UartBlePeerOobValid = true;
		SysLogPrintf(SysLogGet(), "OOB peer data loaded\r\n");
		return true;
	}

	SysLogPrintf(SysLogGet(), "OOB peer format: oob peer <r+c hex> or <addrtype+addr+r+c hex>\r\n");
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

	SysLogPrintf(SysLogGet(), "Commands: sec [method], oob, oob peer <hex>, bond del\r\n");
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

	SysLogPrintf(SysLogGet(), "SEC unknown, one of:");
	for (int i = 0; i < UART_BLE_SEC_METHOD_CNT; i++)
	{
		SysLogPrintf(SysLogGet(), " %s", s_SecMethods[i].pName);
	}
	SysLogPrintf(SysLogGet(), "\r\n");

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

	SysLogPrintf(SysLogGet(), "Commands: oob, oob peer <hex>\r\n");
	return true;
}

// The published OOB set is single use. BtSmpPairingComplete stays owned by the
// port, which calls BtAppEvtSecured on a successful pairing. That hook flags a
// refresh and the work runs in app context from UartRxChedHandler, so no
// crypto, UART or NFCT work runs in the pairing event callback.
static void UartBleOobRefresh(void)
{
	if (UartBleSecIsOob() == false)
	{
		return;			// nothing was consumed, nothing to replace
	}

	BtSmpOobDataClear();
	s_UartBlePeerOobValid = false;
	UartBleOobPrintLocal();
}

void BtAppEvtSecured(uint16_t ConnHdl)
{
	(void)ConnHdl;

	// Keep the callback light. Queue the app context handler to do the refresh.
	s_OobRefreshPending = true;
	UartRxQue();
}
#else
static void UartBleOobInit(void)
{
}

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

#if BLE_SC_METHOD != BLE_SC_NONE
// Console command: "bond del" wipes every stored bond, so a reset after it has
// to pair again. Anything else starting with "bond" prints the usage. Returns
// true when the line was a command and must not go on air.
//
// No target conditional here. BtSmpBondClearAll is the generic entry and each
// port supplies the one that reaches its own bond storage: the RAM table for
// the SoftDevice ports, pm_peers_delete for the BM one.
static bool UartBleBondTryCommand(const uint8_t *pData, int Len)
{
	// This link is a data bridge, so anything that is not exactly the
	// command goes on air untouched: "bond" must be followed by whitespace,
	// the argument must be exactly "del", and nothing but line endings may
	// follow. "bond delivery data" is payload, not a request to clear the
	// security state.
	if (Len < 5 || memcmp(pData, "bond", 4) != 0 ||
		(pData[4] != ' ' && pData[4] != '\t'))
	{
		return false;
	}

	const uint8_t *p = pData + 5;
	int l = Len - 5;

	while (l > 0 && (*p == ' ' || *p == '\t'))
	{
		p++;
		l--;
	}

	if (l < 3 || memcmp(p, "del", 3) != 0)
	{
		return false;
	}

	p += 3;
	l -= 3;

	while (l > 0 && (*p == '\r' || *p == '\n' || *p == ' ' || *p == '\t'))
	{
		p++;
		l--;
	}

	if (l != 0)
	{
		return false;
	}

	// Requested, not done: the delete is queued and each peer reports as it
	// finishes; the storage adapter applies the deletion asynchronously.
	SysLogPrintf(SysLogGet(), "bond deletion requested\r\n");
	BtSmpBondClearAll();

	return true;
}
#else
static bool UartBleBondTryCommand(const uint8_t *pData, int Len)
{
	(void)pData;
	(void)Len;
	return false;
}
#endif

void UartTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{
	g_Uart.Tx(pData, Len);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void * const pCtx)
{
	//BtGattEvtHandler(Evt, pCtx);
}

void BtAppInitUserServices()
{
	bool res;
	res = BtGattSrvcAdd(&g_UartBleSrvc);
}

void ButEvent(int IntNo, void *pCtx)
{
	if (IntNo == BUT1_INT)
	{
		if (g_bUartState == false)
		{
			g_Uart.Enable();
			g_bUartState = true;
		}
		else
		{
			g_Uart.Disable();
			g_bUartState = false;
		}
	}
}

void HardwareInit()
{
	g_bUartState = g_Uart.Init(g_UartCfg);

	// Route SysLog to the same UART so the SMP/ATT stack traces (SMP_TRACE,
	// DEBUG_PRINTF) appear here alongside the application output. Without this
	// the stack pairs/runs silently and no trace is seen.
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)g_Uart, 0,
			   nullptr, 0);

	IOPinCfg(s_LedPins, s_NbLedPins);

	for (int i = 0; i < s_NbLedPins; i++)
	{
		IOPinSet(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
	}

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinEnableInterrupt(BUT1_INT, IRQ_PRIO_LOW, s_ButPins[0].PortNo,
						 s_ButPins[0].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent, NULL);
}

void BtAppInitUserData()
{
}


// UART data is forwarded by the application; the Bluetooth library owns security.
void UartRxHandler(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bUartRxPending = false;
	int len = g_Uart.Rx(s_UartRxBuff, sizeof(s_UartRxBuff));
	if (len > 0)
	{
		(void)BtAppNotify(&g_UartChars[0], s_UartRxBuff, (uint16_t)len);
	}
}

#if 0
uint32_t BleSrvcCharNotify(BtGattSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	BtGattCharNotify(&pSrvc->pCharArray[Idx], pData, DataLen);

	return 0;
}
#endif

// SysLog sends a record when it is logged. A record the UART had no room for
// waits in the store, so the UART ready event queues one flush to send it.
static void SysLogFlushEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bSysLogFlushPending = false;
	(void)SysLogFlush(SysLogGet());
}

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			UartRxQue();
			break;
		case UART_EVT_TXREADY:
			if (CFifoPeek(SysLogGet()->hFifo) != nullptr)
			{
				SysLogFlushQue();
			}
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
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

	SysLogPrintf(SysLogGet(), "UART over BLE\r\n");

	
	if (!BtAppInit(&s_BleAppCfg))
	{
		SysLogPrintf(SysLogGet(), "BtAppInit failed\r\n");
		while (true)
		{
			__NOP();
		}
	}

	AppRun();

	// AppRun is not expected to return. Keep embedded startup from falling
	// through newlib exit if it ever does.
	SysLogPrintf(SysLogGet(), "AppRun returned\r\n");
	while (true)
	{
		__NOP();
	}
}

bool AppCheckStatus(void)
{
	BtAppCheckStatus();

	// A pending callback on an empty queue was refused. Keep the check and
	// retry together so an interrupt cannot queue the same callback between them.
	uint32_t state = DisableInterrupt();

	if (AppEvtHandlerPending() == false)
	{
		if (s_bUartRxPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, UartRxChedHandler);
		}
		if (s_bSysLogFlushPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
		}
	}
	const bool idle = s_bUartRxPending == false &&
		s_bSysLogFlushPending == false &&
		AppEvtHandlerPending() == false;
	EnableInterrupt(state);
	return idle;
}
