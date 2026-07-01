/**-------------------------------------------------------------------------
@example	ble_smp_test.cpp

@brief	SMP (Security Manager Protocol) test peripheral.

A connectable peripheral with one custom service, used to exercise the SMP
secure-connection path on the SDC backend. It traces every milestone of the
pairing handshake over UART so the exchange can be watched live:

  connect -> pairing request from central -> (SC: pubkey/DHKey) ->
  confirm/random -> LTK request -> encryption change -> pairing complete

The peripheral is a Just Works RESPONDER: it does not push a Security
Request, it reacts when the central starts pairing. To test:

  1. Flash this to the board (SDC build, e.g. nRF52832 Debug_SDC).
  2. Open the UART at 115200 8N1 (CoolTerm). You should see "advertising".
  3. From a phone (nRF Connect app) scan for "SmpTest" and Connect.
  4. In nRF Connect, tap the bond/pair action (three-dot menu -> Bond),
     or trigger pairing by reading the protected characteristic.
  5. Watch the UART trace walk through the SMP states and end with
     "PAIRING COMPLETE".

What each provider does here:
  - SDC build composes crypto: ECDH via uECC, AES via LE Encrypt, RNG via HW
    offloaded to the controller, randomness from LE Rand.
  - The generic bt_smp.cpp drives the state machine; this app observes and
    stores one bond in RAM for reconnect testing.

@author	Hoang Nguyen Hoan
@date	May 2026

@license MIT, (c) 2026 I-SYST. See bt_app.h for full text.
----------------------------------------------------------------------------*/
#include <string.h>

#include "istddef.h"
#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "iopinctrl.h"
#include "syslog.h"

#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"			// BtSmpPairingComplete hook + BtSmpKeys_t
#include "crypto/crypto.h"				// crypto engine instances + BtSmpCrypto self-test
#include "bluetooth/blueio_blesrvc.h"	// BLUEIO_UUID_BASE / service uuids
#include "bluetooth/bt_appearance.h"

#include "board.h"

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

//-----------------------------------------------------------------------------
// Names / identifiers
//-----------------------------------------------------------------------------

#define DEVICE_NAME			"SmpTest"
#define MANUFACTURER_NAME	"I-SYST inc."
#define MODEL_NAME			"SmpTest"

#define APP_ADV_INTERVAL	64		// msec
#define APP_ADV_TIMEOUT		0		// 0 = no timeout, advertise until connected
#define MIN_CONN_INTERVAL	10		// msec
#define MAX_CONN_INTERVAL	40		// msec

#define PACKET_SIZE			20

// Reuse the BlueIO UART service UUIDs as a generic test service. The names
// don't matter here; we only need a connectable GATT DB for the central to
// bind to before it can pair.
#define SMP_UUID_BASE		BLUEIO_UUID_BASE
#define SMP_UUID_SERVICE	BLUEIO_UUID_UART_SERVICE
#define SMP_UUID_VALUE_CHAR	BLUEIO_UUID_UART_RX_CHAR

//-----------------------------------------------------------------------------
// UART trace
//-----------------------------------------------------------------------------

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void SmpValueWriteCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

#define UARTFIFOSIZE		CFIFO_MEMSIZE(256)
static uint8_t s_UartRxFifo[UARTFIFOSIZE];
static uint8_t s_UartTxFifo[UARTFIFOSIZE];

static IOPinCfg_t s_UartPins[] = UART_PINS;

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
	.IntPrio = 6,
	.EvtCallback = UartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
};

UART g_Uart;

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	(void)pDev;
	(void)EvtId;
	(void)pBuffer;
	(void)BufferLen;

	return 0;
}

// Hex dump helper for keys.
static void TraceHex(const char *pLabel, const uint8_t *pData, size_t Len)
{
	g_Uart.printf("%s", pLabel);
	for (size_t i = 0; i < Len; i++)
	{
		g_Uart.printf("%02x", pData[i]);
	}
	g_Uart.printf("\r\n");
}

//-----------------------------------------------------------------------------
// GATT service (one writable/readable value char so the central has a reason
// to connect; pairing is driven by the central, not by reads here).
//-----------------------------------------------------------------------------

static const char s_ValueCharDesc[] = "SMP test value";

static uint8_t s_ValueData[PACKET_SIZE] = { 0 };

BtGattChar_t g_SmpChars[] = {
	BT_CHAR(SMP_UUID_VALUE_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_NOTIFY,
			s_ValueCharDesc,
			.WrCB = SmpValueWriteCallback),
};

uint8_t g_LWrBuffer[512];

BtGattSrvc_t g_SmpSrvc = BT_SRVC_CUSTOM(SMP_UUID_BASE, SMP_UUID_SERVICE, g_SmpChars);

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = { SMP_UUID_SERVICE, }
};

static const BtAppDevInfo_t s_SmpDevInfo = {
	MODEL_NAME,
	MANUFACTURER_NAME,
	"001",		// serial
	"0.0",		// fw
	"0.0",		// hw
};

uint8_t g_ManData[4] = { 0 };

//-----------------------------------------------------------------------------
// RAM bond store for this test app.
//
// This is intentionally small: one central, one generated LTK. It proves the
// bonding path without pulling in a flash settings layer yet. It survives
// disconnect/reconnect while the board remains powered. It does not survive
// reset, reflash, or power-cycle.
//-----------------------------------------------------------------------------

static BtSmpKeys_t s_SmpBondKeys;
static bool s_SmpBondValid = false;

extern "C" bool BtSmpBondLtkLookup(uint16_t ConnHdl, uint64_t Rand,
								   uint16_t Ediv, uint8_t Ltk[16])
{
	(void)ConnHdl;

	// LE Secure Connections uses EDIV=0 and Rand=0 for the generated LTK.
	// This SMP test is single-peer, so one cached bond record is enough.
	if (s_SmpBondValid && s_SmpBondKeys.bValid &&
		s_SmpBondKeys.bSc && Ediv == 0 && Rand == 0)
	{
		memcpy(Ltk, s_SmpBondKeys.Ltk, 16);
		g_Uart.printf("SMP bond lookup: found SC LTK\r\n");
		return true;
	}

	g_Uart.printf("SMP bond lookup: no key (ediv=%u rand=%llu)\r\n",
				  (unsigned)Ediv, (unsigned long long)Rand);
	return false;
}

//-----------------------------------------------------------------------------
// App configuration. SecType requests Just Works (no MITM). Switch to
// BTGAP_SECTYPE_LESC_MITM later to exercise the MITM / numeric-comparison
// path once an IO backend exists.
//-----------------------------------------------------------------------------

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0,
	.PeriLinkCount = 1,
	.pDevName = DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 0,
	.Appearance = BT_APPEAR_UNKNOWN_GENERIC,
	.pDevInfo = &s_SmpDevInfo,
	.bExtAdv = false,
	.pAdvManData = g_ManData,
	.AdvManDataLen = sizeof(g_ManData),
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_STATICKEY_NO_MITM,	// Just Works pairing + bonding
	.SecExchg = BTAPP_SECEXCHG_KEYBOARD,		// distribute keys (IRK/CSRK)
	.bCompleteUuidList = true,
	.pAdvUuid = &s_AdvUuid,
	.AdvInterval = APP_ADV_INTERVAL,
	.AdvTimeout = APP_ADV_TIMEOUT,
	.AdvSlowInterval = 0,
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = -1,
	.ConnLedPin = -1,
	.TxPower = 0,
	.pLongWrPoolMem = g_LWrBuffer,
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

//-----------------------------------------------------------------------------
// App hooks
//-----------------------------------------------------------------------------

void BtAppInitUserServices()
{
	BtGattSrvcAdd(&g_SmpSrvc);

	// Give the value characteristic a defined initial value so a read during
	// discovery returns data rather than an empty value.
	BtGattCharSetValue(&g_SmpChars[0], s_ValueData, PACKET_SIZE);

	g_Uart.printf("service added\r\n");
}

// Echo every write back to the central as a notification (if subscribed), so
// read/write/notify can all be exercised against the encrypted link.
void SmpValueWriteCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{
	(void)Offset;

	int l = Len > PACKET_SIZE ? PACKET_SIZE : Len;
	memcpy(s_ValueData, pData, l);

	g_Uart.printf("GATT write %d bytes: ", l);
	for (int i = 0; i < l; i++) g_Uart.printf("%02x", s_ValueData[i]);
	g_Uart.printf("\r\n");

	if (isBtGattCharNotifyEnabled(pChar))
	{
		bool sent = BtAppNotify(pChar, s_ValueData, (uint16_t)l);
		g_Uart.printf("GATT notify echo: %s\r\n", sent ? "sent" : "failed");
	}
	else
	{
		g_Uart.printf("GATT notify not enabled by central\r\n");
	}
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	g_Uart.printf("CONNECTED hdl=%d - requesting security\r\n", ConnHdl);
	// Peripheral-initiated security: prompt the central to encrypt with an
	// existing bond or start pairing. Without this a central has no signal to
	// secure the link.
	BtSmpRequestSecurity(ConnHdl);
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	g_Uart.printf("DISCONNECTED hdl=%d\r\n", ConnHdl);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void * const pCtx)
{
	(void)Evt;
	(void)pCtx;
}

// SMP completion hook (weak default lives in bt_smp.cpp; this overrides it).
extern "C" void BtSmpPairingComplete(uint16_t ConnHdl, bool Success, const BtSmpKeys_t *pKeys)
{
	if (!Success || pKeys == nullptr)
	{
		g_Uart.printf("\r\n*** PAIRING FAILED on hdl=%d ***\r\n", ConnHdl);
		return;
	}

	memcpy(&s_SmpBondKeys, pKeys, sizeof(s_SmpBondKeys));
	s_SmpBondValid = true;

	g_Uart.printf("\r\n*** PAIRING COMPLETE on hdl=%d ***\r\n", ConnHdl);
	g_Uart.printf("  mode      : %s\r\n", pKeys->bSc ? "LE Secure Connections" : "Legacy");
	g_Uart.printf("  MITM auth : %s\r\n", pKeys->bAuthenticated ? "yes" : "no (Just Works)");
	g_Uart.printf("  key size  : %d bytes\r\n", pKeys->EncKeySize);
	TraceHex("  LTK       : ", pKeys->Ltk, 16);
	if (pKeys->bValid)
	{
		TraceHex("  peer IRK  : ", pKeys->Irk, 16);
		TraceHex("  peer CSRK : ", pKeys->Csrk, 16);
		g_Uart.printf("  peer ID   : type=%d addr=", pKeys->IdAddrType);
		for (int i = 5; i >= 0; i--) g_Uart.printf("%02x", pKeys->IdAddr[i]);
		g_Uart.printf("\r\n");
	}
	g_Uart.printf("  bond      : saved in RAM for reconnect\r\n");
	g_Uart.printf("  link is now encrypted\r\n\r\n");
}

//-----------------------------------------------------------------------------
// Main
//-----------------------------------------------------------------------------

void HardwareInit()
{
	g_Uart.Init(s_UartCfg);

	// Route SysLog to the same UART so the library's SMP_TRACE output (and any
	// HCI DEBUG_PRINTF) appears here. No timer -> no timestamps; emit all.
	SysLogInit(SysLogGet(), (DevIntrf_t*)g_Uart, 0, nullptr, 0);

	// SMP crypto (software uECC ECDH + BLE controller AES) is owned and brought
	// up by the BLE app layer during BtAppInit; the application does not set it
	// up here.
}

int main()
{
	HardwareInit();

	g_Uart.printf("\r\n=== IOsonata SMP test peripheral ===\r\n");
	g_Uart.printf("device name : %s\r\n", DEVICE_NAME);
	g_Uart.printf("sec type    : Just Works + bonding (responder)\r\n");

	BtAppInit(&s_BleAppCfg);

	// --- AES provider self-test (NIST FIPS-197 C.1 known vector) ---------
	// Runs after BtAppInit so the SDC controller is enabled (the SDC provider
	// offloads AES to LE Encrypt, which needs the controller up). If this
	// FAILs, every SMP confirm/check value is wrong and pairing dies with
	// CONFIRM_VALUE - check this line first. A FAIL almost always means the
	// byte order in BtSmpCryptoAes128 is flipped.
	{
		static const uint8_t k[16] = {
			0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,
			0x08,0x09,0x0a,0x0b,0x0c,0x0d,0x0e,0x0f };
		static const uint8_t pt[16] = {
			0x00,0x11,0x22,0x33,0x44,0x55,0x66,0x77,
			0x88,0x99,0xaa,0xbb,0xcc,0xdd,0xee,0xff };
		static const uint8_t expect[16] = {
			0x69,0xc4,0xe0,0xd8,0x6a,0x7b,0x04,0x30,
			0xd8,0xcd,0xb7,0x80,0x70,0xb4,0xc5,0x5a };
		uint8_t out[16];
		BtSmpCryptoAes128(nullptr, k, pt, out);
		bool ok = (memcmp(out, expect, 16) == 0);
		g_Uart.printf("AES self-test: %s\r\n", ok ? "PASS" : "FAIL");
		if (!ok)
		{
			g_Uart.printf("  got    : ");
			for (int i = 0; i < 16; i++) g_Uart.printf("%02x", out[i]);
			g_Uart.printf("\r\n  expect : 69c4e0d86a7b0430d8cdb78070b4c55a\r\n");
		}
	}
	// ---------------------------------------------------------------------

	// --- ECDH self-test: provider runs the BLE spec DH known vector --------
	// If this FAILs, the software P-256/ECDH or its byte order is wrong on
	// this target and SC pairing cannot work.
	{
		int r = BtSmpCryptoSelfTest();
		g_Uart.printf("ECDH self-test: %s (rc=%d)\r\n", r == 0 ? "PASS" : "FAIL", r);
	}
	// ---------------------------------------------------------------------

	// --- f4 self-test: SmpF4 against the BLE spec worked example -----------
	{
		int r = BtSmpF4SelfTest();
		g_Uart.printf("f4 self-test: %s (rc=%d)\r\n", r == 0 ? "PASS" : "FAIL", r);
	}
	// ---------------------------------------------------------------------

	g_Uart.printf("advertising - connect from nRF Connect and tap Bond\r\n");

	BtAppRun();

	return 0;
}
