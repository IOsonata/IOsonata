/**-------------------------------------------------------------------------
@example	uart_ble.cpp


@brief	Uart BLE demo

This application demo shows UART Rx/Tx over BLE custom service using EHAL library.
For evaluating power consumption of the UART, the button 1 is used to enable/disable it.
This example also demonstrates passkey paring mode.

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

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "app_evt_handler.h"
#include "syslog.h"

#include "board.h"

// LE Secure Connections method selector.
//
// This board has no screen and no buttons: the UART console is both the display
// and the keyboard. The SMP core prints the numeric value or the passkey to the
// terminal and reads the y/n or the typed passkey back from it, so one firmware
// can exercise any association model. Pick a method here and build the peer
// (uart_ble_central) with a compatible row:
//
//   this side             peer side             resulting method
//   BLE_SC_NONE           BLE_SC_NONE           open link, no pairing
//   BLE_SC_JUSTWORKS      BLE_SC_JUSTWORKS      Just Works (bonded, no MITM)
//   BLE_SC_NUMCOMP        BLE_SC_NUMCOMP        Numeric Comparison
//   BLE_SC_PASSKEY_DISP   BLE_SC_PASSKEY_INPUT  Passkey Entry (this side shows)
//   BLE_SC_PASSKEY_INPUT  BLE_SC_PASSKEY_DISP   Passkey Entry (this side types)
//   BLE_SC_OOB            BLE_SC_OOB            LESC OOB via UART copy/paste
#define BLE_SC_NONE				0
#define BLE_SC_JUSTWORKS		1
#define BLE_SC_NUMCOMP			2
#define BLE_SC_PASSKEY_DISP		3
#define BLE_SC_PASSKEY_INPUT	4
#define BLE_SC_OOB				5

#ifndef BLE_SC_METHOD
#define BLE_SC_METHOD			BLE_SC_OOB
#endif

#if BLE_SC_METHOD != BLE_SC_NONE
#include "bluetooth/bt_smp.h"		// SMP IO caps, console pairing callbacks, bond hooks
#if BLE_SC_METHOD == BLE_SC_OOB
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_OOB
#else
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_KEYBOARD		// distribute IRK/CSRK
#endif

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
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_NONE
#define BLE_SC_NAME				"NONE (open link)"
#endif

#define DEVICE_NAME                     "UARTDemo"

#define PACKET_SIZE						20

#define MANUFACTURER_NAME               "I-SYST inc."
#define MODEL_NAME                      "Generic"

#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID

#define APP_ADV_INTERVAL                64	// in msec

#define APP_ADV_TIMEOUT					0	// in msec

#define MIN_CONN_INTERVAL               10	// in msec
#define MAX_CONN_INTERVAL               40	// in msec

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
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
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

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

static uint8_t s_UartRxFifo[UARTFIFOSIZE];
static uint8_t s_UartTxFifo[UARTFIFOSIZE];

/// UART pins definitions
static IOPinCfg_t s_UartPins[] = UART_PINS;

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = 0,							// Device number zero based
	.pIOPinMap = s_UartPins,				// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),	// Total number of UART pins used
	.Rate = 1000000,						// Baudrate
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


#if BLE_SC_METHOD == BLE_SC_OOB
static bool s_UartBlePeerOobValid = false;

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

	if (BtSmpOobLocalDataGen(g_BtAppData.AppDevice.pHciDev, r, c) != 0)
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
	UartBleOobPrintLocal();
	g_Uart.printf("Enter peer data before pairing. Commands: oob, oob peer <hex>\r\n");
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
#endif


static const IOPinCfg_t s_LedPins[] = LED_PINS;

static int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPinCfg_t);

static const IOPinCfg_t s_ButPins[] = UART_PINS;//{
//	{BUTTON1_PORT, BUTTON1_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},// Button 1
//	{BUTTON2_PORT, BUTTON2_PIN, 0, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},// Button 2
//};

static int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPinCfg_t);

int g_DelayCnt = 0;
volatile bool g_bUartState = false;


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

#if BLE_SC_METHOD != BLE_SC_NONE
void BtAppEvtConnected(uint16_t ConnHdl)
{
	// Security is initiated by the underlying stack when a secure SecType is
	// configured - the application stays SDK-neutral and does not request it here.
	g_Uart.printf("CONNECTED hdl=%d\r\n", ConnHdl);
}

// Bond capture (BtSmpBondAdd) and LTK lookup (BtSmpBondLtkLookup) are provided
// by the library (src/bluetooth/bt_smp_bond.cpp): a multi-slot bond table with
// persistence hooks. The example does not redefine them.
#endif

void ButEvent(int IntNo, void *pCtx)
{
	if (IntNo == 0)
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
	g_Uart.Init(g_UartCfg);

	// Route SysLog to the same UART so the SMP/ATT stack traces (SMP_TRACE,
	// DEBUG_PRINTF) appear here alongside the application output. Without this
	// the stack pairs/runs silently and no trace is seen.
	SysLogInit(SysLogGet(), (DevIntrf_t*)g_Uart, 0, nullptr, 0);

	IOPinCfg(s_LedPins, s_NbLedPins);

	for (int i = 0; i < s_NbLedPins; i++)
	{
		IOPinSet(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
	}

	IOPinCfg(s_ButPins, s_NbButPins);

	IOPinEnableInterrupt(0, IRQ_PRIO_LOW, s_ButPins[0].PortNo, s_ButPins[0].PinNo, IOPINSENSE_LOW_TRANSITION, ButEvent, NULL);
}

void BtAppInitUserData()
{
#if BLE_SC_METHOD != BLE_SC_NONE
	// Set the local IO capability so the SMP core negotiates the selected
	// method; the console then fills whatever role that capability implies.
	BtSmpAuthConfig(BLE_SC_IOCAPS, BLE_SC_AUTHREQ);
#endif
}

#if BLE_SC_METHOD != BLE_SC_NONE
// Console pairing IO. The board has no physical display or keypad, so the SMP
// core routes the user step through the UART: BtSmpNumericComparison and
// BtSmpPasskeyDisplay print, and PairInputPoll (called from the UART RX path)
// reads the y/n or the typed passkey and resumes pairing.
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

// Consume console bytes while a pairing user step is pending. Returns true while
// it owns the RX bytes so the data path does not forward them.
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

//void UartRxChedHandler(void * p_event_data, uint16_t event_size)
void UartRxChedHandler(uint32_t Evt, void *pCtx)
{
#if BLE_SC_METHOD != BLE_SC_NONE
	if (PairInputPoll())
	{
		return;		// console bytes consumed by the pairing step
	}
#endif
	static uint8_t buff[PACKET_SIZE];
	static int bufflen = 0;
	bool flush = false;

	int l = g_Uart.Rx(&buff[bufflen], PACKET_SIZE - bufflen);
	if (l > 0)
	{
		bufflen += l;
		if (bufflen >= PACKET_SIZE)
		{
			flush = true;
		}
	}
	else
	{
		if (bufflen > 0)
		{
			flush = true;
		}
	}
	if (flush)
	{
		if (UartBleOobTryCommand(buff, bufflen))
		{
			bufflen = 0;
			return;
		}
//		if (BleSrvcCharNotify(&g_UartBleSrvc, 0, buff, bufflen) == 0)
		if (BtAppNotify(&g_UartChars[0], buff, (uint16_t)bufflen) == true)
		{
			bufflen = 0;
		}
		//app_sched_event_put(NULL, 0, UartRxChedHandler);
		AppEvtHandlerQue(0, 0, UartRxChedHandler);
	}
}

#if 0
uint32_t BleSrvcCharNotify(BtGattSrvc_t *pSrvc, int Idx, uint8_t *pData, uint16_t DataLen)
{
	BtGattCharNotify(&pSrvc->pCharArray[Idx], pData, DataLen);

	return 0;
}
#endif

int UartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			//app_sched_event_put(NULL, 0, UartRxChedHandler);
			AppEvtHandlerQue(0, 0, UartRxChedHandler);
			break;
		case UART_EVT_TXREADY:
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

    g_Uart.printf("UART over BLE Demo\r\n");
    g_Uart.printf("security    : %s\r\n", BLE_SC_NAME);

    //g_Uart.Disable();

    BtAppInit(&s_BleAppCfg);
    UartBleOobInit();

    BtAppRun();

	return 0;
}
