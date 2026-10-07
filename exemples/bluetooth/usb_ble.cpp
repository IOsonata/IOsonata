/**-------------------------------------------------------------------------
@example usb_ble.cpp

@brief USB CDC to BLE peripheral bridge

The board enumerates as a USB CDC serial port and exposes the BlueIO UART
service as a BLE peripheral. Bytes written by the USB host are sent as BLE
notifications; writes from the BLE peer are returned to the host on USB CDC.

The USB CDC port is also the security console. It supports the same LE Secure
Connections association models as uart_ble.cpp, including Numeric Comparison,
Passkey Entry and copy/paste LESC OOB data.

USB and Bluetooth share the application event queue. Interrupt callbacks only
queue work; pairing input, command parsing and BLE notification submission run
in application context.

@author Hoang Nguyen Hoan
@date Oct. 5, 2026

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
#include <stdarg.h>
#include <string.h>

#include "istddef.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "syslog.h"
#include "usb/usb.h"
#include "usb/usbd_cdc.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/blueio_blesrvc.h"

#include "board.h"

#define BLE_SC_NONE                 0
#define BLE_SC_JUSTWORKS            1
#define BLE_SC_NUMCOMP              2
#define BLE_SC_PASSKEY_DISP         3
#define BLE_SC_PASSKEY_INPUT        4
#define BLE_SC_OOB                  5

#ifndef BLE_SC_METHOD
#define BLE_SC_METHOD               BLE_SC_NUMCOMP
#endif

#if BLE_SC_METHOD == BLE_SC_JUSTWORKS
#define BLE_SEC_EXCHG               BTAPP_SECEXCHG_NONE
#define BLE_SEC_TYPE                BTGAP_SECTYPE_STATICKEY_NO_MITM
#define BLE_SC_IOCAPS               BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#define BLE_SC_AUTHREQ              BT_SMP_AUTHREQ_BONDING_FLAG_BONDING
#define BLE_SC_NAME                 "Just Works (bonded, no MITM)"
#elif BLE_SC_METHOD == BLE_SC_NUMCOMP
#define BLE_SEC_EXCHG               (BTAPP_SECEXCHG_DISPLAY | BTAPP_SECEXCHG_YESNO)
#define BLE_SEC_TYPE                BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS               BT_SMP_IOCAPS_DISPLAY_YESNO
#define BLE_SC_AUTHREQ              (BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME                 "Numeric Comparison"
#elif BLE_SC_METHOD == BLE_SC_PASSKEY_DISP
#define BLE_SEC_EXCHG               BTAPP_SECEXCHG_DISPLAY
#define BLE_SEC_TYPE                BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS               BT_SMP_IOCAPS_DISPLAY_ONLY
#define BLE_SC_AUTHREQ              (BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME                 "Passkey Entry (display)"
#elif BLE_SC_METHOD == BLE_SC_PASSKEY_INPUT
#define BLE_SEC_EXCHG               BTAPP_SECEXCHG_KEYBOARD
#define BLE_SEC_TYPE                BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS               BT_SMP_IOCAPS_KEYBOARD_ONLY
#define BLE_SC_AUTHREQ              (BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME                 "Passkey Entry (keyboard)"
#elif BLE_SC_METHOD == BLE_SC_OOB
#define BLE_SEC_EXCHG               BTAPP_SECEXCHG_OOB
#define BLE_SEC_TYPE                BTGAP_SECTYPE_LESC_MITM
#define BLE_SC_IOCAPS               BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#define BLE_SC_AUTHREQ              (BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM)
#define BLE_SC_NAME                 "LESC OOB"
#else
#define BLE_SEC_EXCHG               BTAPP_SECEXCHG_NONE
#define BLE_SEC_TYPE                BTGAP_SECTYPE_NONE
#define BLE_SC_IOCAPS               BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT
#define BLE_SC_AUTHREQ              0
#define BLE_SC_NAME                 "NONE (open link)"
#endif

#define DEVICE_NAME                 "UsbBleDemo"
#define PACKET_SIZE                 20

#define MANUFACTURER_NAME           "I-SYST inc."
#define MODEL_NAME                  "Generic"

#define APP_ADV_INTERVAL            64
#define APP_ADV_TIMEOUT             0
#define MIN_CONN_INTERVAL           10
#define MAX_CONN_INTERVAL           40

#define BLE_UART_UUID_BASE          BLUEIO_UUID_BASE
#define BLE_UART_UUID_SERVICE       BLUEIO_UUID_UART_SERVICE
#define BLE_UART_UUID_TX_CHAR       BLUEIO_UUID_UART_TX_CHAR
#define BLE_UART_UUID_RX_CHAR       BLUEIO_UUID_UART_RX_CHAR

#define USB_DEVNO                   0
#define USB_PKT_SIZE                USB_CTRLR_PKT_LEN_MAX(USB_DEVNO, BULK)

#define CDC_RXFIFO_PKTCNT           4
#define CDC_RXFIFO_MEMSIZE          USB_INTRF_RXMEM_SIZE(CDC_RXFIFO_PKTCNT, USB_PKT_SIZE)
#define CDC_TXFIFO_MEMSIZE          CFIFO_MEMSIZE(1024)

alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(16)];

static int CdcEvtHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId,
						 uint8_t *pBuffer, int Len);
static void UsbRxEvt(uint32_t Evt, void *pCtx);
static void SysLogFlushEvt(uint32_t Evt, void *pCtx);

alignas(4) static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE];
alignas(4) static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE];

static const UsbdCdcCfg_t s_CdcCfg = {
	.DevNo = USB_DEVNO,
	.bBlocking = true,
	.RxFifoMemSize = CDC_RXFIFO_MEMSIZE,
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = CDC_TXFIFO_MEMSIZE,
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = CdcEvtHandler,
};

static const UsbCfg_t s_UsbCfg = {
	.DevNo = USB_DEVNO,
	.Mode = USB_MODE_DEVICE,
	.Vid = 0x1209,
	.Pid = 0x000A,
	.DevVer = 0x0100,
	.pManufacturer = "I-SYST",
	.pProduct = "IOsonata USB BLE Demo",
	.pSerial = nullptr,
	.pFuncName = "IOsonata CDC",
	.IntPrio = 6,
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

alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];
static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = false
};

static volatile bool s_bUsbRxPending = false;
static volatile bool s_bSysLogFlushPending = false;

static const IOPinCfg_t s_LedPins[] = LED_PINS;
static const int s_NbLedPins = sizeof(s_LedPins) / sizeof(IOPinCfg_t);

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif

static void UsbTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len);

static const BtUuidArr_t s_AdvUuid = {
	.BaseIdx = 1,
	.Type = BT_UUID_TYPE_16,
	.Count = 1,
	.Uuid16 = { BLE_UART_UUID_SERVICE, }
};

static const char s_RxCharDescString[] = "USB Rx characteristic";
static const char s_TxCharDescString[] = "USB Tx characteristic";

BtGattChar_t g_UsbBleChars[] = {
	BT_CHAR(BLE_UART_UUID_RX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
			s_RxCharDescString),
	BT_CHAR(BLE_UART_UUID_TX_CHAR, PACKET_SIZE,
			BT_GATT_CHAR_PROP_WRITE_WORESP,
			s_TxCharDescString,
			.WrCB = UsbTxSrvcCallback),
};

static uint8_t s_LongWrBuffer[512];

BtGattSrvc_t g_UsbBleSrvc = BT_SRVC_CUSTOM(BLE_UART_UUID_BASE,
										   BLE_UART_UUID_SERVICE,
										   g_UsbBleChars);

static const BtAppDevInfo_t s_UsbBleDevDesc = {
	MODEL_NAME,
	MANUFACTURER_NAME,
	"123",
	"0.0",
	"0.0",
};

static uint8_t s_ManData[8];

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.PeriphDevMax = 0,
	.CentralDevMax = 2,
	.pDevName = DEVICE_NAME,
	.VendorId = ISYST_BLUETOOTH_ID,
	.ProductId = 1,
	.ProductVer = 0,
	.Appearance = 0,
	.pDevInfo = &s_UsbBleDevDesc,
	.pAdvManData = s_ManData,
	.AdvManDataLen = sizeof(s_ManData),
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLE_SEC_TYPE,
	.SecExchg = BLE_SEC_EXCHG,
	.bCompleteUuidList = false,
	.pAdvUuid = &s_AdvUuid,
	.AdvInterval = APP_ADV_INTERVAL,
	.AdvTimeout = APP_ADV_TIMEOUT,
	.AdvSlowInterval = 0,
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = CONNECT_LED_PORT,
	.ConnLedPin = CONNECT_LED_PIN,
	.ConnLedActLevel = CONNECT_LED_LOGIC,
	.TxPower = 0,
	.pLongWrPoolMem = s_LongWrBuffer,
	.LongWrPoolMemSize = sizeof(s_LongWrBuffer),
};

#if BLE_SC_METHOD != BLE_SC_NONE
typedef struct {
	const char *pName;
	uint8_t IoCaps;
	uint8_t AuthReq;
	const char *pDesc;
} UsbBleSecMethod_t;

static const UsbBleSecMethod_t s_SecMethods[] = {
	{ "justworks", BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING,
	  "Just Works (bonded, no MITM)" },
	{ "numcomp", BT_SMP_IOCAPS_DISPLAY_YESNO,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "Numeric Comparison" },
	{ "passkey-disp", BT_SMP_IOCAPS_DISPLAY_ONLY,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "Passkey Entry (display)" },
	{ "passkey-input", BT_SMP_IOCAPS_KEYBOARD_ONLY,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "Passkey Entry (keyboard)" },
	{ "oob", BT_SMP_IOCAPS_NO_INPUT_NO_OUTPUT,
	  BT_SMP_AUTHREQ_BONDING_FLAG_BONDING | BT_SMP_AUTHREQ_MITM,
	  "LESC OOB" },
};

#define USB_BLE_SEC_METHOD_CNT ((int)(sizeof(s_SecMethods) / sizeof(s_SecMethods[0])))

enum {
	PAIR_INPUT_NONE = 0,
	PAIR_INPUT_NUMERIC,
	PAIR_INPUT_PASSKEY
};

static int s_SecMethodIdx = 0;
static volatile int s_PairInput = PAIR_INPUT_NONE;
static uint16_t s_PairConnHdl = 0;
static uint8_t s_PairDigits = 0;
static uint32_t s_PairPasskey = 0;
static volatile bool s_OobRefreshPending = false;
static bool s_UsbBlePeerOobValid = false;
#endif

static uint8_t s_UsbRxBuff[PACKET_SIZE];
static int s_UsbRxBuffLen = 0;

static void ConsolePrintf(const char *pFormat, ...)
{
	va_list args;
	va_start(args, pFormat);
	(void)SysLogVPrintf(SysLogGet(), pFormat, args);
	va_end(args);
}

static void UsbRxQue(void)
{
	uint32_t state = DisableInterrupt();
	if (!s_bUsbRxPending)
	{
		s_bUsbRxPending = true;
		(void)AppEvtHandlerQue(0, nullptr, UsbRxEvt);
	}
	EnableInterrupt(state);
}

static void SysLogFlushQue(void)
{
	uint32_t state = DisableInterrupt();
	if (!s_bSysLogFlushPending)
	{
		s_bSysLogFlushPending = true;
		(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
	}
	EnableInterrupt(state);
}

static void UsbTxSrvcCallback(BtGattChar_t *pChar, uint8_t *pData, int Offset, int Len)
{
	(void)pChar;
	(void)Offset;
	(void)g_Cdc.Tx(0, pData, Len);
}

#if BLE_SC_METHOD != BLE_SC_NONE
static bool UsbBleSecIsOob(void)
{
	return strcmp(s_SecMethods[s_SecMethodIdx].pName, "oob") == 0;
}

static void UsbBleSecPrint(void)
{
	const UsbBleSecMethod_t *m = &s_SecMethods[s_SecMethodIdx];
	ConsolePrintf("SEC method=%s iocaps=%d authreq=0x%02x desc=%s\r\n",
				  m->pName, m->IoCaps, m->AuthReq, m->pDesc);
}

static void UsbBleSecApply(int Idx)
{
	if (Idx < 0 || Idx >= USB_BLE_SEC_METHOD_CNT)
	{
		return;
	}
	s_SecMethodIdx = Idx;
	BtSmpAuthConfig(s_SecMethods[Idx].IoCaps, s_SecMethods[Idx].AuthReq);
	UsbBleSecPrint();
}

static void UsbBleSecInit(void)
{
	for (int i = 0; i < USB_BLE_SEC_METHOD_CNT; i++)
	{
		if (s_SecMethods[i].IoCaps == BLE_SC_IOCAPS &&
			s_SecMethods[i].AuthReq == BLE_SC_AUTHREQ)
		{
#if BLE_SC_METHOD == BLE_SC_OOB
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

static int UsbBleHexVal(uint8_t c)
{
	if (c >= '0' && c <= '9') return c - '0';
	if (c >= 'a' && c <= 'f') return c - 'a' + 10;
	if (c >= 'A' && c <= 'F') return c - 'A' + 10;
	return -1;
}

static int UsbBleHexDecode(const uint8_t *pText, int Len, uint8_t *pOut, int MaxOut)
{
	int high = -1;
	int out = 0;

	for (int i = 0; i < Len; i++)
	{
		int v = UsbBleHexVal(pText[i]);
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
	return high < 0 ? out : -1;
}

static void UsbBlePrintHex(const uint8_t *pData, int Len)
{
	for (int i = 0; i < Len; i++)
	{
		ConsolePrintf("%02X", pData[i]);
	}
}

static void UsbBleOobPrintLocal(void)
{
	uint8_t r[16];
	uint8_t c[16];

	if (BtSmpOobLocalDataGen(g_BtAppData.AppDevice.pHciDev, r, c) != 0)
	{
		ConsolePrintf("OOB local data generation failed\r\n");
		return;
	}

	ConsolePrintf("OOB local data. Paste this line on peer:\r\noob peer ");
	UsbBlePrintHex(r, sizeof(r));
	UsbBlePrintHex(c, sizeof(c));
	ConsolePrintf("\r\n");
}

static bool UsbBleOobSetPeer(const uint8_t *pText, int Len)
{
	uint8_t raw[1 + 6 + 16 + 16];
	int cnt = UsbBleHexDecode(pText, Len, raw, sizeof(raw));

	if (cnt == 32)
	{
		BtSmpOobPeerDataSet(&raw[0], &raw[16]);
		s_UsbBlePeerOobValid = true;
		ConsolePrintf("OOB peer data loaded\r\n");
		return true;
	}
	if (cnt == 39)
	{
		BtSmpOobPeerDataSet(&raw[7], &raw[23]);
		s_UsbBlePeerOobValid = true;
		ConsolePrintf("OOB peer data loaded\r\n");
		return true;
	}

	ConsolePrintf("OOB peer format: oob peer <r+c hex> or <addrtype+addr+r+c hex>\r\n");
	return false;
}

static void UsbBleOobInit(void)
{
	UsbBleSecPrint();
	if (UsbBleSecIsOob())
	{
		UsbBleOobPrintLocal();
	}
	ConsolePrintf("Commands: sec [method], oob, oob peer <hex>, bond del\r\n");
}

static bool UsbBleSecTryCommand(const uint8_t *pData, int Len)
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
		UsbBleSecPrint();
		return true;
	}

	while (l > 0 && (p[l - 1] == '\r' || p[l - 1] == '\n' ||
					 p[l - 1] == ' ' || p[l - 1] == '\t'))
	{
		l--;
	}

	for (int i = 0; i < USB_BLE_SEC_METHOD_CNT; i++)
	{
		int nl = (int)strlen(s_SecMethods[i].pName);
		if (nl == l && memcmp(p, s_SecMethods[i].pName, nl) == 0)
		{
			UsbBleSecApply(i);
			if (UsbBleSecIsOob())
			{
				UsbBleOobPrintLocal();
			}
			return true;
		}
	}

	ConsolePrintf("SEC unknown, one of:");
	for (int i = 0; i < USB_BLE_SEC_METHOD_CNT; i++)
	{
		ConsolePrintf(" %s", s_SecMethods[i].pName);
	}
	ConsolePrintf("\r\n");
	return true;
}

static bool UsbBleOobTryCommand(const uint8_t *pData, int Len)
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
		UsbBleOobPrintLocal();
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
		(void)UsbBleOobSetPeer(p, l);
		return true;
	}

	ConsolePrintf("Commands: oob, oob peer <hex>\r\n");
	return true;
}

static bool UsbBleBondTryCommand(const uint8_t *pData, int Len)
{
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

	ConsolePrintf("bond deletion requested\r\n");
	BtSmpBondClearAll();
	return true;
}

void BtAppEvtSecured(uint16_t ConnHdl)
{
	(void)ConnHdl;
	if (UsbBleSecIsOob())
	{
		s_OobRefreshPending = true;
		UsbRxQue();
	}
}

void BtSmpNumericComparison(uint16_t ConnHdl, uint32_t Value)
{
	ConsolePrintf("\r\nSMP numeric comparison: %06u\r\n", (unsigned)Value);
	ConsolePrintf("Do both devices show this value? type y or n\r\n");
	s_PairConnHdl = ConnHdl;
	s_PairInput = PAIR_INPUT_NUMERIC;
}

void BtSmpPasskeyDisplay(uint16_t ConnHdl, uint32_t Passkey)
{
	(void)ConnHdl;
	ConsolePrintf("\r\nSMP passkey (enter this on the peer): %06u\r\n",
				  (unsigned)Passkey);
}

void BtSmpPasskeyRequest(uint16_t ConnHdl)
{
	ConsolePrintf("\r\nSMP passkey entry: type the 6 digits shown on the peer\r\n");
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
	while (g_Cdc.Rx(0, &c, 1) == 1)
	{
		if (s_PairInput == PAIR_INPUT_NUMERIC)
		{
			if (c == 'y' || c == 'Y')
			{
				s_PairInput = PAIR_INPUT_NONE;
				ConsolePrintf("match\r\n");
				BtSmpNumericComparisonReply(s_PairConnHdl, true);
				return true;
			}
			if (c == 'n' || c == 'N')
			{
				s_PairInput = PAIR_INPUT_NONE;
				ConsolePrintf("no match\r\n");
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
				(void)g_Cdc.Tx(0, &c, 1);
				if (s_PairDigits == 6)
				{
					s_PairInput = PAIR_INPUT_NONE;
					ConsolePrintf("\r\n");
					BtSmpPasskeyReply(s_PairConnHdl, s_PairPasskey);
					return true;
				}
			}
			else if (c == 0x1b)
			{
				s_PairInput = PAIR_INPUT_NONE;
				ConsolePrintf("\r\ncancelled\r\n");
				BtSmpPasskeyReply(s_PairConnHdl, BT_SMP_PASSKEY_INVALID);
				return true;
			}
		}
	}
	return true;
}

#else

static void UsbBleOobInit(void) {}
static bool UsbBleSecTryCommand(const uint8_t *, int) { return false; }
static bool UsbBleOobTryCommand(const uint8_t *, int) { return false; }
static bool UsbBleBondTryCommand(const uint8_t *, int) { return false; }

#endif

void BtAppInitUserServices(void)
{
	(void)BtGattSrvcAdd(&g_UsbBleSrvc);
}

void BtAppInitUserData(void)
{
#if BLE_SC_METHOD != BLE_SC_NONE
	BtAppSecInit();
	UsbBleSecInit();
#endif
}

void BtAppEvtConnected(uint16_t ConnHdl)
{
	ConsolePrintf("CONNECTED hdl=%d\r\n", ConnHdl);
}

void BtAppPeriphEvtHandler(uint32_t Evt, void * const pCtx)
{
	(void)Evt;
	(void)pCtx;
}

static void UsbRxEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bUsbRxPending = false;

#if BLE_SC_METHOD != BLE_SC_NONE
	if (s_OobRefreshPending)
	{
		s_OobRefreshPending = false;
		BtSmpOobDataClear();
		s_UsbBlePeerOobValid = false;
		UsbBleOobPrintLocal();
	}

	if (PairInputPoll())
	{
		return;
	}
#endif

	bool flush = false;

	int l = g_Cdc.Rx(0, &s_UsbRxBuff[s_UsbRxBuffLen], PACKET_SIZE - s_UsbRxBuffLen);
	if (l > 0)
	{
		int start = s_UsbRxBuffLen;
		s_UsbRxBuffLen += l;
		if (s_UsbRxBuffLen >= PACKET_SIZE)
		{
			flush = true;
		}

		for (int i = start; i < s_UsbRxBuffLen; i++)
		{
			if (s_UsbRxBuff[i] == '\r' || s_UsbRxBuff[i] == '\n')
			{
				flush = true;
				break;
			}
		}
	}
	else if (s_UsbRxBuffLen > 0)
	{
		flush = true;
	}

	if (flush)
	{
		if (UsbBleSecTryCommand(s_UsbRxBuff, s_UsbRxBuffLen) ||
			UsbBleOobTryCommand(s_UsbRxBuff, s_UsbRxBuffLen) ||
			UsbBleBondTryCommand(s_UsbRxBuff, s_UsbRxBuffLen))
		{
			s_UsbRxBuffLen = 0;
			UsbRxQue();
			return;
		}

		if (BtAppNotify(&g_UsbBleChars[0], s_UsbRxBuff, (uint16_t)s_UsbRxBuffLen))
		{
			s_UsbRxBuffLen = 0;
		}
		UsbRxQue();
	}
	else if (s_UsbRxBuffLen > 0)
	{
		UsbRxQue();
	}
}

static void SysLogFlushEvt(uint32_t Evt, void *pCtx)
{
	(void)Evt;
	(void)pCtx;
	s_bSysLogFlushPending = false;
	(void)SysLogFlush(SysLogGet());
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
			UsbRxQue();
			break;

		case DEVINTRF_EVT_TX_FIFO_EMPTY:
			if (CFifoPeek(SysLogGet()->hFifo) != nullptr)
			{
				SysLogFlushQue();
			}
			break;

		case DEVINTRF_EVT_STATECHG:
			if (Len)
			{
				SysLogFlushQue();
				UsbRxQue();
			}
			break;

		default:
			break;
	}
	return 0;
}

static void HardwareInit(void)
{
	IOPinCfg(s_LedPins, s_NbLedPins);
	for (int i = 0; i < s_NbLedPins; i++)
	{
		IOPinSet(s_LedPins[i].PortNo, s_LedPins[i].PinNo);
	}
}

int main()
{
	AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
	HardwareInit();

	if (!UsbInit(&s_UsbCfg) || !g_Cdc.Init(s_CdcCfg))
	{
		while (true)
		{
			__NOP();
		}
	}

	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)g_Cdc, 0, nullptr, 0);

	ConsolePrintf("USB CDC over BLE\r\n");
	ConsolePrintf("security    : %s\r\n", BLE_SC_NAME);

	// Bring up the Bluetooth stack before enabling the USB controller. This
	// matches the working USB/BLE central example and avoids enabling USBD
	// across SoftDevice clock/interrupt initialization.
	const bool btOk = BtAppInit(&s_BleAppCfg);
	if (!btOk)
	{
		ConsolePrintf("BtAppInit failed\r\n");
	}
	else
	{
		UsbBleOobInit();
	}

	// Enable USB regardless of Bluetooth status so the CDC console still
	// enumerates and can report a Bluetooth initialization failure.
	(void)UsbEnable(USB_DEVNO);

	AppRun();

	while (true)
	{
		__NOP();
	}
}

bool AppCheckStatus(void)
{
	BtAppCheckStatus();
	UsbCheckStatus();

	uint32_t state = DisableInterrupt();
	if (!AppEvtHandlerPending())
	{
		if (s_bUsbRxPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, UsbRxEvt);
		}
		if (s_bSysLogFlushPending)
		{
			(void)AppEvtHandlerQue(0, nullptr, SysLogFlushEvt);
		}
	}

	const bool idle = !s_bUsbRxPending &&
					  !s_bSysLogFlushPending &&
					  !AppEvtHandlerPending();
	EnableInterrupt(state);
	return idle;
}
