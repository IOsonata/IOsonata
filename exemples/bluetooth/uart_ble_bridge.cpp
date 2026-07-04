/**-------------------------------------------------------------------------
@example	uart_ble_bridge.cpp

@brief	Uart BLE streaming demo

This application demo shows UART Rx/Tx streaming over BLE custom service
using EHAL library.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017

@license

MIT License

Copyright (c) 2017, I-SYST inc., all rights reserved

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

#include "istddef.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/bt_smp.h"
#include "bluetooth/blueio_blesrvc.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"
#include "app_evt_handler.h"
#include "prbs.h"

#include "board.h"

//#define NORDIC_NUS_SERVICE

#define DEVICE_NAME                     "UARTBridge"                            /**< Name of device. Will be included in the advertising data. */

#define BLE_SC_NONE				0
#define BLE_SC_OOB				5

#ifndef BLE_SC_METHOD
#define BLE_SC_METHOD			BLE_SC_OOB
#endif

#if BLE_SC_METHOD == BLE_SC_OOB
#define BLE_SEC_TYPE			BTGAP_SECTYPE_LESC_MITM
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_OOB
#define BLE_SC_NAME				"LESC OOB"

#else
#define BLE_SEC_TYPE			BTGAP_SECTYPE_NONE
#define BLE_SEC_EXCHG			BTAPP_SECEXCHG_NONE
#define BLE_SC_NAME				"NONE (open link)"
#endif

#define PACKET_SIZE						244

#define MANUFACTURER_NAME               "I-SYST inc."							/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME                      "IMM-NRF51x"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID						/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID						/**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                64	// in msec

#define APP_ADV_TIMEOUT					0	// in msec

#define MIN_CONN_INTERVAL               10 	// in msec
#define MAX_CONN_INTERVAL               40	// in msec

#ifdef NORDIC_NUS_SERVICE
#define BLE_UART_UUID_BASE			NUS_BASE_UUID

#define BLE_UART_UUID_SERVICE		BLE_UUID_NUS_SERVICE			/**< The UUID of the Nordic UART Service. */
#define BLE_UART_UUID_READ_CHAR		BLE_UUID_NUS_TX_CHARACTERISTIC	/**< The UUID of the TX Characteristic. */
#define BLE_UART_UUID_WRITE_CHAR	BLE_UUID_NUS_RX_CHARACTERISTIC	/**< The UUID of the RX Characteristic. */
#else
#define BLE_UART_UUID_BASE			BLUEIO_UUID_BASE

#define BLE_UART_UUID_SERVICE		BLUEIO_UUID_UART_SERVICE		//!< BlueIO default service
#define BLE_UART_UUID_READ_CHAR		BLUEIO_UUID_UART_RX_CHAR		//!< Data characteristic
#define BLE_UART_UUID_WRITE_CHAR	BLUEIO_UUID_UART_TX_CHAR		//!< Command control characteristic
#endif

int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

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

#define BLESRV_READ_CHAR_IDX		0
#define BLESRV_WRITE_CHAR_IDX		1

BtGattChar_t g_UartChars[] = {
	// Read + Notify (server-pushed)
	BT_CHAR(BLE_UART_UUID_READ_CHAR, PACKET_SIZE,
	        BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY,
	        s_RxCharDescString),
	// Write + Write Without Response (peer sink; BtIntrf handles writes)
	BT_CHAR(BLE_UART_UUID_WRITE_CHAR, PACKET_SIZE,
	        BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP,
	        s_TxCharDescString),
};

uint8_t g_LWrBuffer[512];

BtGattSrvc_t g_UartBleSrvc = BT_SRVC_CUSTOM(BLE_UART_UUID_BASE,
                                            BLE_UART_UUID_SERVICE,
                                            g_UartChars);

const BtAppDevInfo_t s_UartBleDevDesc {
	MODEL_NAME,           	// Model name
	MANUFACTURER_NAME,      // Manufacturer name
	"",                     // Serial number string
	"0.0",                  // Firmware version string
	"0.0",                  // Hardware version string
};

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.pDevInfo = &s_UartBleDevDesc,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLE_SEC_TYPE,    		// Secure connection type
	.SecExchg = BLE_SEC_EXCHG,		// Security key exchange
	.pAdvUuid = NULL,      				// Service uuids to advertise
	//.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,// Led pin number
	.TxPower = 0,						// Tx power
	.MaxMtu = 244,
	.pLongWrPoolMem = g_LWrBuffer,		// Long-write reassembly pool (split across peer slots)
	.LongWrPoolMemSize = sizeof(g_LWrBuffer),
};

#define BLEINTRF_FIFOSIZE			BTINTRF_CFIFO_TOTAL_MEMSIZE(10, PACKET_SIZE)

alignas(4) static uint8_t s_BleIntrfRxFifo[BLEINTRF_FIFOSIZE];
alignas(4) static uint8_t s_BleIntrfTxFifo[BLEINTRF_FIFOSIZE];

static const BtIntrfCfg_t s_BleInrfCfg = {
	.pSrvc = &g_UartBleSrvc,
	.RxCharIdx = BLESRV_WRITE_CHAR_IDX,
	.TxCharIdx = BLESRV_READ_CHAR_IDX,
	.PacketSize = PACKET_SIZE,			// Packet size : use default
	.bBlocking = false,
	.RxFifoMemSize = BLEINTRF_FIFOSIZE,			// Rx Fifo mem size
	.pRxFifoMem = s_BleIntrfRxFifo,		// Rx Fifo mem pointer
	.TxFifoMemSize = BLEINTRF_FIFOSIZE,			// Tx Fifo mem size
	.pTxFifoMem = s_BleIntrfTxFifo,		// Tx Fifo mem pointer
	.EvtCB = BleIntrfEvtCallback
};

BtIntrf g_BtIntrf;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

/// UART pin configuration

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];


static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},		// TX
	//{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	//{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
};

/// UART configuration
const UARTCfg_t g_UartCfg = {
	.DevNo = 0,							// Device number zero based
	.pIOPinMap = s_UartPins,				// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),	// Total number of UART pins used
	.Rate = 115200,						// Baudrate
	.DataBits = 8,						// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,						// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
	.bIntMode = true,					// Interrupt mode
	.IntPrio = 6,//APP_IRQ_PRIORITY_LOW,	// Interrupt priority
	.EvtCallback = nRFUartEvthandler,	// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = true,
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


int g_DelayCnt = 0;

int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	if (EvtId == DEVINTRF_EVT_RX_DATA)
	{
		uint8_t buff[128];

		int l = g_BtIntrf.Rx(0, buff, 128);
		if (l > 0)
		{
			g_Uart.Tx(buff, l);
		}
		cnt += l;
	}

	return cnt;
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	//BtGattEvtHandler(Evt, pCtx);
}

void BtAppInitUserServices()
{
    bool res;
    res = BtGattSrvcAdd(&g_UartBleSrvc);
}

void BtAppInitUserData()
{

}

static volatile bool s_bStreamming = false;

//void UartRxChedHandler(void * p_event_data, uint16_t event_size)
static uint32_t drop = 0;
void UartRxChedHandler(uint32_t Evt, void *pCtx)
{
	static uint8_t buff[PACKET_SIZE];
	static int bufflen = 0;
	static uint8_t d = 0;
	bool flush = false;

	int l = PACKET_SIZE - bufflen;
	if (l > 0)
	{
		l = g_Uart.Rx(&buff[bufflen], l);
		bufflen += l;
		if (bufflen >= PACKET_SIZE || l == 0)
		{
			flush = true;
		}
	}
	else
	{
		flush = true;
	}

	if (flush)
	{
		if (UartBleOobTryCommand(buff, bufflen))
		{
			bufflen = 0;
			flush = false;
			return;
		}

		for (int i = 0; i < bufflen; i++)
		{
			if (d != Prbs8(buff[i]) & d != 0)
			{
				drop++;
			}
			d = Prbs8(buff[i]);
		}
//		g_Uart.printf("drop : %d\r\n", drop);
		if (isConnected())
		{
			g_BtIntrf.Tx(0, buff, bufflen);
		}
		bufflen = 0;
		flush = false;
	}
//	app_sched_event_put(NULL, 0, UartRxChedHandler);
	s_bStreamming = true;
	AppEvtHandlerQue(0, 0, UartRxChedHandler);
}
uint32_t schedcnt = 0;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:

		case UART_EVT_RXDATA:
			if (s_bStreamming == false)
			{
				schedcnt++;
				//app_sched_event_put(NULL, 0, UartRxChedHandler);
				AppEvtHandlerQue(0, 0, UartRxChedHandler);
			}
			//UartRxChedHandler(0, 0);
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

	g_Uart.printf("UartBleBridge\r\n");
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
    HardwareInit();

    BtAppInit(&s_BleAppCfg);
    g_Uart.printf("security    : %s\r\n", BLE_SC_NAME);
    UartBleOobInit();

    g_BtIntrf.Init(s_BleInrfCfg);

    BtAppRun();

	return 0;
}
