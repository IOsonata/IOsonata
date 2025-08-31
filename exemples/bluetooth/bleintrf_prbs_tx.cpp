/**-------------------------------------------------------------------------
@example	bleintrf_prbs_tx.cpp

@brief	BLE PRBS streaming demo

This application demo shows Tx streaming over BLE custom service

@author	Hoang Nguyen Hoan
@date	Dec. 4, 2023

@license

MIT License

Copyright (c) 2023, I-SYST inc., all rights reserved

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
#include "idelay.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/bt_gatt.h"
#include "bluetooth/bt_intrf.h"
#include "bluetooth/blueio_blesrvc.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "app_evt_handler.h"
#include "prbs.h"
#include "coredev/system_core_clock.h"
#include "blueio_board.h"

//#define APP_SCHED		// use Nordic app scheduler

#ifdef APP_SCHED
#include "app_scheduler.h"
#endif

#include "board.h"

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif


//#define NORDIC_NUS_SERVICE
#ifdef S132
#define DEVICE_NAME                     "BlePrbs_S132"                            /**< Name of device. Will be included in the advertising data. */
#elif defined(S140)
#define DEVICE_NAME                     "BlePrbs_S140"                            /**< Name of device. Will be included in the advertising data. */
#else
#define DEVICE_NAME                     "BlePrbs_SDC"                            /**< Name of device. Will be included in the advertising data. */
#endif

#define PACKET_SIZE						244

#define MANUFACTURER_NAME               "I-SYST inc."							/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME                      "IMM-NRF52x"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID						/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID						/**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                64	// in msec

#define APP_ADV_TIMEOUT					0	// in msec

#define MIN_CONN_INTERVAL               7.5 // in  msec
#define MAX_CONN_INTERVAL               40	// in msec

#define BLE_UART_UUID_BASE				BLUEIO_UUID_BASE

#define BLE_UART_UUID_SERVICE			BLUEIO_UUID_UART_SERVICE		//!< BlueIO default service
#define BLE_UART_UUID_READ_CHAR			BLUEIO_UUID_UART_RX_CHAR		//!< Data characteristic
#define BLE_UART_UUID_WRITE_CHAR		BLUEIO_UUID_UART_TX_CHAR		//!< Command control characteristic

int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void ReadCharSetNotif(BtGattChar_t *pChar, bool bEnable);

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

static uint8_t s_UartRxCharMem[PACKET_SIZE];
static uint8_t s_UartTxCharMem[PACKET_SIZE];

static uint8_t s_RxCharValMem[PACKET_SIZE];

static BtGattChar_t g_UartChars[] = {
	{
		// Read characteristic
		.Uuid = BLE_UART_UUID_READ_CHAR,
		.MaxDataLen = PACKET_SIZE,
		.Property = BT_GATT_CHAR_PROP_READ | BT_GATT_CHAR_PROP_NOTIFY | BT_GATT_CHAR_PROP_VALEN,
		.pDesc = s_RxCharDescString,		// char UTF-8 description string
		.WrCB = NULL,                       // Callback for write char, set to NULL for read char
		.SetNotifCB = ReadCharSetNotif,		// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
	},
	{
		// Write characteristic
		.Uuid = BLE_UART_UUID_WRITE_CHAR,	// char UUID
		.MaxDataLen = PACKET_SIZE,
		.Property = BT_GATT_CHAR_PROP_WRITE | BT_GATT_CHAR_PROP_WRITE_WORESP | BT_GATT_CHAR_PROP_VALEN,	// char properties define by BLUEIOSVC_CHAR_PROP_...
		.pDesc = s_TxCharDescString,		// char UTF-8 description string
		.WrCB = NULL,                       // Callback for write char, set to NULL for read char
		.SetNotifCB = NULL,					// Callback on set notification
		.TxCompleteCB = NULL,				// Tx completed callback
	},
};

static const int s_BleUartNbChar = sizeof(g_UartChars) / sizeof(BtGattChar_t);

uint8_t g_LWrBuffer[512];

static const BtGattSrvcCfg_t s_UartSrvcCfg = {
	.SecType = BT_GAP_SECTYPE_NONE,		// Secure or Open service/char
	.bCustom = true,
	.UuidBase = BLE_UART_UUID_BASE,		// Base UUID
	//1,
	.UuidSrvc = BLE_UART_UUID_SERVICE,	// Service UUID
	.NbChar = s_BleUartNbChar,			// Total number of characteristics for the service
	.pCharArray = g_UartChars,			// Pointer a an array of characteristic
	.pLongWrBuff = g_LWrBuffer,			// pointer to user long write buffer
	.LongWrBuffSize = sizeof(g_LWrBuffer)	// long write buffer size
};

BtGattSrvc_t g_UartBleSrvc;

static const BtAppDevInfo_t s_UartBleDevDesc {
	MODEL_NAME,           	// Model name
	MANUFACTURER_NAME,      // Manufacturer name
	"",                     // Serial number string
	"0.0",                  // Firmware version string
	"0.0",                  // Hardware version string
};

static const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
	.pDevName = DEVICE_NAME,			// Device name
	.VendorId = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.pDevInfo = &s_UartBleDevDesc,
	.bExtAdv = false,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BTGAP_SECTYPE_NONE,    	// Secure connection type
	.SecExchg = BTAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuid = NULL,      				// Service uuids to advertise
	//.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,	// Led pin number
	.TxPower = 0,							// Tx power
	.SDEvtHandler = NULL,					// RTOS Softdevice handler
	.MaxMtu = 247,
};

#define BLEINTRF_FIFOSIZE			BTINTRF_CFIFO_TOTAL_MEMSIZE(10, PACKET_SIZE)

alignas(4) static uint8_t s_BleIntrfRxFifo[BLEINTRF_FIFOSIZE];
alignas(4) static uint8_t s_BleIntrfTxFifo[BLEINTRF_FIFOSIZE];

static const BtIntrfCfg_t s_BleInrfCfg = {
	.pSrvc = &g_UartBleSrvc,
	.RxCharIdx = BLESRV_WRITE_CHAR_IDX,
	.TxCharIdx = BLESRV_READ_CHAR_IDX,
	.PacketSize = PACKET_SIZE,			// Packet size : use default
	.bBlocking = true,
	.RxFifoMemSize = BLEINTRF_FIFOSIZE,	// Rx Fifo mem size
	.pRxFifoMem = s_BleIntrfRxFifo,		// Rx Fifo mem pointer
	.TxFifoMemSize = BLEINTRF_FIFOSIZE,	// Tx Fifo mem size
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
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RTS
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
	.IntPrio = 6,	// Interrupt priority
	.EvtCallback = nRFUartEvthandler,	// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
};

/// UART object instance
UART g_Uart;

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

#ifdef APP_SCHED
void PrbsChedHandler(void * p_event_data, uint16_t event_size)
#else
void PrbsChedHandler(uint32_t Evt, void *pCtx)
#endif
{
	static uint8_t buff[PACKET_SIZE];
	static int bufflen = 0;
	static uint8_t d = 0xff;

	if (bufflen == 0)
	{
		for (;bufflen < PACKET_SIZE; bufflen++)
		{
			d = Prbs8(d);
			buff[bufflen] = d;
		}
	}

	if (isConnected())
	{
//		g_Uart.Tx(buff, bufflen);
		if (g_BtIntrf.Tx(0, buff, bufflen) > 0)
		{
			bufflen = 0;
		}
#ifdef APP_SCHED
		app_sched_event_put(NULL, 0, PrbsChedHandler);
#else
		AppEvtHandlerQue(0, 0, PrbsChedHandler);
#endif
	}
}

void ReadCharSetNotif(BtGattChar_t *pChar, bool bEnable)
{
	if (bEnable)
	{
#ifdef APP_SCHED
		app_sched_event_put(NULL, 0, PrbsChedHandler);
#else
		AppEvtHandlerQue(0, 0, PrbsChedHandler);
#endif
	}
}

void BtAppPeriphEvtHandler(uint32_t Evt, void *pCtx)
{
	//BtGattEvtHandler(Evt, pCtx);
}

void BtAppInitUserServices()
{
    bool res;
    res = BtGattSrvcAdd(&g_UartBleSrvc, &s_UartSrvcCfg);
}

void BtAppInitUserData()
{

}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[PACKET_SIZE];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
//			app_sched_event_put(NULL, 0, UartRxChedHandler);
//			AppEvtHandlerQue(0, 0, UartRxChedHandler);
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

#ifdef S132
	g_Uart.printf("BleIntrfPrbsTx_SoftDevice132\r\n");
#elif defined(S140)
	g_Uart.printf("BleIntrfPrbsTx_SoftDevice140\r\n");
#else
	g_Uart.printf("BleIntrfPrbsTx_SDC\r\n");
#endif
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

    g_BtIntrf.Init(s_BleInrfCfg);

    //AppEvtHandlerQue(0, 0, PrbsChedHandler);
    BtAppRun();

	return 0;
}
