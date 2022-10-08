/**-------------------------------------------------------------------------
@example	UartBleBridge.cpp

@brief	Uart BLE streaming demo

This application demo shows UART Rx/Tx streaming over BLE custom service
using EHAL library.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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

#include "app_util_platform.h"
#include "app_scheduler.h"

#include "istddef.h"
#include "bluetooth/ble_app.h"
#include "ble_app_nrf5.h"
#include "ble_service.h"
#include "ble_intrf.h"
#include "bluetooth/blueio_blesrvc.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"
#include "coredev/iopincfg.h"

#include "board.h"

//#define NORDIC_NUS_SERVICE

#define DEVICE_NAME                     "UARTBridge"                            /**< Name of device. Will be included in the advertising data. */

#define PACKET_SIZE						20

#define MANUFACTURER_NAME               "I-SYST inc."							/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME                      "IMM-NRF51x"                            /**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID                 ISYST_BLUETOOTH_ID						/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID                   ISYST_BLUETOOTH_ID						/**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define APP_ADV_INTERVAL                64//MSEC_TO_UNITS(64, UNIT_0_625_MS)		/**< The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms). */

#if (NRF_SD_BLE_API_VERSION < 6)
#define APP_ADV_TIMEOUT			      	180										/**< The advertising timeout (in units of seconds). */
#else
#define APP_ADV_TIMEOUT					0//MSEC_TO_UNITS(180000, UNIT_10_MS)		/**< The advertising timeout (in units of 10ms seconds). */
#endif

#define MIN_CONN_INTERVAL               10 //MSEC_TO_UNITS(10, UNIT_1_25_MS)			/**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL               40//MSEC_TO_UNITS(40, UNIT_1_25_MS)			/**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#ifdef NORDIC_NUS_SERVICE
#define BLE_UART_UUID_BASE			NUS_BASE_UUID

#define BLE_UART_UUID_SERVICE		BLE_UUID_NUS_SERVICE			/**< The UUID of the Nordic UART Service. */
#define BLE_UART_UUID_READ_CHAR		BLE_UUID_NUS_TX_CHARACTERISTIC	/**< The UUID of the TX Characteristic. */
#define BLE_UART_UUID_WRITE_CHAR	BLE_UUID_NUS_RX_CHARACTERISTIC	/**< The UUID of the RX Characteristic. */
#else
#define BLE_UART_UUID_BASE			BLUEIO_UUID_BASE

#define BLE_UART_UUID_SERVICE		BLUEIO_UUID_UART_SERVICE		//!< BlueIO default service
#define BLE_UART_UUID_READ_CHAR		BLUEIO_UUID_UART_RX_CHAR		//!< Data characteristic
#define BLE_UART_UUID_WRITE_CHAR		BLUEIO_UUID_UART_TX_CHAR		//!< Command control characteristic
#endif

int BleIntrfEvtCallback(DevIntrf_t *pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const ble_uuid_t  s_AdvUuids[] = {
	{BLE_UART_UUID_SERVICE, BLE_UUID_TYPE_VENDOR_BEGIN}
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

BLESRVC_CHAR g_UartChars[] = {
	{
		// Read characteristic
		BLE_UART_UUID_READ_CHAR,
		PACKET_SIZE,
		BLESVC_CHAR_PROP_READ | BLESVC_CHAR_PROP_NOTIFY | BLESVC_CHAR_PROP_VARLEN,
		s_RxCharDescString,         // char UTF-8 description string
		NULL,                       // Callback for write char, set to NULL for read char
		NULL,						// Callback on set notification
		NULL,						// Tx completed callback
		NULL,						// pointer to char default values
		0,							// Default value length in bytes
	},
	{
		// Write characteristic
		BLE_UART_UUID_WRITE_CHAR,	// char UUID
		PACKET_SIZE,                // char max data length
		BLESVC_CHAR_PROP_WRITE | BLESVC_CHAR_PROP_WRITEWORESP | BLESVC_CHAR_PROP_VARLEN,	// char properties define by BLUEIOSVC_CHAR_PROP_...
		s_TxCharDescString,			// char UTF-8 description string
		NULL,         				// Callback for write char, set to NULL for read char
		NULL,						// Callback on set notification
		NULL,						// Tx completed callback
		NULL,						// pointer to char default values
		0							// Default value length in bytes
	},
};

static const int s_BleUartNbChar = sizeof(g_UartChars) / sizeof(BLESRVC_CHAR);

uint8_t g_LWrBuffer[512];

const BleSrvcCfg_t s_UartSrvcCfg = {
	BLESRVC_SECTYPE_NONE,		// Secure or Open service/char
	{BLE_UART_UUID_BASE,},		// Base UUID
	1,
	BLE_UART_UUID_SERVICE,   	// Service UUID
	s_BleUartNbChar,            // Total number of characteristics for the service
	g_UartChars,                // Pointer a an array of characteristic
	g_LWrBuffer,                // pointer to user long write buffer
	sizeof(g_LWrBuffer)         // long write buffer size
};

BLESRVC g_UartBleSrvc;

const BleAppDevInfo_t s_UartBleDevDesc {
	MODEL_NAME,           	// Model name
	MANUFACTURER_NAME,      // Manufacturer name
	"",                     // Serial number string
	"0.0",                  // Firmware version string
	"0.0",                  // Hardware version string
};

const BleAppCfg_t s_BleAppCfg = {
	.Role = BLEAPP_ROLE_PERIPHERAL,
	.CentLinkCount = 0, 				// Number of central link
	.PeriLinkCount = 1, 				// Number of peripheral link
	.pDevName = DEVICE_NAME,			// Device name
	.VendorID = ISYST_BLUETOOTH_ID,		// PnP Bluetooth/USB vendor id
	.ProductId = 1,						// PnP Product ID
	.ProductVer = 0,					// Pnp prod version
	.pDevDesc = &s_UartBleDevDesc,
	.bExtAdv = false,
	.pAdvManData = g_ManData,			// Manufacture specific data to advertise
	.AdvManDataLen = sizeof(g_ManData),	// Length of manufacture specific data
	.pSrManData = NULL,
	.SrManDataLen = 0,
	.SecType = BLEAPP_SECTYPE_NONE,    // Secure connection type
	.SecExchg = BLEAPP_SECEXCHG_NONE,	// Security key exchange
	.pAdvUuids = NULL,      			// Service uuids to advertise
	.NbAdvUuid = 0, 					// Total number of uuids
	.AdvInterval = APP_ADV_INTERVAL,	// Advertising interval in msec
	.AdvTimeout = APP_ADV_TIMEOUT,		// Advertising timeout in sec
	.AdvSlowInterval = 0,				// Slow advertising interval, if > 0, fallback to
										// slow interval on adv timeout and advertise until connected
	.ConnIntervalMin = MIN_CONN_INTERVAL,
	.ConnIntervalMax = MAX_CONN_INTERVAL,
	.ConnLedPort = BLUEIO_CONNECT_LED_PORT,// Led port nuber
	.ConnLedPin = BLUEIO_CONNECT_LED_PIN,// Led pin number
	.TxPower = 0,						// Tx power
	.SDEvtHandler = NULL				// RTOS Softdevice handler
};

#define BLEINTRF_FIFOSIZE			CFIFO_TOTAL_MEMSIZE(10, 22)

alignas(4) static uint8_t s_BleIntrfRxFifo[BLEINTRF_FIFOSIZE];
alignas(4) static uint8_t s_BleIntrfTxFifo[BLEINTRF_FIFOSIZE];

static const BleIntrfCfg_t s_BleInrfCfg = {
	.pBleSrv = &g_UartBleSrvc,
	.RxCharIdx = BLESRV_WRITE_CHAR_IDX,
	.TxCharIdx = BLESRV_READ_CHAR_IDX,
	.PacketSize = 20,			// Packet size : use default
	.bBlocking = false,
	.RxFifoMemSize = BLEINTRF_FIFOSIZE,			// Rx Fifo mem size
	.pRxFifoMem = s_BleIntrfRxFifo,		// Rx Fifo mem pointer
	.TxFifoMemSize = BLEINTRF_FIFOSIZE,			// Tx Fifo mem size
	.pTxFifoMem = s_BleIntrfTxFifo,		// Tx Fifo mem pointer
	.EvtCB = BleIntrfEvtCallback
};

BleIntrf g_BleIntrf;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// UART configuration data

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
	.IntPrio = APP_IRQ_PRIORITY_LOW,	// Interrupt priority
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

		int l = g_BleIntrf.Rx(0, buff, 128);
		if (l > 0)
		{
			g_Uart.Tx(buff, l);
		}
		cnt += l;
	}

	return cnt;
}

void BlePeriphEvtUserHandler(ble_evt_t * p_ble_evt)
{
    BleSrvcEvtHandler(&g_UartBleSrvc, p_ble_evt);
}

void BleAppInitUserServices()
{
    uint32_t       err_code;

    err_code = BleSrvcInit(&g_UartBleSrvc, &s_UartSrvcCfg);
    APP_ERROR_CHECK(err_code);
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);
}

void BleAppInitUserData()
{

}

void UartRxChedHandler(void * p_event_data, uint16_t event_size)
{
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
		g_BleIntrf.Tx(0, buff, bufflen);
		bufflen = 0;
		app_sched_event_put(NULL, 0, UartRxChedHandler);
	}
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			app_sched_event_put(NULL, 0, UartRxChedHandler);
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
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
    HardwareInit();

    BleAppInit((const BleAppCfg_t *)&s_BleAppCfg);//, true);

    g_BleIntrf.Init(s_BleInrfCfg);

    BleAppRun();

	return 0;
}
