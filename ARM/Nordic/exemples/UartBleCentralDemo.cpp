/**-------------------------------------------------------------------------
@example	UartBleCentralDemo.cpp


@brief	Uart BLE Central demo

This application demo shows UART Rx/Tx over BLE central using EHAL library.

@author	Hoang Nguyen Hoan
@date	Feb. 4, 2017
@author Thinh Tran
@date 	May 9, 2022

@license

Copyright (c) 2017-2022, I-SYST inc., all rights reserved

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
#include "ble_gap.h"
#include "ble_advdata.h"
#include "nrf_ble_scan.h"

#include "istddef.h"
#include "bluetooth/ble_app.h"
#include "ble_app_nrf5.h"
#include "ble_service.h"
#include "bluetooth/blueio_blesrvc.h"
#include "ble_dev.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"//Does this make the LED_2 & LED_3 do not work?
#include "coredev/iopincfg.h"
#include "stddev.h"
#include "board.h"
#include "idelay.h"
#include "ble_gattc.h"
#include "cfifo.h"
#include "iopinctrl.h"

//#define DEBUG_PRINT

// BLE
#define DEVICE_NAME             "UARTCentral"                   		/**< Name of device. Will be included in the advertising data. */

#define MANUFACTURER_NAME       "I-SYST inc."                   		/**< Manufacturer. Will be passed to Device Information Service. */
#define MODEL_NAME              "IMM-NRF5x"                     		/**< Model number. Will be passed to Device Information Service. */
#define MANUFACTURER_ID         ISYST_BLUETOOTH_ID              		/**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
#define ORG_UNIQUE_ID           ISYST_BLUETOOTH_ID              		/**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */

#define MIN_CONN_INTERVAL       10//MSEC_TO_UNITS(10, UNIT_1_25_MS) 		/**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
#define MAX_CONN_INTERVAL       40//MSEC_TO_UNITS(40, UNIT_1_25_MS) 		/**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */

#define SCAN_INTERVAL           MSEC_TO_UNITS(1000, UNIT_0_625_MS)      /**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             MSEC_TO_UNITS(100, UNIT_0_625_MS)       /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0                                 		/**< Timout when scanning. 0x0000 disables timeout. */

#define TARGET_BRIDGE_DEV_NAME	"BlueIO832Mini"							/**< Name of BLE bridge/client device to be scanned */

// UART
#define BLE_MTU_SIZE			256//byte
#define PACKET_SIZE				128
#define UART_MAX_DATA_LEN  		(PACKET_SIZE)
#define UARTFIFOSIZE			CFIFO_MEMSIZE(UART_MAX_DATA_LEN)

// BLE
#define BLEFIFOSIZE				CFIFO_MEMSIZE(PACKET_SIZE)

// BLE clients
#define BLE_CLIENT_NAME			"UartBleBridge"
#define BLE_CLIENT_ID_01		{0xEF, 0x58, 0x1A, 0xFC, 0x50, 0xAE}
#define BLE_CLIENT_ID_02		{0xD2, 0x12, 0x36, 0xA1, 0x99, 0x9C}

// Clients for testing
#define BLE_CLIENT_ID_03		{0xC1, 0x10, 0x89, 0xE7, 0xD6, 0xB5} // BlueIO832
#define BLE_CLIENT_ID_04		{0xE4, 0x68, 0xE7, 0x6A, 0xB1, 0x12} // BlueIO832
#define BLE_CLIENT_ID_05		{0xD9, 0xC7, 0xEE, 0x21, 0xB0, 0xE1} // Client IBK board
#define BLE_CLIENT_ID_06		{0xCA, 0x23, 0x83, 0x8B, 0xE5, 0x09} // Client IBK board

#define BLE_CLIENT_ID_07 		{0xD9, 0xC7, 0xEE, 0x21, 0xB0, 0xE1}
#define BLE_CLIENT_ID_08		{0xEF, 0x58, 0x1A, 0xFC, 0x50, 0xAE}

uint8_t g_clientMacAddr[6] = BLE_CLIENT_ID_08;
uint8_t g_searchCnt = 0;

// CFIFO for BleAppWrite
alignas(4) uint8_t g_UartRx2BleBuff[BLEFIFOSIZE];//FIFO buffer for UART-to-BLE
std::atomic<int> g_UartRx2BleBuffLen(0);//buffer counter for g_UartRx2BleBuff
HCFIFO g_UartRx2BleFifo;

alignas(4) uint8_t g_UartRxExtBuff[PACKET_SIZE*2];//external buffer for Uart Rx data
volatile int g_UartRxExtBuffLen = 0;

volatile uint32_t g_DropCnt = 0;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt);
void BleTxSchedHandler(void * p_event_data, uint16_t event_size);

IOPinCfg_t s_Leds[] = LED_PIN_MAP;
static int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

const BleAppCfg_t s_BleAppCfg = {
	.Role = BLEAPP_ROLE_PERIPHERAL,
	1, 							// Number of central link
	0, 							// Number of peripheral link
	DEVICE_NAME,                // Device name
	ISYST_BLUETOOTH_ID,     	// PnP Bluetooth/USB vendor id
	1,                      	// PnP Product ID
	0,							// Pnp prod version
//	false,						// Enable device information service (DIS)
	NULL,//&s_UartBleDevDesc,
	.AdvType = BLEADV_TYPE_ADV_IND,
	NULL,//g_ManData,              // Manufacture specific data to advertise
	0,//sizeof(g_ManData),      // Length of manufacture specific data
	NULL,
	0,
	BLEAPP_SECTYPE_NONE,    	// Secure connection type
	BLEAPP_SECEXCHG_NONE,   	// Security key exchange
	NULL,      					// Service uuids to advertise
	0, 							// Total number of uuids
	0,       					// Advertising interval in msec
	0,							// Advertising timeout in sec
	0,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	MIN_CONN_INTERVAL,
	MAX_CONN_INTERVAL,
	LED_BLUE_PORT,//BLUEIO_CONNECT_LED_PORT,    // Led port nuber
	LED_BLUE_PIN,//BLUEIO_CONNECT_LED_PIN,     // Led pin number
	0,
	0,							// Tx power
	NULL,						// RTOS Softdevice handler
	.MaxMtu = BLE_MTU_SIZE,
	//.PeriphDevCnt = 1,			//Max number of peripheral connection
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
	.Rate = 115200,								// Baudrate
	.DataBits = 8,								// Data bits
	.Parity = UART_PARITY_NONE,					// Parity
	.StopBits = 1,								// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,			// Flow control
	.bIntMode = true,							// Interrupt mode
	.IntPrio = APP_IRQ_PRIORITY_LOW,			// Interrupt priority
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


/** @brief Parameters used when scanning. */
static ble_gap_scan_params_t const g_ScanParams =
{
#if (NRF_SD_BLE_API_VERSION >= 6)
	0,
	0,
#endif
    1,		// Active scan
#if (NRF_SD_BLE_API_VERSION <= 2)
	0,	// .selective
	NULL,	// .p_whitelist
#endif
#if (NRF_SD_BLE_API_VERSION >= 3)
	0,				// Use whitelist
	0, 				// Report directed advertisement
#endif
	SCAN_INTERVAL,	// Scan interval
	SCAN_WINDOW,	// Scan window
	SCAN_TIMEOUT,	// Scan timeout
};

uint8_t g_ScanBuff[BLE_GAP_SCAN_BUFFER_EXTENDED_MAX];

ble_data_t g_AdvScanReportData = {
	.p_data = g_ScanBuff,
	.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MAX
};

static ble_gap_conn_params_t s_ConnParams = {
	.min_conn_interval = (uint16_t)MSEC_TO_UNITS(NRF_BLE_SCAN_MIN_CONNECTION_INTERVAL, UNIT_1_25_MS),
	.max_conn_interval = (uint16_t)MSEC_TO_UNITS(NRF_BLE_SCAN_MAX_CONNECTION_INTERVAL, UNIT_1_25_MS),
	.slave_latency = (uint16_t)NRF_BLE_SCAN_SLAVE_LATENCY,
	.conn_sup_timeout = (uint16_t)MSEC_TO_UNITS(NRF_BLE_SCAN_SUPERVISION_TIMEOUT, UNIT_10_MS),
};


//BLE_UUID_TYPE_BLE
const ble_uuid_t s_UartBleSrvAdvUuid = {
	.uuid = BLUEIO_UUID_UART_SERVICE,
	.type = BLE_UUID_TYPE_BLE,
};

BleAppScanCfg_t s_bleScanInitCfg = {
		.Interval = SCAN_INTERVAL,
		.Duration = SCAN_WINDOW,
		.Timeout = SCAN_TIMEOUT,
		.BaseUid = BLUEIO_UUID_BASE,
		.ServUid = s_UartBleSrvAdvUuid.uuid,
};

BLEPERIPH_DEV g_ConnectedDev = {
	.ConnHdl = BLE_CONN_HANDLE_INVALID,
};

uint16_t g_BleTxCharHdl = BLE_CONN_HANDLE_INVALID;
uint16_t g_BleRxCharHdl = BLE_CONN_HANDLE_INVALID;

void BleDevDiscovered(BLEPERIPH_DEV *pDev)
{
#ifdef DEBUG_PRINT
	g_Uart.printf("Number service discovered: %d\r\n", g_ConnectedDev.NbSrvc);
    for (int i = 0; i < pDev->NbSrvc; i++)
    {
    	g_Uart.printf("Service_ID %d: 0x%x,  Num_Characteristic : %d\r\n",
    			i, g_ConnectedDev.Services[i].srv_uuid.uuid, g_ConnectedDev.Services[i].char_count);
    	for (int j = 0; j < g_ConnectedDev.Services[i].char_count; j++)
    	{
    		g_Uart.printf("	Char_ID %d: 0x%x\r\n",
    				j, g_ConnectedDev.Services[i].charateristics[j].characteristic.uuid.uuid);
    	}
    }
    // Find the desired UART-BLE Service
    g_Uart.printf("Looking for UART Service with UUID = 0x%x ...", BLUEIO_UUID_UART_SERVICE);
#endif
    int idx = BleDevFindService(pDev, BLUEIO_UUID_UART_SERVICE);
    if (idx != -1)
    {
#ifdef DEBUG_PRINT
    	g_Uart.printf("Found!\r\n");
#endif
    	// Rx characteristic
    	int dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_RX_CHAR);
#ifdef DEBUG_PRINT
    	g_Uart.printf("Find UART_RX_CHAR idx = 0x%x (%d)...", idx, idx);
#endif
    	if (dcharidx >= 0 && pDev->Services[idx].charateristics[dcharidx].characteristic.char_props.notify)
    	{
    		// Enable Notify
        	//g_Uart.printf("Enable notify\r\n");
        	BleAppEnableNotify(pDev->ConnHdl, pDev->Services[idx].charateristics[dcharidx].cccd_handle);
        	g_BleRxCharHdl = pDev->Services[idx].charateristics[dcharidx].characteristic.handle_value;
#ifdef DEBUG_PRINT
        	g_Uart.printf("Found!\r\n");
#endif
    	}
    	else
		{
#ifdef DEBUG_PRINT
			g_Uart.printf("Not Found!\r\n");
#endif
		}

    	// Tx characteristic
    	dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_TX_CHAR);
#ifdef DEBUG_PRINT
    	g_Uart.printf("Find UART_TX_CHAR idx = 0x%x (%d) ...", idx, idx);
#endif
    	if (dcharidx >= 0)
    	{
    		g_BleTxCharHdl = pDev->Services[idx].charateristics[dcharidx].characteristic.handle_value;
#ifdef DEBUG_PRINT
    		g_Uart.printf("Found!\r\n");
#endif
    	}
    	else
    	{
#ifdef DEBUG_PRINT
    		g_Uart.printf("Not Found!\r\n");
#endif
    	}
    }
    else
    {
#ifdef DEBUG_PRINT
    	g_Uart.printf("Not Found!\r\n");
#endif
    }

}

void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt)
{
    ret_code_t err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    const ble_common_evt_t *p_common_evt = &p_ble_evt->evt.common_evt;
    const ble_gattc_evt_t *p_gattc_evt = &p_ble_evt->evt.gattc_evt;
    //uint8_t addr[6] = { 0xda, 0x02, 0xe8, 0xfe, 0xac, 0xd1};
    uint8_t mac[6];
    char s[256];
    uint8_t len;

    switch (p_ble_evt->header.evt_id)
    {
    	case BLE_GAP_EVT_CONNECTED:
    		{
				g_ConnectedDev.ConnHdl = p_gap_evt->conn_handle;
				err_code = BleAppDiscoverDevice(&g_ConnectedDev);
    		}
    		break;
        case BLE_GAP_EVT_ADV_REPORT:
			{
				// Scan data report
				const ble_gap_evt_adv_report_t * p_adv_report = &p_gap_evt->params.adv_report;

				//inverse the macAddr array
				for (int i=0; i<6; i++)
				{
					mac[i] = p_adv_report->peer_addr.addr[5-i];
				}

				// Find device by name
//				if (ble_advdata_name_find(p_adv_report->data.p_data, p_adv_report->data.len, TARGET_BRIDGE_DEV_NAME))
//	            if (memcmp(addr, p_adv_report->peer_addr.addr, 6) == 0)
				if (memcmp(g_clientMacAddr, mac, 6) == 0)
				{
					// Connect to the found BLE bridge device
					err_code = BleAppConnect((ble_gap_addr_t *)&p_adv_report->peer_addr, &s_ConnParams);
					msDelay(10);
				}
				else
				{
					g_searchCnt++;
					BleAppScan();
				}
			}
			break;
        case BLE_GAP_EVT_TIMEOUT:
        	{
        	    ble_gap_evt_timeout_t const * p_timeout = &p_gap_evt->params.timeout;
        	    if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
        	    {
//        	    	g_Uart.printf("Scan timeout\r\n");
        	    }
        	}
        	break;
        case BLE_GAP_EVT_SCAN_REQ_REPORT:
        	{
        	    ble_gap_evt_scan_req_report_t const * p_req_report = &p_gap_evt->params.scan_req_report;
        	}
        	break;
        case BLE_GATTC_EVT_HVX:
        	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == g_BleRxCharHdl)
        	{
        		g_Uart.Tx(p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);
        	}
        	break;
  }
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);

	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);
	IOPinClear(LED_RED_PORT, LED_RED_PIN);

	// Retarget printf to uart if semihosting is not used
	//UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#ifdef DEBUG_PRINT
	g_Uart.printf("UART BLE Central Demo\r\n");
	msDelay(100);
	g_Uart.printf("UART Configuration: Baudrate %d, FLow Control (%s), Parity (%s)\r\n",
			g_UartCfg.Rate,
			(g_UartCfg.FlowControl == UART_FLWCTRL_NONE) ? "No" : "Yes",
			(g_UartCfg.Parity == UART_PARITY_NONE) ? "No" : "Yes");
#endif

	// Init CFIFO instance
	g_UartRx2BleFifo = CFifoInit(g_UartRx2BleBuff, BLEFIFOSIZE, 1, true);
}

void BleAppInitUserData()
{
//	// Add passkey pairing
//	ble_opt_t opt;
//	opt.gap_opt.passkey.p_passkey = (uint8_t*)"123456";
//	uint32_t err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt);
//	APP_ERROR_CHECK(err_code);
}


//void UartRxSchedHandler(void * p_event_data, uint16_t event_size)
//{
//	uint8_t buff[PACKET_SIZE];
//
//	int l = g_Uart.Rx(buff, PACKET_SIZE);
//	if (l > 0)
//	{
//		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
//		{
//			BleAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, buff, l);
//			g_Uart.Tx(buff, l);
//		}
//	}
//}

void BleTxSchedHandler(void * p_event_data, uint16_t event_size)
{
	IOPinToggle(LED_RED_PORT, LED_RED_PIN);

	int len = PACKET_SIZE;
	uint8_t *p = CFifoGetMultiple(g_UartRx2BleFifo, &len);

	if (p !=NULL)
	{
		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
			BleAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, p, len);
		}

		// Schedule this func again if g_UartRx2BleFifo is not empty
		app_sched_event_put(NULL, 0, BleTxSchedHandler);
	}

	IOPinToggle(LED_RED_PORT, LED_RED_PIN);
}


void UartRxSchedHandler(void * p_event_data, uint16_t event_size)
{
	bool flush = false;
	uint8_t *p = NULL;

	// Internal UartRxBuffer -> External Uart Rx Buffer
	int l1 = g_Uart.Rx(&g_UartRxExtBuff[g_UartRxExtBuffLen], PACKET_SIZE - g_UartRxExtBuffLen);
//	g_Uart.Tx(&g_UartRxExtBuff[g_UartRxExtBuffLen], l1);
	//g_Uart.printf("here_1\r\n");

	int cnt = 0;
	if (l1 > 0)
	{
		g_UartRxExtBuffLen += l1;
		if(g_UartRxExtBuffLen >= PACKET_SIZE)
		{
			flush = true;
		}
	}
	else
	{
		if(g_UartRxExtBuffLen > 0)
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
		app_sched_event_put(NULL, 0, BleTxSchedHandler);
	}

	// Schedule the UartRxSchedHandler if ExternalUartRxBuffer still has data
	if (g_UartRxExtBuffLen > 0)
	{
		app_sched_event_put(NULL,  0,  UartRxSchedHandler);
	}

}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			if (g_UartRxExtBuffLen <= 0)
			{
				app_sched_event_put(NULL, 0, UartRxSchedHandler);
			}

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

    BleAppInit((const BleAppCfg_t *)&s_BleAppCfg);//, true);

    //uint32_t ret = sd_ble_gap_scan_start(&g_ScanParams, &g_AdvScanReportData);
   // APP_ERROR_CHECK(ret);

    // Register the non-GATT service and its characteristics
    BleAppScanInit((BleAppScanCfg_t *)&s_bleScanInitCfg);

    BleAppScan();

    BleAppRun();

	return 0;
}

