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
#ifndef NRFXLIB_SDC
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "nrf_ble_scan.h"
#include "ble_gatt_db.h"
#include "ble_gattc.h"
#endif

#include "istddef.h"
#include "bluetooth/bt_app.h"
//#include "ble_app_nrf5.h"
//#include "ble_service.h"
#include "bluetooth/blueio_blesrvc.h"
#include "bluetooth/bt_dev.h"
#include "blueio_board.h"
#include "coredev/uart.h"
#include "custom_board.h"//Does this make the LED_2 & LED_3 do not work?
#include "coredev/iopincfg.h"
#include "stddev.h"
#include "board.h"
#include "idelay.h"
#include "cfifo.h"
#include "iopinctrl.h"

#define DEBUG_PRINT		// Enable printing debug info over UART interface

// BLE
#define DEVICE_NAME             "UARTCentral"          /**< Name of device. Will be included in the advertising data. */

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
hCfifo_t g_UartRx2BleFifo;

alignas(4) uint8_t g_UartRxExtBuff[PACKET_SIZE*2];//external buffer for Uart Rx data
volatile int g_UartRxExtBuffLen = 0;

volatile uint32_t g_DropCnt = 0;

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
//void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt);
void BleTxSchedHandler(void * p_event_data, uint16_t event_size);

IOPinCfg_t s_Leds[] = LED_PIN_MAP;
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
		.bExtAdv = false,						// Conventional 31-byte Advertising packet
		.pAdvManData = NULL,//g_ManData,		// Manufacture specific data to advertise
		.AdvManDataLen = 0,//sizeof(g_ManData),	// Length of manufacture specific data
		.pSrManData = NULL,						// Addition Manufacture specific data to advertise in scan response
		.SrManDataLen = 0,						// Length of manufacture specific data in scan response
		.SecType = BTGAP_SECTYPE_NONE,			// Secure connection type
		.SecExchg = BTAPP_SECEXCHG_NONE,		// Security key exchange
		.bCompleteUuidList = false,				// true - Follow is a complete uuid list. false - incomplete list (more uuid than listed here)
		.pAdvUuid = NULL,      					// Service uuids to advertise
		//	.NbAdvUuid = 0, 					// Total number of uuids
		.AdvInterval = APP_ADV_INTERVAL,		// Advertising interval in msec
		.AdvTimeout = APP_ADV_TIMEOUT,			// Advertising timeout in sec
		.AdvSlowInterval = 0,					// Slow advertising interval, if > 0, fallback to
							 	 	 	 	 	// slow interval on adv timeout and advertise until connected
		.ConnIntervalMin = MIN_CONN_INTERVAL,
		.ConnIntervalMax = MAX_CONN_INTERVAL,
		.ConnLedPort = LED_BLUE_PORT,			// Led port nuber
		.ConnLedPin = LED_BLUE_PIN,				// Led pin number
		.ConnLedActLevel = 0,					// Connection LED ON logic level (0: Logic low, 1: Logic high)
		.TxPower = 0,							// Tx power
		.SDEvtHandler = NULL,					// RTOS Softdevice handler
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

BtDev_t g_ConnectedDev = {
	.ConnHdl = BT_CONN_HDL_INVALID,
};

#ifndef NRFXLIB_SDC
#if 0
/** @brief Parameters used when scanning. */
static const ble_gap_scan_params_t g_ScanParams =
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
#endif

uint8_t g_ScanBuff[BLE_GAP_SCAN_BUFFER_EXTENDED_MAX];

ble_data_t g_AdvScanReportData = {
	.p_data = g_ScanBuff,
	.len = BLE_GAP_SCAN_BUFFER_EXTENDED_MAX
};


//BLE_UUID_TYPE_BLE
const ble_uuid_t s_UartBleSrvAdvUuid = {
	.uuid = BLUEIO_UUID_UART_SERVICE,
	.type = BLE_UUID_TYPE_BLE,
};

uint16_t g_BleTxCharHdl = BLE_CONN_HANDLE_INVALID;
uint16_t g_BleRxCharHdl = BLE_CONN_HANDLE_INVALID;

void BleDevDiscovered(BtDev_t *pDev)
{
	int flag = 3;
#if 0
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
#endif

    int idx = BleDevFindService(pDev, BLUEIO_UUID_UART_SERVICE);
    if (idx != -1)
    {
    	flag--;
#ifdef DEBUG_PRINT
    	g_Uart.printf("Device found and paired with!\r\n");
#endif
    	// Rx characteristic
    	int dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_RX_CHAR);
#ifdef DEBUG_PRINT
    	g_Uart.printf("Find UART_RX_CHAR idx = 0x%x (%d)...", dcharidx, dcharidx);
#endif
    	if (dcharidx >= 0 && pDev->Services[idx].charateristics[dcharidx].characteristic.char_props.notify)
    	{
    		flag--;
    		// Enable Notify
        	//g_Uart.printf("Enable notify\r\n");
        	uint32_t ec = BtAppEnableNotify(pDev->ConnHdl, pDev->Services[idx].charateristics[dcharidx].cccd_handle);

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
    	g_Uart.printf("Find UART_TX_CHAR idx = 0x%x (%d) ...", dcharidx, dcharidx);
#endif
    	if (dcharidx >= 0)
    	{
    		flag--;
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

    if (flag == 0)
    {
    	g_Uart.printf("Start data communication\r\n\r\n");
    	msDelay(10);
    }
}

void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx)
{

	ble_evt_t * p_ble_evt = (ble_evt_t*)Evt;
    ret_code_t err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    const ble_common_evt_t *p_common_evt = &p_ble_evt->evt.common_evt;
    const ble_gattc_evt_t *p_gattc_evt = &p_ble_evt->evt.gattc_evt;
    //uint8_t addr[6] = { 0xda, 0x02, 0xe8, 0xfe, 0xac, 0xd1};
    uint8_t mac[6];
    char s[256];
    uint8_t len;

   // g_Uart.printf("BtAppCentralEvtHandler %d (0x%x)\r\n", p_ble_evt->header.evt_id, p_ble_evt->header.evt_id);

    switch (p_ble_evt->header.evt_id)
    {
#if 0
    	case BLE_GAP_EVT_CONNECTED:
    		{
				g_ConnectedDev.ConnHdl = p_gap_evt->conn_handle;
				BtAppDiscoverDevice(&g_ConnectedDev);
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
				//g_Uart.printf("%x:%x:\r\n", mac[0], mac[1]);

				// Find device by name
//				if (ble_advdata_name_find(p_adv_report->data.p_data, p_adv_report->data.len, TARGET_BRIDGE_DEV_NAME))
//	            if (memcmp(addr, p_adv_report->peer_addr.addr, 6) == 0)
				if (memcmp(g_clientMacAddr, mac, 6) == 0)
				//if (mac[0] == 0xCB || mac[0] == 0x84)
				{
					// Connect to the found BLE bridge device
					BtGapPeerAddr_t addr = {.Type = p_adv_report->peer_addr.addr_type,};
					memcpy(addr.Addr, p_adv_report->peer_addr.addr, 6);

					BtAppConnect(&addr, &s_ConnParams);
					msDelay(10);
				}
				else
				{
					g_searchCnt++;
					//BtAppScan();
				}
			}
			break;
#endif
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
        	//g_Uart.printf("handle = %d \r\n", p_ble_evt->evt.gattc_evt.params.hvx.handle);
        	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == g_BleRxCharHdl)
        	{
        		g_Uart.Tx(p_ble_evt->evt.gattc_evt.params.hvx.data, p_ble_evt->evt.gattc_evt.params.hvx.len);
        	}
        	break;
  }
}
#endif

void BtAppEvtConnected(uint16_t ConnHdl)
{
	g_ConnectedDev.ConnHdl = ConnHdl;
	g_Uart.printf("BtAppEvtConnected ConnHdl = %d (0x%x)\r\n", ConnHdl, ConnHdl);

	//g_Uart.printf("This device's Role = %s\r\n", s_BleAppCfg.Role == BT_GAP_ROLE_CENTRAL ? "Central" : "Peripheral");
	if (s_BleAppCfg.Role & (BTAPP_ROLE_CENTRAL | BTAPP_ROLE_OBSERVER))
	{
		BtAppDiscoverDevice(&g_ConnectedDev);
	}
}

void BtAppEvtDisconnected(uint16_t ConnHdl)
{
	g_Uart.printf("BtAppEvtDisconnected ConnHdl = %d (0x%x) \r\n", ConnHdl, ConnHdl);
}

bool BtAppScanReport(int8_t Rssi, uint8_t AddrType, uint8_t Addr[6], size_t AdvLen, uint8_t *pAdvData)
{
	char name[32];
	size_t l = BtAdvDataGetDevName(pAdvData, AdvLen, name, 32);

	if (l > 0)
	{
		name[l-1] = 0;
		g_Uart.printf("%s, \r\n", name);
		//g_Uart.printf(
		//		"MAC Addr: %02X:%02X:%02X:%02X:%02X:%02X, RSSI = %d \r\n",
		//		Addr[0], Addr[1], Addr[2], Addr[3], Addr[4], Addr[5], Rssi);
	//}

	//if (memcmp(g_clientMacAddr, Addr, 6) == 0)
	//{
		g_Uart.printf("Found matching device. Stop Scan\r\n");
		BtGapScanStop();

		BtGapPeerAddr_t add = {.Type = AddrType, };
		memcpy(add.Addr, Addr, 6);
		BtGapConnect(&add, &s_ConnParams);

		return false;
	}

	return true;
}

void HardwareInit()
{
	g_Uart.Init(g_UartCfg);

	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinSet(LED_GREEN_PORT, LED_GREEN_PIN);
	IOPinSet(LED_RED_PORT, LED_RED_PIN);
	IOPinSet(LED4_PORT, LED4_PIN);

	// Retarget printf to uart if semihosting is not used
	//UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#ifdef DEBUG_PRINT
	g_Uart.printf("UART BLE Central Demo\r\n");
	msDelay(10);
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
#ifndef NRFXLIB_SDC
		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
			BtAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, p, len);
		}

		// Schedule this func again if g_UartRx2BleFifo is not empty
		app_sched_event_put(NULL, 0, BleTxSchedHandler);
#endif
	}

	IOPinToggle(LED_RED_PORT, LED_RED_PIN);
}


void UartRxSchedHandler(void * p_event_data, uint16_t event_size)
{
	bool flush = false;
	uint8_t *p = NULL;

	// Internal UartRxBuffer -> External Uart Rx Buffer
	int l1 = g_Uart.Rx(&g_UartRxExtBuff[g_UartRxExtBuffLen], PACKET_SIZE - g_UartRxExtBuffLen);

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

    BtAppInit(&s_BleAppCfg);

    // Register the non-GATT service and its characteristics
    BtAppScanInit((BtGapScanCfg_t*)&g_ScanParams);

    BtAppScan();

    BtAppRun();

	return 0;
}

