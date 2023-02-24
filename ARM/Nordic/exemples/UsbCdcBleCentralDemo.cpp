/**-------------------------------------------------------------------------
@example main.cpp

@brief UsbCcBleCentral demo

This firmware scans for BLE-clients (via their names or MAC address), 
communicates with them, and bridges them with the host computer via USB interface.

NOTE: For compatible with C++ compiler, this C++ project must include 2 header files:
	"app_usbd_cdc_acm_internal.h"
	"app_usbd_cdc_acm.h"
	These 2 files are modified versions of the ones in:
	..\nRF5_SDK\components\libraries\usbd\class\cdc\acm


@author: Thinh Tran
@date: June 30, 2022

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

//#include <stdint.h>
//#include <stdbool.h>
//#include <stddef.h>
//#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
//#include "nrf_gpio.h"
//#include "nrf_delay.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
//#include "app_usbd_cdc_acm_internal.h"
//#include "app_usbd_class_base.h"

// IOsonata's UART, PinCfg
#include "board.h"
#include "blueio_board.h"
//#include "blueio_types.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "iopinctrl.h"
#include "idelay.h"

#include "istddef.h"
#include "stddev.h"
#include "idelay.h"

// BLE
#include "app_util_platform.h"
#include "app_scheduler.h"
#include "ble_gap.h"
#include "ble_advdata.h"
#include "nrf_ble_scan.h"
#include "bluetooth/bt_app.h"
#include "bluetooth/blueio_blesrvc.h"
#include "ble_dev.h"
#include "ble_gattc.h"

#include "cfifo.h"

/**
 * @brief CLI interface over UART
 */
//NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
//NRF_CLI_DEF(m_cli_uart,
//            "uart_cli:~$ ",
//            &m_cli_uart_transport.transport,
//            '\r',
//            4);

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

#define DEBUG_PRINT	//printf data for debug via UART
#define FIND_CLIENT_BY_NAME

#ifdef DEBUG_PRINT
#define DEBUG_ENABLE	true
#else
#define DEBUG_ENABLE 	false
#endif



#define SCAN_INTERVAL           MSEC_TO_UNITS(1000, UNIT_0_625_MS)		/**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             MSEC_TO_UNITS(100, UNIT_0_625_MS)       /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0                                 		/**< Timout when scanning. 0x0000 disables timeout. */


#ifdef FIND_CLIENT_BY_NAME
// BLE clients
#define BLE_CLIENT_NAME			"UARTBridge"

#else
#define BLE_CLIENT_ID_01		{0xEF, 0x58, 0x1A, 0xFC, 0x50, 0xAE}
#define BLE_CLIENT_ID_02		{0xD2, 0x12, 0x36, 0xA1, 0x99, 0x9C}
#define BLE_CLIENT_ID_03		{0xEF, 0x58, 0x1A, 0xFC, 0x50, 0xAE}

#define BLE_CLIENT_ID_04 		{0xD9, 0xC7, 0xEE, 0x21, 0xB0, 0xE1}
#define BLE_CLIENT_ID_05		{0xE3, 0x9C, 0xC8, 0xD8, 0xF2, 0x89}
#define BLE_CLIENT_ID_06		{0xF7, 0xD6, 0xE0, 0xCE, 0xCA, 0xD8}
#define BLE_CLIENT_ID_07		{0xEF, 0x58, 0x1A, 0xFC, 0x50, 0xAE}
#define BLE_CLIENT_ID_08		{0xD3, 0xDE, 0x73, 0x53, 0xF6, 0x1B}

#define BLE_CLIENT_ID_T1		{0xC1, 0x10, 0x89, 0xE7, 0xD6, 0xB5} // BlueIO832 for testing
#define BLE_CLIENT_ID_T2		{0xE4, 0x68, 0xE7, 0x6A, 0xB1, 0x12} // BlueIO832 for testing

#define BLE_CLIENT_ID_T3		{0xD9, 0xC7, 0xEE, 0x21, 0xB0, 0xE1} // Client IBK board
#define BLE_CLIENT_ID_T4		{0xCA, 0x23, 0x83, 0x8B, 0xE5, 0x09} // Client IBK board

#define BLUEPYRO_M3225_01		{0xC7, 0x63, 0x06, 0x6F, 0x8C, 0x93} // board #01

uint8_t g_clientMacAddr[6] = BLE_CLIENT_ID_T2;
#endif

uint8_t g_searchCnt = 0;

/* Enable power USB detection
 * Configure if example supports USB port connection */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

#define READ_SIZE (PACKET_SIZE / 2)//1

#define PRINT_DEBUG_UART(text)		\
	if (DEBUG_ENABLE)				\
		g_Uart.printf(text);			\

#define PRINT_DEBUG_USB(s, len)								\
	if (DEBUG_ENABLE)										\
	{														\
		app_usbd_cdc_acm_write(&m_app_cdc_acm, s, len);		\
	}														\

#define PRINT_DEBUG(s,len)				\
	PRINT_DEBUG_UART(s)					\
	PRINT_DEBUG_USB(s,len)				\


///////////////////////////////////////////////////

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event);
static void usbd_user_ev_handler(app_usbd_event_type_t event);

void HardwareInit();
int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void UartRxSchedHandler(void * p_event_data, uint16_t event_size);

void BleTxSchedHandler(void * p_event_data, uint16_t event_size);

void UsbInit();
void UsbRxSchedHandler(void * p_event_data, uint16_t event_size);
void UsbTxSchedHandler(void * p_event_data, uint16_t event_size);


/************************************************************
 * USBD CDC ACM section
 ************************************************************/
#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

#define INTERFACE_CONFIGS APP_USBD_CDC_ACM_CONFIG(CDC_ACM_COMM_INTERFACE, CDC_ACM_COMM_EPIN, CDC_ACM_DATA_INTERFACE, CDC_ACM_DATA_EPIN, CDC_ACM_DATA_EPOUT)

#define CLASS_CONFIG_PART (APP_USBD_CDC_ACM_INST_CONFIG(cdc_acm_user_ev_handler, \
        					CDC_ACM_COMM_INTERFACE,                       \
							CDC_ACM_COMM_EPIN,                            \
							CDC_ACM_DATA_INTERFACE,                       \
							CDC_ACM_DATA_EPIN,                            \
							CDC_ACM_DATA_EPOUT,                           \
							APP_USBD_CDC_COMM_PROTOCOL_AT_V250,           \
							&m_app_cdc_acm_ep))


alignas(4) uint8_t g_UsbRxBuff[PACKET_SIZE];
volatile int g_UsbRxBuffLen = 0;


alignas(4) uint8_t g_UsbTxBuff[PACKET_SIZE];//NRF_DRV_USBD_EPSIZE
volatile uint32_t g_UsbTxBuffLen = 0;

alignas(4) uint8_t g_UsbRxFifoMem[USBFIFOSIZE];
HCFIFO g_UsbRxFifo;

alignas(4) uint8_t g_UsbTxFifoMem[USBFIFOSIZE];
HCFIFO g_UsbTxFifo;

uint8_t g_extern_usbd_serial_number[12 + 1] = { "23062022"};
uint8_t g_extern_usbd_product_string[12 + 1] = { "BLE Central" };

volatile int g_frameCnt = 0;

//std::atomic<int> g_UsbRxBuffLen(0);
std::atomic<int> g_Usb2ExtBuffLen(0);

volatile int g_dropCnt = 0;
volatile bool g_UsbTxDone = true;// true: USB's internal buffer is available. false: otherwise

alignas(4) uint8_t g_BleRxBuff[PACKET_SIZE];


volatile uint32_t g_BleRxBuffLen = 0;

volatile bool g_TxSuccess = true;

/*
 * app_usbd_cdc_acm_data_t
 * APP_USBD_CLASS_DATA_TYPEDEF(type_name, class_data_dec);
/*
typedef struct                                             			\
{                                                          			\
	app_usbd_class_data_t base;                            			\
	app_usbd_cdc_acm_ctx_t ctx;	//class_data_dec = APP_USBD_CDC_ACM_DATA_SPECIFIC_DEC;
} app_usbd_cdc_acm_data_t;		//APP_USBD_CLASS_DATA_TYPE(type_name)
*/

/*
 * app_usbd_cdc_acm_t
 * APP_USBD_CLASS_INSTANCE_TYPEDEF(type_name, interface_configs, class_config_dec)
 */
/*typedef union app_usbd_cdc_acm_u                                    \
{                                                                   \
	app_usbd_class_inst_t base;                                     \
	struct                                                          \
	{                                                               \
		app_usbd_cdc_acm_data_t * p_data;                           \
		app_usbd_class_methods_t const * p_class_methods;           \
		struct                                                      \
		{                                                           \
			uint8_t cnt;                                            \
			app_usbd_class_iface_conf_t								\
				config[NUM_VA_ARGS(BRACKET_EXTRACT(INTERFACE_CONFIGS))];    				\
			app_usbd_class_ep_conf_t								\
				ep[APP_USBD_CLASS_CONF_TOTAL_EP_COUNT(INTERFACE_CONFIGS)];  	\
		} iface;                                                    \
		app_usbd_cdc_acm_inst_t inst; //class_config_dec = APP_USBD_CDC_ACM_INSTANCE_SPECIFIC_DEC                              \
	} specific;                                                                          \
} app_usbd_cdc_acm_t;
*/


/**
 * @brief CDC_ACM class instance
 * */
//APP_USBD_CDC_ACM_GLOBAL_DEF(m_app_cdc_acm,
//		cdc_acm_user_ev_handler,
//		CDC_ACM_COMM_INTERFACE,
//		CDC_ACM_DATA_INTERFACE,
//		CDC_ACM_COMM_EPIN,
//		CDC_ACM_DATA_EPIN,
//		CDC_ACM_DATA_EPOUT,
//		APP_USBD_CDC_COMM_PROTOCOL_AT_V250
//);

extern const app_usbd_class_methods_t app_usbd_cdc_acm_class_methods;

static uint8_t m_app_cdc_acm_ep = { (APP_USBD_EXTRACT_INTERVAL_FLAG(CDC_ACM_COMM_EPIN) ?
		APP_USBD_EXTRACT_INTERVAL_VALUE(CDC_ACM_COMM_EPIN) : APP_USBD_CDC_ACM_DEFAULT_INTERVAL)};

//static APP_USBD_CLASS_DATA_TYPE(type_name) CONCAT_2(instance_name, _data);
static app_usbd_cdc_acm_data_t	m_app_cdc_acm_data;

// Define a USB instance
static const app_usbd_cdc_acm_inst_t s_usb_inst =
{
		.comm_interface = CDC_ACM_COMM_INTERFACE,
		.comm_epin = CDC_ACM_COMM_EPIN,
		.data_interface = CDC_ACM_DATA_INTERFACE,
		.data_epout = CDC_ACM_DATA_EPOUT,
		.data_epin = CDC_ACM_DATA_EPIN,
		.protocol = APP_USBD_CDC_COMM_PROTOCOL_AT_V250,
		.user_ev_handler = cdc_acm_user_ev_handler,
		.p_ep_interval = &m_app_cdc_acm_ep,
};

const app_usbd_cdc_acm_t m_app_cdc_acm =
{
		//.base = 0,
		.specific =
		{
				.p_data = &m_app_cdc_acm_data,
				.p_class_methods = &app_usbd_cdc_acm_class_methods,
				.iface =
				{
						.cnt = NUM_VA_ARGS(BRACKET_EXTRACT(INTERFACE_CONFIGS)),
						.config = { APP_USBD_CLASS_IFACES_CONFIG_EXTRACT(INTERFACE_CONFIGS) },
						.ep = { APP_USBD_CLASS_IFACES_EP_EXTRACT(INTERFACE_CONFIGS) },
				},
				.inst = s_usb_inst,//BRACKET_EXTRACT(CLASS_CONFIG_PART),
		},
};

// USBD config
static const app_usbd_config_t usbd_config =
{
	.ev_state_proc = usbd_user_ev_handler
};


// LED pin config
static const IOPinCfg_t s_Leds[] = LED_PIN_MAP;
static const int s_NbLeds = sizeof(s_Leds) / sizeof(IOPinCfg_t);

// Button pin config
static const IOPinCfg_t s_ButPins[] = BUTTON_PIN_MAP;
static const int s_NbButPins = sizeof(s_ButPins) / sizeof(IOPinCfg_t);


/************************************************************
 * UART section
 ************************************************************/
// Uart pin config
IOPinCfg_t s_UartPins[] = UART_PIN_MAP;
static int s_NbUartPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t);

// UART operation mode config
alignas(4) uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) uint8_t s_UartTxFifo[UARTFIFOSIZE];

UARTCfg_t g_UartCfg = {
	.DevNo = 0,									// Device number zero based
	.pIOPinMap = s_UartPins,					// UART assigned pins
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),					// Total number of UART pins used
	.Rate = 115200,								// Baudrate
	.DataBits = 8,								// Data bits
	.Parity = UART_PARITY_NONE,			// Parity
	.StopBits = 1,								// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,	// Flow control
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

/************************************************************
 * BLE section
 ************************************************************/

const BtAppCfg_t s_BleAppCfg = {
	.Role = BTAPP_ROLE_CENTRAL,
	1,//1, 						// Number of central link
	0, 							// Number of peripheral link
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     	// PnP Bluetooth/USB vendor id
	1,                      	// PnP Product ID
	0,							// Pnp prod version
	0,
	NULL,//&s_UartBleDevDesc,
	false,						// Enable device information service (DIS)
	NULL,//g_ManData,           // Manufacture specific data to advertise
	0,//sizeof(g_ManData),      // Length of manufacture specific data
	NULL,
	0,
	BTGAP_SECTYPE_NONE,    	// Secure connection type
	BTAPP_SECEXCHG_NONE,   	// Security key exchange
	NULL,      					// Service uuids to advertise
	0, 							// Total number of uuids
	0,       					// Advertising interval in msec
	0,							// Advertising timeout in sec
	0,                          // Slow advertising interval, if > 0, fallback to
								// slow interval on adv timeout and advertise until connected
	MIN_CONN_INTERVAL,
	MAX_CONN_INTERVAL,
	LED_BLUE_PORT, //BLUEIO_CONNECT_LED_PORT,    // Led port nuber
	LED_BLUE_PIN, //BLUEIO_CONNECT_LED_PIN,     // Led pin number
	0,
	0,							// Tx power
	NULL,						// RTOS Softdevice handler
	.MaxMtu = BLE_MTU_SIZE,		// BLE MTU packet size
};


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

static BtGapConnParams_t s_ConnParams = {
	.IntervalMin = 7.5,//NRF_BLE_SCAN_MIN_CONNECTION_INTERVAL,
	.IntervalMax = 40,//NRF_BLE_SCAN_MAX_CONNECTION_INTERVAL,
	.Latency = NRF_BLE_SCAN_SLAVE_LATENCY,
	.Timeout = NRF_BLE_SCAN_SUPERVISION_TIMEOUT,
};


//BLE_UUID_TYPE_BLE
const ble_uuid_t s_UartBleSrvAdvUuid = {
	.uuid = BLUEIO_UUID_UART_SERVICE,
	.type = BLE_UUID_TYPE_BLE,
};

BtGapScanCfg_t s_bleScanInitCfg = {
		.Interval = SCAN_INTERVAL,
		.Duration = SCAN_WINDOW,
		.Timeout = SCAN_TIMEOUT,
		.BaseUid = BLUEIO_UUID_BASE,
		.ServUid = BLUEIO_UUID_UART_SERVICE,//s_UartBleSrvAdvUuid,
};

BLEPERIPH_DEV g_ConnectedDev = {
	.ConnHdl = BLE_CONN_HANDLE_INVALID,
};

uint16_t g_BleTxCharHdl = BLE_CONN_HANDLE_INVALID;
uint16_t g_BleRxCharHdl = BLE_CONN_HANDLE_INVALID;

void BleDevDiscovered(BLEPERIPH_DEV *pDev)
{
	char s[256];
	int l;
//	l = sprintf(s, "Number service discovered: %d\r\n", g_ConnectedDev.NbSrvc);
//	PRINT_DEBUG(s,l)
	g_Uart.printf("Number service discovered: %d\r\n", g_ConnectedDev.NbSrvc);
    for (int i = 0; i < pDev->NbSrvc; i++)
    {
//    	l = sprintf(s, "Service_ID %d: 0x%x,  Num_Characteristic : %d\r\n",
//    			i, g_ConnectedDev.Services[i].srv_uuid.uuid, g_ConnectedDev.Services[i].char_count);
//    	PRINT_DEBUG(s,l)
    	g_Uart.printf("Service_ID %d: 0x%x,  Num_Characteristic : %d\r\n",
    			i, g_ConnectedDev.Services[i].srv_uuid.uuid, g_ConnectedDev.Services[i].char_count);
    	for (int j = 0; j < g_ConnectedDev.Services[i].char_count; j++)
    	{
//    		l = sprintf(s, "Char_ID %d: 0x%x\r\n",
//    				j, g_ConnectedDev.Services[i].charateristics[j].characteristic.uuid.uuid);
//    		PRINT_DEBUG(s,l)
    		g_Uart.printf("Char_ID %d: 0x%x\r\n",
    				j, g_ConnectedDev.Services[i].charateristics[j].characteristic.uuid.uuid);
    	}
    }

    // Find the desired UART-BLE Service
    l = sprintf(s, "Looking for UART Service with UUID = 0x%x ...", BLUEIO_UUID_UART_SERVICE);
    PRINT_DEBUG(s,l)
    int idx = BleDevFindService(pDev, BLUEIO_UUID_UART_SERVICE);
    if (idx != -1)
    {
    	l = sprintf(s, "Found!\r\n");
    	PRINT_DEBUG(s,l);
    	// Rx characteristic
    	int dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_RX_CHAR);
    	l = sprintf(s, "Find UART_RX_CHAR idx = 0x%x (%d)...", idx, idx);
    	PRINT_DEBUG(s,l);
    	if (dcharidx >= 0 && pDev->Services[idx].charateristics[dcharidx].characteristic.char_props.notify)
    	{
    		// Enable Notify
        	BtAppEnableNotify(pDev->ConnHdl, pDev->Services[idx].charateristics[dcharidx].cccd_handle);
        	g_BleRxCharHdl = pDev->Services[idx].charateristics[dcharidx].characteristic.handle_value;
        	l = sprintf(s, "Found!\r\n");
        	PRINT_DEBUG(s,l);
    	}
    	else
		{
    		l = sprintf(s, "Not Found!\r\n");
    		PRINT_DEBUG(s,l);
		}

    	// Tx characteristic
    	dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_TX_CHAR);
    	l = sprintf(s, "Find UART_TX_CHAR idx = 0x%x (%d) ...", idx, idx);
    	PRINT_DEBUG(s,l);
    	if (dcharidx >= 0)
    	{
    		g_BleTxCharHdl = pDev->Services[idx].charateristics[dcharidx].characteristic.handle_value;
    		l = sprintf(s, "Found!\r\n");
    		PRINT_DEBUG(s,l);
    	}
    	else
    	{
    		l = sprintf(s, "Not Found!\r\n");
    		PRINT_DEBUG(s,l);
    	}
    }
    else
    {
    	l = sprintf(s, "Not Found!\r\n");
    	PRINT_DEBUG(s,l);
    }

}

void BtAppCentralEvtHandler(uint32_t Evt, void *pCtx)
{
    ret_code_t err_code;
	ble_evt_t * p_ble_evt = (ble_evt_t*)Evt;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    const ble_common_evt_t *p_common_evt = &p_ble_evt->evt.common_evt;
    const ble_gattc_evt_t *p_gattc_evt = &p_ble_evt->evt.gattc_evt;
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

#ifdef FIND_CLIENT_BY_NAME
				bool res = ble_advdata_name_find(p_adv_report->data.p_data, p_adv_report->data.len, BLE_CLIENT_NAME);
				if (res == false)
				{
					res = ble_advdata_short_name_find(p_adv_report->data.p_data, p_adv_report->data.len, BLE_CLIENT_NAME, strlen(BLE_CLIENT_NAME));
				}
				if (res == true)
#else
				if (memcmp(g_clientMacAddr, mac, 6) == 0)// Find device by MAC address
#endif
				{
					len = sprintf(s, "Target client device FOUND!\r\n");
					PRINT_DEBUG(s,len);
					BtGapPeerAddr_t addr = {.Type = p_adv_report->peer_addr.addr_type,};
					memcpy(addr.Addr, p_adv_report->peer_addr.addr, 6);

					res = BtAppConnect(&addr, &s_ConnParams);
					len = sprintf(s, "res = %x\r\n", res);
					PRINT_DEBUG(s,len);

					msDelay(10);
				}
				else
				{
					g_searchCnt++;
					len = sprintf(s, "Keep searching #%d\r\n", g_searchCnt);
					PRINT_DEBUG(s,len)
					BtAppScan();
				}
			}
			break;
        case BLE_GAP_EVT_TIMEOUT:
        	{
        	    ble_gap_evt_timeout_t const * p_timeout = &p_gap_evt->params.timeout;
        	    if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
        	    {
        	    	len = sprintf(s, "Scan time out\r\n");
        	    	PRINT_DEBUG(s,len)
        	    }
        	}
        	break;
        case BLE_GAP_EVT_SCAN_REQ_REPORT:
        	{
        	    ble_gap_evt_scan_req_report_t const * p_req_report = &p_gap_evt->params.scan_req_report;
        	}
        	break;
        case BLE_GATTC_EVT_HVX:
        	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
        	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == g_BleRxCharHdl)
        	{
        		int l = p_ble_evt->evt.gattc_evt.params.hvx.len;
        		int fifoAvai = l;
        		int idx = 0;
        		uint8_t *p = NULL;

        		while (l > 0)
        		{
        			fifoAvai = l;
        			p = CFifoPutMultiple(g_UsbTxFifo, &fifoAvai);
        			if (p != NULL)
        			{
						memcpy(p, &p_ble_evt->evt.gattc_evt.params.hvx.data[idx], fifoAvai);
						l -= fifoAvai;
						idx = fifoAvai;
//						if (l > 0)
//							g_Uart.printf("Still have %d byte data \r\n", l);
        			}
        			else
        			{

//        				g_Uart.printf("Fifo buffer full!\r\n");
        				break;
        			}
        		}
				app_sched_event_put(NULL, 0, UsbTxSchedHandler);
        	}
        	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
        	break;
  }
}


/********************************************************************************/
/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
	static uint8_t usbRxBuff[PACKET_SIZE];
	static int usbReadLen;
	ret_code_t ret;
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    //static uint16_t dataLen;

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
            /*!!! Always need to setup first transfer*/
        	ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, usbRxBuff, PACKET_SIZE);
//        	g_Uart.printf("USB port open! Setup first USB transfer\r\n");
        	PRINT_DEBUG_UART("USB port open! Setup first USB transfer\r\n");
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        	IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        	g_UsbTxDone = true;
            break;
		case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
		//app_sched_event_put(NULL, 0, UsbRxSchedHandler);
		{
			IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
			usbReadLen = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
			uint8_t *p = NULL;
			int fifoAvai = usbReadLen;
			int idx = 0;
			// Put data to CFifo
			while (usbReadLen > 0)
			{
				fifoAvai = usbReadLen;
				p = CFifoPutMultiple(g_UsbRxFifo, &fifoAvai);
//				g_Uart.printf("usbReadLen = %d | fifoAvai = %d\r\n", usbReadLen, fifoAvai);
				if (p != NULL)
				{
					memcpy(p, &usbRxBuff[idx], fifoAvai);
					usbReadLen -= fifoAvai;
					idx = fifoAvai;
				}
				else
				{
					PRINT_DEBUG_UART("Fifo buffer full!\r\n")
					break;
				}
			}
			app_sched_event_put(NULL, 0, BleTxSchedHandler);
			app_usbd_cdc_acm_read_any(&m_app_cdc_acm, usbRxBuff, PACKET_SIZE);
			IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
			break;
		}
        default:
            break;
    }
}

/* Send data from Ble -> USB */
void UsbTxSchedHandler(void * p_event_data, uint16_t event_size)
{
	int l = 64;// internal buffer of USB is max 64 bytes

	if (CFifoUsed(g_UsbTxFifo) > 0)
	{
		if (g_UsbTxDone)
		{
			uint8_t *p = CFifoGetMultiple(g_UsbTxFifo, &l);
			//g_Uart.printf("nByteRead = %d\r\n", l);
			if (p != NULL)
			{
				g_UsbTxDone = false;
				app_usbd_cdc_acm_write(&m_app_cdc_acm, p, l);
			}
		}
		app_sched_event_put(NULL, 0, UsbTxSchedHandler);
	}
}


/* Send data from USB -> Ble */
void UsbRxSchedHandler(void * p_event_data, uint16_t event_size)
{
#if 0
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
	int usbReadLen = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
	uint8_t usbRxBuff[PACKET_SIZE];
	uint8_t *p = NULL;
	int fifoAvai = usbReadLen;
	int idx = 0;

	// Put data to CFifo
	while (usbReadLen > 0)
	{
		fifoAvai = usbReadLen;
		p = CFifoPutMultiple(g_UsbRxFifo, &fifoAvai);
		//				g_Uart.printf("usbReadLen = %d | fifoAvai = %d\r\n", usbReadLen, fifoAvai);
		if (p != NULL)
		{
			memcpy(p, &usbRxBuff[idx], fifoAvai);
			usbReadLen -= fifoAvai;
			idx = fifoAvai;
		}
		else
		{
			PRINT_DEBUG_UART("Fifo buffer full!\r\n")
			break;
		}
	}
	app_sched_event_put(NULL, 0, BleTxSchedHandler);
	ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, usbRxBuff, PACKET_SIZE);
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
#endif
}


/* Send data from Fifo buffer to Ble interface */
void BleTxSchedHandler(void * p_event_data, uint16_t event_size)
{
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
	int len = PACKET_SIZE;
	uint8_t *p = NULL;
	bool flush = false;

	// Transfer data from Fifo buffer to global buffer
	if (g_TxSuccess)
	{
		p = CFifoGetMultiple(g_UsbRxFifo, &len);
		if (p != NULL)
		{
			memcpy(g_UsbRxBuff, p, len);
			g_UsbRxBuffLen = len;
			flush = true;
			//g_Uart.printf("Flush 1: Get %d byte NEW data from CFifo\r\n", g_UsbRxBuffLen);
		}
		else
		{
			g_UsbRxBuffLen = 0;
			//g_Uart.printf("Empty Cfifo buffer\r\n");
		}
	}
	else
	{
		flush = true;
//		g_Uart.printf("g_TxSuccess = %d\r\n", g_TxSuccess);
//		g_Uart.printf("Flush 2: Send data from Global buffer\r\n");
	}

	// Transfer data from global buffer to Ble interface
	if (flush)
	{
		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
			g_TxSuccess = BtAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, g_UsbRxBuff, g_UsbRxBuffLen);
		}
	}

	if (g_UsbRxBuffLen > 0)
	{
		app_sched_event_put(NULL, 0, BleTxSchedHandler);
	}
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
}

#if 0
void UsbRxSchedHandler(void * p_event_data, uint16_t event_size)
{
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
	ret_code_t ret;
	int len = PACKET_SIZE;

	g_UsbRxBuffLen = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
	ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, g_UsbRxBuff, PACKET_SIZE);
	uint8_t *p = CFifoPutMultiple(g_UsbRxFifo, &len);

	// Send to BLE interface
	if (g_UsbRxBuffLen > 0)
	{
		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
			BleAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, g_UsbRxBuff, g_UsbRxBuffLen);
		}
	}
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
}

void BleTxSchedHandler(void * p_event_data, uint16_t event_size)
{
	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);

	int len = PACKET_SIZE;
	uint8_t *p = CFifoGetMultiple(g_UsbRxFifo, &len);

	if (p !=NULL)
	{
		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
			BleAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, p, len);
		}

		app_sched_event_put(NULL, 0, BleTxSchedHandler);
	}

	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
}
#endif

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
            break;
        case APP_USBD_EVT_DRV_RESUME:
            break;
        case APP_USBD_EVT_STARTED:
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);
            break;
        case APP_USBD_EVT_POWER_DETECTED:
        	//g_Uart.printf("APP_USBD_EVT_POWER_DETECTED\r\n");
            if (!nrf_drv_usbd_is_enabled())
            {
            	for (int i = 0; i < 4; i++)
				{
					IOPinToggle(LED_RED_PORT, LED_RED_PIN);
					msDelay(125);
				}
                app_usbd_enable();
            }
            break;
        case APP_USBD_EVT_POWER_REMOVED:
        	//g_Uart.printf("APP_USBD_EVT_POWER_REMOVED\r\n");
            app_usbd_stop();
            IOPinClear(LED_RED_PORT, LED_RED_PIN);
            break;
		case APP_USBD_EVT_POWER_READY:
			//g_Uart.printf("APP_USBD_EVT_POWER_READY\r\n");
			app_usbd_start();
#ifdef UDG
            IOPinSet(LED_GREEN_PORT, LED_GREEN_PIN);
#else
            IOPinSet(LED_RED_PORT, LED_RED_PIN);
#endif
            break;
        default:
            break;
    }
}


void HardwareInit()
{
    /* TODO: Hardware init
     * LEDs
     * UART
     * Timer
     */

	// Clock
	ret_code_t ret;
	int len;
	char s[256];
	ret = nrf_drv_clock_init();

	// Uart
	g_Uart.Init(g_UartCfg);
	len = sprintf(s, "System clock init...%s\r\n", (ret ? "Failed!" : "Done!"));
	PRINT_DEBUG(s,len)
	len = sprintf(s, "UART Configuration: %d, %d, %d\r\n",
			g_UartCfg.Rate, g_UartCfg.FlowControl, g_UartCfg.Parity);
	PRINT_DEBUG(s,len)
	msDelay(100);

	// LEDs
	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinClear(LED_RED_PORT, LED_RED_PIN);
	IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);

	//CFifoInit(pMemBlk, TotalMemSize, BlkSize, bBlocking)
	g_UsbRxFifo = CFifoInit(g_UsbRxFifoMem, USBFIFOSIZE, 1, true);
	g_UsbTxFifo = CFifoInit(g_UsbTxFifoMem, USBFIFOSIZE, 1, true);

	// USB module
	UsbInit();// !!! Must be executed before BLE initialization
	msDelay(500);
	len = sprintf(s, "USB-BLE is on\r\n");
	PRINT_DEBUG(s,len)
}

void UsbInit()
{
	ret_code_t ret;

	// Init USB
#ifdef DEBUG_PRINT
	g_Uart.printf("Init USB...");
#endif
	ret = app_usbd_init(&usbd_config);
#ifdef DEBUG_PRINT
	g_Uart.printf("%s\r\n", (ret ? "Failed" : "Done!"));
#endif

	//USBD serial number generator
	app_usbd_serial_num_generate();

	// Append USB class instance
	const app_usbd_class_inst_t *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
	ret = app_usbd_class_append(class_cdc_acm);
	//APP_ERROR_CHECK(ret);
#ifdef DEBUG_PRINT
	g_Uart.printf("%s\r\n", (ret ? "Failed" : "Done!"));
#endif

	// Check power
	if (USBD_POWER_DETECTION)
	{
#ifdef DEBUG_PRINT
		g_Uart.printf("USB power detection...");
#endif
		ret = app_usbd_power_events_enable();
#ifdef DEBUG_PRINT
		g_Uart.printf("%s\r\n", (ret ? "Failed" : "Done!"));
#endif
	}
	else
	{
#ifdef DEBUG_PRINT
		g_Uart.printf("No USB power detection enabled\r\nStarting USB now");
#endif
		app_usbd_enable();
		app_usbd_start();
	}
}

void BleAppInitUserData()
{
	// TODO: Add passkey pairing
/*	ble_opt_t opt;
	opt.gap_opt.passkey.p_passkey = (uint8_t*)"123456";
	uint32_t err_code =  sd_ble_opt_set(BLE_GAP_OPT_PASSKEY, &opt);
	APP_ERROR_CHECK(err_code);*/
}

void UartRxSchedHandler(void * p_event_data, uint16_t event_size)
{
	uint8_t buff[PACKET_SIZE];

	int l = g_Uart.Rx(buff, PACKET_SIZE);
	if (l > 0)
	{
		if (g_ConnectedDev.ConnHdl != BLE_CONN_HANDLE_INVALID && g_BleTxCharHdl != BLE_CONN_HANDLE_INVALID)
		{
//			BleAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, buff, l);
		}
	}
}

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
//	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			app_sched_event_put(NULL, 0, UartRxSchedHandler);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}


int main(void)
{
	char s[256];
	uint8_t len;
	ret_code_t ret;

	HardwareInit();

    // BLE init
    ret = BtAppInit(&s_BleAppCfg);
    msDelay(250);
    len = sprintf(s, "Bluetooth init...Done!\r\n");
    PRINT_DEBUG(s,len)

    // BLE run
#ifdef FIND_CLIENT_BY_NAME
    len = sprintf(s, "Searching for client with Name = %s", BLE_CLIENT_NAME);
    PRINT_DEBUG(s,len);
#else
    len = sprintf(s, "Searching for client with MAC addr = ");
    PRINT_DEBUG(s,len);
    len = 0;
	for (int i=0; i<6; i++)
	{
		len += sprintf(&s[len], "%02X%s", g_clientMacAddr[i], i<5 ? ":" : "\r\n");
	}
	PRINT_DEBUG(s,len)
#endif

	BtAppScanInit(&s_bleScanInitCfg);// Register the non-GATT BLE services and their characteristics
	BtAppScan();
	BtAppRun();

	return 0;
}
