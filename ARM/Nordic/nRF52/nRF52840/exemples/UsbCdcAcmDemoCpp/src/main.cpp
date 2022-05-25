/**
 *
 *
 @author: Thinh Tran
 @date: May 10, 2022

 * Copyright (c) 2017 - 2019, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
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
#include "ble_app.h"
#include "ble_service.h"
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

#define DEVICE_NAME             "BleCentral"                    /**< Name of device. Will be included in the advertising data. */

//#define MANUFACTURER_NAME       "I-SYST inc."                    /**< Manufacturer. Will be passed to Device Information Service. */
//#define MODEL_NAME              "IMM-NRF5x"                      /**< Model number. Will be passed to Device Information Service. */
//#define MANUFACTURER_ID         ISYST_BLUETOOTH_ID               /**< Manufacturer ID, part of System ID. Will be passed to Device Information Service. */
//#define ORG_UNIQUE_ID           ISYST_BLUETOOTH_ID               /**< Organizational Unique ID, part of System ID. Will be passed to Device Information Service. */
//
//#define MIN_CONN_INTERVAL       MSEC_TO_UNITS(10, UNIT_1_25_MS)  /**< Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units. */
//#define MAX_CONN_INTERVAL       MSEC_TO_UNITS(40, UNIT_1_25_MS)  /**< Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units. */
//
#define SCAN_INTERVAL           MSEC_TO_UNITS(1000, UNIT_0_625_MS)		/**< Determines scan interval in units of 0.625 millisecond. */
#define SCAN_WINDOW             MSEC_TO_UNITS(100, UNIT_0_625_MS)       /**< Determines scan window in units of 0.625 millisecond. */
#define SCAN_TIMEOUT            0                                 		/**< Timout when scanning. 0x0000 disables timeout. */

// BLE
#define BLE_BRIDGE_NAME			"BlueWizard840"//"UartBleBridge"

/**
 * @brief Enable power USB detection
 *
 * Configure if example supports USB port connection
 */
#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

#define READ_SIZE (PACKET_SIZE / 2)//1

///////////////////////////////////////////////////

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event);
static void usbd_user_ev_handler(app_usbd_event_type_t event);

void HardwareInit();
int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void UartRxSchedHandler(void * p_event_data, uint16_t event_size);

void BleAppInitUserData();
void BleDevDiscovered(BLEPERIPH_DEV *pDev);
void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt);
void BleTxSchedHandler(void * p_event_data, uint16_t event_size);

void UsbInit();
void UsbRxSchedHandler(void * p_event_data, uint16_t event_size);


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
alignas(4) uint8_t g_UsbTxBuff[PACKET_SIZE];//NRF_DRV_USBD_EPSIZE

alignas(4) uint8_t g_UsbRxFifoMem[USBFIFOSIZE];
HCFIFO g_UsbRxFifo;

uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456"};
uint8_t g_extern_usbd_product_string[12 + 1] = { "Test" };

volatile int g_frameCnt = 0;

std::atomic<int> g_UsbRxBuffLen(0);
std::atomic<int> g_Usb2ExtBuffLen(0);

volatile int g_dropCnt = 0;


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

const BLEAPP_CFG s_BleAppCfg = {
	{ // Clock config nrf_clock_lf_cfg_t
#ifdef IMM_NRF51822
		NRF_CLOCK_LF_SRC_RC,	// Source RC
		1, 1, 0
#else
		NRF_CLOCK_LF_SRC_XTAL,	// Source 32KHz XTAL
		0, 0, NRF_CLOCK_LF_ACCURACY_20_PPM
#endif

	},
	1,//1, 						// Number of central link
	0, 							// Number of peripheral link
	BLEAPP_MODE_APPSCHED,   	// Use scheduler
	DEVICE_NAME,                 // Device name
	ISYST_BLUETOOTH_ID,     	// PnP Bluetooth/USB vendor id
	1,                      	// PnP Product ID
	0,							// Pnp prod version
	false,						// Enable device information service (DIS)
	NULL,//&s_UartBleDevDesc,
	NULL,//g_ManData,           // Manufacture specific data to advertise
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
	BLUEIO_CONNECT_LED_PORT,    // Led port nuber
	BLUEIO_CONNECT_LED_PIN,     // Led pin number
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
		.ServUid = s_UartBleSrvAdvUuid,
};

BLEPERIPH_DEV g_ConnectedDev = {
	.ConnHdl = BLE_CONN_HANDLE_INVALID,
};

uint16_t g_BleTxCharHdl = BLE_CONN_HANDLE_INVALID;
uint16_t g_BleRxCharHdl = BLE_CONN_HANDLE_INVALID;

void BleDevDiscovered(BLEPERIPH_DEV *pDev)
{
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
    int idx = BleDevFindService(pDev, BLUEIO_UUID_UART_SERVICE);

    if (idx != -1)
    {
    	g_Uart.printf("Found!\r\n");
    	// Rx characteristic
    	int dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_RX_CHAR);
    	g_Uart.printf("Find UART_RX_CHAR idx = 0x%x (%d)...", idx, idx);
    	if (dcharidx >= 0 && pDev->Services[idx].charateristics[dcharidx].characteristic.char_props.notify)
    	{
    		// Enable Notify
        	//g_Uart.printf("Enable notify\r\n");
        	BleAppEnableNotify(pDev->ConnHdl, pDev->Services[idx].charateristics[dcharidx].cccd_handle);
        	g_BleRxCharHdl = pDev->Services[idx].charateristics[dcharidx].characteristic.handle_value;
        	g_Uart.printf("Found!\r\n");
    	}
    	else
		{
			g_Uart.printf("Not Found!\r\n");
		}

    	// Tx characteristic
    	dcharidx = BleDevFindCharacteristic(pDev, idx, BLUEIO_UUID_UART_TX_CHAR);
    	g_Uart.printf("Find UART_TX_CHAR idx = 0x%x (%d) ...", idx, idx);
    	if (dcharidx >= 0)
    	{
    		g_BleTxCharHdl = pDev->Services[idx].charateristics[dcharidx].characteristic.handle_value;
    		g_Uart.printf("Found!\r\n");
    	}
    	else
    	{
    		g_Uart.printf("Not Found!\r\n");
    	}
    }
    else
    {
    	g_Uart.printf("Not Found!\r\n");
    }

}

void BleCentralEvtUserHandler(ble_evt_t * p_ble_evt)
{
    ret_code_t err_code;
    const ble_gap_evt_t * p_gap_evt = &p_ble_evt->evt.gap_evt;
    const ble_common_evt_t *p_common_evt = &p_ble_evt->evt.common_evt;
    const ble_gattc_evt_t *p_gattc_evt = &p_ble_evt->evt.gattc_evt;
    uint8_t addr[6] = { 0xda, 0x02, 0xe8, 0xfe, 0xac, 0xd1};

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

				// Find device by name
				if (ble_advdata_name_find(p_adv_report->data.p_data, p_adv_report->data.len, BLE_BRIDGE_NAME))
				{
					err_code = BleAppConnect((ble_gap_addr_t *)&p_adv_report->peer_addr, &s_ConnParams);
					msDelay(100);
				}
				else
				{
					BleAppScan();
				}
			}
			break;
        case BLE_GAP_EVT_TIMEOUT:
        	{
        	    ble_gap_evt_timeout_t const * p_timeout = &p_gap_evt->params.timeout;
        	    if (p_timeout->src == BLE_GAP_TIMEOUT_SRC_SCAN)
        	    {
        	    	// Scan timeout
        	    }
        	}
        	break;
        case BLE_GAP_EVT_SCAN_REQ_REPORT:
        	{
        	    ble_gap_evt_scan_req_report_t const * p_req_report = &p_gap_evt->params.scan_req_report;
        	}
        	break;
        case BLE_GATTC_EVT_HVX:
        	IOPinToggle(LED_RED_PORT, LED_RED_PIN);
        	if (p_ble_evt->evt.gattc_evt.params.hvx.handle == g_BleRxCharHdl)
        	{
        		//uint16_t lenTx = p_ble_evt->evt.gattc_evt.params.hvx.len;
        		//memcpy(&g_UsbTxBuff, &p_ble_evt->evt.gattc_evt.params.hvx.data, lenTx);
        		//app_usbd_cdc_acm_write(&m_app_cdc_acm, g_UsbTxBuff, lenTx);
        		g_Uart.Tx(p_ble_evt->evt.gattc_evt.params.hvx.data,
        				p_ble_evt->evt.gattc_evt.params.hvx.len);

        		app_usbd_cdc_acm_write(&m_app_cdc_acm,
        				&p_ble_evt->evt.gattc_evt.params.hvx.data,
						p_ble_evt->evt.gattc_evt.params.hvx.len);
        	}
        	IOPinToggle(LED_RED_PORT, LED_RED_PIN);
        	break;
  }
}


/********************************************************************************/
//static uint8_t g_extern_usbd_product_string[] = APP_USBD_STRING_DESC("Test");
//static uint8_t g_extern_usbd_serial_number[] = { "12345" };
/**
 * @brief User event handler @ref app_usbd_cdc_acm_user_ev_handler_t (headphones)
 * */
static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    app_usbd_cdc_acm_t const * p_cdc_acm = app_usbd_cdc_acm_class_get(p_inst);
    //static uint16_t dataLen;

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
        {
        	IOPinSet(LED_GREEN_PORT, LED_GREEN_PIN);
            /*Setup first transfer*/
            ret_code_t ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm,
                                                   g_UsbRxBuff,
                                                   READ_SIZE);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
        	IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);
            break;
        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
        {
        	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
            break;
        }
        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
        {
        	//app_sched_event_put(NULL, 0, UsbRxSchedHandler);
        	ret_code_t ret;
			int len;
			uint8_t *p = NULL;

			ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, g_UsbRxBuff, PACKET_SIZE);
			g_Uart.printf("ret = 0x%x\r\n", ret);

			uint16_t nUsbByteRead = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
			g_Uart.printf("nUsbByteRead = %d\r\n", nUsbByteRead);
			len = min(nUsbByteRead, PACKET_SIZE);

//			if (ret == NRF_SUCCESS)
			{


				// Cfifo buffer has data, so schedule BleTxSchedHandler
				// for sending data via BLE
				// g_Uart.printf("User buffer is ready\r\n");
				g_UsbRxBuffLen += len;

				// g_UsbRxBuff to Cfifo buff
				int l;
				while (len > 0)
				{
					l = len;
					p = CFifoPutMultiple(g_UsbRxFifo, &l);
					if (p == NULL)
					{
						g_dropCnt++;
						break;
					}

//					g_Uart.printf("len = %d| l = %d\r\n", len, l);
					memcpy(p, g_UsbRxBuff, l);
					len -= l;
				}

				g_Uart.printf("dropt count = %d \r\n", g_dropCnt);
				app_sched_event_put(NULL, 0, BleTxSchedHandler);


//				if (l > 0)
//				{
//					if (l == g_UsbRxBuffLen)
//					{
//						g_UsbRxBuffLen = 0;
//					}
//					else
//					{
//						memcpy(g_UsbRxBuff, &g_UsbRxBuff[l], g_UsbRxBuffLen - l);
//						g_UsbRxBuffLen -= l;
//
//					}
//
//					app_sched_event_put(NULL, 0, BleTxSchedHandler);
//				}

			}

            break;
        }
        default:
            break;
    }
}


//void UsbRxSchedHandler(void * p_event_data, uint16_t event_size)
//{
//	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
//
//	ret_code_t ret;
//	int len;
//	uint8_t *p = NULL;
//
//	uint16_t nUsbByteRead = app_usbd_cdc_acm_rx_size(&m_app_cdc_acm);
//	g_Uart.printf("nUsbByteRead = %d\r\n", nUsbByteRead);
//	len = min(nUsbByteRead, PACKET_SIZE);
//
//
//	ret = app_usbd_cdc_acm_read_any(&m_app_cdc_acm, &g_UsbRxBuff[g_UsbRxBuff], len);
//	g_Uart.printf("ret = 0x%x\r\n", ret);
//
//	if (ret != NRF_SUCCESS)
//	{
//		g_Uart.printf("schedule UsbRxSchedHandler\r\n");
//		app_sched_event_put(NULL, 0, UsbRxSchedHandler);
//	}
//	else
//	{
//		g_Uart.printf("app_usbd_cdc_acm_read_any: NRF_SUCCESS\r\n");
//		//p = CFifoPutMultiple(g_UsbRxFifo, &len);
//		len = CFifoPush(g_UsbRxFifo, g_UsbRxBuff, len);
//		if (len < nUsbByteRead)
//		{
//			g_Uart.printf("len < g_UsbRxFifo\r\n");
//		}
//
//
//	}
//
//	// Schedule BleTxSchedHandler for sending data via BLE
//	app_sched_event_put(NULL, 0, BleTxSchedHandler);
//
//
//
//
////	// Data exists in USB's internal buffer --> Transfer it to g_UsbRxBuff
////	while (nUsbByteRead > 0)
////	{
////		len = min(nUsbByteRead, PACKET_SIZE);
////		// Prepare Fifo buffer
////		p = CFifoPutMultiple(g_UsbRxFifo, &len);
////		if (p == NULL)
////		{
////			break;
////		}
////
////		// Transfer USB'internal buff -> Cfifo buff
////		ret = app_usbd_cdc_acm_read_any(p_cdc_acm, p, len);
////		nUsbByteRead -= len;
////
////		g_Uart.printf("updated nUsbByteRead = %d\r\n", nUsbByteRead);
////	}
//
//	// Schedule BleTxSchedHandler for sending data via BLE
//
//
//	IOPinToggle(LED_GREEN_PORT, LED_GREEN_PIN);
//
//}


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




static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SUSPEND:
        	IOPinClear(LED_RED_PORT, LED_RED_PIN);
            break;
        case APP_USBD_EVT_DRV_RESUME:
        	IOPinSet(LED_RED_PORT, LED_RED_PIN);
            break;
        case APP_USBD_EVT_STARTED:
        	IOPinSet(LED_RED_PORT, LED_RED_PIN);
            break;
        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            IOPinSet(LED_RED_PORT, LED_RED_PIN);
            IOPinSet(LED_GREEN_PORT, LED_GREEN_PIN);
            IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
            break;
        case APP_USBD_EVT_POWER_DETECTED:
        	//g_Uart.printf("APP_USBD_EVT_POWER_DETECTED\r\n");
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            IOPinToggle(LED_RED_PORT, LED_RED_PIN);
            msDelay(250);
            IOPinToggle(LED_RED_PORT, LED_RED_PIN);
            break;
        case APP_USBD_EVT_POWER_REMOVED:
        	//g_Uart.printf("APP_USBD_EVT_POWER_REMOVED\r\n");
            app_usbd_stop();
            IOPinClear(LED_RED_PORT, LED_RED_PIN);
            break;
        case APP_USBD_EVT_POWER_READY:
        	//g_Uart.printf("APP_USBD_EVT_POWER_READY\r\n");
            app_usbd_start();
            IOPinSet(LED_RED_PORT, LED_RED_PIN);
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
	ret = nrf_drv_clock_init();

	// Uart
	g_Uart.Init(g_UartCfg);
	g_Uart.printf("Clock init...%s\r\n", (ret ? "Failed!" : "Done!"));
	g_Uart.printf("UART Configuration: %d, %d, %d\r\n",
			g_UartCfg.Rate, g_UartCfg.FlowControl, g_UartCfg.Parity);
	msDelay(100);

	// LEDs
	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinClear(LED_RED_PORT, LED_RED_PIN);
	IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);

	//CFifoInit(pMemBlk, TotalMemSize, BlkSize, bBlocking)
	g_UsbRxFifo = CFifoInit(g_UsbRxFifoMem, USBFIFOSIZE, 1, true);

}

void UsbInit()
{
	ret_code_t ret;

	// Init USB
	g_Uart.printf("Init USB...");
	ret = app_usbd_init(&usbd_config);
	g_Uart.printf("%s\r\n", (ret ? "Failed" : "Done!"));

	//USBD serial number generator
	app_usbd_serial_num_generate();

	// Append USB class instance
	g_Uart.printf("USBD class append...");
	const app_usbd_class_inst_t *class_cdc_acm = app_usbd_cdc_acm_class_inst_get(&m_app_cdc_acm);
	ret = app_usbd_class_append(class_cdc_acm);
	g_Uart.printf("%s\r\n", (ret ? "Failed" : "Done!"));

	// Check power
	if (USBD_POWER_DETECTION)
	{
		g_Uart.printf("USB power detection...");
		ret = app_usbd_power_events_enable();
		//APP_ERROR_CHECK(ret);
		g_Uart.printf("%s\r\n", (ret ? "Failed" : "Done!"));
	}
	else
	{
		g_Uart.printf("No USB power detection enabled\r\nStarting USB now");
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
			BleAppWrite(g_ConnectedDev.ConnHdl, g_BleTxCharHdl, buff, l);
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
	ret_code_t ret;

	HardwareInit();

    UsbInit();// Should be called before BLE initialization
    g_Uart.printf("USB CDC ACM is on\r\n");
    msDelay(500);

    // BLE
    g_Uart.printf("Init BLE...");
    BleAppInit((const BLEAPP_CFG *)&s_BleAppCfg, true);
    g_Uart.printf("Done!\r\n");
    msDelay(250);

    g_Uart.printf("All initializations are done. Start the main tasks!\r\n");
    // Register the non-GATT BLE services and their characteristics
	BleAppScanInit((BleAppScanCfg_t *)&s_bleScanInitCfg);
	BleAppScan();
	BleAppRun();

	return 0;
}
