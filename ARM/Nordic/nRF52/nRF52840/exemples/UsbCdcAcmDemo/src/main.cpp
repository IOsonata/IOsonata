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

#include "nrf.h"
#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
//#include "app_usbd_cdc_acm_internal.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "nrf_cli.h"
#include "nrf_cli_cdc_acm.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"

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

#include "cfifo.h"


#if NRF_CLI_ENABLED
/**
 * @brief Macro for defining a command line interface instance.
 *
 * @param[in] name              Instance name.
 * @param[in] cli_prefix        CLI prefix string.
 * @param[in] p_transport_iface Pointer to the transport interface.
 * @param[in] newline_ch        Deprecated parameter, not used any more. Any uint8_t value can be used.
 * @param[in] log_queue_size    Logger processing queue size.
 */
#define NRF_CLI_DEFCPP(name, cli_prefix, p_transport_iface, newline_ch, log_queue_size)    \
	    extern nrf_cli_t const name;                                            \
        static nrf_cli_ctx_t CONCAT_2(name, _ctx);                              \
        NRF_FPRINTF_DEF(CONCAT_2(name, _fprintf_ctx),                           \
                        &name,                                                  \
                        CONCAT_2(name, _ctx).printf_buff,                       \
                        NRF_CLI_PRINTF_BUFF_SIZE,                               \
                        false,                                                  \
                        nrf_cli_print_stream);                                  \
        NRF_LOG_BACKEND_CLI_DEF(CONCAT_2(name, _log_backend), log_queue_size);  \
        NRF_CLI_HISTORY_MEM_OBJ(name);                                          \
        /*lint -save -e31*/                                                     \
        nrf_cli_t const name = {                                         		\
            .p_name = cli_prefix,                                               \
            .p_iface = p_transport_iface,                                       \
            .p_ctx = &CONCAT_2(name, _ctx),                                     \
            .p_log_backend = NRF_CLI_BACKEND_PTR(name),                         \
            .p_fprintf_ctx = &CONCAT_2(name, _fprintf_ctx),                     \
            .p_cmd_hist_mempool = NRF_CLI_MEMOBJ_PTR(name),                     \
        } /*lint -restore*/


//NRF_CLI_CDC_ACM_DEF(m_cli_cdc_acm_transport);
//#define NRF_CLI_CDC_ACM_DEF(_name_)
static nrf_cli_cdc_acm_internal_cb_t m_cli_cdc_acm_transport_cb;
static const nrf_cli_cdc_acm_internal_t m_cli_cdc_acm_transport = {
	.transport = {.p_api = &nrf_cli_cdc_acm_transport_api},
	.p_cb = &m_cli_cdc_acm_transport_cb,
};

NRF_CLI_DEFCPP(m_cli_cdc_acm,
            "usb_cli:~$ ",
            &m_cli_cdc_acm_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);
#endif

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief USBD CDC ACM example
 *
 */

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
/**
 * @brief User event handler.
 *
 * @param[in] p_inst    Class instance.
 * @param[in] event     User event.
 *
 */
//typedef void (*app_usbd_cdc_acm_user_ev_handler_t)(app_usbd_class_inst_t const *    p_inst,
//                                                 enum app_usbd_cdc_acm_user_event_e event);

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst, app_usbd_cdc_acm_user_event_t event);
static void usbd_user_ev_handler(app_usbd_event_type_t event);

void HardwareInit();
int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);
void UartRxSchedHandler(void * p_event_data, uint16_t event_size);


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
hCFifo_t g_UsbRxFifo;

uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456"};
uint8_t g_extern_usbd_product_string[15 + 1] = { "UsbCdcAcmDemo" };

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



int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
//	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
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


void HardwareInit()
{
	ret_code_t ret;

	ret = nrf_drv_clock_init();

	// Uart
	g_Uart.Init(g_UartCfg);

	// LEDs
	IOPinCfg(s_Leds, s_NbLeds);
	IOPinSet(LED_BLUE_PORT, LED_BLUE_PIN);
	IOPinClear(LED_RED_PORT, LED_RED_PIN);
	IOPinClear(LED_GREEN_PORT, LED_GREEN_PIN);

	g_UsbRxFifo = CFifoInit(g_UsbRxFifoMem, USBFIFOSIZE, 1, true);
}

int main(void)
{
	ret_code_t ret;

	HardwareInit();

    UsbInit();

    while (1)
    {
    	__WFE();
       // while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
    }

	return 0;
}
