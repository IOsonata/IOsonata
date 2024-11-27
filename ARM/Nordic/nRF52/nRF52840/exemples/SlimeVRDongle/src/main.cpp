/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "sdk_common.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "fds.h"
#include "nrf_cli.h"
#include "nrf_drv_clock.h"
#include "nrf_drv_power.h"
//#include "nrf_cli_uart.h"
#include "nrf_cli_cdc_acm.h"

#include "istddef.h"
#include "idelay.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "blueio_board.h"
#include "coredev/uart.h"

#include "SlimeVRDongle.h"
#include "board.h"
#include "stddev.h"

#define FIRMWARE_VERSION	0
#define DEVICE_NAME			"SlimeVRDongle"

#define FDS_DATA_FILE     (0xF010)
#define FDS_DATA_REC_KEY  (0x7010)

typedef struct __App_Data {
	uint8_t NbTracker;
	uint64_t TrackerAddr[MAX_TRACKERS];
} AppData_t;

__attribute__ ((section(".Version"), used))
const AppInfo_t g_AppInfo = {
	DEVICE_NAME, {FIRMWARE_VERSION, 0, BUILDN},
	{'I', 'O', 's', 'o', 'n', 'a', 't', 'a', 'S', 'l', 'i', 0x55, 0xA5, 0x5A, 0xA5, 0x5A},
};

AppData_t g_AppData = { (uint8_t)-1, };

uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456"};
uint8_t g_extern_usbd_product_string[15 + 1] = { "SlimeVRDongle" };

alignas(4) static fds_record_t const g_AppDataRecord =
{
    .file_id           = FDS_DATA_FILE,
    .key               = FDS_DATA_REC_KEY,
    .data = {
		.p_data = &g_AppData,
    /* The length of a record is always expressed in 4-byte units (words). */
		.length_words = (sizeof(AppData_t) + 3) >> 2,
    }
};
volatile bool g_FdsInitialized = false;


#if 0//NRF_CLI_ENABLED
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


int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

const UARTCfg_t g_UartCfg = {
	0,
	s_UartPins,
	sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	1000000,
	8,
	UART_PARITY_NONE,
	1,	// Stop bit
	UART_FLWCTRL_HW,
	true,
	7,
	nRFUartEvthandler,
	true,
};

// UART object instance
UART g_Uart;

static void fds_evt_handler(fds_evt_t const * p_evt)
{
    switch (p_evt->id)
    {
        case FDS_EVT_INIT:
            if (p_evt->result == NRF_SUCCESS)
            {
            	g_FdsInitialized = true;
            }
            break;

        case FDS_EVT_WRITE:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
            }
        } break;

        case FDS_EVT_DEL_RECORD:
        {
            if (p_evt->result == NRF_SUCCESS)
            {
            }
        } break;

        default:
            break;
    }
}

void UpdateRecord()
{
    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    uint32_t rc = fds_record_find(FDS_DATA_FILE, FDS_DATA_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
    	rc = fds_record_update(&desc, &g_AppDataRecord);
    	APP_ERROR_CHECK(rc);
    }
}

uint16_t AddTracker(uint64_t Addr)
{
	uint16_t retid = g_AppData.NbTracker;

	for (int i = 0; i < g_AppData.NbTracker; i++) // Check if the device is already stored
	{
		if (Addr != 0 && g_AppData.TrackerAddr[i] == Addr)
		{
			//LOG_INF("Found device linked to id %d with address %012llX", i, found_addr);
			return i;
		}
	}

	if (g_AppData.NbTracker < MAX_TRACKERS)
	{
		g_AppData.TrackerAddr[g_AppData.NbTracker] = Addr;

		g_AppData.NbTracker++;
	}

	UpdateRecord();

	return retid;
}


/*lint -save -esym(40, BUTTON_1) -esym(40, BUTTON_2) -esym(40, BUTTON_3) -esym(40, BUTTON_4) -esym(40, LED_1) -esym(40, LED_2) -esym(40, LED_3) -esym(40, LED_4) */
#if 0
void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
    switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
         //   NRF_LOG_DEBUG("TX SUCCESS EVENT\r\n");
            break;
        case NRF_ESB_EVENT_TX_FAILED:
        //    NRF_LOG_DEBUG("TX FAILED EVENT\r\n");
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
         //   NRF_LOG_DEBUG("RX RECEIVED EVENT\r\n");
            while (nrf_esb_read_rx_payload(&rx_payload) == NRF_SUCCESS)
            {
                if (rx_payload.length > 0)
                {
                	IOPinSet(0, 23);
                	g_Uart.printf("Received : %d bytes\n", rx_payload.length);
                	IOPinClear(0, 23);

                    //NRF_LOG_DEBUG("RX RECEIVED PAYLOAD\r\n");
                }
            }
            break;
    }
    NRF_GPIO->OUTCLR = 0xFUL << 12;
    NRF_GPIO->OUTSET = (p_event->tx_attempts & 0x0F) << 12;
}
#endif

void clocks_start( void )
{
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}
#if 0
uint32_t esb_init( void )
{
    uint32_t err_code;
    uint8_t base_addr_0[4] = {0xE6, 0xE6, 0xE6, 0xE6};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.payload_length           = 2;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PRX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = true;

    err_code = nrf_esb_init(&nrf_esb_config);

    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return err_code;
}
#endif

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[20];

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			//app_sched_event_put(NULL, 0, UartRxChedHandler);
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
	IOPinConfig(BUT_PORT, BUT_PIN, BUT_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

	printf("UART BLE Central Demo\r\n");

    (void) fds_register(fds_evt_handler);
    uint32_t rc = fds_init();
    APP_ERROR_CHECK(rc);

    while (g_FdsInitialized != true)
    {
        __WFE();
    }

    fds_record_desc_t desc = {0};
    fds_find_token_t  tok  = {0};

    rc = fds_record_find(FDS_DATA_FILE, FDS_DATA_REC_KEY, &desc, &tok);

    if (rc == NRF_SUCCESS)
    {
    	AppData_t data;

    	do {
			/* A config file is in flash. Let's update it. */
			fds_flash_record_t appdata = {0};

			/* Open the record and read its contents. */
			rc = fds_record_open(&desc, &appdata);
			APP_ERROR_CHECK(rc);

			memcpy(&data, appdata.p_data, sizeof(AppData_t));
			/* Close the record when done reading. */
			rc = fds_record_close(&desc);
			APP_ERROR_CHECK(rc);

			rc = fds_record_find(FDS_DATA_FILE, FDS_DATA_REC_KEY, &desc, &tok);
    	} while (rc == NRF_SUCCESS);

    	uint8_t *p = (uint8_t *)&data;
		uint8_t cs = 0;

		for (int i = 0; i < sizeof(AppData_t); i++, p++)
		{
			cs += *p;
		}

		if (cs == 0)
		{
			memcpy(&g_AppData, &data, sizeof(AppData_t));
		}
		else
		{
			UpdateRecord();
		}

    }
    else
    {
        rc = fds_record_write(&desc, &g_AppDataRecord);
        APP_ERROR_CHECK(rc);
    }

    init_cli();
    UsbInit();
}

int main(void)
{
	static bool pairmode = true;
    ret_code_t err_code;

	ret_code_t ret;

	ret = nrf_drv_clock_init();
    //clocks_start();

    HardwareInit();
//    cli_init();
  //  cli_start();

    //err_code = NRF_LOG_INIT(NULL);
    //APP_ERROR_CHECK(err_code);


#if 0
    err_code = esb_init();
   // APP_ERROR_CHECK(err_code);

    //bsp_board_leds_init();
    err_code = nrf_esb_start_rx();
    //APP_ERROR_CHECK(err_code);
#endif

    //NRF_LOG_DEBUG("Enhanced ShockBurst Transmitter Example running.\r\n");

    while (true)
    {
    	EsbSetAddr(pairmode);


        err_code = esb_init();
       // APP_ERROR_CHECK(err_code);

        //bsp_board_leds_init();
        err_code = nrf_esb_start_rx();
        //APP_ERROR_CHECK(err_code);

        do {
        	__WFE();
        } while (IOPinRead(BUT_PORT, BUT_PIN) == false);

        pairmode != pairmode;
    }
}
/*lint -restore */
