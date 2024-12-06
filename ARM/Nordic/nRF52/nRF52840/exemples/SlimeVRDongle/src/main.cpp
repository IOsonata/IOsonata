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

#pragma pack(push, 1)
typedef struct __App_Data {
	uint8_t Cs;		// Checksum
	int8_t NbTracker;
	uint64_t TrackerAddr[MAX_TRACKERS];
} AppData_t;
#pragma pack(pop)

__attribute__ ((section(".Version"), used))
const AppInfo_t g_AppInfo = {
	DEVICE_NAME, {FIRMWARE_VERSION, 0, BUILDN},
	{'I', 'O', 's', 'o', 'n', 'a', 't', 'a', 'S', 'l', 'i', 0x55, 0xA5, 0x5A, 0xA5, 0x5A},
};

alignas(4) AppData_t g_AppData = { 0, -1, };

//uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456"};
uint8_t g_extern_usbd_product_string[40 + 1] = { "SlimeNRF Receiver BLYST840 Dongle" };

#if NRF_CLI_ENABLED
//NRF_CLI_CDC_ACM_DEF(m_cli_cdc_acm_transport);
//#define NRF_CLI_CDC_ACM_DEF(_name_)
static nrf_cli_cdc_acm_internal_cb_t m_cli_cdc_acm_transport_cb;
static const nrf_cli_cdc_acm_internal_t m_cli_cdc_acm_transport = {
	.transport = {.p_api = &nrf_cli_cdc_acm_transport_api},
	.p_cb = &m_cli_cdc_acm_transport_cb,
};

NRF_CLI_DEF(m_cli_cdc_acm,
            "Slime:~$ ",
            &m_cli_cdc_acm_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);
#endif

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
volatile bool g_FdsCleaned = false;

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

        case FDS_EVT_GC:
        	g_FdsCleaned = true;
        	break;

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

    	if (rc == FDS_ERR_NO_SPACE_IN_FLASH)
    	{
    		// Remove deleted record to make space
    		g_FdsCleaned = false;
    		fds_gc();

    		while (g_FdsCleaned == false) __WFE();

        	rc = fds_record_update(&desc, &g_AppDataRecord);
        	APP_ERROR_CHECK(rc);
    	}
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



void HardwareInit()
{
	IOPinConfig(BUT_PORT, BUT_PIN, BUT_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);

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

    UsbInit();
}

static void nrfx_clock_irq_handler(nrfx_clock_evt_type_t evt)
{
    if (evt == NRFX_CLOCK_EVT_HFCLK_STARTED)
    {
    }
    if (evt == NRFX_CLOCK_EVT_LFCLK_STARTED)
    {
    }
}

void clocks_start( void )
{
#if 0
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;

    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
#else
    ret_code_t err_code = nrfx_clock_init(nrfx_clock_irq_handler);
    nrfx_clock_hfclk_start();
    while (nrfx_clock_hfclk_is_running() == false);
#endif
}

int main(void)
{
	static bool pairmode = true;
    ret_code_t err_code;

	ret_code_t ret;

	clocks_start();

	// Update default checksum
	uint8_t *p = (uint8_t*)&g_AppData;

	g_AppData.Cs = 0;

	for (int i = 0; i < sizeof(AppData_t); i++, p++)
	{
		g_AppData.Cs += *p;
	}

	g_AppData.Cs = 0 - g_AppData.Cs;

    ret = nrf_cli_init(&m_cli_cdc_acm, nullptr, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);

	HardwareInit();

    msDelay(1000);

    nrf_cli_print(&m_cli_cdc_acm, "\nSlimeNRF Receiver BLYST840 Dongle");

    ret = nrf_cli_start(&m_cli_cdc_acm);
    APP_ERROR_CHECK(ret);

    while (true)
    {
    	EsbSetAddr(pairmode);


        err_code = esb_init();
       // APP_ERROR_CHECK(err_code);

        //bsp_board_leds_init();
        err_code = nrf_esb_start_rx();
        //APP_ERROR_CHECK(err_code);

        do {
        	nrf_cli_process(&m_cli_cdc_acm);
        	__WFE();
        } while (IOPinRead(BUT_PORT, BUT_PIN) == false);

        pairmode != pairmode;
    }
}
/*lint -restore */
