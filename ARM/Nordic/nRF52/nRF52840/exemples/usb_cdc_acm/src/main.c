/**-------------------------------------------------------------------------
@example	usb_cdc_acm/main.c

@brief	Nordic nRF5 SDK USB CDC PRBS transmit performance comparison.

This benchmark uses the same PRBS producer and BYTE_MODE switch as
usb_cdc_prbs_tx.cpp and tinyusb_cdc_prbs_tx/main.cpp. The application writes
into the nRF5 SDK ring buffer while the CDC completion path drains queued data
in USB-sized chunks, matching the queued transmit model used by IOsonata.

@author	Nguyen Hoan Hoang
@date	Sep. 6, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdbool.h>
#include <stdint.h>

#include "nrf_drv_clock.h"
#include "nrf_drv_usbd.h"

#include "app_error.h"
#include "app_timer.h"
#include "app_usbd.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_core.h"
#include "app_usbd_serial_num.h"
#include "nrf_ringbuf.h"

#include "prbs.h"


#define BYTE_MODE

#define TEST_BUFSIZE			16
#define TX_RING_SIZE			2048

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION	true
#endif

#define CDC_ACM_COMM_INTERFACE	0
#define CDC_ACM_COMM_EPIN		NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE	1
#define CDC_ACM_DATA_EPIN		NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT		NRF_DRV_USBD_EPOUT1

static void CdcEvtHandler(app_usbd_class_inst_t const *pInst,
						  app_usbd_cdc_acm_user_event_t Event);
static void TxStart(void);

APP_USBD_CDC_ACM_GLOBAL_DEF(s_Cdc,
						CdcEvtHandler,
						CDC_ACM_COMM_INTERFACE,
						CDC_ACM_DATA_INTERFACE,
						CDC_ACM_COMM_EPIN,
						CDC_ACM_DATA_EPIN,
						CDC_ACM_DATA_EPOUT,
						APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

NRF_RINGBUF_DEF(s_TxRing, TX_RING_SIZE);

// These names are selected by the existing Nordic SDK string configuration.
uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456" };
uint8_t g_extern_usbd_product_string[12 + 1] = { "SDK PRBS Tx" };

static volatile bool s_PortOpen;
static volatile bool s_TxActive;
static size_t s_TxLength;

static void TxStart(void)
{
	uint8_t *pData;
	size_t len;
	ret_code_t ret;

	if (!s_PortOpen || s_TxActive)
	{
		return;
	}

	len = NRF_DRV_USBD_EPSIZE;
	ret = nrf_ringbuf_get(&s_TxRing, &pData, &len, true);
	if (ret != NRF_SUCCESS || len == 0)
	{
		return;
	}

	ret = app_usbd_cdc_acm_write(&s_Cdc, pData, len);
	if (ret == NRF_SUCCESS)
	{
		s_TxLength = len;
		s_TxActive = true;
	}
	else
	{
		// Release the ring read reservation without consuming data.
		(void)nrf_ringbuf_free(&s_TxRing, 0);
	}
}

static void CdcEvtHandler(app_usbd_class_inst_t const *pInst,
						  app_usbd_cdc_acm_user_event_t Event)
{
	(void)pInst;

	switch (Event)
	{
		case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
			nrf_ringbuf_init(&s_TxRing);
			s_TxLength = 0;
			s_TxActive = false;
			s_PortOpen = true;
			break;

		case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
			s_PortOpen = false;
			s_TxLength = 0;
			s_TxActive = false;
			break;

		case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
			if (s_TxActive)
			{
				(void)nrf_ringbuf_free(&s_TxRing, s_TxLength);
				s_TxLength = 0;
				s_TxActive = false;
			}
			TxStart();
			break;

		default:
			break;
	}
}

static void UsbEvtHandler(app_usbd_event_type_t Event)
{
	switch (Event)
	{
		case APP_USBD_EVT_STOPPED:
			s_PortOpen = false;
			s_TxLength = 0;
			s_TxActive = false;
			app_usbd_disable();
			break;

		case APP_USBD_EVT_POWER_DETECTED:
			if (!nrf_drv_usbd_is_enabled())
			{
				app_usbd_enable();
			}
			break;

		case APP_USBD_EVT_POWER_REMOVED:
			s_PortOpen = false;
			s_TxLength = 0;
			s_TxActive = false;
			app_usbd_stop();
			break;

		case APP_USBD_EVT_POWER_READY:
			app_usbd_start();
			break;

		default:
			break;
	}
}

int main(void)
{
	ret_code_t ret;
	uint8_t d = 0xff;
#ifndef BYTE_MODE
	uint8_t buff[TEST_BUFSIZE];
#endif

	static const app_usbd_config_t usbCfg = {
		.ev_state_proc = UsbEvtHandler,
	};

	ret = nrf_drv_clock_init();
	APP_ERROR_CHECK(ret);

	nrf_drv_clock_lfclk_request(NULL);
	while (!nrf_drv_clock_lfclk_is_running())
	{
	}

	ret = app_timer_init();
	APP_ERROR_CHECK(ret);

	nrf_ringbuf_init(&s_TxRing);
	app_usbd_serial_num_generate();

	ret = app_usbd_init(&usbCfg);
	APP_ERROR_CHECK(ret);

	app_usbd_class_inst_t const *pCdc =
		app_usbd_cdc_acm_class_inst_get(&s_Cdc);
	ret = app_usbd_class_append(pCdc);
	APP_ERROR_CHECK(ret);

	if (USBD_POWER_DETECTION)
	{
		ret = app_usbd_power_events_enable();
		APP_ERROR_CHECK(ret);
	}
	else
	{
		app_usbd_enable();
		app_usbd_start();
	}

	while (true)
	{
#if APP_USBD_CONFIG_EVENT_QUEUE_ENABLE
		while (app_usbd_event_queue_process())
		{
		}
#endif

		if (!s_PortOpen)
		{
			continue;
		}

#ifdef BYTE_MODE
		// Demo transfer byte by byte. The value advances only when the byte
		// was accepted into the SDK ring buffer. If the ring is full, retry
		// this same byte while TX completions make room.
		size_t len = 1;
		ret = nrf_ringbuf_cpy_put(&s_TxRing, &d, &len);
		if (ret == NRF_SUCCESS && len > 0)
		{
			d = Prbs8(d);
		}
		TxStart();
#else
		// Demo transfer buffer
		for (int i = 0; i < TEST_BUFSIZE; i++)
		{
			d = Prbs8(d);
			buff[i] = d;
		}

		int len = TEST_BUFSIZE;
		uint8_t *p = buff;

		while (len > 0)
		{
			size_t l = (size_t)len;
			ret = nrf_ringbuf_cpy_put(&s_TxRing, p, &l);
			if (ret == NRF_SUCCESS)
			{
				len -= (int)l;
				p += l;
			}

			TxStart();

			if (!s_PortOpen)
			{
				break;
			}
		}
#endif
	}

	return 0;
}
