/**-------------------------------------------------------------------------
@example	usb_cdc_acm/main.c

@brief	Nordic nRF5 SDK USB CDC PRBS transmit performance comparison.

This benchmark uses the same PRBS producer and BYTE_MODE switch as
usb_cdc_prbs_tx.cpp and tinyusb_cdc_prbs_tx/main.cpp. Only the USB stack calls
differ, so byte mode and buffered mode exercise the same application workload.

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

#include "prbs.h"


#define BYTE_MODE

#define TEST_BUFSIZE			16

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

APP_USBD_CDC_ACM_GLOBAL_DEF(s_Cdc,
						CdcEvtHandler,
						CDC_ACM_COMM_INTERFACE,
						CDC_ACM_DATA_INTERFACE,
						CDC_ACM_COMM_EPIN,
						CDC_ACM_DATA_EPIN,
						CDC_ACM_DATA_EPOUT,
						APP_USBD_CDC_COMM_PROTOCOL_AT_V250);

// These names are selected by the existing Nordic SDK string configuration.
uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456" };
uint8_t g_extern_usbd_product_string[12 + 1] = { "SDK PRBS Tx" };

static volatile bool s_PortOpen;
static volatile bool s_TxReady;

static void CdcEvtHandler(app_usbd_class_inst_t const *pInst,
						  app_usbd_cdc_acm_user_event_t Event)
{
	(void)pInst;

	switch (Event)
	{
		case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
			s_PortOpen = true;
			s_TxReady = true;
			break;

		case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
			s_PortOpen = false;
			s_TxReady = false;
			break;

		case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
			s_TxReady = true;
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
			s_TxReady = false;
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
			s_TxReady = false;
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
#ifdef BYTE_MODE
	// app_usbd_cdc_acm_write is asynchronous. Keep the submitted byte in a
	// separate object so advancing d cannot modify the in-flight DMA buffer.
	uint8_t txByte = d;
#else
	uint8_t buff[TEST_BUFSIZE];
	bool bufferPending = false;
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

		if (!s_PortOpen || !s_TxReady)
		{
			continue;
		}

#ifdef BYTE_MODE
		// Demo transfer byte by byte. The value advances only when the octet
		// was accepted. If the endpoint is busy, retry this same byte after the
		// TX completion callback makes the interface ready again.
		txByte = d;
		s_TxReady = false;
		ret = app_usbd_cdc_acm_write(&s_Cdc, &txByte, 1);
		if (ret == NRF_SUCCESS)
		{
			d = Prbs8(d);
		}
		else
		{
			s_TxReady = true;
		}
#else
		// Demo transfer buffer
		if (!bufferPending)
		{
			for (int i = 0; i < TEST_BUFSIZE; i++)
			{
				d = Prbs8(d);
				buff[i] = d;
			}
			bufferPending = true;
		}

		s_TxReady = false;
		ret = app_usbd_cdc_acm_write(&s_Cdc, buff, TEST_BUFSIZE);
		if (ret == NRF_SUCCESS)
		{
			// Do not refill buff until TX_DONE sets s_TxReady again.
			bufferPending = false;
		}
		else
		{
			s_TxReady = true;
		}
#endif
	}

	return 0;
}
