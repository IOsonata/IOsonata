/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
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
#include <stdint.h>
#include <stdbool.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd.h"
#include "app_usbd_serial_num.h"
#include "app_timer.h"

#include "boards.h"
#include "bsp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "prbs.h"
#include "usbd_cdc_intrf.h"

/*
 * This SDK example links external sources individually through its Eclipse
 * project. Keep the evaluation adapter local without changing project metadata
 * by compiling its implementation in this translation unit.
 */
#include "usbd_cdc_intrf.c"

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief Nordic CDC ACM through the IOsonata DeviceIntrf byte-stream path
 *
 * This benchmark keeps Nordic SDK 17.1.0 as the USB stack and replaces the
 * direct 64-byte producer with the old IOsonata DeviceIntrf + CFifo interface.
 * The application still transmits one PRBS byte per DeviceIntrfTx call.
 */

#define LED_USB_RESUME			(BSP_BOARD_LED_0)

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION	true
#endif

#define CDC_RXFIFO_MEMSIZE		CFIFO_MEMSIZE(256)
#define CDC_TXFIFO_MEMSIZE		CFIFO_MEMSIZE(2048)

static uint8_t s_CdcRxFifoMem[CDC_RXFIFO_MEMSIZE] __attribute__((aligned(4)));
static uint8_t s_CdcTxFifoMem[CDC_TXFIFO_MEMSIZE] __attribute__((aligned(4)));
static UsbdCdcDevIntrf_t s_CdcIntrf;

static const UsbdCdcIntrfCfg_t s_CdcCfg = {
	.bBlocking = true,
	.RxFifoMemSize = sizeof(s_CdcRxFifoMem),
	.pRxFifoMem = s_CdcRxFifoMem,
	.TxFifoMemSize = sizeof(s_CdcTxFifoMem),
	.pTxFifoMem = s_CdcTxFifoMem,
	.EvtCB = NULL,
};

uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456"};
uint8_t g_extern_usbd_product_string[12 + 1] = { "Test" };

static void UsbdUserEvtHandler(app_usbd_event_type_t Event)
{
	switch (Event)
	{
		case APP_USBD_EVT_DRV_SUSPEND:
			bsp_board_led_off(LED_USB_RESUME);
			break;

		case APP_USBD_EVT_DRV_RESUME:
			bsp_board_led_on(LED_USB_RESUME);
			break;

		case APP_USBD_EVT_STARTED:
			break;

		case APP_USBD_EVT_STOPPED:
			app_usbd_disable();
			bsp_board_leds_off();
			break;

		case APP_USBD_EVT_POWER_DETECTED:
			NRF_LOG_INFO("USB power detected");
			if (!nrf_drv_usbd_is_enabled())
			{
				app_usbd_enable();
			}
			break;

		case APP_USBD_EVT_POWER_REMOVED:
			NRF_LOG_INFO("USB power removed");
			app_usbd_stop();
			break;

		case APP_USBD_EVT_POWER_READY:
			NRF_LOG_INFO("USB ready");
			app_usbd_start();
			break;

		default:
			break;
	}
}

int main(void)
{
	ret_code_t ret;
	uint8_t data = 0xff;
	bool wasOpen = false;

	static const app_usbd_config_t usbd_config = {
		.ev_state_proc = UsbdUserEvtHandler
	};

	ret = NRF_LOG_INIT(NULL);
	APP_ERROR_CHECK(ret);

	ret = nrf_drv_clock_init();
	APP_ERROR_CHECK(ret);

	nrf_drv_clock_lfclk_request(NULL);
	while (!nrf_drv_clock_lfclk_is_running())
	{
		/* Just waiting */
	}

	ret = app_timer_init();
	APP_ERROR_CHECK(ret);

	bsp_board_init(BSP_INIT_LEDS);
	app_usbd_serial_num_generate();

	ret = app_usbd_init(&usbd_config);
	APP_ERROR_CHECK(ret);

	if (!UsbdCdcIntrfInit(&s_CdcIntrf, &s_CdcCfg))
	{
		return -1;
	}

	NRF_LOG_INFO("Nordic CDC through IOsonata DeviceIntrf started.");

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
			/* Drain queued USB events when configured. */
		}
#endif

		if (!UsbdCdcIntrfIsPortOpen(&s_CdcIntrf))
		{
			wasOpen = false;
			UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
			__WFE();
			continue;
		}

		if (!wasOpen)
		{
			data = 0xff;
			wasOpen = true;
		}

		/*
		 * Match the native IOsonata benchmark: one byte per call. Advance the
		 * PRBS only after the blocking FIFO accepts that byte.
		 */
		if (DeviceIntrfTx(&s_CdcIntrf.DevIntrf, 0, &data, 1) == 1)
		{
			data = Prbs8(data);
		}
	}
}

/** @} */
