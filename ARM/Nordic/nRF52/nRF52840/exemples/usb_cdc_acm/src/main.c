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
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_usbd.h"
#include "nrf_drv_clock.h"

#include "app_error.h"
#include "app_util.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#include "app_usbd_serial_num.h"
#include "app_timer.h"

#include "boards.h"
#include "bsp.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "cfifo.h"
#include "prbs.h"
#include "usbd_cdc_intrf.h"

/**@file
 * @defgroup usbd_cdc_acm_example main.c
 * @{
 * @ingroup usbd_cdc_acm_example
 * @brief Nordic USBD CDC ACM PRBS transmit stream benchmark
 */

#define LED_USB_RESUME      (BSP_BOARD_LED_0)
#define LED_CDC_ACM_OPEN    (BSP_BOARD_LED_1)
#define LED_CDC_ACM_RX      (BSP_BOARD_LED_2)
#define LED_CDC_ACM_TX      (BSP_BOARD_LED_3)

#ifndef USBD_POWER_DETECTION
#define USBD_POWER_DETECTION true
#endif

/*
 * 1: direct 64-byte CFifo -> app_usbd_cdc_acm_write benchmark.
 * 0: byte-at-a-time DeviceIntrfTx -> UsbdCdcIntrf benchmark.
 *
 * This may also be selected from the compiler command line with
 * -DUSB_CDC_TEST_DIRECT=0 or -DUSB_CDC_TEST_DIRECT=1.
 */
#ifndef USB_CDC_TEST_DIRECT
#define USB_CDC_TEST_DIRECT 1
#endif

uint8_t g_extern_usbd_serial_number[12 + 1] = { "123456"};
uint8_t g_extern_usbd_product_string[12 + 1] = { "Test" };

#if USB_CDC_TEST_DIRECT

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event);

#define CDC_ACM_COMM_INTERFACE  0
#define CDC_ACM_COMM_EPIN       NRF_DRV_USBD_EPIN2

#define CDC_ACM_DATA_INTERFACE  1
#define CDC_ACM_DATA_EPIN       NRF_DRV_USBD_EPIN1
#define CDC_ACM_DATA_EPOUT      NRF_DRV_USBD_EPOUT1

APP_USBD_CDC_ACM_GLOBAL_DEF(m_test_cdc_acm,
                            cdc_acm_user_ev_handler,
                            CDC_ACM_COMM_INTERFACE,
                            CDC_ACM_DATA_INTERFACE,
                            CDC_ACM_COMM_EPIN,
                            CDC_ACM_DATA_EPIN,
                            CDC_ACM_DATA_EPOUT,
                            APP_USBD_CDC_COMM_PROTOCOL_AT_V250
);

#define TX_FIFO_MEMSIZE CFIFO_MEMSIZE(2048)

static uint8_t m_tx_fifo_mem[TX_FIFO_MEMSIZE] __attribute__((aligned(4)));
static hCFifo_t m_tx_fifo;
static volatile bool m_port_open;
static volatile bool m_tx_ready;
static uint8_t m_prbs = 0xff;

static uint8_t prbs8(uint8_t cur)
{
    uint8_t newbit = (((cur >> 6) ^ (cur >> 5)) & 1U);
    return (uint8_t)(((cur << 1) | newbit) & 0x7fU);
}

static void cdc_acm_user_ev_handler(app_usbd_class_inst_t const * p_inst,
                                    app_usbd_cdc_acm_user_event_t event)
{
    (void)p_inst;

    switch (event)
    {
        case APP_USBD_CDC_ACM_USER_EVT_PORT_OPEN:
            bsp_board_led_on(LED_CDC_ACM_OPEN);
            m_prbs = 0xff;
            CFifoFlush(m_tx_fifo);
            m_port_open = true;
            m_tx_ready = true;
            break;

        case APP_USBD_CDC_ACM_USER_EVT_PORT_CLOSE:
            m_port_open = false;
            m_tx_ready = false;
            bsp_board_led_off(LED_CDC_ACM_OPEN);
            break;

        case APP_USBD_CDC_ACM_USER_EVT_TX_DONE:
            m_tx_ready = true;
            bsp_board_led_invert(LED_CDC_ACM_TX);
            break;

        case APP_USBD_CDC_ACM_USER_EVT_RX_DONE:
            bsp_board_led_invert(LED_CDC_ACM_RX);
            break;

        default:
            break;
    }
}

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
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
            m_port_open = false;
            m_tx_ready = false;
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
            m_port_open = false;
            m_tx_ready = false;
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

#else

#define CDC_RXFIFO_MEMSIZE      CFIFO_MEMSIZE(256)
#define CDC_TXFIFO_MEMSIZE      CFIFO_MEMSIZE(2048)

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

static void usbd_user_ev_handler(app_usbd_event_type_t event)
{
    switch (event)
    {
        case APP_USBD_EVT_DRV_SOF:
            UsbdCdcIntrfTxKick(&s_CdcIntrf);
            break;

        case APP_USBD_EVT_DRV_SUSPEND:
            bsp_board_led_off(LED_USB_RESUME);
            break;

        case APP_USBD_EVT_DRV_RESUME:
            bsp_board_led_on(LED_USB_RESUME);
            break;

        case APP_USBD_EVT_STOPPED:
            app_usbd_disable();
            bsp_board_leds_off();
            break;

        case APP_USBD_EVT_POWER_DETECTED:
            if (!nrf_drv_usbd_is_enabled())
            {
                app_usbd_enable();
            }
            break;

        case APP_USBD_EVT_POWER_REMOVED:
            app_usbd_stop();
            break;

        case APP_USBD_EVT_POWER_READY:
            app_usbd_start();
            break;

        default:
            break;
    }
}

#endif

int main(void)
{
    ret_code_t ret;
#if !USB_CDC_TEST_DIRECT
    uint8_t data = 0xff;
#endif
    static const app_usbd_config_t usbd_config = {
        .ev_state_proc = usbd_user_ev_handler,
#if !USB_CDC_TEST_DIRECT
        .enable_sof = true,
#endif
    };

    ret = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(ret);

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);

    nrf_drv_clock_lfclk_request(NULL);
    while (!nrf_drv_clock_lfclk_is_running())
    {
    }

    ret = app_timer_init();
    APP_ERROR_CHECK(ret);

    bsp_board_init(BSP_INIT_LEDS);
    app_usbd_serial_num_generate();

#if USB_CDC_TEST_DIRECT
    m_tx_fifo = CFifoInit(m_tx_fifo_mem, sizeof(m_tx_fifo_mem), 1, true);
    if (m_tx_fifo == NULL)
    {
        return -1;
    }
#endif

    ret = app_usbd_init(&usbd_config);
    APP_ERROR_CHECK(ret);

#if USB_CDC_TEST_DIRECT
    app_usbd_class_inst_t const * class_cdc_acm =
        app_usbd_cdc_acm_class_inst_get(&m_test_cdc_acm);
    ret = app_usbd_class_append(class_cdc_acm);
    APP_ERROR_CHECK(ret);
#else
    if (!UsbdCdcIntrfInit(&s_CdcIntrf, &s_CdcCfg))
    {
        return -1;
    }
#endif

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

#if USB_CDC_TEST_DIRECT
        if (m_port_open && m_tx_ready)
        {
            int count = NRF_DRV_USBD_EPSIZE;
            uint8_t * p_put = CFifoPutMultiple(m_tx_fifo, &count);

            if (p_put != NULL && count == NRF_DRV_USBD_EPSIZE)
            {
                uint8_t next = m_prbs;

                for (int i = 0; i < count; ++i)
                {
                    p_put[i] = next;
                    next = prbs8(next);
                }

                count = NRF_DRV_USBD_EPSIZE;
                uint8_t * p_tx = CFifoGetMultiple(m_tx_fifo, &count);

                if (p_tx != NULL && count == NRF_DRV_USBD_EPSIZE)
                {
                    m_tx_ready = false;
                    ret = app_usbd_cdc_acm_write(&m_test_cdc_acm, p_tx, count);

                    if (ret == NRF_SUCCESS)
                    {
                        m_prbs = next;
                    }
                    else
                    {
                        m_tx_ready = true;
                    }
                }
            }
        }

        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
        __WFE();
#else
        if (DeviceIntrfTx(&s_CdcIntrf.DevIntrf, 0, &data, 1) == 1)
        {
            data = Prbs8(data);
        }
#endif
    }
}

/** @} */
