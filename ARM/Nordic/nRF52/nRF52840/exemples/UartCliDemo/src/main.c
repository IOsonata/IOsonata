/**
 * Copyright (c) 2016 - 2020, Nordic Semiconductor ASA
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
#include <stdio.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf.h"
#include "nrf_drv_clock.h"
#include "nrf_gpio.h"
#include "nrf_delay.h"

#include "app_timer.h"
#include "fds.h"
#include "app_error.h"
#include "app_util.h"
#include "nrf_memobj.h"

#include "nrf_cli.h"
//#include "nrf_cli_rtt.h"
#include "nrf_cli_types.h"

#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_log_backend_flash.h"
#include "nrf_fstorage_nvmc.h"

#include "nrf_mpu_lib.h"
#include "nrf_stack_guard.h"

#include "coredev/iopincfg.h"
#include "board.h"

#if defined(APP_USBD_ENABLED) && APP_USBD_ENABLED
#define CLI_OVER_USB_CDC_ACM 0
#else
#define CLI_OVER_USB_CDC_ACM 0
#endif

#if CLI_OVER_USB_CDC_ACM
#include "nrf_cli_cdc_acm.h"
#include "nrf_drv_usbd.h"
#include "app_usbd_core.h"
#include "app_usbd.h"
#include "app_usbd_string_desc.h"
#include "app_usbd_cdc_acm.h"
#endif //CLI_OVER_USB_CDC_ACM

#if 1
#if defined(TX_PIN_NUMBER) && defined(RX_PIN_NUMBER)
#define CLI_OVER_UART 1
#else
#define CLI_OVER_UART 0
#endif
#endif

#if CLI_OVER_UART
#include "nrf_cli_uart.h"
#endif

/* If enabled then CYCCNT (high resolution) timestamp is used for the logger. */
#define USE_CYCCNT_TIMESTAMP_FOR_LOG 0

/**@file
 * @defgroup CLI_example main.c
 *
 * @{
 *
 */

#if NRF_LOG_BACKEND_FLASHLOG_ENABLED
NRF_LOG_BACKEND_FLASHLOG_DEF(m_flash_log_backend);
#endif

#if NRF_LOG_BACKEND_CRASHLOG_ENABLED
NRF_LOG_BACKEND_CRASHLOG_DEF(m_crash_log_backend);
#endif

/* Counter timer. */
APP_TIMER_DEF(m_timer_0);

/* Declared in demo_cli.c */
extern uint32_t m_counter;
extern bool m_counter_active;

/**
 * @brief Command line interface instance
 * */
#define CLI_EXAMPLE_LOG_QUEUE_SIZE  (4)

#if 0
NRF_CLI_UART_DEF(m_cli_uart_transport, 0, 64, 16);
NRF_CLI_DEF(m_cli_uart,
            "uart_cli:~$ ",
            &m_cli_uart_transport.transport,
            '\r',
            CLI_EXAMPLE_LOG_QUEUE_SIZE);
#else
extern const nrf_cli_transport_api_t g_nRFCliUartApi;

nrf_cli_transport_t m_cli_uart_transport = {
	.p_api = &g_nRFCliUartApi
};

NRF_LOG_BACKEND_CLI_DEF(m_cli_uart_log_backend, CLI_EXAMPLE_LOG_QUEUE_SIZE);
NRF_CLI_HISTORY_MEM_OBJ(m_cli_uart);

//NRF_MEMOBJ_POOL_DEF(m_cli_uart_cmd_hist_memobj,_NRF_CLI_HISTORY_HEADER_SIZE + NRF_CLI_HISTORY_ELEMENT_SIZE, NRF_CLI_HISTORY_ELEMENT_COUNT);

//nrf_cli_uart_internal_t m_cli_uart_transport;
nrf_cli_ctx_t m_cli_uart_ctx;
nrf_cli_t m_cli_uart;
NRF_FPRINTF_DEF(m_cli_uart_fprintf_ctx, &m_cli_uart, m_cli_uart_ctx.printf_buff, NRF_CLI_PRINTF_BUFF_SIZE, false, nrf_cli_print_stream);
nrf_cli_t m_cli_uart = {
	.p_name = "uart_cli:~$ ",
	.p_iface = &m_cli_uart_transport,
	.p_ctx = &m_cli_uart_ctx,//CONCAT_2(name, _ctx),
	.p_log_backend = NRF_CLI_BACKEND_PTR(m_cli_uart),
	.p_fprintf_ctx = &m_cli_uart_fprintf_ctx,//CONCAT_2(name, _fprintf_ctx),
	.p_cmd_hist_mempool = NRF_CLI_MEMOBJ_PTR(m_cli_uart),
};


#endif

static void timer_handle(void * p_context)
{
    UNUSED_PARAMETER(p_context);

    if (m_counter_active)
    {
        m_counter++;
        NRF_LOG_RAW_INFO("counter = %d\n", m_counter);
    }
}

static void cli_start(void)
{
    ret_code_t ret;

    ret = nrf_cli_start(&m_cli_uart);
    APP_ERROR_CHECK(ret);
}

static void cli_init(void)
{
    ret_code_t ret;

//    nrf_drv_uart_config_t uart_config = NRF_DRV_UART_DEFAULT_CONFIG;
// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const IOPinCfg_t s_UartPortPins[] = UART_PINS;

// UART configuration data
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_NO,
	.pIOPinMap = s_UartPortPins,
	.NbIOPins = sizeof(s_UartPortPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = NULL,//nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = 0,
	.pTxMem = NULL,
	.bDMAMode = true,
};

//    uart_config.pseltxd = TX_PIN_NUMBER;
//    uart_config.pselrxd = RX_PIN_NUMBER;
//    uart_config.hwfc    = NRF_UART_HWFC_DISABLED;
    ret = nrf_cli_init(&m_cli_uart, &g_UartCfg, true, true, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK(ret);
}

static void cli_process(void)
{
    nrf_cli_process(&m_cli_uart);
}


static void flashlog_init(void)
{
    ret_code_t ret;
    int32_t backend_id;
#if 0
    ret = nrf_log_backend_flash_init(&nrf_fstorage_nvmc);
    APP_ERROR_CHECK(ret);
#if NRF_LOG_BACKEND_FLASHLOG_ENABLED
    backend_id = nrf_log_backend_add(&m_flash_log_backend, NRF_LOG_SEVERITY_WARNING);
    APP_ERROR_CHECK_BOOL(backend_id >= 0);

    nrf_log_backend_enable(&m_flash_log_backend);
#endif

#if NRF_LOG_BACKEND_CRASHLOG_ENABLED
    backend_id = nrf_log_backend_add(&m_crash_log_backend, NRF_LOG_SEVERITY_INFO);
    APP_ERROR_CHECK_BOOL(backend_id >= 0);

    nrf_log_backend_enable(&m_crash_log_backend);
#endif
#endif
}

static inline void stack_guard_init(void)
{
    APP_ERROR_CHECK(nrf_mpu_lib_init());
    APP_ERROR_CHECK(nrf_stack_guard_init());
}

uint32_t cyccnt_get(void)
{
    return DWT->CYCCNT;
}


int main(void)
{
    ret_code_t ret;

    if (USE_CYCCNT_TIMESTAMP_FOR_LOG)
    {
        DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        DWT->CYCCNT = 0;
        APP_ERROR_CHECK(NRF_LOG_INIT(cyccnt_get, 64000000));
    }
    else
    {
        APP_ERROR_CHECK(NRF_LOG_INIT(app_timer_cnt_get));
    }

    ret = nrf_drv_clock_init();
    APP_ERROR_CHECK(ret);
    nrf_drv_clock_lfclk_request(NULL);

//    ret = app_timer_init();
//    APP_ERROR_CHECK(ret);

//    ret = app_timer_create(&m_timer_0, APP_TIMER_MODE_REPEATED, timer_handle);
//    APP_ERROR_CHECK(ret);

//    ret = app_timer_start(m_timer_0, APP_TIMER_TICKS(1000), NULL);
//    APP_ERROR_CHECK(ret);

    cli_init();

//    usbd_init();

   // ret = fds_init();
   // APP_ERROR_CHECK(ret);


    //UNUSED_RETURN_VALUE(nrf_log_config_load());

    cli_start();

    flashlog_init();

    stack_guard_init();

    NRF_LOG_RAW_INFO("Command Line Interface example started.\n");
    NRF_LOG_RAW_INFO("Please press the Tab key to see all available commands.\n");

    while (true)
    {
        UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
#if CLI_OVER_USB_CDC_ACM && APP_USBD_CONFIG_EVENT_QUEUE_ENABLE
        while (app_usbd_event_queue_process())
        {
            /* Nothing to do */
        }
#endif
        cli_process();
    }
}

/** @} */
