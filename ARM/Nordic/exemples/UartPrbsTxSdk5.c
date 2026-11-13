/**-------------------------------------------------------------------------
@example	UartPrbsTxSdk5.c


@brief	UART PRBS transmit test with NRF5_SDK

Demo code using Nordic NRF5_SDK to do PRBS transmit test using UART


@author	Hoang Nguyen Hoan
@date	Dec. 18, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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

#include <stdio.h>
#include <memory.h>

#include "app_uart.h"
#include "app_error.h"
#include "nrf.h"
#include "nrfx_uarte.h"

#include "prbs.h"

// This include contain i/o definition the board in use
#include "board.h"

//#define BYTE_MODE
#define TEST_BUFSIZE		16

#define SDK_NRFX	// Use nrfx_uarte, comment out this define to use app_uart_fifo

static volatile bool s_bTxDone = false;

#ifndef SDK_NRFX
#define UART_TX_BUF_SIZE 512                         /**< UART TX buffer size. */
#define UART_RX_BUF_SIZE 512                         /**< UART RX buffer size. */

const app_uart_comm_params_t comm_params = {
		UART_RX_PIN,
		UART_TX_PIN,
		UART_RTS_PIN,
		UART_CTS_PIN,
		APP_UART_FLOW_CONTROL_DISABLED,
      false,
	  NRF_UARTE_BAUDRATE_1000000
};

void uart_error_handle(app_uart_evt_t * p_event)
{
    if (p_event->evt_type == APP_UART_COMMUNICATION_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_communication);
    }
    else if (p_event->evt_type == APP_UART_FIFO_ERROR)
    {
        APP_ERROR_HANDLER(p_event->data.error_code);
    }
}

#else

static uint8_t s_CacheBuff[TEST_BUFSIZE];
static nrfx_uarte_t g_Uarte = NRFX_UARTE_INSTANCE(NRF_UARTE0);

static void uart_error_handle(nrfx_uarte_event_t const *p_event, void *ctx)
{
	(void)ctx;

	switch (p_event->type)
	{
		case NRFX_UARTE_EVT_TX_DONE:
			s_bTxDone = true;
			break;
		case NRFX_UARTE_EVT_RX_DONE:
			break;
		case NRFX_UARTE_EVT_ERROR:
			break;
		default:
			break;
	}
}

#if !NRFX_CHECK(NRFX_PRS_ENABLED)
void UARTE0_UART0_IRQHandler(void)
{
	nrfx_uarte_irq_handler(&g_Uarte);
}
#endif

static int UartInit(void)
{
	nrfx_uarte_config_t cfg =
		NRFX_UARTE_DEFAULT_CONFIG(
			(UART_TX_PORT << 5) | UART_TX_PIN,
			(UART_RX_PORT << 5) | UART_RX_PIN
		);

	cfg.tx_cache.p_buffer = s_CacheBuff;
	cfg.tx_cache.length = sizeof(s_CacheBuff);
	cfg.baudrate = NRF_UARTE_BAUDRATE_1000000;
	cfg.interrupt_priority = 1;
	cfg.config.hwfc = NRF_UARTE_HWFC_DISABLED;
	cfg.config.parity = NRF_UARTE_PARITY_EXCLUDED;

	return nrfx_uarte_init(&g_Uarte, &cfg, uart_error_handle);
}
#endif

int main()
{
    uint32_t err_code;
	uint8_t d = 0xff;
	uint8_t buff[TEST_BUFSIZE];

#ifdef SDK_NRFX

	err_code = UartInit();
	if (err_code != NRF_SUCCESS)
	{
		while (1)
		{
		}
	}

	while(1)
	{
		// NOTE : NRFX_UARTE_TX_BLOCKING is extremely slow
#ifdef BYTE_MODE
		if (nrfx_uarte_tx(&g_Uarte, &d, 1, NRFX_UARTE_TX_BLOCKING) == NRF_SUCCESS)
		{
			//while (s_bTxDone == false);
			// If success send next code
			d = Prbs8(d);
		}
#else
		for (int i = 0; i < TEST_BUFSIZE; i++)
		{
			d = Prbs8(d);
			buff[i] = d;
		}

		s_bTxDone = false;
		if (nrfx_uarte_tx(&g_Uarte, buff, TEST_BUFSIZE, 0) == NRFX_SUCCESS)
		{
			while (s_bTxDone == false);
		}
#endif
	}
#else

    APP_UART_FIFO_INIT(&comm_params,
                         UART_RX_BUF_SIZE,
                         UART_TX_BUF_SIZE,
                         uart_error_handle,
                         APP_IRQ_PRIORITY_LOWEST,
                         err_code);

    APP_ERROR_CHECK(err_code);

    while (1)
    {
    	if (app_uart_put(d) == NRF_SUCCESS)
    	{
			// If success send next code
			d = Prbs8(d);
    	}
    }
#endif
	return 0;
}
