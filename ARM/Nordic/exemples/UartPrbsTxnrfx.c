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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrfx_uarte.h"

#include "prbs.h"

// This include contains the I/O definition for the board in use.
#include "board.h"

//#define BYTE_MODE
#define TEST_BUFSIZE        16

static volatile bool s_bTxDone = false;

static uint8_t s_CacheBuff[TEST_BUFSIZE];

/*
 * nrfx 4.3 keeps mutable driver state inside nrfx_uarte_t.
 * The instance must not be const.
 *
 * nRF52832 has one UARTE peripheral: NRF_UARTE0.
 * NRFX_UARTE_INSTANCE() now takes the peripheral register pointer.
 */
static nrfx_uarte_t g_Uarte = NRFX_UARTE_INSTANCE(NRFX_UART_INST);

static void uart_event_handler(nrfx_uarte_event_t const *p_event, void *ctx)
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

/*
 * Do not define UARTE0_UART0_IRQHandler() here when nrfx_prs.c is linked.
 * nrfx_prs owns the shared IRQ handler and dispatches it to nrfx_uarte.
 */

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

    return nrfx_uarte_init(&g_Uarte, &cfg, uart_event_handler);
}

int main(void)
{
    int err_code;
    uint8_t d = 0xff;
    uint8_t buff[TEST_BUFSIZE];

    err_code = UartInit();
    if (err_code != 0)
    {
        while (1)
        {
        }
    }

    while (1)
    {
#ifdef BYTE_MODE
        if (nrfx_uarte_tx(&g_Uarte, &d, 1, NRFX_UARTE_TX_BLOCKING) == 0)
        {
            d = Prbs8(d);
        }
#else
        for (int i = 0; i < TEST_BUFSIZE; i++)
        {
            d = Prbs8(d);
            buff[i] = d;
        }

        s_bTxDone = false;

        if (nrfx_uarte_tx(&g_Uarte, buff, sizeof(buff), 0) == 0)
        {
            while (s_bTxDone == false)
            {
            }
        }
#endif
    }
}

void _exit(void)
{
    while (1)
    {
    }
}
