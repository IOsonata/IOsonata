/**-------------------------------------------------------------------------
@example	EsbPtx.cpp

@brief	ESB transmitter example

Generic example of ESB transmitter

@author	Hoang Nguyen Hoan
@date	Nov. 6, 2019

@license

MIT License

Copyright (c) 2019, I-SYST inc., all rights reserved

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
#include <string.h>

#include "sdk_macros.h"
#include "sdk_config.h"
#include "nrf_error.h"
#include "nrf.h"
#include "nrf_esb.h"
#include "nrfx_clock.h"

#include "istddef.h"
#include "idelay.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "miscdev/led.h"
#include "board.h"

#define UART_FIFO_MEMSIZE				CFIFO_MEMSIZE(256)
static uint8_t s_UartTxFifoMem[UART_FIFO_MEMSIZE];

static IOPinCfg_t s_UartPins[] = UART_PORTPINS;

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = UART_FIFO_MEMSIZE,
	.pTxMem = s_UartTxFifoMem,
	.bDMAMode = true,
};

// UART object instance
UART g_Uart;

Led g_Led1;

static nrf_esb_payload_t s_TxPayload = {0x08, 0, 0x00, 0x00, 0x00, 0x11, 0x00, 0x00, 0x00};

void nrf_esb_event_handler(nrf_esb_evt_t const * p_event)
{
	nrf_esb_payload_t rxpayload;

	switch (p_event->evt_id)
    {
        case NRF_ESB_EVENT_TX_SUCCESS:
        	g_Led1.Off();

            break;
        case NRF_ESB_EVENT_TX_FAILED:
        	g_Uart.printf("Tx failed\r\n");
            (void) nrf_esb_flush_tx();
            break;
        case NRF_ESB_EVENT_RX_RECEIVED:
            // Get the most recent element from the RX FIFO.
            while (nrf_esb_read_rx_payload(&rxpayload) == NRF_SUCCESS) ;

            g_Uart.printf("rx = %d bytes\r\n", rxpayload.length);

            break;
    }

}


void clocks_start( void )
{
    // Start HFCLK and wait for it to start.
    NRF_CLOCK->EVENTS_HFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0);
}


uint32_t esb_init( void )
{
    uint32_t err_code;
#if 0
    uint8_t base_addr_0[4] = {0xE7, 0xE7, 0xE7, 0xE7};
    uint8_t base_addr_1[4] = {0xC2, 0xC2, 0xC2, 0xC2};
    uint8_t addr_prefix[8] = {0xE7, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8 };
#else
    uint8_t base_addr_0[4] = {0x62, 0x39, 0x8A, 0xF2};
    uint8_t base_addr_1[4] = {0x28, 0xFF, 0x50, 0xB8};
    uint8_t addr_prefix[8] = {0xFE, 0xFF, 0x29, 0x27, 0x09, 0x02, 0xB2, 0xD6 };
#endif

    nrf_esb_config_t nrf_esb_config         = NRF_ESB_DEFAULT_CONFIG;
    nrf_esb_config.retransmit_count         = 6;
    nrf_esb_config.protocol                 = NRF_ESB_PROTOCOL_ESB_DPL;
    nrf_esb_config.bitrate                  = NRF_ESB_BITRATE_2MBPS;
    nrf_esb_config.mode                     = NRF_ESB_MODE_PTX;
    nrf_esb_config.event_handler            = nrf_esb_event_handler;
    nrf_esb_config.selective_auto_ack       = false;

    err_code = nrf_esb_init(&nrf_esb_config);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_0(base_addr_0);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_base_address_1(base_addr_1);
    VERIFY_SUCCESS(err_code);

    err_code = nrf_esb_set_prefixes(addr_prefix, 8);
    VERIFY_SUCCESS(err_code);

    return NRF_SUCCESS;
}

int main(void)
{
    uint32_t err_code;
    // Initialize
    clocks_start();

    g_Uart.Init(s_UartCfg);
    g_Led1.Init(LED1_PORT, LED1_PIN, LED1_LOGIC);

    err_code = esb_init();
    APP_ERROR_CHECK(err_code);


    while (true)
    {
    	g_Led1.On();

    	if (nrf_esb_write_payload(&s_TxPayload) == NRF_SUCCESS)
        {
        	g_Uart.printf("Send OK\r\n");
        }
        else
        {
        	g_Uart.printf("Send failed\r\n");
        }
    	__WFE();
    	msDelay(5000);
    }
}
