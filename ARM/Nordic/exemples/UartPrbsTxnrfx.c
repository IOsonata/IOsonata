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

#include "nrfx_uarte.h"

#include "prbs.h"

// This include contain i/o definition the board in use
#include "board.h"

//#define BYTE_MODE
#define TEST_BUFSIZE		16

static volatile bool s_bTxDone = false;

uint8_t s_CacheBuff[TEST_BUFSIZE];

const nrfx_uarte_t g_Uarte = NRFX_UARTE_INSTANCE(NRFX_UART_INST);
const nrfx_uarte_config_t g_UarteCfg = {
	.rxd_pin = (UART_RX_PORT << 5) | UART_RX_PIN,
	.txd_pin = (UART_TX_PORT << 5) | UART_TX_PIN,
	.tx_cache = { s_CacheBuff, TEST_BUFSIZE},
	.config = {.hwfc = NRF_UARTE_HWFC_DISABLED, 0, 1, 0 },
	.baudrate = NRF_UARTE_BAUDRATE_1000000,
	.interrupt_priority = 1
};

void uart_event_handler(nrfx_uarte_event_t const * p_event, void *ctx)
{
//	printf("Uarte Handler\r\n");
	switch(p_event->type)
	{
		case NRFX_UARTE_EVT_TX_DONE: ///< Requested TX transfer completed.
			s_bTxDone = true;
			break;
		case NRFX_UARTE_EVT_RX_DONE: ///< Requested RX transfer completed.
		 	 break;
		case NRFX_UARTE_EVT_ERROR:   ///< Error reported by UART peripheral.
			break;
	}
}

int main()
{
    uint32_t err_code;
	uint8_t d = 0xff;
	uint8_t buff[TEST_BUFSIZE];


    err_code = nrfx_uarte_init(&g_Uarte, &g_UarteCfg, uart_event_handler);
    if (err_code != NRFX_SUCCESS)
    {
    	//printf("Error %x\n\r", err_code);
    }

    //sprintf(buff, "UART PRBS Test\n\r");
    //while (nrfx_uarte_tx(&g_Uarte, buff, strlen(buff)) != NRFX_SUCCESS);

	while(1)
	{
#ifdef BYTE_MODE
		if (nrfx_uarte_tx(&g_Uarte, &d, 1, NRFX_UARTE_TX_BLOCKING) == NRFX_SUCCESS)
		{
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
		if (nrfx_uarte_tx(&g_Uarte, buff, TEST_BUFSIZE, 0) == NRFX_SUCCESS)//NRFX_UARTE_TX_LINK) == NRFX_SUCCESS)
		{
			while (s_bTxDone == false);
		}
#endif
	}
	return 0;
}

void _exit()
{
	while(1);
}
