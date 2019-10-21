/**-------------------------------------------------------------------------
@example	uart_slip_prbs_rx.cpp

@brief	UART PRBS transmit test over SLIP protocol

This example sends PRBS byte though UART over SLIP protocole. The example
shows UART & SLIP interface use in both C and C++.

To compile in C, rename the file to .c and uncomment the line #define DEMO_C


@author	Hoang Nguyen Hoan
@date	Oct. 7, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

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

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "prbs.h"
#include "slip_intrf.h"

// This include contain i/o definition the board in use
#include "board.h"

#define SLIPTEST_BUFSIZE		600

//#define DEMO_C

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOSIZE			CFIFO_MEMSIZE(SLIPTEST_BUFSIZE * 4)

uint8_t g_RxBuff[FIFOSIZE];

static IOPINCFG s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
const UARTCFG g_UartCfg = {
	.DevNo = UART_NO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPINCFG),
	.Rate = 460800,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = FIFOSIZE,
	.pRxMem = g_RxBuff,
	.TxMemSize = 0,//FIFOSIZE,
	.pTxMem = NULL,//g_TxBuff,
	.bDMAMode = false,
};

#ifdef DEMO_C
// For C programming
UARTDEV g_UartDev;
SLIPDEV g_SlipDev;
#else
// For C++ object programming
// UART object instance
UART g_Uart;
Slip g_Slip;
#endif

int nRFUartEvthandler(UARTDEV *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[SLIPTEST_BUFSIZE];
	uint8_t *p;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			//UARTRx(pDev, buff, BufferLen);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

int main()
{
	bool res;

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &g_UartCfg);
	SlipInit(&g_SlipDev, &g_UartDev.DevIntrf);
#else
	res = g_Uart.Init(g_UartCfg);
	g_Uart.printf("UART PRBS Test\n\r");
	g_Slip.Init(&g_Uart);
#endif

    uint8_t val = 0;
    uint32_t errcnt = 0;
    uint32_t cnt = 0;
	uint8_t d = 0xff;
	uint8_t buf[SLIPTEST_BUFSIZE];

	while(1)
	{
#ifdef DEMO_C
        int l = SlipRx(&g_SlipDev, buf, SLIPTEST_BUFSIZE);
#else
        int l = g_Slip.Rx(0, buf, SLIPTEST_BUFSIZE);
#endif
//		e += difftime(time(NULL), t);
		for (int i = 0; i < l; i++)
		{
            cnt++;

			if (val == 0)
			{
				val = buf[i];
			}
			// If success send next code
            if (val != buf[i])
            {
                errcnt++;
                if ((cnt & 0xf) == 0)
                {
                	printf("Err %d\n", errcnt);
                }
               // printf("PRBS %u errors %x %x\n", errcnt, val, d);
            }
/*            else if ((cnt & 0x7fff) == 0)
            {
                printf("PRBS %d rate %.3f B/s, err : %u\n", l, cnt / e, errcnt);
//                printf("PRBS rate %.3f B/s, err : %u\n", cnt / elapse.count(), errcnt);

            }*/
			val = Prbs8(buf[i]);
		}
	}
	return 0;
}
