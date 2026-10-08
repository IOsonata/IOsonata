/**-------------------------------------------------------------------------
@example	uart_slip_prbs_rx.cpp

@brief	UART PRBS receive test over SLIP protocol

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

#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "prbs.h"
#include "slip_intrf.h"

// This include contain i/o definition the board in use
#include "board.h"

#ifndef UART_BAUDRATE
#define UART_BAUDRATE 115200
#endif

#define DEMO_C

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define SLIPTEST_BUFSIZE		600
#define FIFOSIZE				CFIFO_MEMSIZE(SLIPTEST_BUFSIZE * 4)
uint8_t g_RxBuff[FIFOSIZE];

// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const IOPinCfg_t s_UartPortPins[] = UART_PINS;

// UART configuration data
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPortPins,
	.NbIOPins = sizeof(s_UartPortPins) / sizeof(IOPINCFG),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = NULL,//nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = FIFOSIZE,
	.pRxMem = g_RxBuff,
	.TxMemSize = 0,//FIFOSIZE,
	.pTxMem = NULL,//g_TxBuff,
	.bDMAMode = false,
};

#ifdef DEMO_C
// For C programming
UARTDev_t g_UartDev;
SLIPDEV g_SlipDev;
#else
// For C++ object programming
// UART object instance
UART g_Uart;
Slip g_Slip;
#endif

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	//uint8_t buff[SLIPTEST_BUFSIZE];

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
	if (!res) return 1;
	SlipInit(&g_SlipDev, &g_UartDev.DevIntrf, false);
#else
	res = g_Uart.Init(g_UartCfg);
	if (!res) return 1;
	g_Slip.Init(&g_Uart, false);
#endif

	printf("UART PRBS Test\n\r");

	uint8_t val = 0;
	bool haveValue = false;
	uint32_t errcnt = 0;
	uint32_t pkcnt = 0;
	uint8_t buf[SLIPTEST_BUFSIZE] = {};

	while (1)
	{
#ifdef DEMO_C
		int len = SlipRx(&g_SlipDev, buf, sizeof(buf));
		bool complete = SlipRxCompleted(&g_SlipDev);
#else
		int len = g_Slip.Rx(0, buf, sizeof(buf));
		bool complete = g_Slip.RxCompleted();
#endif
		if (len < 0) continue;
		// Validate payload chunks as they arrive, including frames larger than buf.
		// Rx excludes END; completion can also accompany a zero-byte read.
		for (int i = 0; i < len; i++)
		{
			if (haveValue && val != buf[i])
			{
				errcnt++;
				printf("PRBS %u errors %x %x\n", errcnt, val, buf[i]);
			}
			val = Prbs8(buf[i]);
			haveValue = true;
		}
		// Nonblocking SLIP leaves a pending escape in the next output byte.
		// Preserve that byte when reusing the buffer for the next chunk.
		buf[0] = !complete && len < (int)sizeof(buf) ? buf[len] : 0;
		if (complete)
		{
			pkcnt++;
			if ((pkcnt & 0xff) == 0) printf("frames %u errors %u\n", pkcnt, errcnt);
		}
	}
	return 0;
}
