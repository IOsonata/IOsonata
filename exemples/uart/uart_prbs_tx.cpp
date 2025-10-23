/**-------------------------------------------------------------------------
@example	uart_prbs_tx.cpp

@brief	UART PRBS transmit test

This example sends PRBS byte though UART. The example shows UART interface use
in both C and C++.

To compile in C, rename the file to .c and uncomment the line #define DEMO_C


@author	Hoang Nguyen Hoan
@date	Aug. 31, 2016

@license

Copyright (c) 2016, I-SYST inc., all rights reserved

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
#include "coredev/system_core_clock.h"

// This include contain i/o definition the board in use
#include "board.h"

//#define DEMO_C	// Select demo C code
#define BYTE_MODE

#define TEST_BUFSIZE		16

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

// Uncomment to enable FIFO
//#define UARTFIFOSIZE			CFIFO_MEMSIZE(TEST_BUFSIZE)

#ifdef UARTFIFOSIZE
alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];
#endif

static const IOPinCfg_t s_UartPins[] = UART_PINS;

// UART configuration data
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
	.EvtCallback = nRFUartEvthandler,
	.bFifoBlocking = true,
#ifdef UARTFIFOSIZE
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,//FIFOSIZE,
	.pTxMem = s_UartTxFifo,//g_TxBuff,
#else
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = 0,
	.pTxMem = NULL,
#endif
	.bDMAMode = true,
};

#ifdef DEMO_C
// For C programming
UARTDev_t g_UartDev;
#else
// For C++ object programming
// UART object instance
UART g_Uart;
#endif

int nRFUartEvthandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return 0;
}

int main()
{
	bool res;

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &s_UartCfg);
	UARTprintf(&g_UartDev, "UART PRBS Test\n\r");
#else
	res = g_Uart.Init(s_UartCfg);
	g_Uart.printf("UART PRBS Test\n\r");
#endif

	uint8_t d = 0xff;
	uint8_t buff[TEST_BUFSIZE];

	while (1)
	{
#ifdef BYTE_MODE
		// Demo transfer byte by byte
#ifdef DEMO_C
		if (UARTTx(&g_UartDev, &d, 1) > 0)
#else
		if (g_Uart.Tx(&d, 1) > 0)
#endif
		{
			d = Prbs8(d);
		}
#else
		// Demo transfer buffer
		for (int i = 0; i < TEST_BUFSIZE; i++)
		{
			d = Prbs8(d);
			buff[i] = d;
		}
		int len = TEST_BUFSIZE;
		uint8_t *p = buff;
		while (len > 0)
		{
#ifdef DEMO_C
			int l = UARTTx(&g_UartDev, p, len);
#else
			int l = g_Uart.Tx(p, len);
#endif
			len -= l;
			p += l;
		}
#endif
	}
	return 0;
}
