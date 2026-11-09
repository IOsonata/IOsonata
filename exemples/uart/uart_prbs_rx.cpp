/**-------------------------------------------------------------------------
@example	uart_prbs_rx.cpp


@brief	UART PRBS test

Demo code using EHAL library to do PRBS transmit test using UART

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
#include <stdint.h>
#include <string.h>

//#include <time.h>
#include "istddef.h"
#include "coredev/uart.h"
#include "coredev/iopincfg.h"
#include "stddev.h"
#include "prbs.h"
#include "board.h"

#ifndef UART_PRBS_VERBOSE
#define UART_PRBS_VERBOSE 1
#endif
#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif
#ifndef UART_INT_MODE
#define UART_INT_MODE false
#endif
#ifndef UART_DMA_MODE
#define UART_DMA_MODE true
#endif
#ifndef UART_INT_PRIO
#define UART_INT_PRIO 6
#endif

#define UARTFIFOSIZE			CFIFO_MEMSIZE(256)

alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];

// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const UART_PORTPINS;

// UART configuration data
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_NO,
	.pIOPinMap = s_UartPortPins,
	.NbIOPins = UART_PORTPIN_COUNT,
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = UART_INT_MODE,					// Interrupt mode
	.IntPrio = UART_INT_PRIO,//APP_IRQ_PRIORITY_LOW,	// Interrupt priority
	.EvtCallback = NULL,//nRFUartEvthandler,	// UART event handler
	.bFifoBlocking = true,				// Blocking FIFO
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
	.bDMAMode = UART_DMA_MODE,
};

#define DEMO_C
#ifdef DEMO_C
// For C
UARTDev_t g_UartDev;
#else
// For C++
// UART object instance
UART g_Uart;
#endif

int main()
{
	bool res;
	bool haveValue = false;
	uint8_t expected = 0;
	uint8_t data[64];
	uint32_t errors = 0;
	uint32_t count = 0;
	uint32_t report = 0;
	int length;
	int i;

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &g_UartCfg);
#else
	res = g_Uart.Init(g_UartCfg);
#endif
	if (!res)
	{
		return 1;
	}
#ifdef UART_RETARGET
#ifdef DEMO_C
	UARTRetargetEnable(&g_UartDev, STDOUT_FILENO);
#else
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
#endif
#endif
	while (1)
	{
#ifdef DEMO_C
		length = UARTRx(&g_UartDev, data, sizeof(data));
#else
		length = g_Uart.Rx(data, sizeof(data));
#endif
		for (i = 0; i < length; i++)
		{
			if (haveValue && expected != data[i])
			{
				errors++;
			}
			expected = Prbs8(data[i]);
			haveValue = true;
			count++;
#if UART_PRBS_VERBOSE
			printf("%02x\r\n", data[i]);
#endif
		}
		if (count - report >= 32768U)
		{
			printf("PRBS bytes %lu errors %lu\r\n", (unsigned long)count, (unsigned long)errors);
			report = count;
		}
	}
}
