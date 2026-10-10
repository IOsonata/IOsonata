/**-------------------------------------------------------------------------
@example	uart_loopback.cpp


@brief	UART loopback test

Demo code using IOsonata library to read from UART Rx and Send it out to Tx

@author	Hoang Nguyen Hoan
@date	Dec. 22, 2023

@license

MIT License

Copyright (c) 2023, I-SYST inc., all rights reserved

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

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#include "coredev/uart.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"

#include "board.h"

//#define DEMO_C
//#define BYTE_MODE

#define BUFFER_SIZE				16

#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif

#ifdef MCUOSC
McuOsc_t g_McuOsc = MCUOSC;
#endif

#define UARTFIFOSIZE CFIFO_MEMSIZE(256)

#ifdef UARTFIFOSIZE
alignas(4) static uint8_t s_UartRxFifo[UARTFIFOSIZE];
alignas(4) static uint8_t s_UartTxFifo[UARTFIFOSIZE];
#endif

// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const IOPinCfg_t s_UartPortPins[] = UART_PINS;
#define UART_PORTPIN_COUNT (sizeof(s_UartPortPins) / sizeof(s_UartPortPins[0]))


// UART configuration data
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPortPins,
	.NbIOPins = UART_PORTPIN_COUNT,
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = UART_INT_MODE,
	.IntPrio = 1,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
#ifdef UARTFIFOSIZE
	.RxMemSize = UARTFIFOSIZE,
	.pRxMem = s_UartRxFifo,
	.TxMemSize = UARTFIFOSIZE,
	.pTxMem = s_UartTxFifo,
#else
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
#endif
	.bDMAMode = UART_DMA_MODE,
};

#ifdef DEMO_C
// For C
UARTDev_t g_UartDev;
#else
// For C++
// UART object instance
UART g_Uart;
#endif

volatile bool g_UartInitOk = false;

int main()
{
	uint8_t buff[BUFFER_SIZE];
#ifdef BYTE_MODE
	int len = 1;
#else
	int len = BUFFER_SIZE;
#endif

#ifdef DEMO_C
	g_UartInitOk = UARTInit(&g_UartDev, &g_UartCfg);
#else
	g_UartInitOk = g_Uart.Init(g_UartCfg);
#endif
	if (!g_UartInitOk) return 1;

	int pending = 0;
	int offset = 0;
	while (1)
	{
		// Keep the unsent suffix until accepted; never overwrite it with RX.
		if (pending == 0)
		{
#ifdef DEMO_C
			pending = UARTRx(&g_UartDev, buff, len);
#else
			pending = g_Uart.Rx(buff, len);
#endif
			offset = 0;
		}
		if (pending > 0)
		{
#ifdef DEMO_C
			int sent = UARTTx(&g_UartDev, buff + offset, pending);
#else
			int sent = g_Uart.Tx(buff + offset, pending);
#endif
			if (sent > 0)
			{
				offset += sent;
				pending -= sent;
			}
		}
	}
	return 0;
}
