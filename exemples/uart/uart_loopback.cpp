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
#include <chrono>
#include <time.h>

#include "coredev/uart.h"
#include "prbs.h"
#include "board.h"

//#define DEMO_C
//#define BYTE_MODE

#define BUFFER_SIZE				16

// This defines the s_UartPortPins map and pin count.
// See board.h for target device specific definitions
static const UART_PORTPINS;

// UART configuration data
const UARTCfg_t g_UartCfg = {
	.DevNo = UART_NO,
	.pIOPinMap = s_UartPortPins,
	.NbIOPins = UART_PORTPIN_COUNT,
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = nullptr,//nRFUartEvthandler,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

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
	uint8_t buff[BUFFER_SIZE];
#ifdef BYTE_MODE
	int len = 1;
#else
	int len = BUFFER_SIZE;
#endif

#ifdef DEMO_C
	res = UARTInit(&g_UartDev, &g_UartCfg);
	UARTprintf("UART Loopback Test\r\n");
#else
	res = g_Uart.Init(g_UartCfg);
	g_Uart.printf("UART Loopback Test\r\n");
#endif



	while(1)
	{
#ifdef DEMO_C
        int l = UARTRx(&g_UartDev, buff, len);
#else
        int l = g_Uart.Rx(buff, len);
#endif
        if (l > 0)
        {
#ifdef DEMO_C
        	UARTTx(&g_UartDev, buff, l);
#else
        	g_Uart.Tx(buff, l);
#endif
        }
	}
	return 0;
}
