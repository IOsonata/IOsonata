/**-------------------------------------------------------------------------
@example	wdt_demo.cpp

@brief	Watchdog demo

Starts the watchdog with a WDT_DEMO_TIMEOUT msec timeout and reloads it every
WDT_DEMO_PERIOD msec, WDT_DEMO_RELOADS times, printing each reload on the
console UART. Then it stops reloading: the watchdog resets the MCU and the
demo starts over, which shows as the banner printed again.

When the watchdog still runs from before the reset (on MCU where a reset
does not stop it), the banner says so and the demo goes on with the running
configuration.

The board.h of the project gives the console UART pins.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>

#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/wdt.h"
#include "idelay.h"

#include "board.h"

// Watchdog timeout in msec
#ifndef WDT_DEMO_TIMEOUT
#define WDT_DEMO_TIMEOUT		2000
#endif

// Reload period in msec, shorter than the timeout
#ifndef WDT_DEMO_PERIOD
#define WDT_DEMO_PERIOD			500
#endif

// Reloads before the demo lets the watchdog reset the MCU
#ifndef WDT_DEMO_RELOADS
#define WDT_DEMO_RELOADS		10
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

static void WdtDemoTimeout(WdtDev_t * const pDev);

static const WdtCfg_t s_WdtCfg = {
	.DevNo = 0,
	.msTimeout = WDT_DEMO_TIMEOUT,
	.msWindow = 0,
	.NbChan = 1,
	.bRunSleep = true,
	.bRunHalt = false,
	.IntPrio = 6,
	.EvtHandler = WdtDemoTimeout,
};

static UART s_Uart;
static Wdt s_Wdt;

// Interrupt context, the reset follows: nothing more than a flag
static volatile bool s_bWdtDemoTimeout = false;

static void WdtDemoTimeout(WdtDev_t * const pDev)
{
	(void)pDev;
	s_bWdtDemoTimeout = true;
}

int main()
{
	s_Uart.Init(s_UartCfg);

	s_Uart.printf("WdtDemo\r\n");

	if (s_Wdt.Init(s_WdtCfg) == false)
	{
		s_Uart.printf("Watchdog init failed\r\n");

		while (1)
		{
			__WFE();
		}
	}

	// Init does not start the watchdog: running here means from before
	if (s_Wdt.Running())
	{
		s_Uart.printf("Watchdog already running, timeout %u ms\r\n", (unsigned)s_Wdt.Timeout());
	}
	else
	{
		s_Uart.printf("Watchdog timeout %u ms\r\n", (unsigned)s_Wdt.Timeout());
	}

	s_Wdt.Start();

	for (int i = 0; i < WDT_DEMO_RELOADS; i++)
	{
		msDelay(WDT_DEMO_PERIOD);
		s_Wdt.Reload(0);
		s_Uart.printf("Reload %d\r\n", i + 1);
	}

	s_Uart.printf("No more reloads, reset in %u ms\r\n", (unsigned)s_Wdt.Timeout());

	while (1)
	{
		__WFE();
	}

	return 0;
}
