/**-------------------------------------------------------------------------
@example	wdt_demo.cpp

@brief	Watchdog demo

Starts the watchdog with a WDT_DEMO_TIMEOUT msec timeout and reloads it every
WDT_DEMO_PERIOD msec, WDT_DEMO_RELOADS times, logging each reload with
SysLog on the console UART. Then it stops reloading: the watchdog resets the
MCU and the demo starts over, which shows as the banner logged again.

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
#include "syslog.h"
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

// SysLog on the console UART, a record goes out as soon as it is logged.
// 16 records of 128 bytes. A record the UART does not take in full stays in
// the store and the rest goes out at the next log call. bBlocking keeps the
// oldest records when the store is full.
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = true,
};

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
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);

	SysLogPrintf(SysLogGet(), "WdtDemo\r\n");

	if (s_Wdt.Init(s_WdtCfg) == false)
	{
		SysLogPrintf(SysLogGet(), "Watchdog init failed\r\n");

		while (1)
		{
			__WFE();
		}
	}

	// Init does not start the watchdog: running here means from before
	if (s_Wdt.Running())
	{
		SysLogPrintf(SysLogGet(), "Watchdog already running, timeout %u ms\r\n", (unsigned)s_Wdt.Timeout());
	}
	else
	{
		SysLogPrintf(SysLogGet(), "Watchdog timeout %u ms\r\n", (unsigned)s_Wdt.Timeout());
	}

	s_Wdt.Start();

	for (int i = 0; i < WDT_DEMO_RELOADS; i++)
	{
		msDelay(WDT_DEMO_PERIOD);
		s_Wdt.Reload(0);
		SysLogPrintf(SysLogGet(), "Reload %d\r\n", i + 1);
	}

	SysLogPrintf(SysLogGet(), "No more reloads, reset in %u ms\r\n", (unsigned)s_Wdt.Timeout());

	while (1)
	{
		__WFE();
	}

	return 0;
}
