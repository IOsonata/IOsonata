/**-------------------------------------------------------------------------
@example	hfxo_test.cpp

@brief	Nordic high frequency crystal oscillator request test

Checks first request, shared requests and last release through the public
SystemCoreClockSelect API. Debug writes to the debugger semihosting console.
Release writes to the board UART at 1 Mbaud. The test reports the HFXO
running state after each operation and does not initialize a BLE stack.

A BLE stack already enabled by the application must retain ownership of
CLOCK. Tests of stack initialization and handover require a separate BLE
example.

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026

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
#include <stdbool.h>

#include "nrf.h"
#include "coredev/system_core_clock.h"

#ifdef NDEBUG
#include "coredev/uart.h"
#include "syslog.h"
#include "board.h"

#define HFXO_PRINT(...) SysLogPrintf(SysLogGet(), __VA_ARGS__)

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

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
	.IntPrio = 6,
	.EvtCallback = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = true,
};

static UART s_Uart;
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = true,
};

static void HfxoConsoleInit(void)
{
	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);
}
#else
#include <stdio.h>
#define HFXO_PRINT(...) printf(__VA_ARGS__)

static void HfxoConsoleInit(void)
{
}
#endif

// CLOCK register status: verify the crystal source, not merely HFCLK running.
static bool HfxoRunning(void)
{
	return (NRF_CLOCK->HFCLKSTAT &
		(CLOCK_HFCLKSTAT_STATE_Msk | CLOCK_HFCLKSTAT_SRC_Msk)) ==
		(CLOCK_HFCLKSTAT_STATE_Msk |
		 (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos));
}

static void HfxoResult(const char *pName, bool Result, bool Running)
{
	HFXO_PRINT("%s: %s, HFXO %s\r\n", pName,
		Result ? "PASS" : "FAIL",
		Running ? "running" : "stopped");
}

int main(void)
{
	bool initial;
	bool first;
	bool second;
	bool release1;
	bool release2;
	bool running1;
	bool running2;
	bool running3;
	bool final;
	bool failed;
	int pass;

	HfxoConsoleInit();
	HFXO_PRINT("HfxoTest\r\n");

	initial = HfxoRunning();
	HFXO_PRINT("Initial HFXO: %s\r\n", initial ? "running" : "stopped");

	first = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U);
	running1 = HfxoRunning();
	second = SystemCoreClockSelect(OSC_TYPE_XTAL, 32000000U);
	running2 = HfxoRunning();
	release1 = SystemCoreClockSelect(OSC_TYPE_RC, 0U);
	running3 = HfxoRunning();
	release2 = SystemCoreClockSelect(OSC_TYPE_RC, 0U);
	final = HfxoRunning();
	failed = SystemCoreClockSelect(OSC_TYPE_RC, 0U) == false;

	pass = 0;
	if (first && running1)
	{
		pass++;
	}
	HfxoResult("First request", first && running1, running1);
	if (second && running2)
	{
		pass++;
	}
	HfxoResult("Second request", second && running2, running2);
	if (release1 && running3)
	{
		pass++;
	}
	HfxoResult("First release", release1 && running3, running3);
	if (release2 && final == initial)
	{
		pass++;
	}
	HfxoResult("Last release", release2 && final == initial, final);
	if (failed)
	{
		pass++;
	}
	HfxoResult("Unmatched release", failed, final);
	HFXO_PRINT("HfxoTest: %d/5 checks PASS\r\n", pass);

	while (1)
	{
		__WFE();
	}

	return 0;
}
