/**-------------------------------------------------------------------------
@example	hfxo_test.cpp

@brief	Shared high-frequency clock request test

Exercises the generic shared clock request and release API without directly
accessing MCU registers. Debug uses semihosting; Release uses the board UART
at 1 Mbaud. Physical oscillator state is checked in MCU-specific tests, not by
this portable example.

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

int main(void)
{
	bool first;
	bool second;
	bool third;
	int pass;

	HfxoConsoleInit();
	HFXO_PRINT("HfxoTest\r\n");

	// Requests must be balanced. Neither the app nor this test selects
	// oscillator hardware or reads the target's CLOCK registers.
	first = SystemHighFreqClockEnable();
	HFXO_PRINT("First enable: %s\r\n", first ? "PASS" : "FAIL");

	second = first && SystemHighFreqClockEnable();
	HFXO_PRINT("Second enable: %s\r\n", second ? "PASS" : "FAIL");

	if (second)
	{
		SystemHighFreqClockDisable();
		HFXO_PRINT("First request released\r\n");
	}
	if (first)
	{
		SystemHighFreqClockDisable();
		HFXO_PRINT("Last request released\r\n");
	}

	third = SystemHighFreqClockEnable();
	HFXO_PRINT("Re-enable after release: %s\r\n",
		third ? "PASS" : "FAIL");
	if (third)
	{
		SystemHighFreqClockDisable();
	}

	// The MCU clock implementation must ignore a release with no active request.
	SystemHighFreqClockDisable();
	HFXO_PRINT("Unmatched release requested\r\n");

	pass = (first ? 1 : 0) + (second ? 1 : 0) + (third ? 1 : 0);
	HFXO_PRINT("HfxoTest: %d/3 requests PASS\r\n", pass);

	while (1)
	{
		__WFE();
	}
	return pass == 3 ? 0 : 1;
}
