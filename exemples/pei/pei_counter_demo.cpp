/**-------------------------------------------------------------------------
@example	pei_counter_demo.cpp

@brief	Button presses counted in hardware, an LED toggled every N presses

A Counter counts the presses of the button of board.h with no CPU: the GPIO
event of the button goes to the COUNT task of the counter through PEI (PPI on
the nRF52). Every PEI_COUNTER_DEMO_LIMIT presses the limit event of the counter
toggles the LED of board.h, also through PEI with no CPU.

The limit event handler counts the limit events. The main loop reads the count
every 100 ms and logs it with SysLog on the console UART when it changed. A
button with no debounce can count more than one edge for one press.

The board.h of the project gives the console UART pins, the button and the
LED. The counter takes any free timer instance, DevNo -1.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

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
#include "coredev/counter.h"
#include "coredev/pei.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "syslog.h"
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE				1000000
#endif

// Presses between two LED toggles
#ifndef PEI_COUNTER_DEMO_LIMIT
#define PEI_COUNTER_DEMO_LIMIT		5
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
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

// SysLog on the console UART, 16 records of 128 bytes
alignas(4) static uint8_t s_SysLogMem[SYSLOG_MEMSIZE(16, 128)];

static const SysLogCfg_t s_SysLogCfg = {
	.pMem = s_SysLogMem,
	.MemSize = sizeof(s_SysLogMem),
	.RecordLen = 128,
	.bBlocking = true,
};

static const IOPinCfg_t s_Button = PEI_COUNTER_DEMO_BUTTON;
static const IOPinCfg_t s_Led = PEI_COUNTER_DEMO_LED;

static void PeiCounterDemoLimit(CounterDev_t * const pDev, uint64_t Count);

// Events routed with PEI, on any free timer instance
static const CounterCfg_t s_CounterCfg = {
	.DevNo = -1,
	.Src = COUNTER_SRC_EVENT,
	.Pin = {},
	.InputNo = 0,
	.Edge = IOPINSENSE_DISABLE,
	.Limit = PEI_COUNTER_DEMO_LIMIT,
	.IntPrio = 6,
	.EvtHandler = PeiCounterDemoLimit,
};

static Counter s_Counter;
static Pei s_Pei;

// Interrupt context: count only, main logs
static volatile uint32_t s_NbLimit = 0;

static void PeiCounterDemoLimit(CounterDev_t * const pDev, uint64_t Count)
{
	(void)pDev;
	(void)Count;

	s_NbLimit = s_NbLimit + 1;
}

static void PeiCounterDemoFail(const char *pMsg)
{
	SysLogPrintf(SysLogGet(), "%s failed\r\n", pMsg);

	while (1)
	{
		__WFE();
	}
}

int main()
{
	PeiEvent_t butEvt = 0U;
	PeiEvent_t limEvt = 0U;

	s_Uart.Init(s_UartCfg);
	SysLogInit(SysLogGet(), &s_SysLogCfg, (DevIntrf_t *)s_Uart, 0, nullptr, 0);

	SysLogPrintf(SysLogGet(), "PeiCounterDemo\r\n");

	// LED off, active low
	IOPinConfig(s_Led.PortNo, s_Led.PinNo, s_Led.PinOp, s_Led.PinDir, s_Led.Res, s_Led.Type);
	IOPinSet(s_Led.PortNo, s_Led.PinNo);
	IOPinConfig(s_Button.PortNo, s_Button.PinNo, s_Button.PinOp, s_Button.PinDir, s_Button.Res,
				s_Button.Type);

	if (s_Counter.Init(s_CounterCfg) == false)
	{
		PeiCounterDemoFail("Counter init");
	}

	// Button presses to the COUNT task
	if (IOPinEnableEvent(s_Pei, butEvt, s_Button.PortNo, s_Button.PinNo, IOPINSENSE_LOW_TRANSITION) == false)
	{
		PeiCounterDemoFail("Button event");
	}
	if (s_Counter.ConnectTask(s_Pei, butEvt, COUNTER_TASK_COUNT) == false)
	{
		PeiCounterDemoFail("COUNT task");
	}

	// Limit event to the LED toggle task
	if (s_Counter.EnablePeiEvent(s_Pei, limEvt) == false)
	{
		PeiCounterDemoFail("Limit event");
	}
	if (IOPinEnableTask(s_Pei, limEvt, s_Led.PortNo, s_Led.PinNo, IOPIN_TASK_TOGGLE) == false)
	{
		PeiCounterDemoFail("LED task");
	}

	if (s_Pei.Enable(butEvt) == false || s_Pei.Enable(limEvt) == false)
	{
		PeiCounterDemoFail("PEI enable");
	}
	if (s_Counter.Start() == false)
	{
		PeiCounterDemoFail("Counter start");
	}

	SysLogPrintf(SysLogGet(), "Counter DevNo %d, LED toggled every %u presses\r\n",
				 ((CounterDev_t *)s_Counter)->DevNo, (unsigned)s_Counter.Limit());

	uint64_t last = 0;

	while (1)
	{
		msDelay(100);

		uint64_t count = s_Counter.Count();

		if (count != last)
		{
			last = count;
			SysLogPrintf(SysLogGet(), "Count %u, limit events %u\r\n", (unsigned)count, (unsigned)s_NbLimit);
		}
	}

	return 0;
}
