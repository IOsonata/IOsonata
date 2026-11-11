/**-------------------------------------------------------------------------
@example	timer_dual_irq_test.cpp

@brief	Generic one/two high-frequency timer interrupt and alarm validation.

The first timer uses C++ initialization; an optional second uses the C API.
Single-shot checks use the C API on both, including single-timer targets.

@author	IOsonata contributors
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
#include <stdio.h>
#include <unistd.h>

#include "idelay.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "board.h"

#if defined(MCUOSC)
McuOsc_t g_McuOsc = MCUOSC;
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

alignas(4) static uint8_t s_UartTxMem[CFIFO_MEMSIZE(256)];
static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = IRQ_PRIO_NORMAL,
	.bFifoBlocking = true,
	.TxMemSize = sizeof(s_UartTxMem),
	.pTxMem = s_UartTxMem,
	.bDMAMode = false,
};

static UART s_Uart;
static Timer s_Timer0;
static TimerDev_t s_Timer1 = {};

volatile uint32_t g_TimerIrqCount[2] = {0U, 0U};
volatile uint32_t g_TimerUnexpectedEvents = 0U;

static void OnTimerEvent(TimerDev_t * const pTimer, uint32_t Evt)
{
	int devNo;

	if (pTimer == nullptr)
	{
		g_TimerUnexpectedEvents = g_TimerUnexpectedEvents + 1;
		return;
	}
	devNo = pTimer == (TimerDev_t *)s_Timer0 ? 0 :
		(pTimer == &s_Timer1 ? 1 : -1);
	if ((Evt & TIMER_EVT_TRIGGER(0)) == 0U || devNo < 0 || devNo > 1)
	{
		g_TimerUnexpectedEvents = g_TimerUnexpectedEvents + 1;
		return;
	}
	g_TimerIrqCount[devNo] = g_TimerIrqCount[devNo] + 1;
}

static void Snapshot(uint32_t *pCount0, uint32_t *pCount1)
{
	uint32_t state;

	state = DisableInterrupt();
	*pCount0 = g_TimerIrqCount[0];
	*pCount1 = g_TimerIrqCount[1];
	EnableInterrupt(state);
}

static void Check(const char *pName, bool Success, int *pFailures)
{
	printf("  [%s] %s\r\n", Success ? "PASS" : "FAIL", pName);
	if (!Success)
	{
		(*pFailures)++;
	}
}

int main(void)
{
	static char stdoutBuffer[256];
	TimerCfg_t config = {};
	TimerDev_t *timers[2];
	uint32_t counts[2];
	uint32_t stopped[2];
	uint32_t singleStart[2];
	uint32_t singleEnd[2];
	uint64_t period;
	int failures;
	int count;
	int i;
	bool initialized;

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, stdoutBuffer, _IOLBF, sizeof(stdoutBuffer));
	printf("\r\nGeneric timer IRQ test\r\n");
	failures = 0;
	count = TimerGetHighFreqDevCount();
	Check("at least one HF timer", count > 0, &failures);
	if (count <= 0)
	{
		msDelay(200U);
		return 1;
	}
	if (count > 2)
	{
		count = 2;
	}
	printf("Testing %d timer(s), first DevNo %d\r\n",
		count, TimerGetHighFreqDevNo());
	timers[0] = s_Timer0;
	timers[1] = &s_Timer1;
	config.DevNo = TimerGetHighFreqDevNo();
	config.ClkSrc = TIMER_CLKSRC_DEFAULT;
	config.Freq = 1000000U;
	config.IntPrio = 2;
	config.EvtHandler = OnTimerEvent;
	initialized = s_Timer0.Init(config);
	Check("first timer initialized through C++", initialized, &failures);
	if (!initialized)
	{
		msDelay(200U);
		return 1;
	}
	if (count == 2)
	{
		config.DevNo++;
		initialized = TimerInit(&s_Timer1, &config);
		Check("second timer initialized through C", initialized, &failures);
	}
	if (!initialized)
	{
		s_Timer0.Disable();
		msDelay(200U);
		return 1;
	}
	for (i = 0; i < count; i++)
	{
		Check("timer frequency is 1 MHz",
			TimerGetFrequency(timers[i]) == 1000000U, &failures);
		Check("timer has an alarm", TimerGetMaxTrigger(timers[i]) > 0, &failures);
	}
	period = s_Timer0.EnableTimerTrigger(0, (uint32_t)100U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	Check("first periodic alarm: 100 ms", period == 100U, &failures);
	if (count == 2)
	{
		period = msTimerEnableTrigger(&s_Timer1, 0, 250U,
			TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
		Check("second periodic alarm: 250 ms", period == 250U, &failures);
	}
	for (i = 0; i < 3; i++)
	{
		msDelay(1000U);
		Snapshot(&counts[0], &counts[1]);
		printf("  elapsed %d s: first=%lu second=%lu\r\n", i + 1,
			(unsigned long)counts[0], (unsigned long)counts[1]);
	}
	Check("first periodic callbacks: about 30",
		counts[0] >= 24U && counts[0] <= 36U, &failures);
	if (count == 2)
	{
		Check("second periodic callbacks: about 12",
			counts[1] >= 9U && counts[1] <= 15U, &failures);
	}
	for (i = 0; i < count; i++)
	{
		TimerDisableTrigger(timers[i], 0);
		TimerDisable(timers[i]);
	}
	Snapshot(&stopped[0], &stopped[1]);
	msDelay(300U);
	Snapshot(&counts[0], &counts[1]);
	Check("disabled timers stop callbacks",
		counts[0] == stopped[0] && counts[1] == stopped[1], &failures);
	Snapshot(&singleStart[0], &singleStart[1]);
	for (i = 0; i < count; i++)
	{
		Check("restart through C API", TimerEnable(timers[i]), &failures);
		period = msTimerEnableTrigger(timers[i], 0, 120U,
			TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr);
		Check("single-shot alarm: 120 ms", period == 120U, &failures);
	}
	msDelay(350U);
	Snapshot(&singleEnd[0], &singleEnd[1]);
	for (i = 0; i < count; i++)
	{
		Check("single-shot fires exactly once",
			singleEnd[i] == singleStart[i] + 1U, &failures);
	}
	msDelay(250U);
	Snapshot(&counts[0], &counts[1]);
	for (i = 0; i < count; i++)
	{
		Check("single-shot does not rearm", counts[i] == singleEnd[i], &failures);
		TimerDisable(timers[i]);
	}
	Check("no unexpected timer events", g_TimerUnexpectedEvents == 0U, &failures);
	printf("%s: %d failed\r\n", failures == 0 ? "PASS" : "FAIL", failures);
	msDelay(200U);
	return failures == 0 ? 0 : 1;
}
