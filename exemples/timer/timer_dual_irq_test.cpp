/**-------------------------------------------------------------------------
@example	timer_dual_irq_test.cpp

@brief	ESP32-C3 TG0/T0 and TG1/T0 interrupt and alarm validation.

DevNo 0 uses the C++ Timer API; DevNo 1 uses the C TimerDev_t API.

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
		g_TimerUnexpectedEvents++;
		return;
	}
	devNo = pTimer->DevNo;
	if ((Evt & TIMER_EVT_TRIGGER(0)) == 0U || devNo < 0 || devNo > 1)
	{
		g_TimerUnexpectedEvents++;
		return;
	}
	g_TimerIrqCount[devNo]++;
}

static const TimerCfg_t s_Cfg0 = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000U,
	.IntPrio = 2,
	.EvtHandler = OnTimerEvent,
};

static const TimerCfg_t s_Cfg1 = {
	.DevNo = 1,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000U,
	.IntPrio = 2,
	.EvtHandler = OnTimerEvent,
};

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
	uint32_t count0;
	uint32_t count1;
	uint32_t stopped0;
	uint32_t stopped1;
	uint32_t singleStart;
	uint32_t singleEnd;
	uint32_t period0;
	uint64_t period1;
	int failures;
	int i;
	bool firstInit;
	bool secondInit;

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
	setvbuf(stdout, stdoutBuffer, _IOLBF, sizeof(stdoutBuffer));

	printf("\r\nESP32-C3 dual Timer Group IRQ test\r\n");
	printf("TG0/T0: C++ Timer, TG1/T0: C TimerDev_t\r\n");
	failures = 0;

	Check("two HF timer instances",
		TimerGetHighFreqDevCount() == 2 &&
		TimerGetHighFreqDevNo() == 0, &failures);

	firstInit = s_Timer0.Init(s_Cfg0);
	secondInit = TimerInit(&s_Timer1, &s_Cfg1);
	Check("TG0/T0 initialization at 1 MHz",
		firstInit && s_Timer0.Frequency() == 1000000U, &failures);
	Check("TG1/T0 initialization at 1 MHz",
		secondInit && TimerGetFrequency(&s_Timer1) == 1000000U, &failures);

	if (!firstInit || !secondInit)
	{
		msDelay(200U);
		return 1;
	}
	Check("one alarm per Timer Group",
		s_Timer0.MaxTimerTrigger() == 1 &&
		TimerGetMaxTrigger(&s_Timer1) == 1, &failures);

	period0 = s_Timer0.EnableTimerTrigger(0, (uint32_t)100U,
		TIMER_TRIG_TYPE_CONTINUOUS);
	period1 = msTimerEnableTrigger(&s_Timer1, 0, 250U,
		TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	Check("TG0 periodic alarm programmed to 100 ms",
		period0 == 100U, &failures);
	Check("TG1 periodic alarm programmed to 250 ms",
		period1 == 250U, &failures);

	if (period0 == 0U || period1 == 0U)
	{
		s_Timer0.Disable();
		TimerDisable(&s_Timer1);
		msDelay(200U);
		return 1;
	}

	for (i = 0; i < 30; i++)
	{
		msDelay(100U);
		if ((i + 1) % 10 == 0)
		{
			Snapshot(&count0, &count1);
			printf("  elapsed %d00 ms: TG0=%lu TG1=%lu\r\n",
				i + 1, (unsigned long)count0, (unsigned long)count1);
		}
	}

	Snapshot(&count0, &count1);
	Check("TG0 periodic interrupt callbacks, 100 ms",
		count0 >= 24U && count0 <= 36U, &failures);
	Check("TG1 periodic interrupt callbacks, 250 ms",
		count1 >= 9U && count1 <= 15U, &failures);
	Check("no unexpected timer events",
		g_TimerUnexpectedEvents == 0U, &failures);

	s_Timer0.DisableTimerTrigger(0);
	TimerDisableTrigger(&s_Timer1, 0);
	s_Timer0.Disable();
	TimerDisable(&s_Timer1);

	Snapshot(&stopped0, &stopped1);
	msDelay(300U);
	Snapshot(&count0, &count1);
	Check("both timers stop generating interrupts",
		count0 == stopped0 && count1 == stopped1, &failures);

	if (!TimerEnable(&s_Timer1))
	{
		Check("TG1 timer restart", false, &failures);
	}
	else
	{
		period1 = msTimerEnableTrigger(&s_Timer1, 0, 120U,
			TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr);
		Check("TG1 single-shot alarm programmed",
			period1 == 120U, &failures);

		Snapshot(&count0, &singleStart);
		msDelay(350U);
		Snapshot(&count0, &singleEnd);
		Check("TG1 single-shot fires exactly once",
			singleEnd == singleStart + 1U, &failures);
		msDelay(250U);
		Snapshot(&count0, &count1);
		Check("TG1 single-shot does not rearm",
			count1 == singleEnd, &failures);
		TimerDisable(&s_Timer1);
	}
	printf("%s: %d failed\r\n",
		failures == 0 ? "PASS" : "FAIL", failures);
	msDelay(200U);
	return failures == 0 ? 0 : 1;
}
