/**-------------------------------------------------------------------------
@example	timer_test.cpp

@brief	TimerTest: generic test of the timers and of the Timer API.

Every timer DevNo, the low power ones first, goes through the generic Timer
API: Init, frequency set and get, Init by another Timer object refused, count
against a reference timer, single and continuous triggers, Disable, Enable
and PowerOff. sDelay(1) is then timed by a high frequency timer: as the first
call after reset, with the low frequency clock held running, and with every
low power timer held so sDelay takes a high frequency one. Each is within
2 percent. Results go to the UART, the last line gives the failure count.

The test uses the generic interfaces only. board.h gives the UART pins.

@author	Hoang Nguyen Hoan
@date	Nov. 27, 2026

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

#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include "idelay.h"
#include "iopinctrl.h"
#include "board.h"

#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif
#ifndef TIMER_TEST_IRQ_PRIO
#define TIMER_TEST_IRQ_PRIO		IRQ_PRIO_LOW
#endif
// UART interrupt mode is the default, DMA a choice of the board
#ifndef TIMER_TEST_UART_INT
#define TIMER_TEST_UART_INT		true
#endif
#ifndef TIMER_TEST_UART_DMA
#define TIMER_TEST_UART_DMA		false
#endif

// Most low power timers held at once by the sDelay check
#define TIMER_TEST_MAX_LF		8
// Time the count is compared with the reference timer
#define TIMER_TEST_WINDOW_MS	200U
// Trigger period, and the triggers of a continuous trigger counted
#define TIMER_TEST_TRIG_MS		10U
#define TIMER_TEST_TRIG_CNT		10U

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = TIMER_TEST_UART_INT,
	.IntPrio = TIMER_TEST_IRQ_PRIO,
	.EvtCB = nullptr,
	.bFifoBlocking = true,
	.bDMAMode = TIMER_TEST_UART_DMA,
};

static const TimerCfg_t s_TimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0U,			// Highest rate of the timer
	.IntPrio = TIMER_TEST_IRQ_PRIO,
	.EvtHandler = nullptr,
	.bTickInt = false,
};

static UARTDev_t s_Uart;
static TimerDev_t s_Timer;		// Timer under test
static TimerDev_t s_Other;		// Another Timer object for the same DevNo
static TimerDev_t s_RefTimer;	// Reference of the times
static TimerDev_t s_LfTimer[TIMER_TEST_MAX_LF];
static volatile uint32_t s_TrigCount;
static unsigned s_Failures;

static void TimerTestResult(const char *pName, bool Passed)
{
	UARTprintf(&s_Uart, "%s: %s\r\n", pName, Passed ? "PASS" : "FAIL");
	if (!Passed)
	{
		s_Failures++;
	}
}

static void TimerTestTrigger(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	(void)pTimer;
	(void)TrigNo;
	(void)pContext;

	s_TrigCount = s_TrigCount + 1U;
}

// Reference of a timer: the first high frequency timer, or another timer
// when that one is under test
static int TimerTestRefDevNo(int DevNo)
{
	int hf;

	hf = TimerGetHighFreqDevNo();
	if (DevNo != hf)
	{
		return hf;
	}
	if (TimerGetHighFreqDevCount() > 1)
	{
		return hf + 1;
	}

	return TimerGetLowFreqDevCount() - 1;
}

static bool TimerTestRefInit(int DevNo)
{
	TimerCfg_t cfg;

	cfg = s_TimerCfg;
	cfg.DevNo = DevNo;

	return DevNo >= 0 && TimerInit(&s_RefTimer, &cfg);
}

// Count of the timer against the reference over the same time
static void TimerTestCount(int DevNo)
{
	uint32_t state;
	uint32_t refStart;
	uint32_t start;
	uint32_t refUs;
	uint32_t us;
	int ref;

	ref = TimerTestRefDevNo(DevNo);
	if (!TimerTestRefInit(ref))
	{
		TimerTestResult("Reference timer", false);
		return;
	}
	state = DisableInterrupt();
	refStart = TimerGetMicrosecond(&s_RefTimer);
	start = TimerGetMicrosecond(&s_Timer);
	EnableInterrupt(state);
	msDelay(TIMER_TEST_WINDOW_MS);
	state = DisableInterrupt();
	refUs = TimerGetMicrosecond(&s_RefTimer) - refStart;
	us = TimerGetMicrosecond(&s_Timer) - start;
	EnableInterrupt(state);
	(void)TimerPowerOff(&s_RefTimer);

	UARTprintf(&s_Uart, "Count %lu us, reference timer %d %lu us\r\n",
		(unsigned long)us, ref, (unsigned long)refUs);
	TimerTestResult("Count within 2 percent of the reference",
		refUs != 0U && us >= refUs - refUs / 50U && us <= refUs + refUs / 50U);
}

static void TimerTestTriggers(void)
{
	const uint32_t windowMs = TIMER_TEST_TRIG_MS * TIMER_TEST_TRIG_CNT + TIMER_TEST_TRIG_MS / 2U;
	uint64_t period;
	uint32_t expected;
	uint32_t count;
	int trig;

	trig = s_Timer.FindAvailTrigger != nullptr ? s_Timer.FindAvailTrigger(&s_Timer) : -1;
	TimerTestResult("Trigger available", TimerGetMaxTrigger(&s_Timer) > 0 && trig >= 0);
	if (trig < 0)
	{
		return;
	}

	s_TrigCount = 0U;
	period = nsTimerEnableTrigger(&s_Timer, trig, TIMER_TEST_TRIG_MS * 1000000ULL,
		TIMER_TRIG_TYPE_SINGLE, TimerTestTrigger, nullptr);
	msDelay(TIMER_TEST_TRIG_MS * 5U);
	count = s_TrigCount;
	TimerTestResult("Single trigger", period != 0U && count == 1U);

	s_TrigCount = 0U;
	period = nsTimerEnableTrigger(&s_Timer, trig, TIMER_TEST_TRIG_MS * 1000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, TimerTestTrigger, nullptr);
	msDelay(windowMs);
	TimerDisableTrigger(&s_Timer, trig);
	count = s_TrigCount;
	expected = period ? (uint32_t)(windowMs * 1000000ULL / period) : 0U;
	UARTprintf(&s_Uart, "Continuous trigger, period %lu us: %lu triggers in %lu ms\r\n",
		(unsigned long)(period / 1000U), (unsigned long)count,
		(unsigned long)windowMs);
	TimerTestResult("Continuous trigger", count >= 2U && expected >= 1U &&
		count >= expected - 1U && count <= expected + 1U);
	msDelay(TIMER_TEST_TRIG_MS * 3U);
	TimerTestResult("Trigger disabled", s_TrigCount == count);
}

// Init of the timer under test by another Timer object is refused. One that
// was not refused is powered off again.
static bool TimerTestRefused(const TimerCfg_t *pCfg)
{
	s_Other = {};
	if (!TimerInit(&s_Other, pCfg))
	{
		return true;
	}
	(void)TimerPowerOff(&s_Other);

	return false;
}

// The Timer API on one timer
static void TimerTestOne(int DevNo)
{
	TimerCfg_t cfg;
	uint64_t start;
	uint32_t freq;
	uint32_t asked;
	uint32_t got;
	bool ok;

	UARTprintf(&s_Uart, "Timer %d, %s\r\n", DevNo,
		DevNo < TimerGetLowFreqDevCount() ? "low power" : "high frequency");
	cfg = s_TimerCfg;
	cfg.DevNo = DevNo;
	ok = TimerInit(&s_Timer, &cfg);
	TimerTestResult("Init", ok);
	if (!ok)
	{
		return;
	}

	// The closest frequency the timer has, then back to the highest
	freq = TimerGetFrequency(&s_Timer);
	asked = freq / 2U;
	got = TimerSetFrequency(&s_Timer, asked);
	UARTprintf(&s_Uart, "Frequency %lu Hz, asked %lu Hz, got %lu Hz\r\n",
		(unsigned long)freq, (unsigned long)asked, (unsigned long)got);
	TimerTestResult("Frequency set and get", freq != 0U && got != 0U &&
		got <= freq && TimerGetFrequency(&s_Timer) == got);
	TimerTestResult("Frequency restored", TimerSetFrequency(&s_Timer, freq) == freq);

	TimerTestResult("Init refused while held", TimerTestRefused(&cfg));

	TimerTestCount(DevNo);
	TimerTestTriggers();

	TimerDisable(&s_Timer);
	TimerTestResult("Init refused while disabled", TimerTestRefused(&cfg));
	ok = TimerEnable(&s_Timer);
	start = TimerGetTickCount(&s_Timer);
	msDelay(10);
	TimerTestResult("Enable after Disable", ok && TimerGetTickCount(&s_Timer) != start);

	ok = TimerPowerOff(&s_Timer);
	TimerTestResult("PowerOff", ok && !TimerEnable(&s_Timer));
	ok = TimerInit(&s_Other, &cfg);
	TimerTestResult("Init after PowerOff", ok);
	if (ok)
	{
		(void)TimerPowerOff(&s_Other);
	}
}

// sDelay(1) timed by a high frequency timer
static void TimerTestSleepDelay(const char *pName)
{
	uint32_t start;
	uint32_t us;
	int32_t diff;
	bool ok;

	if (!TimerTestRefInit(TimerGetHighFreqDevNo()))
	{
		TimerTestResult("sDelay reference timer", false);
		return;
	}
	start = TimerGetMicrosecond(&s_RefTimer);
	ok = sDelay(1);
	us = TimerGetMicrosecond(&s_RefTimer) - start;
	(void)TimerPowerOff(&s_RefTimer);

	diff = (int32_t)us - 1000000;
	UARTprintf(&s_Uart, "sDelay(1) %s: %lu us, %ld us from 1 s\r\n", pName,
		(unsigned long)us, (long)diff);
	TimerTestResult("sDelay within 2 percent", ok && us >= 980000U && us <= 1020000U);
}

static void TimerTestSleepDelays(void)
{
	TimerCfg_t cfg;
	int count;
	int held;
	int i;

	TimerTestResult("sDelay(0)", sDelay(0));

	// The low frequency clock held by the test, already running for sDelay
	if (SystemLowFreqClockEnable())
	{
		TimerTestSleepDelay("low frequency clock running");
		SystemLowFreqClockDisable();
	}
	else
	{
		TimerTestResult("Low frequency clock", false);
	}

	// Every low power timer held, sDelay takes a high frequency one
	count = TimerGetLowFreqDevCount();
	if (count <= 0 || count > TIMER_TEST_MAX_LF)
	{
		return;
	}
	cfg = s_TimerCfg;
	held = 0;
	for (i = 0; i < count; i++)
	{
		cfg.DevNo = i;
		s_LfTimer[i] = {};
		if (TimerInit(&s_LfTimer[i], &cfg))
		{
			held++;
		}
	}
	TimerTestResult("Low power timers held", held == count);
	TimerTestSleepDelay("low power timers held");
	for (i = 0; i < count; i++)
	{
		(void)TimerPowerOff(&s_LfTimer[i]);
	}
}

int main(void)
{
	int count;
	int i;

	if (!UARTInit(&s_Uart, &s_UartCfg))
	{
		return 1;
	}
	UARTprintf(&s_Uart, "TimerTest\r\nCPU %lu Hz, %d low power and %d high frequency timers\r\n",
		(unsigned long)SystemCoreClockGet(), TimerGetLowFreqDevCount(),
		TimerGetHighFreqDevCount());

	// The first call after reset, the low frequency clock as it starts
	TimerTestSleepDelay("first call");

	count = TimerGetLowFreqDevCount() + TimerGetHighFreqDevCount();
	for (i = 0; i < count; i++)
	{
		TimerTestOne(i);
	}

	TimerTestSleepDelays();

	UARTprintf(&s_Uart, "TimerTest complete: %u failures\r\n", s_Failures);
	msDelay(100);

	while (1)
	{
		__WFI();
	}
}
