/**-------------------------------------------------------------------------
@example	clock_demo.cpp

@brief	Core, peripheral and low-frequency clock selection demo.

Uses the generic clock, UART and Timer interfaces. The board supplies the
source sequence and peripheral clock divisors. Each change is made with the
timer stopped. Both build configurations log to UART without a debugger.
Timer elapsed time and interrupt counts check operation after clock changes.
Clock and timer checks use only the public IOsonata HAL interfaces.
These checks do not measure absolute oscillator accuracy. PowerOffDemo tests
low-power modes, wake sources and peripheral operation following resume.

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
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "coredev/uart.h"
#include <stdarg.h>
#include "idelay.h"
#include "iopinctrl.h"
#include "board.h"

#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif
#ifndef CLOCK_DEMO_IRQ_PRIORITY
#define CLOCK_DEMO_IRQ_PRIORITY 6
#endif

// The generic API is declared for every target. A weak reference lets the
// diagnostic report a missing MCU implementation instead of failing at link
// time. A missing operation is a test failure, never an implicit PASS.
#if defined(__GNUC__)
#pragma weak SystemHighFreqClockEnable
#pragma weak SystemHighFreqClockDisable
#pragma weak SystemLowFreqClockEnable
#pragma weak SystemLowFreqClockDisable
#endif

struct ClockDemoSource_t {
	const char *pName;
	OSC_TYPE Type;
	uint32_t Frequency;
	uint32_t CoreMin;
	uint32_t CoreMax;
	bool Select;

	constexpr ClockDemoSource_t(const char *pLabel, OSC_TYPE Osc, uint32_t Freq,
		uint32_t Min, uint32_t Max, bool bSelect = true) :
		pName(pLabel), Type(Osc), Frequency(Freq), CoreMin(Min), CoreMax(Max), Select(bSelect)
	{
	}
};

struct ClockDemoLowFreq_t {
	const char *pName;
	OSC_TYPE Type;
	uint32_t Frequency;
};

static const ClockDemoSource_t s_Sources[] = CLOCK_DEMO_SOURCES;
#if defined(CLOCK_DEMO_LF_SOURCES)
static const ClockDemoLowFreq_t s_LowFreqSources[] = CLOCK_DEMO_LF_SOURCES;
#endif
static const uint32_t s_PeriphDividers[] = CLOCK_DEMO_PERIPH_DIVIDERS;
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
	.bIntMode = false,
	.IntPrio = CLOCK_DEMO_IRQ_PRIORITY,
	.bFifoBlocking = true,
	.bDMAMode = false,
};
static UARTDev_t s_Uart;
static TimerDev_t s_Timer;
static TimerDev_t s_RetainedTimer;
static volatile uint32_t s_InterruptCount;
static unsigned s_Failures;
static void ClockDemoTimerHandler(TimerDev_t *pTimer, uint32_t Event);
static void ClockDemoFlush(void);

static const TimerCfg_t s_TimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000U,
	.IntPrio = CLOCK_DEMO_IRQ_PRIORITY,
	.EvtHandler = ClockDemoTimerHandler,
};

static void ClockDemoTimerHandler(TimerDev_t *pTimer, uint32_t Event)
{
	(void)pTimer;
	if ((Event & TIMER_EVT_TRIGGER(0)) != 0U)
	{
		s_InterruptCount = s_InterruptCount + 1U;
	}
}

// This demo prints only strings and unsigned decimal values. Direct polling
// output keeps the clock checks small enough for the 16 KB targets.
static void ClockDemoPut(const char *pText)
{
	uint8_t value;

	while (*pText != 0)
	{
		value = (uint8_t)*pText;
		if (UARTTx(&s_Uart, &value, 1) == 1)
		{
			pText++;
		}
	}
}

static void ClockDemoPrint(const char *pFormat, ...)
{
	va_list args;
	char text[11];
	unsigned value;
	unsigned pos;

	va_start(args, pFormat);
	while (*pFormat != 0)
	{
		if (*pFormat != '%')
		{
			text[0] = *pFormat++;
			text[1] = 0;
			ClockDemoPut(text);
		}
		else
		{
			pFormat++;
			if (*pFormat == 's')
			{
				ClockDemoPut(va_arg(args, const char *));
			}
			else if (*pFormat == 'u')
			{
				value = va_arg(args, unsigned);
				pos = sizeof(text) - 1U;
				text[pos] = 0;
				do
				{
					text[--pos] = '0' + value % 10U;
					value /= 10U;
				} while (value != 0U);
				ClockDemoPut(text + pos);
			}
			if (*pFormat != 0)
			{
				pFormat++;
			}
		}
	}
	va_end(args);
}

static void ClockDemoResult(const char *pName, bool Passed)
{
	ClockDemoPrint("%s: %s\r\n", pName, Passed ? "PASS" : "FAIL");
	ClockDemoFlush();
	if (!Passed)
	{
		s_Failures++;
	}
}

// Exercise the same shared oscillator reference-count API on every MCU.
// Physical oscillator on/off and power measurements require target tests.
static void ClockDemoSharedClockTest(void)
{
	bool first;
	bool second;
	bool third;

	if (SystemHighFreqClockEnable == nullptr ||
		SystemHighFreqClockDisable == nullptr)
	{
		ClockDemoResult("HF shared clock API implemented", false);
	}
	else
	{
		first = SystemHighFreqClockEnable();
		second = first && SystemHighFreqClockEnable();
		if (second)
		{
			SystemHighFreqClockDisable();
		}
		if (first)
		{
			SystemHighFreqClockDisable();
		}
		// An unmatched release must not affect a later clock request.
		SystemHighFreqClockDisable();
		third = SystemHighFreqClockEnable();
		if (third)
		{
			SystemHighFreqClockDisable();
		}
		ClockDemoResult("HF shared clock lifecycle",
			first && second && third);
	}

	if (SystemLowFreqClockEnable == nullptr ||
		SystemLowFreqClockDisable == nullptr)
	{
		ClockDemoResult("LF shared clock API implemented", false);
	}
	else
	{
		first = SystemLowFreqClockEnable();
		second = first && SystemLowFreqClockEnable();
		if (second)
		{
			SystemLowFreqClockDisable();
		}
		if (first)
		{
			SystemLowFreqClockDisable();
		}
		SystemLowFreqClockDisable();
		third = SystemLowFreqClockEnable();
		if (third)
		{
			SystemLowFreqClockDisable();
		}
		ClockDemoResult("LF shared clock lifecycle",
			first && second && third);
	}
}

static bool ClockDemoTimerInit(int DevNo)
{
	TimerCfg_t cfg;

	cfg = s_TimerCfg;
	cfg.DevNo = DevNo;
	if (!TimerInit(&s_Timer, &cfg))
	{
		ClockDemoResult("Timer initialization", false);
		return false;
	}
	TimerDisable(&s_Timer);
	return true;
}

// Allow the polling UART to finish its last byte before clock changes.
static void ClockDemoFlush(void)
{
	msDelay(2);
}

static void ClockDemoTimer(int DevNo)
{
	unsigned delay;
	uint32_t frequency;
	uint64_t period;
	uint32_t start;
	uint32_t elapsed;
	uint32_t count;
	uint32_t expected;
	uint32_t difference;

	ClockDemoPrint("Timer device %u\r\n", (unsigned)DevNo);
	if (!ClockDemoTimerInit(DevNo))
	{
		return;
	}
	for (delay = 0U; delay < 2U; delay++)
	{
		// Refresh the divider after a peripheral-clock change, with timer stopped.
		frequency = TimerSetFrequency(&s_Timer, s_TimerCfg.Freq);
		s_InterruptCount = 0U;
		period = nsTimerEnableTrigger(&s_Timer, 0, 10000000ULL,
			TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
		if (frequency == 0U || period == 0U || !TimerEnable(&s_Timer))
		{
			TimerDisableTrigger(&s_Timer, 0);
			TimerDisable(&s_Timer);
			ClockDemoResult("Timer start", false);
			return;
		}
		start = TimerGetMicrosecond(&s_Timer);
		if (delay == 0U)
		{
			msDelay(250);
		}
		else
		{
			nsDelay(250000000U);
		}
		TimerDisable(&s_Timer);
		elapsed = TimerGetMicrosecond(&s_Timer) - start;
		count = s_InterruptCount;
		TimerDisableTrigger(&s_Timer, 0);
		expected = (uint64_t)elapsed * 1000U / period;
		difference = count > expected ? count - expected : expected - count;
		ClockDemoPrint("Timer: %u Hz, %s 250 ms measured %u us, IRQ %u\r\n",
			(unsigned)frequency, delay == 0U ? "msDelay" : "nsDelay",
			(unsigned)elapsed, (unsigned)count);
		ClockDemoResult("Delay within 2 percent", elapsed >= 245000U && elapsed <= 255000U);
		ClockDemoResult("Timer count and IRQ", elapsed > 0U && count > 0U && difference <= 1U);
	}
}


// Check that disabling one timer does not stop an independently
// configured timer. Prefer one LF and one HF device when available.
static void ClockDemoConcurrentTimerTest(int LowCount, int HighCount)
{
	TimerCfg_t firstCfg;
	TimerCfg_t secondCfg;
	int secondNo;
	uint32_t start;
	uint32_t elapsed;
	bool first;
	bool second;

	if (LowCount + HighCount < 2)
	{
		ClockDemoPrint("Concurrent timers: UNSUPPORTED (one device)\r\n");
		return;
	}
	secondNo = LowCount > 0 && HighCount > 0 ? LowCount : 1;
	firstCfg = s_TimerCfg;
	secondCfg = s_TimerCfg;
	firstCfg.DevNo = 0;
	secondCfg.DevNo = secondNo;
	firstCfg.EvtHandler = nullptr;
	secondCfg.EvtHandler = nullptr;
	first = TimerInit(&s_Timer, &firstCfg);
	second = TimerInit(&s_RetainedTimer, &secondCfg);
	if (!first || !second)
	{
		ClockDemoResult("Concurrent timer initialization", false);
		return;
	}
	first = TimerEnable(&s_Timer);
	second = TimerEnable(&s_RetainedTimer);
	if (first && second)
	{
		start = TimerGetMicrosecond(&s_RetainedTimer);
		TimerDisable(&s_Timer);
		msDelay(30);
		elapsed = TimerGetMicrosecond(&s_RetainedTimer) - start;
		ClockDemoResult("Timer independent clock ownership", elapsed > 0U);
	}
	else
	{
		ClockDemoResult("Concurrent timer enable", false);
	}
	TimerDisable(&s_Timer);
	TimerDisable(&s_RetainedTimer);
}

int main(void)
{
	unsigned cycle;
	unsigned sourceIdx;
	bool selected;
	const ClockDemoSource_t *pSource;
#if defined(CLOCK_DEMO_LF_SOURCES)
	unsigned lfIdx;
	const ClockDemoLowFreq_t *pLfSource;
#endif
	uint32_t core;
	uint32_t bus;
	uint32_t readback;
	unsigned dividerIdx;
	unsigned dividerCount;
	uint32_t divider;
	uint32_t request;
	uint32_t actual;
	int clockIdx;
	unsigned domainCount;
	int timerDevNo;
	int timerCount;
	int lowTimerCount;
	int highTimerCount;
	if (!UARTInit(&s_Uart, &s_UartCfg))
	{
		return 1;
	}
	msDelay(1000);
	ClockDemoPrint("ClockDemo\r\nStartup CPU %u Hz, peripheral clock 0 %u Hz\r\n",
		(unsigned)SystemCoreClockGet(), (unsigned)SystemPeriphClockGet(0));
	ClockDemoFlush();
	ClockDemoSharedClockTest();
	if (sizeof(s_Sources) == 0U)
	{
		ClockDemoPrint("Core selection: SKIP (not configured)\r\n");
	}

#if !defined(CLOCK_DEMO_LF_SOURCES)
	{
		ClockDemoPrint("LF selection: SKIP (not configured)\r\n");
	}
#endif

	domainCount = 0U;
	for (cycle = 1U; cycle <= CLOCK_DEMO_CYCLES; cycle++)
	{
		ClockDemoPrint("Cycle %u\r\n", cycle);
		for (sourceIdx = 0U; sourceIdx < sizeof(s_Sources) / sizeof(s_Sources[0]); sourceIdx++)
		{
			pSource = &s_Sources[sourceIdx];
			ClockDemoPrint("%s %s\r\n", pSource->Select ? "Selecting" : "Checking",
				pSource->pName);
			ClockDemoFlush();
			selected = true;
			if (pSource->Select)
			{
				selected = SystemCoreClockSelect(pSource->Type, pSource->Frequency);
			}
			core = SystemCoreClockGet();
			ClockDemoResult(pSource->pName, selected && core >= pSource->CoreMin && core <= pSource->CoreMax);
			if (!selected || core < pSource->CoreMin || core > pSource->CoreMax)
			{
				continue;
			}
			// Check every possible peripheral clock index in a bounded
			// range, not just index zero. An index returning zero is
			// unsupported; all nonzero domains receive Get/Set tests.
			for (clockIdx = 0; clockIdx < 32; clockIdx++)
			{
				bus = SystemPeriphClockGet(clockIdx);
				if (bus == 0U)
				{
					continue;
				}
				domainCount++;
				ClockDemoPrint("Peripheral clock %u baseline %u Hz\r\n",
					(unsigned)clockIdx, (unsigned)bus);
				dividerCount = sizeof(s_PeriphDividers) / sizeof(s_PeriphDividers[0]);
				for (dividerIdx = 0U; dividerIdx <= dividerCount; dividerIdx++)
				{
					divider = dividerIdx == 0U ? 1U : s_PeriphDividers[dividerIdx - 1U];
					if (divider == 0U || bus / divider == 0U)
					{
						ClockDemoResult("Peripheral clock request", false);
						continue;
					}
					request = bus / divider;
					ClockDemoFlush();
					actual = SystemPeriphClockSet(clockIdx, request);
					readback = SystemPeriphClockGet(clockIdx);
					ClockDemoPrint("Domain %u request %u Hz, actual %u Hz\r\n",
						(unsigned)clockIdx, (unsigned)request, (unsigned)actual);
					ClockDemoResult("Peripheral clock", actual != 0U &&
						readback == actual && (request != bus || actual == bus));
				}
				actual = SystemPeriphClockSet(clockIdx, bus);
				ClockDemoResult("Peripheral clock restore",
					actual == bus && SystemPeriphClockGet(clockIdx) == bus);
			}
#if defined(CLOCK_DEMO_LF_SOURCES)
			for (lfIdx = 0U; lfIdx < sizeof(s_LowFreqSources) / sizeof(s_LowFreqSources[0]); lfIdx++)
			{
				pLfSource = &s_LowFreqSources[lfIdx];
				ClockDemoPrint("Selecting LF %s\r\n", pLfSource->pName);
				ClockDemoFlush();
				selected = SystemLowFreqClockSelect(pLfSource->Type, pLfSource->Frequency);
				ClockDemoResult(pLfSource->pName, selected);
				ClockDemoPrint("LF configured %u Hz\r\n", (unsigned)GetLowFreqOscFreq());
			}
#endif
		}
	}
	if (SystemPeriphClockGet(-1) != 0U ||
		SystemPeriphClockGet(32) != 0U)
	{
		ClockDemoResult("Peripheral clock invalid index", false);
	}
	else
	{
		ClockDemoResult("Peripheral clock invalid index", true);
	}
	if (domainCount == 0U)
	{
		ClockDemoResult("Peripheral clock domains present", false);
	}
	// Timer implementations expose their device counts through the generic API.
	// Test every advertised device rather than a board-selected instance.
	lowTimerCount = TimerGetLowFreqDevCount();
	highTimerCount = TimerGetHighFreqDevCount();
	timerCount = lowTimerCount + highTimerCount;
	if (timerCount == 0)
	{
		ClockDemoPrint("Timers: UNSUPPORTED (no devices)\r\n");
	}
	for (timerDevNo = 0; timerDevNo < timerCount; timerDevNo++)
	{
		ClockDemoTimer(timerDevNo);
	}
	ClockDemoConcurrentTimerTest(lowTimerCount, highTimerCount);
	ClockDemoFlush();
	ClockDemoPrint("ClockDemo complete: %u failures\r\n", s_Failures);
	ClockDemoFlush();
	while (1)
	{
		msDelay(1000);
	}
}
