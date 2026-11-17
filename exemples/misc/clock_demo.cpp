/**-------------------------------------------------------------------------
@example	clock_demo.cpp

@brief	Core, peripheral and low-frequency clock selection demo.

Uses the generic clock, UART and Timer interfaces. The board supplies the
source sequence and peripheral clock divisors. Each change is made with the
timer stopped. Both build configurations log to UART without a debugger.
Timer elapsed time and interrupt counts check operation after each change;
they do not measure absolute oscillator accuracy.

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
#include "syslog.h"
#include "idelay.h"
#include "board.h"

struct ClockDemoSource_t {
	const char *pName;
	OSC_TYPE Type;
	uint32_t Frequency;
	uint32_t CoreFrequency;
};

struct ClockDemoLowFreq_t {
	const char *pName;
	OSC_TYPE Type;
	uint32_t Frequency;
};

static const ClockDemoSource_t s_Sources[] = CLOCK_DEMO_SOURCES;
static const ClockDemoLowFreq_t s_LowFreqSources[] = CLOCK_DEMO_LF_SOURCES;
static const uint32_t s_ApbDividers[] = CLOCK_DEMO_APB_DIVIDERS;
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
	.IntPrio = 6,
	.bFifoBlocking = true,
	.bDMAMode = false,
};
static UART s_Uart;
static TimerDev_t s_Timer;
static volatile uint32_t s_InterruptCount;
static unsigned s_Failures;
alignas(4) static uint8_t s_LogMem[SYSLOG_MEMSIZE(4, 160)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = 160,
	.bBlocking = true,
};

static void ClockDemoTimerHandler(TimerDev_t *pTimer, uint32_t Event);

static const TimerCfg_t s_TimerCfg = {
	.DevNo = CLOCK_DEMO_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1000000U,
	.IntPrio = 6,
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

static void ClockDemoResult(const char *pName, bool Passed)
{
	SysLogPrintf(SysLogGet(), "%s: %s\r\n", pName, Passed ? "PASS" : "FAIL");
	if (!Passed)
	{
		s_Failures++;
	}
}

static void ClockDemoTimer(void)
{
	// Refresh the divider after a peripheral-clock change, with timer stopped.
	uint32_t frequency = TimerSetFrequency(&s_Timer, s_TimerCfg.Freq);
	s_InterruptCount = 0U;
	uint64_t period = nsTimerEnableTrigger(&s_Timer, 0, 10000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	if (frequency == 0U || period == 0U || !TimerEnable(&s_Timer))
	{
		TimerDisableTrigger(&s_Timer, 0);
		TimerDisable(&s_Timer);
		ClockDemoResult("Timer start", false);
		return;
	}
	uint32_t start = TimerGetMicrosecond(&s_Timer);
	msDelay(250);
	TimerDisable(&s_Timer);
	uint32_t elapsed = TimerGetMicrosecond(&s_Timer) - start;
	uint32_t count = s_InterruptCount;
	TimerDisableTrigger(&s_Timer, 0);
	uint32_t expected = (uint64_t)elapsed * 1000U / period;
	uint32_t difference = count > expected ? count - expected : expected - count;
	SysLogPrintf(SysLogGet(), "Timer: %u Hz, delay 250 ms measured %u us, IRQ %u\r\n",
		(unsigned)frequency, (unsigned)elapsed, (unsigned)count);
	ClockDemoResult("Timer count and IRQ", elapsed > 0U && count > 0U && difference <= 1U);
}

int main(void)
{
	if (!s_Uart.Init(s_UartCfg) ||
		!SysLogInitNoTimer(SysLogGet(), &s_LogCfg, (DevIntrf_t *)s_Uart, 0, 0))
	{
		return 1;
	}
	msDelay(1000);
	SysLogPrintf(SysLogGet(), "ClockDemo\r\nStartup CPU %u Hz, APB %u Hz\r\n",
		(unsigned)SystemCoreClockGet(), (unsigned)SystemPeriphClockGet(0));
	if (!TimerInit(&s_Timer, &s_TimerCfg))
	{
		ClockDemoResult("Timer initialization", false);
		return 1;
	}
	TimerDisable(&s_Timer);

	for (unsigned cycle = 1U; cycle <= CLOCK_DEMO_CYCLES; cycle++)
	{
		SysLogPrintf(SysLogGet(), "Cycle %u\r\n", cycle);
		for (const ClockDemoSource_t &source : s_Sources)
		{
			SysLogPrintf(SysLogGet(), "Selecting %s\r\n", source.pName);
			bool selected = SystemCoreClockSelect(source.Type, source.Frequency);
			uint32_t core = SystemCoreClockGet();
			ClockDemoResult(source.pName, selected && core == source.CoreFrequency);
			if (!selected || core != source.CoreFrequency)
			{
				continue;
			}
			for (uint32_t divider : s_ApbDividers)
			{
				uint32_t request = source.CoreFrequency / divider;
				uint32_t actual = SystemPeriphClockSet(0, request);
				SysLogPrintf(SysLogGet(), "CPU %u Hz, APB request %u Hz, actual %u Hz\r\n",
					(unsigned)SystemCoreClockGet(), (unsigned)request, (unsigned)actual);
				ClockDemoResult("APB", actual == request &&
					SystemPeriphClockGet(0) == actual && SystemCoreClockGet() == core);
				ClockDemoTimer();
			}
			for (const ClockDemoLowFreq_t &source : s_LowFreqSources)
			{
				SysLogPrintf(SysLogGet(), "Selecting LF %s\r\n", source.pName);
				bool selected = SystemLowFreqClockSelect(source.Type, source.Frequency);
				ClockDemoResult(source.pName, selected);
				SysLogPrintf(SysLogGet(), "LF measured %u Hz\r\n", (unsigned)GetLowFreqOscFreq());
			}
		}
	}
	SysLogPrintf(SysLogGet(), "ClockDemo complete: %u failures\r\n", s_Failures);
	while (1)
	{
		msDelay(1000);
	}
}
