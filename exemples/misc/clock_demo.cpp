/**-------------------------------------------------------------------------
@example	clock_demo.cpp

@brief	Core, peripheral and low-frequency clock selection demo.

Uses the generic clock, UART and Timer interfaces. The board supplies the
source sequence and peripheral clock divisors. Each change is made with the
timer stopped. Both build configurations log to UART without a debugger.
Timer elapsed time and interrupt counts check operation after clock changes.
Boards that defer timer initialization check it after the selection sequence.
These checks do not measure absolute oscillator accuracy. An optional board wake pin
adds sleep/resume checks with UART output drained and the timer stopped.

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
#include "iopinctrl.h"
#include "board.h"

#ifndef CLOCK_DEMO_APB_SHARED_CORE
#define CLOCK_DEMO_APB_SHARED_CORE 0
#endif

#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif
#ifndef CLOCK_DEMO_IRQ_PRIORITY
#define CLOCK_DEMO_IRQ_PRIORITY 6
#endif
#ifndef CLOCK_DEMO_CORE_READONLY
#define CLOCK_DEMO_CORE_READONLY 0
#endif
#ifndef CLOCK_DEMO_APB_READONLY
#define CLOCK_DEMO_APB_READONLY 0
#endif
#ifndef CLOCK_DEMO_DEFER_TIMER
#define CLOCK_DEMO_DEFER_TIMER 0
#endif
#ifndef CLOCK_DEMO_SLEEP_APB_DIVIDER
#define CLOCK_DEMO_SLEEP_APB_DIVIDER 4U
#endif
#ifndef CLOCK_DEMO_UART_RX_TEST
#define CLOCK_DEMO_UART_RX_TEST 0
#endif
#ifndef CLOCK_DEMO_STOP_UART
#define CLOCK_DEMO_STOP_UART 0
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
	.IntPrio = CLOCK_DEMO_IRQ_PRIORITY,
	.bFifoBlocking = true,
	.bDMAMode = false,
};
static UARTDev_t s_Uart;
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
static void ClockDemoFlush(void);

static const TimerCfg_t s_TimerCfg = {
	.DevNo = CLOCK_DEMO_TIMER_DEVNO,
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

static void ClockDemoResult(const char *pName, bool Passed)
{
	SysLogPrintf(SysLogGet(), "%s: %s\r\n", pName, Passed ? "PASS" : "FAIL");
	ClockDemoFlush();
	if (!Passed)
	{
		s_Failures++;
	}
}

#if CLOCK_DEMO_UART_RX_TEST
static void ClockDemoUartRx(void)
{
	uint8_t value;
	bool received = false;
	unsigned remaining = 1024U;

	// Discard bytes from the previous prompt before starting this check.
	while (remaining != 0U && UARTRx(&s_Uart, &value, 1) > 0)
	{
		remaining--;
	}
	if (remaining == 0U)
	{
		ClockDemoResult("UART RX", false);
		return;
	}
	if (!TimerEnable(&s_Timer))
	{
		TimerDisable(&s_Timer);
		ClockDemoResult("UART RX timer", false);
		return;
	}
	uint32_t start = TimerGetMicrosecond(&s_Timer);
	SysLogPrintf(SysLogGet(), "UART RX: send U within 30 s\r\n");
	while ((uint32_t)(TimerGetMicrosecond(&s_Timer) - start) < 30000000U)
	{
		// Drain output without discarding a response received during transmission.
		(void)SysLogFlush(SysLogGet());
		if (UARTRx(&s_Uart, &value, 1) > 0 && value == 'U')
		{
			received = true;
			break;
		}
	}
	TimerDisable(&s_Timer);
	ClockDemoResult("UART RX", received);
}
#endif

static bool ClockDemoTimerInit(void)
{
	if (!TimerInit(&s_Timer, &s_TimerCfg))
	{
		ClockDemoResult("Timer initialization", false);
		return false;
	}
	TimerDisable(&s_Timer);
	return true;
}

// Polling UART may accept only part of a syslog record. Drain both queues
// before stopping its clock, including the final bytes in the shift register.
static void ClockDemoFlush(void)
{
	uint8_t unused;

	while (CFifoUsed(SysLogGet()->hFifo) != 0 || CFifoUsed(s_Uart.hTxFifo) != 0)
	{
		(void)SysLogFlush(SysLogGet());
		(void)UARTRx(&s_Uart, &unused, 1);
	}
	msDelay(2);
}

static void ClockDemoClockBegin(void)
{
	ClockDemoFlush();
#if CLOCK_DEMO_STOP_UART
	UARTPowerOff(&s_Uart);
#endif
}

static bool ClockDemoClockEnd(void)
{
#if CLOCK_DEMO_STOP_UART
	return UARTInit(&s_Uart, &s_UartCfg);
#else
	return true;
#endif
}

static void ClockDemoTimer(void)
{
	for (unsigned delay = 0U; delay < 2U; delay++)
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
		if (delay == 0U)
		{
			msDelay(250);
		}
		else
		{
			nsDelay(250000000U);
		}
		TimerDisable(&s_Timer);
		uint32_t elapsed = TimerGetMicrosecond(&s_Timer) - start;
		uint32_t count = s_InterruptCount;
		TimerDisableTrigger(&s_Timer, 0);
		uint32_t expected = (uint64_t)elapsed * 1000U / period;
		uint32_t difference = count > expected ? count - expected : expected - count;
		SysLogPrintf(SysLogGet(), "Timer: %u Hz, %s 250 ms measured %u us, IRQ %u\r\n",
			(unsigned)frequency, delay == 0U ? "msDelay" : "nsDelay",
			(unsigned)elapsed, (unsigned)count);
		ClockDemoResult("Delay within 2 percent", elapsed >= 245000U && elapsed <= 255000U);
		ClockDemoResult("Timer count and IRQ", elapsed > 0U && count > 0U && difference <= 1U);
	}
}

#if defined(CLOCK_DEMO_WAKE_PIN)
static void ClockDemoSleep(void)
{
	uint32_t core = SystemCoreClockGet();
	uint32_t apb = SystemPeriphClockGet(0);
	OscDesc_t high = *GetHighFreqOscDesc();
	OscDesc_t low = *GetLowFreqOscDesc();
	unsigned released = 0U;

	// The board selects a supported peripheral-clock divider for sleep.
	msDelay(2);
	uint32_t sleepApb = SystemPeriphClockSet(0, core / CLOCK_DEMO_SLEEP_APB_DIVIDER);
	if (sleepApb == 0U)
	{
		ClockDemoResult("Sleep APB selection", false);
		return;
	}
	core = SystemCoreClockGet();
	IOPinConfig(CLOCK_DEMO_WAKE_PORT, CLOCK_DEMO_WAKE_PIN, IOPINOP_GPIO,
		IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	while (released < 20U)
	{
		released = IOPinRead(CLOCK_DEMO_WAKE_PORT, CLOCK_DEMO_WAKE_PIN) != 0 ?
			released + 1U : 0U;
		msDelay(1);
	}
	if (!SystemWakePin(CLOCK_DEMO_WAKE_PORT, CLOCK_DEMO_WAKE_PIN,
		IOPINSENSE_LOW_TRANSITION))
	{
		ClockDemoResult("Sleep wake pin", false);
		(void)SystemPeriphClockSet(0, apb);
		return;
	}
	SysLogPrintf(SysLogGet(), "Sleep: CPU %u Hz, APB %u Hz, LF %u Hz\r\n",
		(unsigned)core, (unsigned)sleepApb, (unsigned)low.Freq);
	SysLogPrintf(SysLogGet(), "Entering sleep; press " CLOCK_DEMO_WAKE_NAME " to resume\r\n");
	ClockDemoFlush();
	UARTDisable(&s_Uart);
	SystemSleep();
	UARTEnable(&s_Uart);
	(void)SystemWakePin(CLOCK_DEMO_WAKE_PORT, CLOCK_DEMO_WAKE_PIN, IOPINSENSE_DISABLE);
	SysLogPrintf(SysLogGet(), "Sleep returned; checking clocks\r\n");
	ClockDemoResult("CPU clock after resume", SystemCoreClockGet() == core &&
		GetHighFreqOscType() == high.Type && GetHighFreqOscFreq() == high.Freq);
	ClockDemoResult("APB clock after resume", SystemPeriphClockGet(0) == sleepApb);
	ClockDemoResult("LF descriptor after resume", GetLowFreqOscType() == low.Type &&
		GetLowFreqOscFreq() == low.Freq);
	ClockDemoTimer();
#if CLOCK_DEMO_UART_RX_TEST
	ClockDemoUartRx();
#endif
	msDelay(2);
	ClockDemoResult("APB restore", SystemPeriphClockSet(0, apb) == apb);
}
#endif

int main(void)
{
	if (!UARTInit(&s_Uart, &s_UartCfg) ||
		!SysLogInitNoTimer(SysLogGet(), &s_LogCfg, &s_Uart.DevIntrf, 0, 0))
	{
		return 1;
	}
	msDelay(1000);
	SysLogPrintf(SysLogGet(), "ClockDemo\r\nStartup CPU %u Hz, APB %u Hz\r\n",
		(unsigned)SystemCoreClockGet(), (unsigned)SystemPeriphClockGet(0));
	ClockDemoFlush();
#if !CLOCK_DEMO_DEFER_TIMER
	if (!ClockDemoTimerInit())
	{
		return 1;
	}
#if CLOCK_DEMO_UART_RX_TEST
	ClockDemoUartRx();
#endif
#endif
	if (sizeof(s_Sources) == 0U)
	{
		SysLogPrintf(SysLogGet(), "Core selection: SKIP (not configured)\r\n");
	}
	if (sizeof(s_ApbDividers) == 0U)
	{
		SysLogPrintf(SysLogGet(), "Peripheral clock setting: SKIP (not configured)\r\n");
	}

#if !defined(CLOCK_DEMO_LF_SOURCES)
	{
		SysLogPrintf(SysLogGet(), "LF selection: SKIP (not configured)\r\n");
	}
#endif

	for (unsigned cycle = 1U; cycle <= CLOCK_DEMO_CYCLES; cycle++)
	{
		SysLogPrintf(SysLogGet(), "Cycle %u\r\n", cycle);
		for (const ClockDemoSource_t &source : s_Sources)
		{
#if CLOCK_DEMO_CORE_READONLY
			SysLogPrintf(SysLogGet(), "Checking %s\r\n", source.pName);
#else
			SysLogPrintf(SysLogGet(), "Selecting %s\r\n", source.pName);
#endif
			// Polling Tx has queued the last bytes in the hardware FIFO.
			ClockDemoClockBegin();
#if CLOCK_DEMO_CORE_READONLY
			bool selected = !source.Select;
#else
			bool selected = !source.Select || SystemCoreClockSelect(source.Type, source.Frequency);
#endif
			if (!ClockDemoClockEnd())
			{
				return 1;
			}
			uint32_t core = SystemCoreClockGet();
			ClockDemoResult(source.pName, selected && core >= source.CoreMin && core <= source.CoreMax);
			if (!selected || core < source.CoreMin || core > source.CoreMax)
			{
				continue;
			}
#if !CLOCK_DEMO_APB_READONLY
			uint32_t bus = SystemPeriphClockGet(0);
			for (uint32_t divider : s_ApbDividers)
			{
				uint32_t request = bus / divider;
				ClockDemoClockBegin();
				uint32_t actual = SystemPeriphClockSet(0, request);
				if (!ClockDemoClockEnd())
				{
					return 1;
				}
				SysLogPrintf(SysLogGet(), "CPU %u Hz, APB request %u Hz, actual %u Hz\r\n",
					(unsigned)SystemCoreClockGet(), (unsigned)request, (unsigned)actual);
				ClockDemoResult("APB", actual == request &&
					SystemPeriphClockGet(0) == actual &&
					SystemCoreClockGet() == (CLOCK_DEMO_APB_SHARED_CORE ? actual : core));

#if !CLOCK_DEMO_DEFER_TIMER
				ClockDemoTimer();
#endif
			}
#endif
#if defined(CLOCK_DEMO_WAKE_PIN) && !CLOCK_DEMO_DEFER_TIMER
			unsigned lfIndex = 0U;
#endif
#if defined(CLOCK_DEMO_LF_SOURCES)
			for (const ClockDemoLowFreq_t &source : s_LowFreqSources)
			{
				SysLogPrintf(SysLogGet(), "Selecting LF %s\r\n", source.pName);
				ClockDemoClockBegin();
				bool selected = SystemLowFreqClockSelect(source.Type, source.Frequency);
				if (!ClockDemoClockEnd())
				{
					return 1;
				}
				ClockDemoResult(source.pName, selected);
				SysLogPrintf(SysLogGet(), "LF measured %u Hz\r\n", (unsigned)GetLowFreqOscFreq());
#if defined(CLOCK_DEMO_WAKE_PIN) && !CLOCK_DEMO_DEFER_TIMER
				// Rotate the LF source used for sleep across the three cycles.
				if (selected && lfIndex == (cycle - 1U) %
					(sizeof(s_LowFreqSources) / sizeof(s_LowFreqSources[0])))
				{
					ClockDemoSleep();
				}
				lfIndex++;
#endif
			}
#endif
		}
	}
#if CLOCK_DEMO_DEFER_TIMER
	// Some ports keep a timer's module clock until reset, even after Disable.
	// Complete clock selection before acquiring that timer.
	if (!ClockDemoTimerInit())
	{
		return 1;
	}
	ClockDemoTimer();
#if CLOCK_DEMO_UART_RX_TEST
	ClockDemoUartRx();
#endif
#if defined(CLOCK_DEMO_WAKE_PIN)
	for (unsigned cycle = 1U; cycle <= CLOCK_DEMO_CYCLES; cycle++)
	{
		SysLogPrintf(SysLogGet(), "Sleep cycle %u\r\n", cycle);
		ClockDemoSleep();
	}
#endif
#endif
	ClockDemoFlush();
	SysLogPrintf(SysLogGet(), "ClockDemo complete: %u failures\r\n", s_Failures);
	ClockDemoFlush();
	while (1)
	{
		msDelay(1000);
	}
}
