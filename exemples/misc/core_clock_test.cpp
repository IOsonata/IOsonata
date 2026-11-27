/**-------------------------------------------------------------------------
@file	core_clock_test.cpp

@brief	CoreClockTest: generic System Core clock and delay validation.

Uses the generic clock, UART and Timer interfaces. Oscillator selections
come from the System Core configuration, not from board-specific test lists.
Both build configurations log to UART without a debugger.
One LF timer and one HF timer check counting after clock requests are
released. Delay calibration can reuse the HF timer at each CPU speed.

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
// UART interrupt mode is the generic default. Boards lacking IRQ support
// may override this option; DMA remains an independent choice.
#ifndef CORE_CLOCK_TEST_UART_INT
#define CORE_CLOCK_TEST_UART_INT true
#endif
#ifndef CORE_CLOCK_TEST_UART_DMA
#define CORE_CLOCK_TEST_UART_DMA false
#endif

#ifndef CORE_CLOCK_TEST_DELAY_CALIBRATION
#define CORE_CLOCK_TEST_DELAY_CALIBRATION 1
#endif

#define CLOCK_TEST_CYCLES	3U

// The generic API is declared for every target. A weak reference lets the
// diagnostic report a missing MCU implementation instead of failing at link
// time. A missing operation is a test failure, never an implicit PASS.
#if defined(__GNUC__)
#pragma weak SystemPowerModeSet
#pragma weak SystemPowerModeGet
#pragma weak SystemHighFreqClockEnable
#pragma weak SystemHighFreqClockDisable
#pragma weak SystemLowFreqClockEnable
#pragma weak SystemLowFreqClockDisable
#pragma weak SystemHighFreqClockCalibrate
#pragma weak SystemLowFreqClockCalibrate
#endif

static const IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
};
static int CoreClockTestUartEvent(DevIntrf_t *pDev, DEVINTRF_EVT Event,
	uint8_t *pBuffer, int Length);

static const UARTCfg_t s_UartCfg = {
	.DevNo = UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(s_UartPins[0]),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = CORE_CLOCK_TEST_UART_INT,
	.IntPrio = CLOCK_DEMO_IRQ_PRIORITY,
	.EvtCB = CORE_CLOCK_TEST_UART_INT && CORE_CLOCK_TEST_UART_DMA ?
		CoreClockTestUartEvent : nullptr,
	.bFifoBlocking = true,
	.bDMAMode = CORE_CLOCK_TEST_UART_DMA,
};
static UARTDev_t s_Uart;
static TimerDev_t s_Timer;
static unsigned s_Failures;
static volatile bool s_UartTxEmpty = true;

static void CoreClockTestDelayCalibrationRecord(void);
static void CoreClockTestFlush(void);

static const TimerCfg_t s_TimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0U,			// Highest supported rate for each timer device
	.IntPrio = CLOCK_DEMO_IRQ_PRIORITY,
	.EvtHandler = nullptr,
};

// The DMA UART driver raises TX_FIFO_EMPTY only after the final byte
// completes. Keep the diagnostic independent of UART FIFO internals.
static int CoreClockTestUartEvent(DevIntrf_t *pDev, DEVINTRF_EVT Event,
	uint8_t *pBuffer, int Length)
{
	(void)pDev;
	(void)pBuffer;
	(void)Length;

	if (Event == DEVINTRF_EVT_TX_FIFO_EMPTY)
	{
		s_UartTxEmpty = true;
	}
	return 0;
}

// Print through bounded multi-byte submissions rather than one EasyDMA
// transaction per character. The UART driver copies accepted bytes into
// its own FIFO; it does not retain this stack buffer. This remains generic
// for MCUs without DMA and requires no full printf/heap implementation.
#define CORE_CLOCK_TEST_TX_BLOCK_SIZE	64U
#define CORE_CLOCK_TEST_TX_RETRY_LIMIT	5000U

static void CoreClockTestWrite(const char *pText, unsigned Length)
{
	unsigned offset;
	unsigned retries;
	int accepted;

	offset = 0U;
	retries = 0U;
	while (offset < Length)
	{
		// Completion belongs to the last accepted byte, not a prior DMA.
		if (CORE_CLOCK_TEST_UART_INT && CORE_CLOCK_TEST_UART_DMA)
		{
			s_UartTxEmpty = false;
		}
		accepted = UARTTx(&s_Uart, (uint8_t *)(pText + offset),
			(int)(Length - offset));
		if (accepted > 0)
		{
			offset += (unsigned)accepted;
			retries = 0U;
		}
		else
		{
			if (++retries >= CORE_CLOCK_TEST_TX_RETRY_LIMIT)
			{
				s_Failures++;
				return;
			}
			usDelay(100U);
		}
	}
}

static void CoreClockTestAppend(char *pBuffer, unsigned *pUsed, char Value)
{
	if (*pUsed == CORE_CLOCK_TEST_TX_BLOCK_SIZE)
	{
		CoreClockTestWrite(pBuffer, *pUsed);
		*pUsed = 0U;
	}
	pBuffer[*pUsed] = Value;
	(*pUsed)++;
}

static void CoreClockTestPrint(const char *pFormat, ...)
{
	va_list args;
	const char *pString;
	char ch;
	char block[CORE_CLOCK_TEST_TX_BLOCK_SIZE];
	char digits[10];
	unsigned used;
	unsigned digitCount;
	unsigned value;

	used = 0U;
	va_start(args, pFormat);
	while (*pFormat != 0)
	{
		ch = *pFormat++;
		if (ch != '%')
		{
			CoreClockTestAppend(block, &used, ch);
			continue;
		}
		// Test messages use only non-null strings and unsigned integers.
		if (*pFormat++ == 's')
		{
			pString = va_arg(args, const char *);
			while (*pString != 0)
			{
				CoreClockTestAppend(block, &used, *pString++);
			}
		}
		else
		{
			value = va_arg(args, unsigned);
			digitCount = sizeof(digits);
			do
			{
				digits[--digitCount] = '0' + value % 10U;
				value /= 10U;
			} while (value != 0U);
			while (digitCount < sizeof(digits))
			{
				CoreClockTestAppend(block, &used, digits[digitCount++]);
			}
		}
	}
	va_end(args);
	if (used != 0U)
	{
		CoreClockTestWrite(block, used);
	}
}

static void CoreClockTestResult(const char *pName, bool Passed)
{
	CoreClockTestPrint("%s: %s\r\n", pName, Passed ? "PASS" : "FAIL");
	CoreClockTestFlush();
	if (!Passed)
	{
		s_Failures++;
	}
}

// Some older ports only implement Disable.
static void CoreClockTestReleaseTimer(void)
{
	if (!TimerPowerOff(&s_Timer))
	{
		TimerDisable(&s_Timer);
	}
}

// Two users share the oscillator, release their requests, then request it again.
static bool CoreClockTestRequests(bool (*Enable)(void), void (*Disable)(void))
{
	bool second;
	bool restarted;

	if (Enable == nullptr || Disable == nullptr || !Enable())
	{
		return false;
	}
	second = Enable();
	if (second)
	{
		Disable();
	}
	Disable();
	restarted = Enable();
	if (restarted)
	{
		Disable();
	}
	return second && restarted;
}

static bool CoreClockTestInitHighTimer(void)
{
	TimerCfg_t cfg = s_TimerCfg;

	cfg.DevNo = TimerGetHighFreqDevNo();
#ifdef CORE_CLOCK_TEST_HF_TIMER_FREQ
	cfg.Freq = CORE_CLOCK_TEST_HF_TIMER_FREQ;
#endif
	return TimerInit(&s_Timer, &cfg);
}

static void CoreClockTestSharedClock(bool high)
{
	uint64_t start;
	bool present;
	bool running;
	bool shared;
	const char *name;

	name = high ? "HF" : "LF";
	present = high ? TimerGetHighFreqDevCount() > 0 : TimerGetLowFreqDevCount() > 0;
	running = present && (high ? CoreClockTestInitHighTimer() : TimerInit(&s_Timer, &s_TimerCfg));
	shared = high ?
		CoreClockTestRequests(SystemHighFreqClockEnable, SystemHighFreqClockDisable) :
		CoreClockTestRequests(SystemLowFreqClockEnable, SystemLowFreqClockDisable);
	CoreClockTestPrint("%s clock\r\n", name);
	CoreClockTestResult("Shared clock", shared);
	if (present)
	{
		CoreClockTestResult("Timer request", running);
	}
	if (running)
	{
		start = TimerGetTickCount(&s_Timer);
		msDelay(10);
		CoreClockTestResult("Count after release", TimerGetTickCount(&s_Timer) != start);
		CoreClockTestReleaseTimer();
	}
}

// Exercise the generic CPU running mode API independently of sleep and
// oscillator ownership. Unsupported low-power modes are reported as such.
static void CoreClockTestPowerMode(void)
{
	uint32_t fastest;
	uint32_t reduced;
	uint32_t restored;
	bool changed;
	bool restoredOk;

	if (SystemPowerModeSet == nullptr || SystemPowerModeGet == nullptr)
	{
		CoreClockTestPrint("CPU low power: UNSUPPORTED\r\n");
		return;
	}

	if (!SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE))
	{
		CoreClockTestResult("CPU performance", false);
		return;
	}
	fastest = SystemCoreClockGet();
	CoreClockTestResult("CPU performance",
		SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);

	CoreClockTestFlush();
	changed = SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER);
	if (!changed)
	{
		CoreClockTestPrint("CPU low power: UNSUPPORTED\r\n");
		return;
	}

	reduced = SystemCoreClockGet();
	CoreClockTestPrint("CPU %u Hz (max %u)\r\n",
		(unsigned)reduced, (unsigned)fastest);
	CoreClockTestResult("CPU low power",
		SystemPowerModeGet() == SYSTEM_POWER_MODE_LOW_POWER &&
		reduced != 0U && reduced < fastest);

	// Measure the low-frequency endpoint before restoring performance mode.
	CoreClockTestDelayCalibrationRecord();
	CoreClockTestFlush();
	restoredOk = SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE);
	restored = SystemCoreClockGet();
	CoreClockTestResult("CPU restored",
		restoredOk && SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE &&
		restored == fastest);
}

static void CoreClockTestCalibrationTest(void)
{
	bool enabled;
	bool calibrated;

	if (GetHighFreqOscType() == OSC_TYPE_RC)
	{
		calibrated = SystemHighFreqClockCalibrate != nullptr && SystemHighFreqClockCalibrate();
		CoreClockTestPrint("HF RC calibration: %s\r\n",
			calibrated ? "PASS" : "UNSUPPORTED or failed");
	}

	if (GetLowFreqOscType() != OSC_TYPE_RC || GetLowFreqOscFreq() == 0U)
	{
		return;
	}
	if (SystemLowFreqClockCalibrate == nullptr)
	{
		CoreClockTestPrint("LF calibration: UNSUPPORTED\r\n");
		return;
	}

	if (SystemLowFreqClockEnable == nullptr ||
		SystemLowFreqClockDisable == nullptr)
	{
		CoreClockTestResult("LF RC clock API", false);
		return;
	}
	enabled = SystemLowFreqClockEnable();

	if (!enabled)
	{
		CoreClockTestResult("LF RC clock enable", false);
		return;
	}
	calibrated = SystemLowFreqClockCalibrate();
	SystemLowFreqClockDisable();

	// The nRF5340 can supply shared LFXO while a core requests LFRC.
	// In that case LFRC is not active and there is nothing to calibrate.
	// Treat unsupported calibration like the optional HFRC operation.
	CoreClockTestPrint("LF RC calibration: %s\r\n",
		calibrated ? "PASS" : "UNSUPPORTED or failed");
}

static void CoreClockTestDelayDivisors(const DelayCalibrationPoint_t *pSample)
{
	uint64_t usDivisor;
	uint64_t nsDivisor;

	if (pSample->UsLoopsPerSecond == 0U || pSample->NsLoopsPerSecond == 0U)
	{
		return;
	}
	usDivisor = (uint64_t)pSample->CpuHz * 1000000ULL /
		pSample->UsLoopsPerSecond;
	nsDivisor = (uint64_t)pSample->CpuHz * 1000000ULL /
		pSample->NsLoopsPerSecond;
	CoreClockTestPrint("Delay divisors %u Hz: us %u, ns %u\r\n",
		(unsigned)pSample->CpuHz, (unsigned)usDivisor, (unsigned)nsDivisor);
}

static bool CoreClockTestDelaySample(DelayCalibrationPoint_t *pSample)
{
	uint32_t cpuHz;
	uint32_t start;
	uint32_t irqState;
	uint32_t usElapsed;
	uint32_t nsPublicElapsed;
	bool valid;

	cpuHz = SystemCoreClockGet();
	if (cpuHz == 0U)
	{
		return false;
	}

	pSample->CpuHz = cpuHz;
	pSample->UsLoopsPerSecond = 0U;
	pSample->NsLoopsPerSecond = 0U;
	pSample->Measured = false;

	// DWT is an optional measurement instrument, never the runtime delay.
	// If unavailable, use the existing HF Timer calibration path.
	if (DelayCalibrationMeasure(pSample))
	{
		CoreClockTestDelayDivisors(pSample);
		return true;
	}

	if (TimerGetHighFreqDevCount() <= 0 || TimerGetHighFreqDevNo() < 0)
	{
		CoreClockTestPrint("No delay timebase\r\n");
		return false;
	}
	if (!CoreClockTestInitHighTimer())
	{
		return false;
	}
	// Sample the busy loop without UART or other interrupt-handler time.
	// The independent HF hardware timer keeps running with PRIMASK set.
	irqState = __get_PRIMASK();
	__disable_irq();
	start = TimerGetMicrosecond(&s_Timer);
	msDelay(100);
	usElapsed = TimerGetMicrosecond(&s_Timer) - start;
	__set_PRIMASK(irqState);

	// Sample the busy loop without UART or other interrupt-handler time.
	// The independent HF hardware timer keeps running with PRIMASK set.
	irqState = __get_PRIMASK();
	__disable_irq();
	start = TimerGetMicrosecond(&s_Timer);
	nsDelay(100000000U);
	nsPublicElapsed = TimerGetMicrosecond(&s_Timer) - start;
	__set_PRIMASK(irqState);

	CoreClockTestReleaseTimer();
	// Recommend factors from the public API, never from a differently
	// placed inline assembly probe.
	valid = DelayCalibrationFromTimer(cpuHz, usElapsed, nsPublicElapsed,
		pSample);
	if (valid)
	{
		CoreClockTestDelayDivisors(pSample);
	}
	return valid;
}

static void CoreClockTestDelayCalibrationRecord(void)
{
	DelayCalibrationPoint_t sample;

	if (!CORE_CLOCK_TEST_DELAY_CALIBRATION)
	{
		return;
	}

	sample = {};

	if (!CoreClockTestDelaySample(&sample))
	{
		CoreClockTestResult("Delay measurement", false);
	}
}

// Wait for the public completion event on interrupt-driven DMA targets.
// Legacy polling targets retain the existing delay until their UART drivers
// expose an equivalent completion event.
static void CoreClockTestFlush(void)
{
#if CORE_CLOCK_TEST_UART_INT && CORE_CLOCK_TEST_UART_DMA
	uint32_t timeout;

	for (timeout = 0U; !s_UartTxEmpty && timeout < 5000U; timeout++)
	{
		usDelay(100U);
	}
	if (!s_UartTxEmpty)
	{
		s_Failures++;
	}
#else
	msDelay(2);
#endif
}

static void CoreClockTestComplete(void)
{
	CoreClockTestFlush();
	CoreClockTestPrint("CoreClockTest complete: %u failures\r\n", s_Failures);
	CoreClockTestFlush();

	while (1)
	{
		__WFI();
	}
}

int main(void)
{
	unsigned cycle;
	uint32_t core;
	uint32_t bus;
	uint32_t readback;
	unsigned shift;
	uint32_t request;
	uint32_t actual;
	int clockIdx;
	unsigned domainCount;

	if (!UARTInit(&s_Uart, &s_UartCfg))
	{
		return 1;
	}
	msDelay(1000);
	CoreClockTestPrint("CoreClockTest\r\nCPU %u Hz, PCLK0 %u Hz\r\n",
		(unsigned)SystemCoreClockGet(), (unsigned)SystemPeriphClockGet(0));
	CoreClockTestFlush();
	CoreClockTestResult("Startup clock", SystemCoreClockGet() != 0U);
	CoreClockTestSharedClock(false);
	CoreClockTestSharedClock(true);
	CoreClockTestDelayCalibrationRecord();
	CoreClockTestPowerMode();
	domainCount = 0U;

	for (cycle = 1U; cycle <= CLOCK_TEST_CYCLES; cycle++)
	{
		CoreClockTestPrint("Cycle %u\r\n", cycle);

		core = SystemCoreClockGet();
		CoreClockTestResult("Core frequency", core != 0U);

		// Every available peripheral domain receives the same fixed requests.
		for (clockIdx = 0; clockIdx < 32; clockIdx++)
		{
			bus = SystemPeriphClockGet(clockIdx);

			if (bus == 0U)
			{
				continue;
			}

			domainCount++;
			for (shift = 0U; shift < 3U; shift++)
			{
				request = bus >> shift;

				if (request == 0U)
				{
					break;
				}

				CoreClockTestFlush();
				actual = SystemPeriphClockSet(clockIdx, request);
				readback = SystemPeriphClockGet(clockIdx);
				CoreClockTestPrint("Clock %u: %u -> %u Hz\r\n",
					(unsigned)clockIdx, (unsigned)request, (unsigned)actual);
				CoreClockTestResult("PCLK", actual != 0U &&
					readback == actual);
			}

			actual = SystemPeriphClockSet(clockIdx, bus);
			CoreClockTestResult("PCLK restore",
				actual == bus && SystemPeriphClockGet(clockIdx) == bus);
		}

	}

	CoreClockTestCalibrationTest();
	CoreClockTestResult("Invalid PCLK index",
		SystemPeriphClockGet(-1) == 0U);
	CoreClockTestResult("Clock scan limit",
		SystemPeriphClockGet(32) == 0U);

	if (domainCount == 0U)
	{
		CoreClockTestResult("Clock domains present", false);
	}
	CoreClockTestComplete();
}
