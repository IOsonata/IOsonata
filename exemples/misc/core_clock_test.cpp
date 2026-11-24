/**-------------------------------------------------------------------------
@file	core_clock_test.cpp

@brief	CoreClockTest: generic System Core clock and delay validation.

Uses the generic clock, UART and Timer interfaces. Oscillator selections
come from the System Core configuration, not from board-specific test lists.
Both build configurations log to UART without a debugger.
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
// UART interrupt mode is the generic default. Boards lacking IRQ support
// may override this option; DMA remains an independent choice.
#ifndef CORE_CLOCK_TEST_UART_INT
#define CORE_CLOCK_TEST_UART_INT true
#endif
#ifndef CORE_CLOCK_TEST_UART_DMA
#define CORE_CLOCK_TEST_UART_DMA false
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
#pragma weak sDelay
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
static TimerDev_t s_RetainedTimer;
static volatile uint32_t s_InterruptCount;
static volatile uint32_t s_TriggerCount;
static uint32_t s_LfTimerInitMask;
static unsigned s_Failures;
static volatile bool s_UartTxEmpty = true;

static DelayCalibrationPoint_t s_DelayHigh;
static DelayCalibrationPoint_t s_DelayLow;
static void CoreClockTestTimerHandler(TimerDev_t *pTimer, uint32_t Event);
static void CoreClockTestTimerTriggers(void);
static void CoreClockTestDelayCalibrationRecord(void);
static void CoreClockTestFlush(void);

static const TimerCfg_t s_TimerCfg = {
	.DevNo = 0,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 0U,			// Highest supported rate for each timer device
	.IntPrio = CLOCK_DEMO_IRQ_PRIORITY,
	.EvtHandler = CoreClockTestTimerHandler,
};

// Test the trigger callback independently of the timer event handler.
static void CoreClockTestTriggerHandler(TimerDev_t *pTimer, int Trigger,
	void *pContext)
{
	(void)pTimer;
	(void)Trigger;
	(void)pContext;
	s_TriggerCount++;
}

static void CoreClockTestTimerHandler(TimerDev_t *pTimer, uint32_t Event)
{
	(void)pTimer;
	if ((Event & TIMER_EVT_TRIGGER(0)) != 0U)
	{
		s_InterruptCount = s_InterruptCount + 1U;
	}
}

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
		s_UartTxEmpty = false;
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
	char block[CORE_CLOCK_TEST_TX_BLOCK_SIZE];
	char digits[11];
	unsigned used;
	unsigned digitCount;
	unsigned value;

	used = 0U;
	va_start(args, pFormat);
	while (*pFormat != 0)
	{
		if (*pFormat != '%')
		{
			CoreClockTestAppend(block, &used, *pFormat);
			pFormat++;
			continue;
		}
		pFormat++;
		if (*pFormat == 's')
		{
			pString = va_arg(args, const char *);
			if (pString != nullptr)
			{
				while (*pString != 0)
				{
					CoreClockTestAppend(block, &used, *pString);
					pString++;
				}
			}
		}
		else if (*pFormat == 'u')
		{
			value = va_arg(args, unsigned);
			digitCount = 0U;
			do
			{
				digits[digitCount] = '0' + value % 10U;
				digitCount++;
				value /= 10U;
			} while (value != 0U);
			while (digitCount != 0U)
			{
				digitCount--;
				CoreClockTestAppend(block, &used, digits[digitCount]);
			}
		}
		if (*pFormat != 0)
		{
			pFormat++;
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

// Exercise the same shared oscillator reference-count API on every MCU.
// Physical oscillator on/off and power measurements require target tests.
static void CoreClockTestSharedClockTest(void)
{
	bool first;
	bool second;
	bool third;

	CoreClockTestPrint("Configured HF source %u, LF source %u (%u Hz)\r\n",
		(unsigned)GetHighFreqOscType(),
		(unsigned)GetLowFreqOscType(),
		(unsigned)GetLowFreqOscFreq());

	if (SystemHighFreqClockEnable == nullptr ||
		SystemHighFreqClockDisable == nullptr)
	{
		CoreClockTestResult("HF shared clock API implemented", false);
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
		CoreClockTestResult("HF shared clock lifecycle",
			first && second && third);
	}

	if (SystemLowFreqClockEnable == nullptr ||
		SystemLowFreqClockDisable == nullptr)
	{
		CoreClockTestResult("LF shared clock API implemented", false);
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
		CoreClockTestPrint("LF lifecycle: first %u, second %u, after release %u\r\n",
			(unsigned)first, (unsigned)second, (unsigned)third);
		CoreClockTestResult("LF shared clock lifecycle",
			first && second && third);
	}
}

// Check one-shot calibration without making the test own a periodic
// timer or depending on the application event dispatcher.
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
		CoreClockTestPrint("CPU low-power mode: UNSUPPORTED\r\n");
		return;
	}

	fastest = SystemCoreClockGet();
	if (!SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE))
	{
		CoreClockTestResult("CPU performance mode", false);
		return;
	}
	fastest = SystemCoreClockGet();
	CoreClockTestResult("CPU performance mode",
		SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);

	CoreClockTestFlush();
	changed = SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER);
	if (!changed)
	{
		CoreClockTestPrint("CPU low-power mode: UNSUPPORTED\r\n");
		return;
	}

	reduced = SystemCoreClockGet();
	CoreClockTestPrint("CPU reduced-speed mode %u Hz (performance %u Hz)\r\n",
		(unsigned)reduced, (unsigned)fastest);
	CoreClockTestResult("CPU low-power mode",
		SystemPowerModeGet() == SYSTEM_POWER_MODE_LOW_POWER &&
		reduced != 0U && reduced < fastest);

	// Measure the low-frequency endpoint before restoring performance mode.
	CoreClockTestDelayCalibrationRecord();
	CoreClockTestFlush();
	restoredOk = SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE);
	restored = SystemCoreClockGet();
	CoreClockTestResult("CPU performance restoration",
		restoredOk && SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE &&
		restored == fastest);
}

static void CoreClockTestCalibrationTest(void)
{
	bool enabled;
	bool calibrated;

	if (SystemHighFreqClockCalibrate == nullptr)
	{
		CoreClockTestPrint("HF RC calibration: UNSUPPORTED (API missing)\r\n");
	}
	else
	{
		calibrated = SystemHighFreqClockCalibrate();

		CoreClockTestPrint("HF RC calibration: %s\r\n",
			calibrated ? "PASS" : "UNSUPPORTED or failed");
	}

	if (SystemLowFreqClockCalibrate == nullptr)
	{
		CoreClockTestPrint("LF RC calibration: UNSUPPORTED (API missing)\r\n");
		return;
	}


	if (SystemLowFreqClockEnable == nullptr ||
		SystemLowFreqClockDisable == nullptr)
	{
		CoreClockTestResult("LF RC calibration clock API", false);
		return;
	}
	enabled = SystemLowFreqClockEnable();

	if (!enabled)
	{
		CoreClockTestResult("LF RC calibration clock enable", false);
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

// The architecture's delay implementation measures its own delay units.
// The generic HF Timer remains a fallback and a timing validation reference.
// Do not use RTC for calibration; never change production delay factors here.
// Measure complete sDelay latency for both LFCLK ownership states.
// Cold-start includes crystal startup; warm-start keeps LFCLK enabled by
// another user. These require separate compensation values in production.
static void CoreClockTestSleepDelayMeasure(const char *pMode)
{
	TimerCfg_t cfg;
	uint32_t started;
	uint32_t elapsed;
	uint32_t difference;
	bool ok;

	cfg = s_TimerCfg;
	cfg.DevNo = TimerGetHighFreqDevNo();
	cfg.EvtHandler = nullptr;
	if (cfg.DevNo < 0 || !TimerInit(&s_RetainedTimer, &cfg) ||
		!TimerEnable(&s_RetainedTimer))
	{
		CoreClockTestPrint("sDelay HF reference: UNSUPPORTED\r\n");
		return;
	}
	started = TimerGetMicrosecond(&s_RetainedTimer);
	ok = sDelay(1U);
	elapsed = TimerGetMicrosecond(&s_RetainedTimer) - started;
	TimerDisable(&s_RetainedTimer);
	if (!ok)
	{
		CoreClockTestPrint("sDelay %s: LF timer unavailable or sleep rejected\r\n",
			pMode);
		CoreClockTestResult("sDelay sleep operation", false);
		return;
	}
	difference = elapsed >= 1000000U ?
		elapsed - 1000000U : 1000000U - elapsed;
	CoreClockTestPrint("sDelay %s end-to-end %u us (target 1000000 us)\r\n",
		pMode, (unsigned)elapsed);
	// Report independent time error in us; the LF timer may be a 1-MHz
	// GRTC rather than a 32,768-Hz RTC on a different MCU.
	CoreClockTestPrint("sDelay %s overhead: %s %u us\r\n",
		pMode, elapsed >= 1000000U ? "subtract" : "add",
		(unsigned)difference);
	CoreClockTestResult("sDelay end-to-end within 2 percent",
		difference <= 20000U);
}

static void CoreClockTestSleepDelay(void)
{
	bool held;

	if (sDelay == nullptr)
	{
		CoreClockTestPrint("sDelay: UNSUPPORTED on this MCU\r\n");
		return;
	}
	CoreClockTestSleepDelayMeasure("cold");
	if (SystemLowFreqClockEnable == nullptr ||
		SystemLowFreqClockDisable == nullptr)
	{
		return;
	}
	held = SystemLowFreqClockEnable();
	if (!held)
	{
		CoreClockTestPrint("sDelay warm: LF clock hold unavailable\r\n");
		return;
	}
	CoreClockTestSleepDelayMeasure("warm");
	SystemLowFreqClockDisable();
	// A second call confirms that the temporary LF timer was returned
	// to its owner pool after the first call. Preserve the existing
	// cold/warm calibration measurements above.
	CoreClockTestSleepDelayMeasure("reuse");
}

// Print the measured factors as loop divisors, CPU Hz x 1000000 / loops
// per second, the unit a port passes to its delay factor setup.
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
	TimerCfg_t cfg;
	uint32_t cpuHz;
	uint32_t start;
	uint32_t irqState;
	uint32_t usElapsed;
	uint32_t nsPublicElapsed;
	uint32_t nsRepeatElapsed;
	uint32_t nsFactorBefore;
	uint32_t nsFactorAfter;
	bool valid;

	cpuHz = SystemCoreClockGet();
	if (pSample == nullptr || cpuHz == 0U)
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
		CoreClockTestPrint("Delay calibration DWT %u Hz: us loop/s %u, ns loop/s %u\r\n",
			(unsigned)cpuHz, (unsigned)pSample->UsLoopsPerSecond,
			(unsigned)pSample->NsLoopsPerSecond);
		CoreClockTestDelayDivisors(pSample);
	}
	else
	{
		CoreClockTestPrint("DWT calibration unavailable; HF Timer will measure delay timing\r\n");
	}

	if (TimerGetHighFreqDevCount() <= 0 || TimerGetHighFreqDevNo() < 0)
	{
		CoreClockTestPrint("Delay timing: no CPU counter or HF Timer\r\n");
		return false;
	}

	cfg = s_TimerCfg;
	cfg.DevNo = TimerGetHighFreqDevNo();
	cfg.EvtHandler = nullptr;
	if (!TimerInit(&s_RetainedTimer, &cfg))
	{
		CoreClockTestPrint("Delay timing: HF Timer init FAIL\r\n");
		return false;
	}
	valid = TimerSetFrequency(&s_RetainedTimer, 0U) != 0U &&
		TimerEnable(&s_RetainedTimer);
	if (!valid)
	{
		TimerDisable(&s_RetainedTimer);
		CoreClockTestPrint("Delay timing: HF Timer start FAIL\r\n");
		return false;
	}

	// Sample the busy loop without UART or other interrupt-handler time.
	// The independent HF hardware timer keeps running with PRIMASK set.
	irqState = __get_PRIMASK();
	__disable_irq();
	start = TimerGetMicrosecond(&s_RetainedTimer);
	msDelay(100);
	usElapsed = TimerGetMicrosecond(&s_RetainedTimer) - start;
	__set_PRIMASK(irqState);

	nsFactorBefore = SystemnsDelayFactor;
	// Sample the busy loop without UART or other interrupt-handler time.
	// The independent HF hardware timer keeps running with PRIMASK set.
	irqState = __get_PRIMASK();
	__disable_irq();
	start = TimerGetMicrosecond(&s_RetainedTimer);
	nsDelay(100000000U);
	nsPublicElapsed = TimerGetMicrosecond(&s_RetainedTimer) - start;
	__set_PRIMASK(irqState);

	// Repeat the real API measurement at a second call site. The raw
	// assembly probe can execute at a different flash alignment, so it
	// must not be treated as the production delay's calibration source.
	// Sample the busy loop without UART or other interrupt-handler time.
	// The independent HF hardware timer keeps running with PRIMASK set.
	irqState = __get_PRIMASK();
	__disable_irq();
	start = TimerGetMicrosecond(&s_RetainedTimer);
	nsDelay(100000000U);
	nsRepeatElapsed = TimerGetMicrosecond(&s_RetainedTimer) - start;
	__set_PRIMASK(irqState);

	nsFactorAfter = SystemnsDelayFactor;

	TimerDisable(&s_RetainedTimer);
	CoreClockTestPrint("nsDelay repeat %u us, factors before %u after %u\r\n",
		(unsigned)nsRepeatElapsed, (unsigned)nsFactorBefore,
		(unsigned)nsFactorAfter);
	CoreClockTestPrint("Delay timing HF Timer %u Hz: msDelay %u us, nsDelay %u us\r\n",
		(unsigned)cpuHz, (unsigned)usElapsed, (unsigned)nsPublicElapsed);
	CoreClockTestResult("nsDelay HF Timer 100 ms",
		nsPublicElapsed >= 98000U && nsPublicElapsed <= 102000U);
	CoreClockTestResult("nsDelay repeated HF Timer 100 ms",
		nsRepeatElapsed >= 98000U && nsRepeatElapsed <= 102000U);
	CoreClockTestResult("nsDelay factor unchanged during timing",
		nsFactorBefore == nsFactorAfter);

	if (usElapsed == 0U || nsPublicElapsed == 0U)
	{
		CoreClockTestResult("Delay HF Timer measurement", false);
		return false;
	}
	// Recommend factors from the public API, never from a differently
	// placed inline assembly probe.
	valid = DelayCalibrationFromTimer(cpuHz, usElapsed, nsPublicElapsed,
		pSample);
	if (valid)
	{
		CoreClockTestPrint("Delay calibration HF Timer %u Hz: us loop/s %u, ns loop/s %u\r\n",
			(unsigned)cpuHz, (unsigned)pSample->UsLoopsPerSecond,
			(unsigned)pSample->NsLoopsPerSecond);
		CoreClockTestDelayDivisors(pSample);
	}
	else
	{
		CoreClockTestPrint("Delay factors unavailable on this MCU; timing only\r\n");
	}
	return valid;
}

static void CoreClockTestDelayCalibrationRecord(void)
{
	DelayCalibrationPoint_t sample;

	sample = {};

	if (!CoreClockTestDelaySample(&sample))
	{
		return;
	}

	if (!s_DelayHigh.Measured || sample.CpuHz > s_DelayHigh.CpuHz)
	{
		s_DelayHigh = sample;
	}

	if (!s_DelayLow.Measured || sample.CpuHz < s_DelayLow.CpuHz)
	{
		s_DelayLow = sample;
	}
}

static void CoreClockTestDelayCalibrationReport(void)
{
	if (!s_DelayHigh.Measured || !s_DelayLow.Measured)
	{
		CoreClockTestPrint("Delay two-point calibration: not available\r\n");
		return;
	}

	CoreClockTestPrint("Delay highest observed %u Hz: us loop/s %u, ns loop/s %u\r\n",
		(unsigned)s_DelayHigh.CpuHz, (unsigned)s_DelayHigh.UsLoopsPerSecond,
		(unsigned)s_DelayHigh.NsLoopsPerSecond);
	CoreClockTestPrint("Delay lowest observed %u Hz: us loop/s %u, ns loop/s %u\r\n",
		(unsigned)s_DelayLow.CpuHz, (unsigned)s_DelayLow.UsLoopsPerSecond,
		(unsigned)s_DelayLow.NsLoopsPerSecond);

	if (s_DelayHigh.CpuHz == s_DelayLow.CpuHz)
	{
		CoreClockTestPrint("Delay two-point calibration: one CPU speed observed\r\n");
		return;
	}
	// Report both measured endpoints. The System Core implementation
	// already owns frequency-dependent factor calculation/interpolation.
	CoreClockTestPrint("Delay two-point calibration: two CPU speeds measured\r\n");
}

static bool CoreClockTestTimerInit(int DevNo)
{
	TimerCfg_t cfg;

	cfg = s_TimerCfg;
	cfg.DevNo = DevNo;
	if (!TimerInit(&s_Timer, &cfg))
	{
		CoreClockTestResult("Timer initialization", false);
		return false;
	}
	// TimerInit owns the initial hardware enable. Do not immediately
	// disable and restart an LF timer: that churns the shared LFCLK
	// oscillator and can race its asynchronous STOP transition.
	return true;
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

static void CoreClockTestTimer(int DevNo)
{
	unsigned delay;
	uint32_t frequency;
	uint64_t period;
	uint32_t start;
	uint32_t elapsed;
	uint32_t count;
	uint32_t expected;
	uint32_t difference;
	uint32_t hfStart;
	uint32_t hfElapsed;
	uint32_t measuredTickRate;
	uint64_t tickStart;
	uint64_t tickEnd;
	TimerCfg_t hfCfg;
	bool hfReady;
	bool enabled;

	CoreClockTestPrint("Timer device %u\r\n", (unsigned)DevNo);
	if (!CoreClockTestTimerInit(DevNo))
	{
		return;
	}
	if (DevNo >= 0 && DevNo < TimerGetLowFreqDevCount() && DevNo < 32)
	{
		s_LfTimerInitMask |= 1UL << DevNo;
	}

	// Diagnose LFCLK timing against an independent HF peripheral timer.
	// The test remains generic: no oscillator registers or CPU counter access.
	hfReady = false;
	if (DevNo < TimerGetLowFreqDevCount() &&
		TimerGetHighFreqDevCount() > 0 &&
		TimerGetHighFreqDevNo() >= 0)
	{
		hfCfg = s_TimerCfg;
		hfCfg.DevNo = TimerGetHighFreqDevNo();
		hfCfg.EvtHandler = nullptr;
		if (TimerInit(&s_RetainedTimer, &hfCfg))
		{
			hfReady = TimerEnable(&s_RetainedTimer);
		}
	}
	for (delay = 0U; delay < 2U; delay++)
	{
		// Refresh the divider after a peripheral-clock change, with timer stopped.
		frequency = TimerSetFrequency(&s_Timer, s_TimerCfg.Freq);
		s_InterruptCount = 0U;
		period = nsTimerEnableTrigger(&s_Timer, 0, 10000000ULL,
			TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
		// Evaluate all three independently. Short-circuiting would conceal
		// whether frequency, compare setup or clock ownership caused failure.
		enabled = TimerEnable(&s_Timer);
		if (frequency == 0U || period == 0U || !enabled)
		{
			CoreClockTestPrint("Timer start diagnostic: device %u, frequency %u Hz, "
				"period %u ns, enable %u, LF source %u\r\n",
				(unsigned)DevNo, (unsigned)frequency, (unsigned)period,
				(unsigned)enabled, (unsigned)GetLowFreqOscType());
			TimerDisableTrigger(&s_Timer, 0);
			TimerDisable(&s_Timer);
			CoreClockTestResult("Timer start", false);
			if (hfReady)
			{
				TimerDisable(&s_RetainedTimer);
			}
			return;
		}
		tickStart = hfReady ? TimerGetTickCount(&s_Timer) : 0U;
		start = TimerGetMicrosecond(&s_Timer);
		hfStart = hfReady ? TimerGetMicrosecond(&s_RetainedTimer) : 0U;
		if (delay == 0U)
		{
			msDelay(250);
		}
		else
		{
			nsDelay(250000000U);
		}
		elapsed = TimerGetMicrosecond(&s_Timer) - start;
		tickEnd = hfReady ? TimerGetTickCount(&s_Timer) : 0U;
		hfElapsed = hfReady ?
			TimerGetMicrosecond(&s_RetainedTimer) - hfStart : 0U;
		TimerDisable(&s_Timer);
		count = s_InterruptCount;
		TimerDisableTrigger(&s_Timer, 0);
		expected = (uint64_t)elapsed * 1000U / period;
		difference = count > expected ? count - expected : expected - count;
		CoreClockTestPrint("Timer: %u Hz, %s 250 ms measured %u us, IRQ %u\r\n",
			(unsigned)frequency, delay == 0U ? "msDelay" : "nsDelay",
			(unsigned)elapsed, (unsigned)count);
		CoreClockTestResult("Delay within 2 percent", elapsed >= 245000U && elapsed <= 255000U);
		CoreClockTestResult("Timer count and IRQ", elapsed > 0U && count > 0U && difference <= 1U);
		if (hfReady)
		{
			CoreClockTestPrint("LF/HF reference: LF %u us, HF %u us\r\n",
				(unsigned)elapsed, (unsigned)hfElapsed);
			if (hfElapsed != 0U && tickEnd >= tickStart)
			{
				// Raw RTC ticks distinguish oscillator drift from the
				// Timer tick-to-time conversion. Keep every MCU generic.
				measuredTickRate = (uint32_t)(((tickEnd - tickStart) *
					1000000ULL + hfElapsed / 2U) / hfElapsed);
				CoreClockTestPrint("Independent LF timer rate: %u Hz (nominal %u Hz)\r\n",
					(unsigned)measuredTickRate, (unsigned)frequency);
			}
		}
	}
	if (hfReady)
	{
		TimerDisable(&s_RetainedTimer);
	}
	CoreClockTestTimerTriggers();
}


// Exercise the public Timer trigger API on each enumerated hardware timer.
// Keep this separate from the clock-change timing and IRQ measurements.
static void CoreClockTestTimerTriggers(void)
{
	uint64_t period;
	uint64_t before;
	uint64_t after;
	uint32_t fired;
	int trigger;
	bool enabled;
	bool valid;

	valid = s_Timer.FindAvailTrigger != nullptr &&
		TimerGetMaxTrigger(&s_Timer) > 0;
	CoreClockTestResult("Timer trigger availability", valid);
	if (!valid)
	{
		return;
	}
	trigger = s_Timer.FindAvailTrigger(&s_Timer);
	CoreClockTestResult("Timer free trigger",
		trigger >= 0 && trigger < TimerGetMaxTrigger(&s_Timer));
	if (trigger < 0 || trigger >= TimerGetMaxTrigger(&s_Timer))
	{
		return;
	}

	// One-shot must invoke the callback once, not periodically.
	s_TriggerCount = 0U;
	period = nsTimerEnableTrigger(&s_Timer, trigger, 20000000ULL,
		TIMER_TRIG_TYPE_SINGLE, CoreClockTestTriggerHandler, nullptr);
	enabled = TimerEnable(&s_Timer);
	if (enabled && period != 0U)
	{
		msDelay(75U);
	}
	fired = s_TriggerCount;
	CoreClockTestResult("Timer one-shot callback",
		enabled && period != 0U && fired == 1U);
	TimerDisableTrigger(&s_Timer, trigger);
	if (!enabled || period == 0U)
	{
		TimerDisable(&s_Timer);
		return;
	}

	// Switch trigger modes while the timer owns its clock. Disable/Enable
	// is tested by the timer matrix; this phase validates the trigger API,
	// not a second asynchronous LFCLK stop/start transition.
	TimerReset(&s_Timer);
	s_TriggerCount = 0U;
	period = nsTimerEnableTrigger(&s_Timer, trigger, 20000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, CoreClockTestTriggerHandler, nullptr);
	before = TimerGetTickCount(&s_Timer);
	if (enabled && period != 0U)
	{
		msDelay(110U);
	}
	after = TimerGetTickCount(&s_Timer);
	fired = s_TriggerCount;
	if (!enabled || period == 0U || fired < 4U || fired > 7U ||
		after <= before)
	{
		CoreClockTestPrint("Timer continuous diagnostics: device %u, enabled %u, "
			"period %u ns, callbacks %u, before %u, after %u\r\n",
			(unsigned)s_Timer.DevNo, (unsigned)enabled,
			(unsigned)period, (unsigned)fired,
			(unsigned)before, (unsigned)after);
	}
	CoreClockTestResult("Timer continuous callback",
		enabled && period != 0U && fired >= 4U && fired <= 7U);
	CoreClockTestResult("Timer counter advances",
		enabled && after > before);
	TimerDisableTrigger(&s_Timer, trigger);
	TimerDisable(&s_Timer);
	before = s_TriggerCount;
	msDelay(40U);
	CoreClockTestResult("Timer trigger disabled",
		s_TriggerCount == before);
	CoreClockTestResult("Timer trigger slot reusable",
		s_Timer.FindAvailTrigger(&s_Timer) == trigger);
}

// Check that an initialized but disabled LF timer remains owned.
// This runs after the device matrix: those TimerInit calls intentionally
// retain their handles across TimerDisable. No hardware-specific RTC map
// or private allocation API is used here.
static void CoreClockTestSleepDelayOwnership(int LowCount)
{
	uint32_t expectedMask;
	bool blocked;

	if (sDelay == nullptr || LowCount <= 0 || LowCount > 32)
	{
		CoreClockTestPrint("sDelay RTC contention: UNSUPPORTED\r\n");
		return;
	}
	expectedMask = LowCount == 32 ? UINT32_MAX :
		(1UL << LowCount) - 1U;
	if ((s_LfTimerInitMask & expectedMask) != expectedMask)
	{
		CoreClockTestPrint("sDelay RTC contention: UNSUPPORTED (not all LF timers initialized)\r\n");
		return;
	}
	// Zero seconds is a no-op even when every LF timer is occupied.
	CoreClockTestResult("sDelay zero with LF timers occupied", sDelay(0U));
	// All advertised LF timers have been initialized, then disabled.
	// A temporary delay may not reuse any of these application handles.
	blocked = !sDelay(1U);
	CoreClockTestResult("sDelay occupied disabled LF timers", blocked);
}

// Check that disabling one timer does not stop an independently
// configured timer. Prefer one LF and one HF device when available.
static void CoreClockTestConcurrentTimerTest(int LowCount, int HighCount)
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
		CoreClockTestPrint("Concurrent timers: UNSUPPORTED (one device)\r\n");
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
		if (first)
		{
			TimerDisable(&s_Timer);
		}
		if (second)
		{
			TimerDisable(&s_RetainedTimer);
		}
		CoreClockTestResult("Concurrent timer initialization", false);
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
		CoreClockTestResult("Timer independent clock ownership", elapsed > 0U);
	}
	else
	{
		CoreClockTestResult("Concurrent timer enable", false);
	}
	TimerDisable(&s_Timer);
	TimerDisable(&s_RetainedTimer);
}

int main(void)
{
	unsigned cycle;
	bool selected;
	OSC_TYPE initialCoreSource;
	OSC_TYPE initialLfSource;
	uint32_t initialCoreFreq;
	uint32_t initialLfFreq;
	uint32_t core;
	uint32_t bus;
	uint32_t readback;
	unsigned dividerIdx;
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
	CoreClockTestPrint("CoreClockTest\r\nStartup CPU %u Hz, peripheral clock 0 %u Hz\r\n",
		(unsigned)SystemCoreClockGet(), (unsigned)SystemPeriphClockGet(0));
	CoreClockTestFlush();
	CoreClockTestResult("Core clock startup", SystemCoreClockGet() != 0U);
	CoreClockTestSharedClockTest();
	CoreClockTestPowerMode();
	CoreClockTestDelayCalibrationRecord();
	CoreClockTestSleepDelay();
	initialCoreSource = GetHighFreqOscType();
	initialCoreFreq = GetHighFreqOscFreq();
	initialLfSource = GetLowFreqOscType();
	initialLfFreq = GetLowFreqOscFreq();
	domainCount = 0U;

	for (cycle = 1U; cycle <= CLOCK_TEST_CYCLES; cycle++)
	{
		CoreClockTestPrint("Cycle %u\r\n", cycle);

		// Exercise the configured core source and RC using the same HAL.
		CoreClockTestFlush();
		selected = SystemCoreClockSelect(initialCoreSource, initialCoreFreq);
		CoreClockTestResult("Configured core source", selected);

		if (initialCoreSource != OSC_TYPE_RC)
		{
			CoreClockTestFlush();
			selected = SystemCoreClockSelect(OSC_TYPE_RC, 0U);

			if (selected)
			{
				CoreClockTestResult("Core RC selection", true);
			}
			else
			{
				CoreClockTestPrint("Core RC: UNSUPPORTED or rejected\r\n");
			}
		}

		core = SystemCoreClockGet();
		CoreClockTestResult("Core frequency", core != 0U);

		if (core != s_DelayHigh.CpuHz && core != s_DelayLow.CpuHz)
		{
			CoreClockTestDelayCalibrationRecord();
		}

		// Every available peripheral domain receives the same fixed requests.
		for (clockIdx = 0; clockIdx < 32; clockIdx++)
		{
			bus = SystemPeriphClockGet(clockIdx);

			if (bus == 0U)
			{
				continue;
			}

			domainCount++;
			CoreClockTestPrint("Peripheral clock %u baseline %u Hz\r\n",
				(unsigned)clockIdx, (unsigned)bus);

			for (dividerIdx = 1U; dividerIdx <= 4U; dividerIdx *= 2U)
			{
				divider = dividerIdx;
				request = bus / divider;

				if (request == 0U)
				{
					CoreClockTestResult("Peripheral clock request", false);
					continue;
				}

				CoreClockTestFlush();
				actual = SystemPeriphClockSet(clockIdx, request);
				readback = SystemPeriphClockGet(clockIdx);
				CoreClockTestPrint("Domain %u request %u Hz, actual %u Hz\r\n",
					(unsigned)clockIdx, (unsigned)request, (unsigned)actual);
				CoreClockTestResult("Peripheral clock", actual != 0U &&
					readback == actual);
			}

			actual = SystemPeriphClockSet(clockIdx, bus);
			CoreClockTestResult("Peripheral clock restore",
				actual == bus && SystemPeriphClockGet(clockIdx) == bus);
		}

		// Test the configured LF source and the internal RC where supported.
		if (initialLfFreq != 0U)
		{
			CoreClockTestFlush();
			selected = SystemLowFreqClockSelect(initialLfSource, initialLfFreq);
			CoreClockTestResult("Configured LF source", selected);
		}

		if (initialLfFreq != 0U)
		{
			CoreClockTestFlush();
			selected = SystemLowFreqClockSelect(OSC_TYPE_RC, 32768U);

			if (selected)
			{
				CoreClockTestResult("LF RC selection", true);

				if (cycle == CLOCK_TEST_CYCLES)
				{
					CoreClockTestCalibrationTest();
					CoreClockTestPrint("LF RC timer timing test\r\n");

					for (timerDevNo = 0;
						timerDevNo < TimerGetLowFreqDevCount(); timerDevNo++)
					{
						CoreClockTestTimer(timerDevNo);
					}
				}
			}
			else
			{
				CoreClockTestPrint("LF RC: UNSUPPORTED or rejected\r\n");
			}
		}

		// Return the test to the configured board oscillator sources.
		if (initialLfFreq != 0U)
		{
			selected = SystemLowFreqClockSelect(initialLfSource, initialLfFreq);
			CoreClockTestResult("LF source restoration", selected);
		}

		if (initialCoreSource != OSC_TYPE_RC)
		{
			selected = SystemCoreClockSelect(initialCoreSource, initialCoreFreq);
			CoreClockTestResult("Core source restoration", selected);
		}
	}

	CoreClockTestResult("Peripheral clock invalid index",
		SystemPeriphClockGet(-1) == 0U);
	CoreClockTestResult("Peripheral domain scan limit",
		SystemPeriphClockGet(32) == 0U);

	if (domainCount == 0U)
	{
		CoreClockTestResult("Peripheral clock domains present", false);
	}
	CoreClockTestPrint("Restored LF source timer timing test\r\n");

	// Timer implementations expose their device counts through the generic API.
	// Test every advertised device rather than a board-selected instance.
	lowTimerCount = TimerGetLowFreqDevCount();
	highTimerCount = TimerGetHighFreqDevCount();
	timerCount = lowTimerCount + highTimerCount;

	if (timerCount == 0)
	{
		CoreClockTestResult("Timer devices available", false);
	}

	for (timerDevNo = 0; timerDevNo < timerCount; timerDevNo++)
	{
		CoreClockTestTimer(timerDevNo);
	}
	CoreClockTestConcurrentTimerTest(lowTimerCount, highTimerCount);
	CoreClockTestSleepDelayOwnership(lowTimerCount);
	CoreClockTestDelayCalibrationReport();
	CoreClockTestFlush();
	CoreClockTestPrint("CoreClockTest complete: %u failures\r\n", s_Failures);
	CoreClockTestFlush();

	while (1)
	{
		msDelay(1000);
	}
}
