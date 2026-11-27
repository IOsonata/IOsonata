/**-------------------------------------------------------------------------
@example	core_power_test.cpp
@brief	Generic power-management API validation

SystemPowerCapabilitiesGet() determines which power modes the test exercises.
For Standby, CORE_POWER_TEST_WAKE_MS selects timed wake (milliseconds) or
button wake (zero). Boards with a wake pin and no System OFF also test
button wake after timed Standby.

WAKE_BUT_SYSTEM_OFF identifies a board button whose pin driver arms wake
from System OFF. The next boot checks reset-on-wake and retention.
The normal CPU-resumable sleep tests run on every boot before System OFF.

MIT License. Copyright (c) 2026, I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "coredev/uart.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"
#include "idelay.h"
#include "syslog.h"
#include "board.h"

// The current CorePowerTest requires the power API implementation to
// be linked. Only older managed projects using the compatibility wrapper
// can treat these functions as optional.
#if defined(CORE_POWER_TEST_LEGACY_OPTIONAL_APIS) && defined(__GNUC__)
#pragma weak SystemPowerCapabilitiesGet
#pragma weak SystemSleep
#endif

#ifndef UART_BAUDRATE
#define UART_BAUDRATE			1000000
#endif
#ifndef CORE_POWER_TEST_WAKE_MS
#define CORE_POWER_TEST_WAKE_MS	1000U
#endif
#ifndef CORE_POWER_TEST_SLEEP_CYCLES
#define CORE_POWER_TEST_SLEEP_CYCLES	3U
#endif
#ifndef CORE_POWER_TEST_TIMER_DEVNO
#define CORE_POWER_TEST_TIMER_DEVNO	0
#endif
#ifndef WAKE_BUT_SENSE
#define WAKE_BUT_SENSE	IOPINSENSE_LOW_TRANSITION
#endif

#ifndef WAKE_BUT_SYSTEM_OFF
#define WAKE_BUT_SYSTEM_OFF 0
#endif

#ifndef CORE_POWER_TEST_POLLING_CONSOLE
#define CORE_POWER_TEST_POLLING_CONSOLE 0
#endif

#define CORE_POWER_TEST_MARKER	0xC3U
#define CORE_POWER_TEST_RAM_WORDS	32U

static const IOPinCfg_t s_UartPins[] = {
	{ UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL },
	{ UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
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
	.bIntMode = !CORE_POWER_TEST_POLLING_CONSOLE,
	.IntPrio = 6,
	.EvtCB = nullptr,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = nullptr,
	.TxMemSize = 0,
	.pTxMem = nullptr,
	.bDMAMode = !CORE_POWER_TEST_POLLING_CONSOLE,
};

#if CORE_POWER_TEST_WAKE_MS > 0
static const TimerCfg_t s_TimerCfg = {
	.DevNo = CORE_POWER_TEST_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1024U,
	.IntPrio = 6,
};
#endif

#if !CORE_POWER_TEST_POLLING_CONSOLE
// Use a bounded default suitable for MCUs with only a few KB of SRAM.
// Larger boards may increase the queue without changing the shared test.
// Blocking SysLog keeps a record pending until the sink accepts it.
#ifndef CORE_POWER_TEST_LOG_RECORDS
#define CORE_POWER_TEST_LOG_RECORDS	4
#endif
#define CORE_POWER_TEST_LOG_RECORD_LEN	128

alignas(4) static uint8_t s_LogMem[
	SYSLOG_MEMSIZE(CORE_POWER_TEST_LOG_RECORDS, CORE_POWER_TEST_LOG_RECORD_LEN)];
static const SysLogCfg_t s_LogCfg = {
	.pMem = s_LogMem,
	.MemSize = sizeof(s_LogMem),
	.RecordLen = CORE_POWER_TEST_LOG_RECORD_LEN,
	.bBlocking = true,
};
#endif

static UARTDev_t s_Uart;
#if defined(WAKE_BUT_PIN)
static int s_WakeIntNo = -1;
static volatile bool s_ButtonWoke;
#endif
#if CORE_POWER_TEST_WAKE_MS > 0
static TimerDev_t s_WakeTimer;
static volatile bool s_TimerWoke;
#endif
static volatile uint32_t s_StaticRam[CORE_POWER_TEST_RAM_WORDS];
static uint32_t s_Checks;
static uint32_t s_Failures;

#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
static volatile uint32_t * const s_RetRam = (volatile uint32_t *)POWER_DEMO_RAM_RET_ADDR;
#if defined(POWER_DEMO_RAM_DROP_ADDR)
static volatile uint32_t * const s_DropRam = (volatile uint32_t *)POWER_DEMO_RAM_DROP_ADDR;
#endif
#endif

#if CORE_POWER_TEST_POLLING_CONSOLE
static void PowerTestPut(const char *pText)
{
	uint8_t c;

	while (*pText)
	{
		c = (uint8_t)*pText++;
		while (UARTTx(&s_Uart, &c, 1) != 1)
		{
		}
	}
}

static void PowerTestHex(uint32_t Value)
{
	char digits[9];
	int i;

	digits[8] = 0;
	for (i = 7; i >= 0; i--)
	{
		digits[i] = "0123456789abcdef"[Value & 15U];
		Value >>= 4;
	}
	PowerTestPut(digits);
}

static void PowerTestText(const char *pText)
{
	uint8_t dummy;

	PowerTestPut(pText);
	// Poll the UART until the last queued byte reaches the transmitter.
	while (CFifoUsed(s_Uart.hTxFifo) != 0)
	{
		UARTRx(&s_Uart, &dummy, 1);
	}
}

static void PowerTestStatus(const char *pName, const char *pStatus)
{
	PowerTestPut(pName);
	PowerTestText(pStatus);
}

static void PowerTestValue(const char *pName, uint32_t Value)
{
	PowerTestPut(pName);
	PowerTestHex(Value);
	PowerTestText("\r\n");
}

static void PowerTestPair(const char *pName, uint32_t First, const char *pMiddle,
                          uint32_t Second, const char *pEnd)
{
	PowerTestPut(pName);
	PowerTestHex(First);
	PowerTestPut(pMiddle);
	PowerTestHex(Second);
	PowerTestText(pEnd);
}
#else
#define PowerTestText(text) SysLogPrintf(SysLogGet(), text)
#define PowerTestStatus(name, status) SysLogPrintf(SysLogGet(), "%s%s", name, status)
#define PowerTestValue(name, value) SysLogPrintf(SysLogGet(), name "%08x\r\n", (unsigned)(value))
#define PowerTestPair(name, first, middle, second, end) \
	SysLogPrintf(SysLogGet(), name "%08x" middle "%08x" end, (unsigned)(first), (unsigned)(second))
#endif

static void CorePowerTestResult(const char *pName, bool Pass)
{
	s_Checks++;
	if (!Pass)
	{
		s_Failures++;
	}
	PowerTestStatus(pName, Pass ? ": PASS\r\n" : ": FAIL\r\n");
}

static void CorePowerTestUnsupported(const char *pName)
{
	PowerTestStatus(pName, ": UNSUPPORTED\r\n");
}

static void CorePowerTestSummary(void)
{
#if !CORE_POWER_TEST_POLLING_CONSOLE
	uint32_t logRejected;
	uint32_t uartUnaccepted;
	int logPending;
	int uartPending;

	// Snapshot transport backpressure independently from the test assertions.
	// TxDropCnt counts unaccepted UART submissions (which SysLog may retry);
	// it is not proof that a byte was lost on the physical wire.
	logRejected = __atomic_load_n(&SysLogGet()->hFifo->DropCnt, __ATOMIC_RELAXED);
	uartUnaccepted = s_Uart.TxDropCnt;
	logPending = CFifoUsed(SysLogGet()->hFifo);
	uartPending = CFifoUsed(s_Uart.hTxFifo);
	SysLogPrintf(SysLogGet(),
		"LogDiag fifo=0x%08x uartfifo=0x%08x evicted=0x%08x uartRejected=0x%08x\r\n",
		(unsigned)logPending, (unsigned)uartPending,
		(unsigned)logRejected, (unsigned)uartUnaccepted);
#endif
	PowerTestPair("CorePowerTest complete: 0x", s_Failures, " failures, 0x", s_Checks, " checks\r\n");
}

static uint32_t CorePowerTestRamPattern(uint32_t Idx)
{
	return 0xA55A1234UL ^ (Idx * 0x9E3779B9UL);
}

#if CORE_POWER_TEST_WAKE_MS > 0
static void CorePowerTestTimerWake(TimerDev_t *pTimer, int Trigger, void *pContext)
{
	(void)pTimer;
	(void)Trigger;
	(void)pContext;
	s_TimerWoke = true;
}
#endif

static void CorePowerTestRetention(bool Marker)
{
#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
	uint32_t i;
	uint32_t count;
	uint32_t unchanged;
	bool valid;

	count = POWER_DEMO_RAM_RET_SIZE / sizeof(uint32_t);
	if (Marker)
	{
		valid = true;
		for (i = 0U; i < count; i++)
		{
			if (s_RetRam[i] != CorePowerTestRamPattern(i))
			{
				valid = false;
				break;
			}
		}
		CorePowerTestResult("RAM retention after System OFF", valid);
#if defined(POWER_DEMO_RAM_DROP_ADDR)
		count = POWER_DEMO_RAM_DROP_SIZE / sizeof(uint32_t);
		unchanged = 0U;
		for (i = 0U; i < count; i++)
		{
			if (s_DropRam[i] == (CorePowerTestRamPattern(i) ^ 0xFFFFFFFFUL))
			{
				unchanged++;
			}
		}
		PowerTestPair("RAM non-retention unchanged words 0x", unchanged, " / 0x", count, "\r\n");
		valid = unchanged != count;
		CorePowerTestResult("RAM non-retention after System OFF", valid);
#endif
	}
	CorePowerTestResult("RAM retention invalid range",
		!SystemRamRetain(0U, POWER_DEMO_RAM_RET_SIZE, true) &&
		!SystemRamRetain(POWER_DEMO_RAM_RET_ADDR, 0U, true));
	CorePowerTestResult("RAM retention enable",
		SystemRamRetain(POWER_DEMO_RAM_RET_ADDR, POWER_DEMO_RAM_RET_SIZE, true));
#if defined(POWER_DEMO_RAM_DROP_ADDR)
	CorePowerTestResult("RAM non-retention disable",
		SystemRamRetain(POWER_DEMO_RAM_DROP_ADDR, POWER_DEMO_RAM_DROP_SIZE, false));
	count = POWER_DEMO_RAM_DROP_SIZE / sizeof(uint32_t);
	for (i = 0U; i < count; i++)
	{
		s_DropRam[i] = CorePowerTestRamPattern(i) ^ 0xFFFFFFFFUL;
	}
#endif
	count = POWER_DEMO_RAM_RET_SIZE / sizeof(uint32_t);
	for (i = 0U; i < count; i++)
	{
		s_RetRam[i] = CorePowerTestRamPattern(i);
	}
#else
	(void)Marker;
	CorePowerTestResult("RAM retention test region configured", false);
#endif
}

static void CorePowerTestRunningMode(uint32_t Caps)
{
	uint32_t before;
	uint32_t reduced;
	bool ok;

	before = SystemCoreClockGet();
	if ((Caps & SYSTEM_POWER_CAP_LOW_POWER) == 0U)
	{
		CorePowerTestUnsupported("Reduced CPU power mode");
		return;
	}
	// Let UART finish pending output before changing the CPU power mode.
	msDelay(100);
	ok = SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER);
	reduced = SystemCoreClockGet();
	CorePowerTestResult("Reduced CPU power mode", ok &&
		SystemPowerModeGet() == SYSTEM_POWER_MODE_LOW_POWER && reduced <= before);
	msDelay(100);
	ok = SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE);
	CorePowerTestResult("CPU performance restoration", ok &&
		SystemCoreClockGet() == before &&
		SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);
}

#if CORE_POWER_TEST_WAKE_MS > 0
static bool CorePowerTestTimedWake(void)
{
	uint32_t state;
	uint32_t start;
	uint32_t elapsed;
	uint64_t period;
	uint64_t expected;
	bool ok;

	if (TimerSetFrequency(&s_WakeTimer, 1024U) == 0U || !TimerEnable(&s_WakeTimer))
	{
		CorePowerTestResult("Wake timer enable", false);
		return false;
	}
	PowerTestText("Entering Standby; automatic timer wake\r\n");
	// Allow queued UART DMA transfers to finish before entering sleep.
	msDelay(100);
	state = DisableInterrupt();
	s_TimerWoke = false;
	start = TimerGetMicrosecond(&s_WakeTimer);
	period = nsTimerEnableTrigger(&s_WakeTimer, 0,
		(uint64_t)CORE_POWER_TEST_WAKE_MS * 1000000ULL,
		TIMER_TRIG_TYPE_SINGLE, CorePowerTestTimerWake, nullptr);
	if (period != 0U && !s_TimerWoke)
	{
		SystemSleep();
	}
	EnableInterrupt(state);
	elapsed = TimerGetMicrosecond(&s_WakeTimer) - start;
	TimerDisableTrigger(&s_WakeTimer, 0);
	TimerDisable(&s_WakeTimer);
	expected = (uint64_t)CORE_POWER_TEST_WAKE_MS * 1000ULL;
	ok = period != 0U && s_TimerWoke &&
		(uint64_t)elapsed >= expected * 98ULL / 100ULL &&
		(uint64_t)elapsed <= expected * 102ULL / 100ULL;
	CorePowerTestResult("Standby timer wake", ok);
	if (!ok)
	{
		PowerTestStatus("Trigger armed", period != 0U ? ": yes\r\n" : ": no\r\n");
		PowerTestStatus("RTC callback", s_TimerWoke ? ": yes\r\n" : ": no\r\n");
		PowerTestValue("Elapsed us 0x", elapsed);
	}
	return true;
}
#endif

#if defined(WAKE_BUT_PIN)
static bool CorePowerTestButtonWake(void)
{
	uint32_t state;

	do
	{
		while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) ==
			(WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? 1 : 0))
		{
			msDelay(1);
		}
		s_ButtonWoke = false;
		PowerTestText("Entering Standby; press wake button\r\n");
		msDelay(100);
		state = DisableInterrupt();
		if (!s_ButtonWoke)
		{
			SystemSleep();
			EnableInterrupt(state);
			break;
		}
		// A press during the prompt already ran the handler. Arm another trial.
		EnableInterrupt(state);
	} while (true);
	CorePowerTestResult("Standby button wake", s_ButtonWoke);
	return true;
}
#endif

static void CorePowerTestSleepCycles(bool (*Wake)(void))
{
	uint32_t cycle;
	uint32_t coreBefore;
	uint32_t periphBefore;
	uint32_t uartBefore;
	uint32_t i;
	uint32_t pattern;
	bool ok;
	volatile uint32_t stackRam[CORE_POWER_TEST_RAM_WORDS];

	for (cycle = 0U; cycle < CORE_POWER_TEST_SLEEP_CYCLES; cycle++)
	{
		coreBefore = SystemCoreClockGet();
		periphBefore = SystemPeriphClockGet(0);
		uartBefore = (uint32_t)UARTGetRate(&s_Uart);
		for (i = 0U; i < CORE_POWER_TEST_RAM_WORDS; i++)
		{
			pattern = CorePowerTestRamPattern(i) ^ cycle;
			s_StaticRam[i] = pattern;
			stackRam[i] = ~pattern;
		}
		if (!Wake())
		{
			return;
		}
		CorePowerTestResult("CPU/peripheral clock after resume",
			SystemCoreClockGet() == coreBefore &&
			SystemPeriphClockGet(0) == periphBefore);
		CorePowerTestResult("UART rate after resume", (uint32_t)UARTGetRate(&s_Uart) == uartBefore);
		ok = true;
		for (i = 0U; i < CORE_POWER_TEST_RAM_WORDS; i++)
		{
			pattern = CorePowerTestRamPattern(i) ^ cycle;
			if (s_StaticRam[i] != pattern || stackRam[i] != ~pattern)
			{
				ok = false;
			}
		}
		CorePowerTestResult("Static/stack RAM after resume", ok);
		PowerTestValue("Standby cycle 0x", cycle + 1U);
	}
}

static void CorePowerTestSleep(uint32_t Caps, bool Button = (CORE_POWER_TEST_WAKE_MS == 0))
{
#if CORE_POWER_TEST_WAKE_MS > 0
	bool ok;
#endif

#if defined(CORE_POWER_TEST_LEGACY_OPTIONAL_APIS) && defined(__GNUC__)
	if (SystemSleep == nullptr)
	{
		CorePowerTestUnsupported("Resumable SystemSleep");
		return;
	}
#endif
	if ((Caps & SYSTEM_POWER_CAP_SLEEP) == 0U)
	{
		CorePowerTestUnsupported("Resumable SystemSleep");
		return;
	}
#if defined(WAKE_BUT_PIN)
	if (Button)
	{
		CorePowerTestSleepCycles(CorePowerTestButtonWake);
		return;
	}
#endif
#if CORE_POWER_TEST_WAKE_MS > 0
	if ((Caps & SYSTEM_POWER_CAP_TIMER_WAKE) == 0U)
	{
		CorePowerTestUnsupported("Resumable timer wake");
		return;
	}

	ok = TimerInit(&s_WakeTimer, &s_TimerCfg);
	CorePowerTestResult("Wake timer initialization", ok);
	if (!ok)
	{
		return;
	}
	TimerDisable(&s_WakeTimer);
	CorePowerTestSleepCycles(CorePowerTestTimedWake);
#endif
}

#if defined(WAKE_BUT_PIN)
#ifndef CORE_POWER_TEST_WAKE_INT_PRIO
#define CORE_POWER_TEST_WAKE_INT_PRIO	6
#endif

// Record the edge even if the button is released before the test resumes.
static void CorePowerTestWakeHandler(int IntNo, void *pCtx)
{
	(void)IntNo;
	(void)pCtx;
	s_ButtonWoke = true;
}
#endif

static bool CorePowerTestWakePin(uint32_t Caps)
{
#if defined(WAKE_BUT_PIN)
	bool ok;

	if ((Caps & SYSTEM_POWER_CAP_WAKE_PIN) == 0U)
	{
		CorePowerTestUnsupported("Wake pin");
		return false;
	}
	IOPinConfig(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_PINOP,
		IOPINDIR_INPUT, WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ?
		IOPINRES_PULLDOWN : IOPINRES_PULLUP, IOPINTYPE_NORMAL);
	// The wake source is the pin interrupt, enabled with the pin driver.
	s_WakeIntNo = IOPinAllocateInterrupt(CORE_POWER_TEST_WAKE_INT_PRIO,
		WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE, CorePowerTestWakeHandler,
		nullptr);
	ok = s_WakeIntNo >= 0;
	CorePowerTestResult("Wake pin configuration", ok);
	if (!ok)
	{
		return false;
	}
	IOPinDisableInterrupt(s_WakeIntNo);
	s_WakeIntNo = IOPinAllocateInterrupt(CORE_POWER_TEST_WAKE_INT_PRIO,
		WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE, CorePowerTestWakeHandler,
		nullptr);
	ok = s_WakeIntNo >= 0;
	CorePowerTestResult("Wake pin rearm", ok);
	return ok;
#else
	(void)Caps;
	CorePowerTestUnsupported("Wake pin (no board pin)");
	return false;
#endif
}

int main(void)
{
	uint32_t caps;
	uint32_t reason;
	uint32_t startCount;
	int retCount;
	bool marker;
	bool pinWake;

	if (!UARTInit(&s_Uart, &s_UartCfg))
	{
		return 1;
	}
#if !CORE_POWER_TEST_POLLING_CONSOLE
	if (!SysLogInitNoTimer(SysLogGet(), &s_LogCfg, &s_Uart.DevIntrf, 0, 0))
	{
		return 1;
	}
#endif
	PowerTestText("CorePowerTest\r\n");
	s_Checks = 0U;
	s_Failures = 0U;
#if defined(CORE_POWER_TEST_LEGACY_OPTIONAL_APIS) && defined(__GNUC__)
	if (SystemPowerCapabilitiesGet == nullptr)
	{
		CorePowerTestUnsupported("Power capability API");
		CorePowerTestSummary();
		return 1;
	}
#endif
	caps = SystemPowerCapabilitiesGet();
	PowerTestValue("Capabilities 0x", caps);
	reason = SystemResetReason();
	PowerTestValue("Reset reason 0x", reason);
	CorePowerTestResult("Reset reason stable", SystemResetReason() == reason);

	retCount = SystemRetRegCount();
	marker = false;
	startCount = 0U;
	if ((caps & SYSTEM_POWER_CAP_RET_REG) != 0U && retCount > 0)
	{
		startCount = SystemRetRegRead(0);
		// Retention marker only proves a previous System OFF test when
		// this reset was caused by a configured System OFF wake source.
		// A reset-pin event must not validate stale RAM patterns.
		marker = retCount > 1 &&
			(reason & (SYSTEM_RESET_WAKE_PIN | SYSTEM_RESET_WAKE_TIMER)) != 0U &&
			SystemRetRegRead(1) == CORE_POWER_TEST_MARKER;
		CorePowerTestResult("Retention register write/read",
			SystemRetRegWrite(0, (startCount + 1U) & 0xFFU) &&
			SystemRetRegRead(0) == ((startCount + 1U) & 0xFFU));
		CorePowerTestResult("Retention register invalid index",
			SystemRetRegRead(-1) == 0U && !SystemRetRegWrite(-1, 1U) &&
			SystemRetRegRead(retCount) == 0U && !SystemRetRegWrite(retCount, 1U));
		if ((reason & (SYSTEM_RESET_WAKE_PIN | SYSTEM_RESET_WAKE_TIMER)) == 0U &&
			retCount > 1 && SystemRetRegRead(1) == CORE_POWER_TEST_MARKER)
		{
			(void)SystemRetRegWrite(1, 0U);
		}
	}
	else
	{
		CorePowerTestUnsupported("Retention registers");
	}

	if (marker)
	{
		CorePowerTestResult("System OFF reset-on-wake",
			(reason & (SYSTEM_RESET_WAKE_PIN | SYSTEM_RESET_WAKE_TIMER)) != 0U);
	}
	if ((caps & SYSTEM_POWER_CAP_RAM_RETENTION) != 0U)
	{
		CorePowerTestRetention(marker);
	}
	else
	{
		CorePowerTestUnsupported("System OFF RAM retention");
	}
	CorePowerTestRunningMode(caps);
#if CORE_POWER_TEST_WAKE_MS == 0
	pinWake = CorePowerTestWakePin(caps);
	if (pinWake)
	{
		CorePowerTestSleep(caps);
	}
#else
	CorePowerTestSleep(caps);
	pinWake = CorePowerTestWakePin(caps);
#if defined(WAKE_BUT_PIN)
	if (pinWake && (caps & SYSTEM_POWER_CAP_SLEEP) != 0U)
	{
		CorePowerTestSleep(caps, true);
	}
#endif
#endif
	CorePowerTestSummary();

	if ((caps & SYSTEM_POWER_CAP_SYSTEM_OFF) == 0U)
	{
		CorePowerTestUnsupported("System OFF");
	}
	else if (pinWake && WAKE_BUT_SYSTEM_OFF)
	{
		if (retCount > 1)
		{
			(void)SystemRetRegWrite(1, CORE_POWER_TEST_MARKER);
		}
		PowerTestText("Entering System OFF; press wake button\r\n");
#if defined(WAKE_BUT_PIN)
		// Do not immediately wake if the button is still held.
		while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) ==
			(WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? 1 : 0))
		{
			msDelay(1);
		}
#endif
		// Keep interrupts enabled while UART sends the final message.
		msDelay(100);
		SystemPowerOff();
		CorePowerTestResult("System OFF returned unexpectedly", false);
		CorePowerTestSummary();
	}
	else
	{
		CorePowerTestUnsupported("System OFF: no board button configured for wake");
	}

#if defined(WAKE_BUT_PIN)
	if (pinWake)
	{
		IOPinDisableInterrupt(s_WakeIntNo);
	}
#endif

	while (1)
	{
		__WFI();
	}
}
