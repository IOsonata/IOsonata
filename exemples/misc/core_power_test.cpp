/**-------------------------------------------------------------------------
@example	core_power_test.cpp
@brief	Generic power-management API validation

The test runs all capabilities reported by SystemPowerCapabilitiesGet().
Board definitions only describe physical UART, wake pins and reserved RAM.
No board define selects System OFF versus resumable sleep.

System OFF is last: the next boot validates reset-on-wake and retention.
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

#define CORE_POWER_TEST_MARKER	0xC3U
#define CORE_POWER_TEST_RAM_WORDS	32U
// At 1 Mbps this gives the ISR time to send several hundred UART bytes.
#define CORE_POWER_TEST_TX_DRAIN_MS	10U

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

static const TimerCfg_t s_TimerCfg = {
	.DevNo = CORE_POWER_TEST_TIMER_DEVNO,
	.ClkSrc = TIMER_CLKSRC_DEFAULT,
	.Freq = 1024U,
	.IntPrio = 6,
};

static UART s_Uart;
static TimerDev_t s_WakeTimer;
static volatile bool s_TimerWoke;
static volatile uint32_t s_StaticRam[CORE_POWER_TEST_RAM_WORDS];
static uint32_t s_Checks;
static uint32_t s_Failures;

#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
static volatile uint32_t * const s_RetRam = (volatile uint32_t *)POWER_DEMO_RAM_RET_ADDR;
#if defined(POWER_DEMO_RAM_DROP_ADDR)
static volatile uint32_t * const s_DropRam = (volatile uint32_t *)POWER_DEMO_RAM_DROP_ADDR;
#endif
#endif

static void CorePowerTestPut(const char *pText)
{
	int remaining;
	int sent;
	bool lineEnd;

	remaining = 0;
	while (pText[remaining] != 0)
	{
		remaining++;
	}
	lineEnd = remaining > 0 && pText[remaining - 1] == '\n';
	while (remaining > 0)
	{
		sent = s_Uart.Tx((const uint8_t *)pText, remaining);
		if (sent > 0)
		{
			pText += sent;
			remaining -= sent;
		}
	}
	// Tx() queues the data. Let the UART IRQ drain a complete record.
	// This is a test-console delay, not a driver flush or TX polling.
	if (lineEnd)
	{
		msDelay(CORE_POWER_TEST_TX_DRAIN_MS);
	}
}

static void CorePowerTestNumber(uint32_t Value)
{
	char output[11];
	uint8_t digit;
	int shift;
	int i;

	output[0] = '0';
	output[1] = 'x';
	i = 2;
	for (shift = 28; shift >= 0; shift -= 4)
	{
		digit = (uint8_t)((Value >> shift) & 15U);
		output[i] = (char)(digit < 10U ? '0' + digit : 'A' + digit - 10U);
		i++;
	}
	output[i] = 0;
	CorePowerTestPut(output);
}

static void CorePowerTestResult(const char *pName, bool Pass)
{
	s_Checks++;
	if (!Pass)
	{
		s_Failures++;
	}
	CorePowerTestPut(pName);
	CorePowerTestPut(Pass ? ": PASS\r\n" : ": FAIL\r\n");
}

static void CorePowerTestUnsupported(const char *pName)
{
	CorePowerTestPut(pName);
	CorePowerTestPut(": UNSUPPORTED\r\n");
}

static void CorePowerTestSummary(void)
{
	CorePowerTestPut("CorePowerTest complete: ");
	CorePowerTestNumber(s_Failures);
	CorePowerTestPut(" failures, ");
	CorePowerTestNumber(s_Checks);
	CorePowerTestPut(" checks\r\n");
}

static uint32_t CorePowerTestRamPattern(uint32_t Idx)
{
	return 0xA55A1234UL ^ (Idx * 0x9E3779B9UL);
}

static void CorePowerTestTimerWake(TimerDev_t *pTimer, int Trigger, void *pContext)
{
	(void)pTimer;
	(void)Trigger;
	(void)pContext;
	s_TimerWoke = true;
}

static void CorePowerTestRetention(uint32_t Caps, uint32_t Reason, bool Marker)
{
#if defined(POWER_DEMO_RAM_RET_TEST) && POWER_DEMO_RAM_RET_TEST
	uint32_t i;
	uint32_t count;
	uint32_t unchanged;
	bool valid;

	if ((Caps & SYSTEM_POWER_CAP_RAM_RETENTION) == 0U)
	{
		CorePowerTestUnsupported("RAM retention");
		return;
	}

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
		CorePowerTestPut("RAM non-retention unchanged words ");
		CorePowerTestNumber(unchanged);
		CorePowerTestPut(" / ");
		CorePowerTestNumber(count);
		CorePowerTestPut("\r\n");
		valid = unchanged != count;
		CorePowerTestResult("RAM non-retention after System OFF", valid);
#endif
	}
	(void)Reason;
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
	(void)Caps;
	(void)Reason;
	(void)Marker;
	CorePowerTestUnsupported("RAM retention (no reserved board region)");
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
	ok = SystemPowerModeSet(SYSTEM_POWER_MODE_LOW_POWER);
	reduced = SystemCoreClockGet();
	CorePowerTestResult("Reduced CPU power mode", ok &&
		SystemPowerModeGet() == SYSTEM_POWER_MODE_LOW_POWER && reduced <= before);
	ok = SystemPowerModeSet(SYSTEM_POWER_MODE_PERFORMANCE);
	CorePowerTestResult("CPU performance restoration", ok &&
		SystemCoreClockGet() == before &&
		SystemPowerModeGet() == SYSTEM_POWER_MODE_PERFORMANCE);
}

static void CorePowerTestSleep(uint32_t Caps)
{
	TimerDev_t *pTimer;
	uint32_t cycle;
	uint32_t coreBefore;
	uint32_t periphBefore;
	uint32_t uartBefore;
	uint32_t state;
	uint32_t start;
	uint32_t elapsed;
	uint32_t i;
	uint32_t pattern;
	uint64_t period;
	uint64_t expected;
	bool ok;
	volatile uint32_t stackRam[CORE_POWER_TEST_RAM_WORDS];

	if ((Caps & SYSTEM_POWER_CAP_SLEEP) == 0U || SystemSleep == nullptr)
	{
		CorePowerTestUnsupported("Resumable SystemSleep");
		return;
	}
	if ((Caps & SYSTEM_POWER_CAP_TIMER_WAKE) == 0U)
	{
		CorePowerTestUnsupported("Resumable timer wake");
		return;
	}

	pTimer = &s_WakeTimer;
	ok = TimerInit(pTimer, &s_TimerCfg);
	CorePowerTestResult("Wake timer initialization", ok);
	if (!ok)
	{
		return;
	}
	TimerDisable(pTimer);
	for (cycle = 0U; cycle < CORE_POWER_TEST_SLEEP_CYCLES; cycle++)
	{
		coreBefore = SystemCoreClockGet();
		periphBefore = SystemPeriphClockGet(0);
		uartBefore = s_Uart.Rate();
		for (i = 0U; i < CORE_POWER_TEST_RAM_WORDS; i++)
		{
			pattern = CorePowerTestRamPattern(i) ^ cycle;
			s_StaticRam[i] = pattern;
			stackRam[i] = ~pattern;
		}
		if (TimerSetFrequency(pTimer, 1024U) == 0U || !TimerEnable(pTimer))
		{
			CorePowerTestResult("Wake timer enable", false);
			return;
		}
		CorePowerTestPut("Entering Standby; automatic timer wake\r\n");
		// All pending test output must finish before CPU sleep.
		msDelay(CORE_POWER_TEST_TX_DRAIN_MS);
		state = DisableInterrupt();
		s_TimerWoke = false;
		start = TimerGetMicrosecond(pTimer);
		period = nsTimerEnableTrigger(pTimer, 0,
			(uint64_t)CORE_POWER_TEST_WAKE_MS * 1000000ULL,
			TIMER_TRIG_TYPE_SINGLE, CorePowerTestTimerWake, nullptr);
		if (period != 0U && !s_TimerWoke)
		{
			SystemSleep();
		}
		EnableInterrupt(state);
		elapsed = TimerGetMicrosecond(pTimer) - start;
		TimerDisableTrigger(pTimer, 0);
		TimerDisable(pTimer);
		expected = (uint64_t)CORE_POWER_TEST_WAKE_MS * 1000ULL;
		CorePowerTestResult("Standby timer wake", period != 0U && s_TimerWoke &&
			(uint64_t)elapsed >= expected * 98ULL / 100ULL &&
			(uint64_t)elapsed <= expected * 102ULL / 100ULL);
		CorePowerTestResult("CPU/peripheral clock after resume",
			SystemCoreClockGet() == coreBefore &&
			SystemPeriphClockGet(0) == periphBefore);
		CorePowerTestResult("UART rate after resume", s_Uart.Rate() == uartBefore);
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
		CorePowerTestPut("Standby cycle ");
		CorePowerTestNumber(cycle + 1U);
		CorePowerTestPut("\r\n");
	}
}

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
	ok = SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE);
	CorePowerTestResult("Wake pin configuration", ok);
	if (!ok)
	{
		return false;
	}
	CorePowerTestResult("Wake pin invalid index",
		!SystemWakePin(WAKE_BUT_PORT, -1, WAKE_BUT_SENSE));
	CorePowerTestResult("Wake pin disable",
		SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, IOPINSENSE_DISABLE));
	ok = SystemWakePin(WAKE_BUT_PORT, WAKE_BUT_PIN, WAKE_BUT_SENSE);
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

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
	CorePowerTestPut("CorePowerTest\r\n");
	s_Checks = 0U;
	s_Failures = 0U;
	if (SystemPowerCapabilitiesGet == nullptr)
	{
		CorePowerTestUnsupported("Power capability API");
		CorePowerTestSummary();
		return 1;
	}
	caps = SystemPowerCapabilitiesGet();
	CorePowerTestPut("Capabilities ");
	CorePowerTestNumber(caps);
	CorePowerTestPut("\r\n");
	reason = SystemResetReason();
	CorePowerTestPut("Reset reason ");
	CorePowerTestNumber(reason);
	CorePowerTestPut("\r\n");
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
			SystemRetRegWrite(0, startCount + 1U) &&
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
	CorePowerTestRetention(caps, reason, marker);
	CorePowerTestRunningMode(caps);
	CorePowerTestSleep(caps);

	pinWake = CorePowerTestWakePin(caps);
	CorePowerTestSummary();

	if ((caps & SYSTEM_POWER_CAP_SYSTEM_OFF) == 0U)
	{
		CorePowerTestUnsupported("System OFF");
	}
	else if (pinWake)
	{
		if (retCount > 1)
		{
			(void)SystemRetRegWrite(1, CORE_POWER_TEST_MARKER);
		}
		CorePowerTestPut("Entering System OFF; press wake button\r\n");
#if defined(WAKE_BUT_PIN)
		// Do not immediately wake if the button is still held.
		while (IOPinRead(WAKE_BUT_PORT, WAKE_BUT_PIN) ==
			(WAKE_BUT_SENSE == IOPINSENSE_HIGH_TRANSITION ? 1 : 0))
		{
			msDelay(1);
		}
#endif
		// Keep interrupts enabled while the final message drains.
		msDelay(100);
		SystemPowerOff();
		CorePowerTestResult("System OFF returned unexpectedly", false);
		CorePowerTestSummary();
	}
	else
	{
		CorePowerTestUnsupported("System OFF: no validated wake source");
	}

	while (1)
	{
		__WFI();
	}
}
