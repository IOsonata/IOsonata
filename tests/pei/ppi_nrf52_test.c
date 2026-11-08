#define _DEFAULT_SOURCE
#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

#include "nrf.h"
#include "coredev/pei.h"

// MCU hooks of the software events, called directly to check exhaustion
bool PeiSwEventAllocMcu(int DevNo, PeiEndpoint_t *pEvent);
void PeiSwEventFreeMcu(PeiEndpoint_t Event);

NRF_PPI_Type g_TestPpi;
int g_critical_depth;
static int g_assign_calls;
static int g_enable_calls;
static int g_disable_calls;
static int g_fail_assign;
static int g_fail_enable;
static int g_fail_disable;

#if defined(SOFTDEVICE_PRESENT)
// SoftDevice programmed (magic number) and enabled, changed by
// TestNoSoftDevice
uint32_t g_TestSdInfo[2] = { 0U, 0x51B1E5DBU };
static int g_sd_enabled = 1;
static int g_sd_enabled_calls;

uint32_t sd_softdevice_is_enabled(uint8_t *p_softdevice_enabled)
{
	assert(g_critical_depth == 0);
	g_sd_enabled_calls++;
	*p_softdevice_enabled = (uint8_t)g_sd_enabled;
	return 0;
}
#endif

uint32_t sd_ppi_channel_assign(uint8_t ch, const volatile void *evt, const volatile void *task)
{
	assert(g_critical_depth == 0);
	g_assign_calls++;
	if (g_fail_assign)
	{
		return 1;
	}
	g_TestPpi.CH[ch].EEP = (uint32_t)(uintptr_t)evt;
	g_TestPpi.CH[ch].TEP = (uint32_t)(uintptr_t)task;
	return 0;
}

uint32_t sd_ppi_channel_enable_set(uint32_t mask)
{
	assert(g_critical_depth == 0);
	g_enable_calls++;
	if (g_fail_enable)
	{
		return 1;
	}
	g_TestPpi.CHENSET = mask;
	g_TestPpi.CHEN |= mask;
	return 0;
}

uint32_t sd_ppi_channel_enable_clr(uint32_t mask)
{
	assert(g_critical_depth == 0);
	g_disable_calls++;
	if (g_fail_disable)
	{
		return 1;
	}
	g_TestPpi.CHENCLR = mask;
	g_TestPpi.CHEN &= ~mask;
	return 0;
}

static PeiCfg_t Config(uint32_t Event, uint32_t Task)
{
	PeiCfg_t cfg = {0};
	cfg.Event = Event;
	cfg.Task = Task;
	return cfg;
}

static int Channel(const PeiConn_t *pConn)
{
	return (int)((pConn->Handle & 31U) - 1U);
}

static void ResetMock(void)
{
	memset(&g_TestPpi, 0, sizeof(g_TestPpi));
	g_assign_calls = 0;
	g_enable_calls = 0;
	g_disable_calls = 0;
	g_fail_assign = 0;
	g_fail_enable = 0;
	g_fail_disable = 0;
	assert(g_critical_depth == 0);
}

static void TestValidation(void)
{
	PeiConn_t c = {0};
	PeiCfg_t cfg = Config(0x40008100U, 0x40006000U);

	assert(!PeiConnect(NULL, &cfg));
	assert(!PeiConnect(&c, NULL));
	cfg.DevNo = 1;
	assert(!PeiConnect(&c, &cfg));
	cfg.DevNo = 0;
	cfg.Options = 1;
	assert(!PeiConnect(&c, &cfg));
	cfg.Options = 0;
	cfg.Event = 0;
	assert(!PeiConnect(&c, &cfg));
	cfg.Event = 0x20000100U;
	assert(!PeiConnect(&c, &cfg));
	cfg.Event = 0x40008102U;
	assert(!PeiConnect(&c, &cfg));
	cfg.Event = 0x40008000U;
	assert(!PeiConnect(&c, &cfg));
	cfg.Event = 0x40008100U;
	cfg.Task = 0x40006100U;
	assert(!PeiConnect(&c, &cfg));
	cfg.Task = 0x40006002U;
	assert(!PeiConnect(&c, &cfg));
	assert(c.Handle == 0U);
	assert(g_TestPpi.CH[2].EEP == 0U);
	assert(!PeiEnable(&c));
	assert(!PeiDisable(&c));
	PeiDisconnect(&c);
	PeiDisconnect(NULL);
	assert(g_critical_depth == 0);
}

static void TestLifecycle(void)
{
	PeiConn_t c = {0};
	PeiConn_t copied;
	PeiCfg_t cfg = Config(0x40008100U, 0x40006000U);
	uint32_t first;
	int ch;

	assert(PeiConnect(&c, &cfg));
	ch = Channel(&c);
	first = c.Handle;
	assert(ch >= 2 && ch < TEST_PPI_CHANNEL_COUNT);
	assert(g_TestPpi.CH[ch].EEP == (uint32_t)cfg.Event);
	assert(g_TestPpi.CH[ch].TEP == (uint32_t)cfg.Task);
	assert((g_TestPpi.CHEN & (1UL << ch)) == 0U);
	assert(!PeiConnect(&c, &cfg));
	copied = c;
	assert(!PeiEnable(&copied));
	PeiDisconnect(&copied);
	assert(copied.Handle == 0U);
	assert(c.Handle == first);
	assert(PeiEnable(&c));
	assert(PeiEnable(&c));
	assert(g_TestPpi.CHENSET == (1UL << ch) || g_enable_calls > 0);
	assert(PeiDisable(&c));
	assert(PeiDisable(&c));
	assert(g_TestPpi.CHENCLR == (1UL << ch) || g_disable_calls > 0);
	PeiDisconnect(&c);
	assert(c.Handle == 0U);
	assert(g_TestPpi.CH[ch].EEP == 0U);
	assert(g_TestPpi.CH[ch].TEP == 0U);
	assert(g_TestPpi.FORK[ch].TEP == 0U);
	assert(g_critical_depth == 0);
	PeiDisconnect(&c);
	assert(PeiConnect(&c, &cfg));
	assert(c.Handle != first);
	PeiDisconnect(&c);
}

static void TestOccupied(void)
{
	PeiConn_t c = {0};
	PeiCfg_t cfg = Config(0x40008100U, 0x40006000U);
	g_TestPpi.CH[2].EEP = 0x40004100;
	g_TestPpi.CH[3].TEP = 0x40004000;
	g_TestPpi.FORK[4].TEP = 0x40005000;
	g_TestPpi.CHEN = (1UL << 5);
	g_TestPpi.CHG[2] = (1UL << 6);
	assert(PeiConnect(&c, &cfg));
	assert(Channel(&c) == 7);
	PeiDisconnect(&c);
	assert(g_TestPpi.CH[2].EEP == 0x40004100);
	assert(g_TestPpi.CH[3].TEP == 0x40004000);
	assert(g_TestPpi.FORK[4].TEP == 0x40005000);
	assert((g_TestPpi.CHEN & (1UL << 5)) != 0U);
	assert(g_TestPpi.CHG[2] == (1UL << 6));
	ResetMock();
}

static void TestMultipleAndExhaustion(void)
{
	PeiConn_t routes[20] = {{0}};
	PeiCfg_t cfg = Config(0x40008100U, 0x40006000U);
	uint32_t seen = 0;
	int count = 0;
	int i;

	for (i = 0; i < 20; i++)
	{
		// Same event to multiple tasks, multiple events to the same task.
		cfg.Event = (i & 1) ? 0x40009100U : 0x40008100U;
		cfg.Task = (i & 1) ? 0x40006000U : 0x40006004U;
		if (!PeiConnect(&routes[i], &cfg))
		{
			break;
		}
		assert((seen & (1UL << Channel(&routes[i]))) == 0U);
		seen |= (1UL << Channel(&routes[i]));
		assert(Channel(&routes[i]) >= 2);
		count++;
	}

#if defined(SOFTDEVICE_PRESENT) || defined(NRFXLIB_SDC)
	assert(count == ((TEST_PPI_CHANNEL_COUNT < 17 ? TEST_PPI_CHANNEL_COUNT : 17) - 2));
#else
	assert(count == TEST_PPI_CHANNEL_COUNT - 2);
#endif
	assert(!PeiConnect(&routes[i], &cfg));
	for (i = 0; i < count; i++)
	{
		PeiDisconnect(&routes[i]);
	}
	assert(g_critical_depth == 0);
}

static void TestSVCFailures(void)
{
#if defined(SOFTDEVICE_PRESENT)
	PeiConn_t c = {0};
	PeiCfg_t cfg = Config(0x40008100U, 0x40006000U);
	int ch;
	g_fail_assign = 1;
	assert(!PeiConnect(&c, &cfg));
	assert(c.Handle == 0U);
	assert(g_TestPpi.CH[2].EEP == 0U);
	g_fail_assign = 0;
	assert(PeiConnect(&c, &cfg));
	ch = Channel(&c);
	g_fail_enable = 1;
	assert(!PeiEnable(&c));
	g_fail_enable = 0;
	assert(PeiEnable(&c));
	g_fail_disable = 1;
	PeiDisconnect(&c);
	assert(c.Handle != 0U);
	g_fail_disable = 0;
	g_fail_assign = 1;
	PeiDisconnect(&c);
	assert(c.Handle != 0U);
	g_fail_assign = 0;
	PeiDisconnect(&c);
	assert(c.Handle == 0U);
	assert(g_TestPpi.CH[ch].EEP == 0U);
#endif
	assert(g_critical_depth == 0);
}

// S132/S140 library with the SoftDevice not enabled, or not programmed:
// the PPI registers are written directly, no SVC. sd_softdevice_is_enabled
// is not called when the SoftDevice image is not there.
static void TestNoSoftDevice(void)
{
#if defined(SOFTDEVICE_PRESENT)
	PeiConn_t c = {0};
	PeiCfg_t cfg = Config(0x40008100U, 0x40006000U);
	int ch;
	int pass;

	for (pass = 0; pass < 2; pass++)
	{
		ResetMock();
		g_sd_enabled_calls = 0;
		if (pass == 0)
		{
			g_sd_enabled = 0;
		}
		else
		{
			g_TestSdInfo[1] = 0U;
		}
		assert(PeiConnect(&c, &cfg));
		ch = Channel(&c);
		assert(g_TestPpi.CH[ch].EEP == 0x40008100U);
		assert(g_TestPpi.CH[ch].TEP == 0x40006000U);
		assert(PeiEnable(&c));
		assert(g_TestPpi.CHENSET == (1UL << ch));
		assert(PeiDisable(&c));
		assert(g_TestPpi.CHENCLR == (1UL << ch));
		PeiDisconnect(&c);
		assert(c.Handle == 0U);
		assert(g_TestPpi.CH[ch].EEP == 0U);
		assert(g_assign_calls == 0 && g_enable_calls == 0 && g_disable_calls == 0);
		assert(pass == 0 ? g_sd_enabled_calls > 0 : g_sd_enabled_calls == 0);
	}
	g_sd_enabled = 1;
	g_TestSdInfo[1] = 0x51B1E5DBU;
#endif
	assert(g_critical_depth == 0);
}

// Exercise the same public matrix API that C++ Pei wraps, from C.
static void TestCMatrix(void)
{
	PeiDev_t pei;
	PeiEvent_t event = 0U;
	PeiEvent_t oldToken;
	const PeiEndpoint_t source = 0x40008100U;
	const PeiEndpoint_t replacement = 0x40009100U;
	const PeiEndpoint_t taskA = 0x40006000U;
	const PeiEndpoint_t taskB = 0x40007000U;

	ResetMock();
	assert(PeiInit(&pei, 0));
	assert(PeiBindEvent(&pei, &event, source));
	assert(event != 0U);
	assert(PeiGetEventEndpoint(&pei, event) == source);
	assert(PeiConnectTask(&pei, event, taskA));
	assert(PeiConnectTask(&pei, event, taskB));
	assert(PeiEnableEvent(&pei, event));
	assert(PeiDisableEvent(&pei, event));
	assert(PeiUnbindEvent(&pei, event, source));
	assert(PeiGetEventEndpoint(&pei, event) == 0U);
	assert(PeiBindEvent(&pei, &event, replacement));
	assert(PeiEnableEvent(&pei, event));
	assert(PeiDisconnectTask(&pei, event, taskA));
	assert(PeiDisconnectTask(&pei, event, taskA));
	oldToken = event;
	assert(PeiReleaseEvent(&pei, &event));
	assert(event == 0U);
	assert(!PeiEnableEvent(&pei, oldToken));
	assert(PeiDeinit(&pei));
	assert(g_critical_depth == 0);
}

// The RTC's implicit COMPARE->CLEAR route is an ordinary connection in
// the same matrix as its application destinations. The RTC driver does
// not own a separate PeiConn_t.
static void TestRtcMatrixRearm(void)
{
	PeiDev_t pei;
	PeiEvent_t evt;
	const PeiEndpoint_t compare = 0x40008100U;
	const PeiEndpoint_t replacement = 0x40009100U;
	const PeiEndpoint_t clear = 0x40006000U;
	const PeiEndpoint_t led = 0x40007000U;

	evt = 0U;
	ResetMock();
	assert(PeiInit(&pei, 0));
	assert(PeiBindEvent(&pei, &evt, compare));
	assert(PeiConnectTask(&pei, evt, clear));
	assert(PeiConnectTask(&pei, evt, led));
	assert(PeiEnableEvent(&pei, evt));
	assert(PeiDisableEvent(&pei, evt));
	assert(PeiEnableEvent(&pei, evt));
	assert(PeiUnbindEvent(&pei, evt, compare));
	assert(PeiDisconnectTask(&pei, evt, clear));
	assert(PeiDisconnectTask(&pei, evt, clear));
	assert(PeiBindEvent(&pei, &evt, replacement));
	assert(PeiGetEventEndpoint(&pei, evt) == replacement);
	assert(PeiEnableEvent(&pei, evt));
	assert(PeiReleaseEvent(&pei, &evt));
	assert(PeiDeinit(&pei));
	assert(g_critical_depth == 0);
}

// Map the EGU registers at their nRF52 addresses in host memory
static void TestEguMap(void)
{
	void *p = mmap((void *)TEST_EGU_BASE, 6 * 0x1000, PROT_READ | PROT_WRITE,
				   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

	assert(p == (void *)TEST_EGU_BASE);
	memset(p, 0, 6 * 0x1000);
}

static int CountPpiRoutes(PeiEndpoint_t Event)
{
	int i;
	int count = 0;

	for (i = 0; i < TEST_PPI_CHANNEL_COUNT; i++)
	{
		if (g_TestPpi.CH[i].EEP == (uint32_t)Event && Event != 0U)
		{
			count++;
		}
	}
	return count;
}

// EGU instances PEI may use for the build profile, as in pei_nrf52.cpp
static int EguUsable(int Inst)
{
	if (Inst >= TEST_EGU_COUNT)
	{
		return 0;
	}
#if defined(SOFTDEVICE_PRESENT)
	return Inst == 0 || Inst == 3;
#elif defined(NRFXLIB_SDC)
	return Inst != 5;
#else
	return 1;
#endif
}

static PeiEndpoint_t EguEvent(int Inst, int Chan)
{
	return TEST_EGU_BASE + 0x1000U * (uint32_t)Inst + 0x100U + 4U * (uint32_t)Chan;
}

static void TestSwEvent(void)
{
	PeiDev_t pei;
	PeiEvent_t sw1 = 0U;
	PeiEvent_t sw2 = 0U;
	PeiEvent_t sw3 = 0U;
	PeiEvent_t hw = 0U;
	PeiEvent_t old;
	PeiEndpoint_t ep;
	PeiEndpoint_t first;
	const PeiEndpoint_t source = 0x40008100U;
	const PeiEndpoint_t taskA = 0x40006000U;
	const PeiEndpoint_t taskB = 0x40007000U;
	int expected;
	int count;
	int i;

	ResetMock();
	TestEguMap();
	assert(PeiInit(&pei, 0));

	// First software event takes channel 0 of the first usable instance
	assert(PeiBindSwEvent(&pei, &sw1));
	assert(sw1 != 0U);
	first = PeiGetEventEndpoint(&pei, sw1);
	assert(first == EguEvent(0, 0));
	assert(PeiGetSwEventTask(&pei, sw1) == first - 0x100U);
	assert(PeiConnectTask(&pei, sw1, taskA));
	assert(PeiConnectTask(&pei, sw1, taskB));
	assert(CountPpiRoutes(first) == 2);
	assert(PeiEnableEvent(&pei, sw1));

	// Firing writes TASKS_TRIGGER of the channel, never INTEN
	NRF_EGU0->EVENTS_TRIGGERED[0] = 1U;
	assert(PeiTriggerSwEvent(&pei, sw1));
	assert(NRF_EGU0->TASKS_TRIGGER[0] == 1U);
	assert(NRF_EGU0->EVENTS_TRIGGERED[0] == 0U);
	assert(NRF_EGU0->INTEN == 0U);

	// Already a software event: no new channel
	old = sw1;
	assert(PeiBindSwEvent(&pei, &sw1));
	assert(sw1 == old && PeiGetEventEndpoint(&pei, sw1) == first);

	// A hardware source is not a software event
	assert(PeiBindEvent(&pei, &hw, source));
	assert(!PeiTriggerSwEvent(&pei, hw));
	assert(PeiGetSwEventTask(&pei, hw) == 0U);
	assert(!PeiBindSwEvent(&pei, &hw));
	assert(!PeiTriggerSwEvent(&pei, 0U));
	assert(!PeiBindSwEvent(&pei, NULL));
	assert(!PeiBindSwEvent(NULL, &sw2));

	// Next channel; a channel with INTEN set by another user is skipped
	assert(PeiBindSwEvent(&pei, &sw2));
	assert(PeiGetEventEndpoint(&pei, sw2) == EguEvent(0, 1));
	NRF_EGU0->INTEN = 1U << 2;
	assert(PeiBindSwEvent(&pei, &sw3));
	assert(PeiGetEventEndpoint(&pei, sw3) == EguEvent(0, 3));
	NRF_EGU0->INTEN = 0U;

	// Unbind frees the channel and keeps the routes, a new bind reconnects them
	assert(PeiUnbindEvent(&pei, sw1, first));
	assert(CountPpiRoutes(first) == 0);
	assert(!PeiTriggerSwEvent(&pei, sw1));
	assert(PeiSwEventAllocMcu(0, &ep));
	assert(ep == first);
	PeiSwEventFreeMcu(ep);
	assert(PeiBindSwEvent(&pei, &sw1));
	assert(PeiGetEventEndpoint(&pei, sw1) == first);
	assert(CountPpiRoutes(first) == 2);
	assert(PeiTriggerSwEvent(&pei, sw1));

	// A failed bind gives the channel back: no PPI channel for the routes
	assert(PeiUnbindEvent(&pei, sw1, first));
	g_TestPpi.CHEN = 0xFFFFFFFFU;
	assert(!PeiBindSwEvent(&pei, &sw1));
	g_TestPpi.CHEN = 0U;
	assert(PeiSwEventAllocMcu(0, &ep));
	assert(ep == first);
	PeiSwEventFreeMcu(ep);
	// Freeing a channel PEI does not own does nothing
	PeiSwEventFreeMcu(source);
	PeiSwEventFreeMcu(EguEvent(0, 9));

	// Release frees the channel
	old = sw2;
	assert(PeiReleaseEvent(&pei, &sw2));
	assert(sw2 == 0U);
	assert(!PeiTriggerSwEvent(&pei, old));
	assert(PeiSwEventAllocMcu(0, &ep));
	assert(ep == first);
	assert(PeiSwEventAllocMcu(0, &ep));
	assert(ep == EguEvent(0, 1));
	PeiSwEventFreeMcu(ep);
	PeiSwEventFreeMcu(first);

	// Exhaustion: every channel of the usable instances except sw3. sw1 has
	// its identity but no channel since the failed bind.
	expected = 0;
	for (i = 0; i < 6; i++)
	{
		expected += EguUsable(i) ? 16 : 0;
	}
	expected -= 1;
	count = 0;
	while (PeiSwEventAllocMcu(0, &ep))
	{
		int inst = (int)((ep - TEST_EGU_BASE) / 0x1000U);

		assert(EguUsable(inst));
		count++;
	}
	assert(count == expected);
	assert(!PeiBindSwEvent(&pei, &sw1));
	for (i = 0; i < 6; i++)
	{
		int ch;

		for (ch = 0; ch < 16; ch++)
		{
			if (EguEvent(i, ch) != PeiGetEventEndpoint(&pei, sw3))
			{
				PeiSwEventFreeMcu(EguEvent(i, ch));
			}
		}
	}
	assert(!PeiSwEventAllocMcu(1, &ep));
	assert(!PeiSwEventAllocMcu(0, NULL));

	// Deinit frees everything
	assert(PeiDeinit(&pei));
	assert(PeiSwEventAllocMcu(0, &ep));
	assert(ep == first);
	PeiSwEventFreeMcu(ep);
	assert(g_critical_depth == 0);
}

int main(void)
{
	ResetMock();
	TestValidation();
	TestLifecycle();
	ResetMock();
	TestOccupied();
	TestMultipleAndExhaustion();
	TestSVCFailures();
	TestNoSoftDevice();
	TestCMatrix();
	TestRtcMatrixRearm();
	TestSwEvent();
	printf("PEI nRF52 %d channels, %d EGU: PASS\n", TEST_PPI_CHANNEL_COUNT, TEST_EGU_COUNT);
	return 0;
}
