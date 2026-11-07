#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nrf.h"
#include "coredev/pei.h"

NRF_PPI_Type g_TestPpi;
int g_critical_depth;
static int g_assign_calls;
static int g_enable_calls;
static int g_disable_calls;
static int g_fail_assign;
static int g_fail_enable;
static int g_fail_disable;

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

int main(void)
{
	ResetMock();
	TestValidation();
	TestLifecycle();
	ResetMock();
	TestOccupied();
	TestMultipleAndExhaustion();
	TestSVCFailures();
	printf("PEI nRF52 %d channels: PASS\n", TEST_PPI_CHANNEL_COUNT);
	return 0;
}
