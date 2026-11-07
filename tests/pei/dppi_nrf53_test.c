#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "nrf.h"
#include "coredev/pei.h"

#if defined(NRF5340_XXAA_APPLICATION) && !defined(NRF_TRUSTZONE_NONSECURE)
#define TEST_ADDR 0x50000000UL
#else
#define TEST_ADDR 0x40000000UL
#endif

NRF_DPPIC_Type g_TestDppic;
int g_TestCriticalDepth;

static struct {
	PeiEndpoint_t Addr;
	uint32_t Value;
} s_Reg[128];
static int s_RegCount;

volatile uint32_t *PeiNrf53TestRegister(PeiEndpoint_t Addr)
{
	int i;

	assert(g_TestCriticalDepth > 0);
	for (i = 0; i < s_RegCount; i++)
	{
		if (s_Reg[i].Addr == Addr)
		{
			return &s_Reg[i].Value;
		}
	}
	assert(s_RegCount < 128);
	s_Reg[s_RegCount].Addr = Addr;
	return &s_Reg[s_RegCount++].Value;
}

static uint32_t ReadReg(PeiEndpoint_t Addr)
{
	uint32_t value;

	g_TestCriticalDepth++;
	value = *PeiNrf53TestRegister(Addr);
	g_TestCriticalDepth--;
	return value;
}

static void WriteReg(PeiEndpoint_t Addr, uint32_t Value)
{
	g_TestCriticalDepth++;
	*PeiNrf53TestRegister(Addr) = Value;
	g_TestCriticalDepth--;
}

static PeiCfg_t Config(int EventNo, int TaskNo)
{
	PeiCfg_t cfg = {0};

	cfg.Event = TEST_ADDR + 0x1100UL + ((uint32_t)EventNo * 0x1000UL);
	cfg.Task = TEST_ADDR + 0x2000UL + ((uint32_t)TaskNo * 0x1000UL);
	return cfg;
}

static void TestFanout(void)
{
	PeiConn_t a = {0};
	PeiConn_t b = {0};
	PeiConn_t copy;
	PeiCfg_t cfg = Config(0, 0);
	uint32_t channel;
	uint32_t mask;
	uint32_t first;

	g_TestDppic.CHG[0] = 1UL << 2;
	g_TestDppic.CHEN = 1UL << 3;
	assert(PeiConnect(&a, &cfg));
	first = a.Handle;
	channel = ReadReg(cfg.Event + 0x80U) & 0xFFU;
	assert(channel == 4U);
	mask = 1UL << channel;
	assert(ReadReg(cfg.Task + 0x80U) == 0U);
	assert(!PeiConnect(&a, &cfg));

	copy = a;
	assert(!PeiEnable(&copy));
	PeiDisconnect(&copy);
	assert(a.Handle == first);
	assert(PeiEnable(&a));
	assert(PeiEnable(&a));
	assert(ReadReg(cfg.Task + 0x80U) == (0x80000000UL | channel));
	assert(g_TestDppic.CHENSET == mask);

	cfg.Task += 0x1000U;
	assert(PeiConnect(&b, &cfg));
	assert((ReadReg(cfg.Event + 0x80U) & 0xFFU) == channel);
	assert(ReadReg(cfg.Task + 0x80U) == 0U);
	assert(PeiEnable(&b));
	g_TestDppic.CHENCLR = 0U;
	assert(PeiDisable(&a));
	assert(PeiDisable(&a));
	assert(g_TestDppic.CHENCLR == 0U);
	assert(ReadReg(cfg.Task + 0x80U) == (0x80000000UL | channel));
	PeiDisconnect(&a);
	assert(ReadReg(cfg.Event + 0x80U) != 0U);
	assert(PeiDisable(&b));
	assert(g_TestDppic.CHENCLR == mask);
	PeiDisconnect(&b);
	assert(ReadReg(cfg.Event + 0x80U) == 0U);
	assert(PeiConnect(&a, &cfg));
	assert(a.Handle != first);
	PeiDisconnect(&a);
	assert(g_TestCriticalDepth == 0);
}

static void TestValidation(void)
{
	PeiConn_t a = {0};
	PeiConn_t b = {0};
	PeiCfg_t cfg = Config(1, 1);
	PeiCfg_t other;

	assert(!PeiConnect(NULL, &cfg));
	assert(!PeiConnect(&a, NULL));
	cfg.DevNo = 1;
	assert(!PeiConnect(&a, &cfg));
	cfg.DevNo = 0;
	cfg.Options = 1;
	assert(!PeiConnect(&a, &cfg));
	cfg.Options = 0;
	cfg.Event = 0x20000100U;
	assert(!PeiConnect(&a, &cfg));
	cfg.Event = TEST_ADDR + 0x1102U;
	assert(!PeiConnect(&a, &cfg));
	cfg.Event = TEST_ADDR + 0x1100U;
	cfg.Task = TEST_ADDR + 0x2000U;
	assert(PeiConnect(&a, &cfg));
	assert(!PeiConnect(&b, &cfg));
	other = cfg;
	other.Event += 0x1000U;
	assert(!PeiConnect(&b, &other));
	other.Task += 0x1000U;
	assert(PeiConnect(&b, &other));
	PeiDisconnect(&a);
	PeiDisconnect(&b);
	assert(!PeiEnable(&a));
	assert(!PeiDisable(&a));
	PeiDisconnect(&a);
	PeiDisconnect(NULL);
	assert(g_TestCriticalDepth == 0);
}

static void TestExternalOwnership(void)
{
	PeiConn_t a = {0};
	PeiCfg_t cfg = Config(4, 4);

	WriteReg(cfg.Event + 0x80U, 0x80000006U);
	assert(!PeiConnect(&a, &cfg));
	WriteReg(cfg.Event + 0x80U, 0U);
	WriteReg(cfg.Task + 0x80U, 0x80000007U);
	assert(!PeiConnect(&a, &cfg));
	WriteReg(cfg.Task + 0x80U, 0U);
	assert(PeiConnect(&a, &cfg));
	WriteReg(cfg.Task + 0x80U, 0x8000001FU);
	assert(!PeiEnable(&a));
	PeiDisconnect(&a);
	assert(a.Handle != 0U);
	WriteReg(cfg.Task + 0x80U, 0U);
	assert(PeiEnable(&a));
	PeiDisconnect(&a);
	assert(g_TestCriticalDepth == 0);
}

static void TestExhaustion(void)
{
	PeiConn_t c[31] = {{0}};
	PeiCfg_t cfg;
	int i;

	for (i = 0; i < 30; i++)
	{
		cfg = Config(i + 8, i + 8);
		assert(PeiConnect(&c[i], &cfg));
	}
	cfg = Config(40, 40);
	assert(!PeiConnect(&c[30], &cfg));
	for (i = 0; i < 30; i++)
	{
		PeiDisconnect(&c[i]);
	}
	assert(g_TestCriticalDepth == 0);
}

int main(void)
{
	memset(&g_TestDppic, 0, sizeof(g_TestDppic));
	TestFanout();
	g_TestDppic.CHG[0] = 0U;
	g_TestDppic.CHEN = 0U;
	TestValidation();
	TestExternalOwnership();
	TestExhaustion();
	puts("PEI nRF53 DPPI host model: PASS");
	return 0;
}
