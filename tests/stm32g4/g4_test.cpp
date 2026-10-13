/**-------------------------------------------------------------------------
@file	g4_test.cpp

@brief	Host test of the STM32G4 system clock and LPTIM1 timer sources.

Runs ARM/ST/STM32G4xx/src/system_stm32g4xx.c and
ARM/ST/STM32G4xx/src/timer_lptim_stm32g4xx.cpp with the ST STM32G474 device
header. The peripheral address range is mapped as plain memory: the test
sets the ready flags and the LPTIM status and counter itself, and calls the
LPTIM1 interrupt handler. Register values only, no timing.

@author	Hoang Nguyen Hoan
@date	October 13, 2026

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
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>

#include "stm32g4xx.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"

TestScb_t g_TestScb;

extern "C" {
// Const has internal linkage in C++ without extern
extern void (* const __Vectors[4])(void);
void (* const __Vectors[4])(void) = {};
void LPTIM1_IRQHandler(void);
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq);
}

static int s_Fail;
static int s_Checks;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

static bool s_NvicEn[128];
static uint32_t s_NvicPrio[128];

extern "C" void TestNvic(int Op, int IrqNo, uint32_t Prio)
{
	if (IrqNo < 0 || IrqNo >= 128)
	{
		return;
	}
	switch (Op)
	{
		case 0: s_NvicEn[IrqNo] = true; break;
		case 1: s_NvicEn[IrqNo] = false; break;
		case 3: s_NvicPrio[IrqNo] = Prio; break;
		default: break;
	}
}

// ---- System clock -----------------------------------------------------------

static void ClockReset(void)
{
	memset((void *)RCC, 0, sizeof(RCC_TypeDef));
	memset((void *)PWR, 0, sizeof(PWR_TypeDef));
	memset((void *)FLASH, 0, sizeof(FLASH_TypeDef));
	RCC->CR = RCC_CR_HSION | RCC_CR_HSIRDY;
	PWR->CR5 = PWR_CR5_R1MODE;
}

// PLL fields of PLLCFGR: source, M, N, R
static uint32_t Pll(uint32_t Src, uint32_t M, uint32_t N, uint32_t R)
{
	return Src | ((M - 1) << RCC_PLLCFGR_PLLM_Pos) | (N << RCC_PLLCFGR_PLLN_Pos) |
		   (((R >> 1) - 1) << RCC_PLLCFGR_PLLR_Pos) | RCC_PLLCFGR_PLLREN;
}

static void TestClock(void)
{
	// HSI16: 16 / 4 x 85 / 2 = 170 MHz, boost mode, 4 wait states
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY;
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	g_McuOsc.CoreOsc.Freq = 16000000;
	SystemInit();
	CHECK(RCC->PLLCFGR == Pll(RCC_PLLCFGR_PLLSRC_HSI, 4, 85, 2));
	CHECK((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_PLL);
	CHECK((RCC->CFGR & (RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2)) == 0);
	CHECK((RCC->CR & RCC_CR_PLLON) != 0);
	CHECK((PWR->CR5 & PWR_CR5_R1MODE) == 0);
	CHECK((FLASH->ACR & FLASH_ACR_LATENCY) == 4);
	CHECK((FLASH->ACR & (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN)) == (FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN));
	CHECK(SystemCoreClock == 170000000);
	CHECK(SystemPeriphClockGet(0) == 170000000 && SystemPeriphClockGet(1) == 170000000);
	CHECK(SystemPeriphClockGet(2) == 0);
	CHECK(g_TestScb.VTOR == (uint32_t)(uintptr_t)__Vectors);

	// HSE crystals of the boards: 24 MHz on the NUCLEO-G474RE, 8 and 12 MHz
	static const struct { uint32_t Freq, M; } hse[] = { { 24000000, 6 }, { 8000000, 2 }, { 12000000, 3 } };
	for (auto &h : hse)
	{
		ClockReset();
		RCC->CR |= RCC_CR_PLLRDY | RCC_CR_HSERDY;
		RCC->CFGR = RCC_CFGR_SWS_PLL;
		CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, h.Freq));
		CHECK(RCC->PLLCFGR == Pll(RCC_PLLCFGR_PLLSRC_HSE, h.M, 85, 2));
		CHECK((RCC->CR & (RCC_CR_HSEON | RCC_CR_HSEBYP)) == RCC_CR_HSEON);
		CHECK(SystemCoreClock == 170000000);
	}

	// External oscillator: HSE bypass
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY | RCC_CR_HSERDY;
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	CHECK(SystemCoreClockSelect(OSC_TYPE_TCXO, 24000000));
	CHECK((RCC->CR & (RCC_CR_HSEON | RCC_CR_HSEBYP)) == (RCC_CR_HSEON | RCC_CR_HSEBYP));

	// No crystal: HSE never ready, back to HSI16 and recorded in g_McuOsc
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY;
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, 24000000));
	CHECK(RCC->PLLCFGR == Pll(RCC_PLLCFGR_PLLSRC_HSI, 4, 85, 2));
	CHECK((RCC->CR & RCC_CR_HSEON) == 0);
	CHECK(g_McuOsc.CoreOsc.Type == OSC_TYPE_RC && g_McuOsc.CoreOsc.Freq == 16000000);
	CHECK(SystemCoreClock == 170000000);

	// Out of the HSE range
	CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, 50000000) == false);
	CHECK(SystemCoreClockSelect(OSC_TYPE_XTAL, 3000000) == false);

	// PLL not ready: stays on HSI16
	ClockReset();
	RCC->CFGR = RCC_CFGR_SWS_HSI;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	SystemInit();
	CHECK((RCC->CR & RCC_CR_PLLON) == 0);
	CHECK((RCC->CFGR & RCC_CFGR_SW) == RCC_CFGR_SW_HSI);
	CHECK(SystemCoreClock == 16000000);

	// SystemCoreClockUpdate from the registers, HSE as SYSCLK and AHB divider
	g_McuOsc.CoreOsc.Freq = 24000000;
	RCC->CFGR = RCC_CFGR_SWS_HSE | RCC_CFGR_HPRE_DIV4;
	SystemCoreClockUpdate();
	CHECK(SystemCoreClock == 6000000);
	RCC->CFGR = RCC_CFGR_SWS_PLL;
	RCC->PLLCFGR = Pll(RCC_PLLCFGR_PLLSRC_HSE, 6, 75, 2);
	SystemCoreClockUpdate();
	CHECK(SystemCoreClock == 150000000);

	// Flash wait states below 150 MHz: range 1 normal mode
	ClockReset();
	RCC->CR |= RCC_CR_PLLRDY;
	g_McuOsc.CoreOsc.Type = OSC_TYPE_RC;
	g_McuOsc.CoreOsc.Freq = 16000000;
	SystemInit();
	CHECK((PWR->CR5 & PWR_CR5_R1MODE) == 0);

	// APB prescaler, closest of HCLK / 1, 2, 4, 8, 16
	SystemCoreClock = 160000000;
	static const struct { uint32_t Req, Got, Ppre; } apb[] = {
		{ 160000000, 160000000, 0 }, { 80000000, 80000000, 4 }, { 60000000, 80000000, 4 },
		{ 40000000, 40000000, 5 }, { 20000000, 20000000, 6 }, { 1000, 10000000, 7 }, { 0, 10000000, 7 },
		{ 200000000, 160000000, 0 } };
	for (auto &a : apb)
	{
		CHECK(SystemPeriphClockSet(1, a.Req) == a.Got);
		CHECK(((RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos) == a.Ppre);
		CHECK(SystemPeriphClockGet(1) == a.Got);
	}
	CHECK(SystemPeriphClockSet(0, 40000000) == 40000000);
	CHECK(((RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos) == 5);
	CHECK(SystemPeriphClockSet(2, 40000000) == 0);
}

// ---- LPTIM1 -----------------------------------------------------------------

static uint32_t s_Evt;
static int s_EvtCnt;
static int s_TrigCnt;

static void EvtHandler(TimerDev_t * const pTimer, uint32_t Evt)
{
	s_Evt |= Evt;
	s_EvtCnt++;
}

static void TrigHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	s_TrigCnt++;
}

static void LptReset(void)
{
	memset((void *)LPTIM1, 0, sizeof(LPTIM_TypeDef));
	memset((void *)RCC, 0, sizeof(RCC_TypeDef));
	memset((void *)PWR, 0, sizeof(PWR_TypeDef));
	// ARR and CMP writes taken at once
	LPTIM1->ISR = LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK;
	memset(s_NvicEn, 0, sizeof(s_NvicEn));
}

// LPTIM status with the write done flags, as the hardware keeps them set
static void LptIrq(uint32_t Flags, uint32_t Cnt)
{
	LPTIM1->CNT = Cnt;
	LPTIM1->ISR = Flags | LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK;
	LPTIM1_IRQHandler();
	LPTIM1->ISR = LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK;
}

static uint32_t Sel(void)
{
	return (RCC->CCIPR & RCC_CCIPR_LPTIM1SEL) >> RCC_CCIPR_LPTIM1SEL_Pos;
}

static void TestLptim(void)
{
	TimerDev_t t;
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler };

	// Only LPTIM1
	memset(&t, 0, sizeof(t));
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	cfg.DevNo = 1;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.DevNo = -1;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(TimerInit(nullptr, &cfg) == false);
	CHECK(TimerInit(&t, nullptr) == false);
	CHECK(TimerGetLowFreqDevCount() == 1 && TimerGetHighFreqDevCount() == 0 && TimerGetHighFreqDevNo() == 1);
	cfg.DevNo = 0;

	// LSI 32 kHz
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel() == 1);
	CHECK((RCC->CSR & RCC_CSR_LSION) != 0);
	CHECK((RCC->APB1ENR1 & RCC_APB1ENR1_LPTIM1EN) != 0);
	CHECK(t.Freq == 32000);
	CHECK(t.nsPeriod == 31250);
	CHECK((LPTIM1->CFGR & LPTIM_CFGR_PRESC) == 0);
	CHECK(LPTIM1->IER == (LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE));
	CHECK(LPTIM1->CR == (LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT));
	CHECK(LPTIM1->ARR == 0xFFFF);
	CHECK(s_NvicEn[LPTIM1_IRQn] && s_NvicPrio[LPTIM1_IRQn] == 5);

	// LSI never ready
	LptReset();
	CHECK(TimerInit(&t, &cfg) == false);

	// HSI16 and the closest prescaler
	static const struct { uint32_t Req, Got, Presc; } hf[] = {
		{ 0, 16000000, 0 }, { 16000000, 16000000, 0 }, { 20000000, 16000000, 0 }, { 1000000, 1000000, 4 },
		{ 3000000, 4000000, 2 }, { 100, 125000, 7 } };
	for (auto &h : hf)
	{
		LptReset();
		RCC->CR = RCC_CR_HSIRDY;
		cfg.ClkSrc = TIMER_CLKSRC_HFRC;
		cfg.Freq = h.Req;
		CHECK(TimerInit(&t, &cfg));
		CHECK(Sel() == 2);
		CHECK(t.Freq == h.Got);
		CHECK(((LPTIM1->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos) == h.Presc);
	}

	// Default clock from g_McuOsc: LSE crystal, then LSI
	LptReset();
	RCC->BDCR = RCC_BDCR_LSERDY;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel() == 3 && t.Freq == 32768);
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel() == 1 && t.Freq == 32000);

	// LSE crystal started with backup domain access, refused when it does
	// not start
	LptReset();
	cfg.ClkSrc = TIMER_CLKSRC_LFXTAL;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((PWR->CR1 & PWR_CR1_DBP) != 0);
	CHECK((RCC->BDCR & RCC_BDCR_LSEON) == 0);

	// Tick count and overflow
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	cfg.ClkSrc = TIMER_CLKSRC_LFRC;
	cfg.Freq = 32000;
	CHECK(TimerInit(&t, &cfg));
	LPTIM1->CNT = 100;
	CHECK(t.GetTickCount(&t) == 100);
	s_Evt = 0;
	s_EvtCnt = 0;
	LptIrq(LPTIM_ISR_ARRM, 5);
	CHECK(t.Rollover == 0x10000 && (s_Evt & TIMER_EVT_COUNTER_OVR) && s_EvtCnt == 1);
	CHECK(t.GetTickCount(&t) == 0x10005);
	// Wrapped, the interrupt not taken yet
	LPTIM1->ISR |= LPTIM_ISR_ARRM;
	LPTIM1->CNT = 3;
	CHECK(t.GetTickCount(&t) == 0x20003);
	LPTIM1->CNT = 0xFFF0;
	CHECK(t.GetTickCount(&t) == 0x1FFF0);
	LPTIM1->ISR &= ~LPTIM_ISR_ARRM;

	// Compare trigger, 10 ms at 32 kHz is 320 counts
	CHECK(t.GetMaxTrigger(&t) == 1);
	CHECK(t.FindAvailTrigger(&t) == 0);
	LPTIM1->CNT = 1000;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, nullptr) == 10000000ULL);
	CHECK(LPTIM1->CMP == 1320);
	CHECK(t.FindAvailTrigger(&t) == -1);
	s_Evt = 0;
	s_TrigCnt = 0;
	LptIrq(LPTIM_ISR_CMPM, 1321);
	CHECK(s_TrigCnt == 1 && (s_Evt & TIMER_EVT_TRIGGER(0)));
	CHECK(LPTIM1->CMP == 1641);
	// Compare wraps at 16 bits
	LptIrq(LPTIM_ISR_CMPM, 0xFF00);
	CHECK(LPTIM1->CMP == ((0xFF00 + 320) & 0xFFFF));
	CHECK(s_TrigCnt == 2);

	// Disabled trigger: no callback, no trigger event
	t.DisableTrigger(&t, 0);
	s_Evt = 0;
	LptIrq(LPTIM_ISR_CMPM, 2000);
	CHECK(s_TrigCnt == 2 && (s_Evt & TIMER_EVT_TRIGGER(0)) == 0);

	// Single shot
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) != 0);
	LptIrq(LPTIM_ISR_CMPM, 2400);
	LptIrq(LPTIM_ISR_CMPM, 2720);
	CHECK(s_TrigCnt == 3);

	// Out of the 16 bits compare range, wrong trigger number
	CHECK(t.EnableTrigger(&t, 0, 31250ULL * 2, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, 0, 31250ULL * 0x10000, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, 1, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableExtTrigger(&t, 0, TIMER_EXTTRIG_SENSE_HIGH_TRANSITION) == false);

	// New frequency restarts the counter
	CHECK(t.SetFrequency(&t, 8000) == 8000);
	CHECK(((LPTIM1->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos) == 2);
	CHECK(t.Rollover == 0 && t.nsPeriod == 125000);
	CHECK(LPTIM1->CR == (LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT));

	// Disable and enable
	t.Disable(&t);
	CHECK(LPTIM1->CR == 0 && (RCC->APB1ENR1 & RCC_APB1ENR1_LPTIM1EN) == 0 && !s_NvicEn[LPTIM1_IRQn]);
	CHECK(t.Enable(&t));
	CHECK(LPTIM1->CR == (LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT) && (RCC->APB1ENR1 & RCC_APB1ENR1_LPTIM1EN));
	CHECK(LPTIM1->IER == (LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE) && s_NvicEn[LPTIM1_IRQn]);
}

int main()
{
	void *p = mmap((void *)0x40000000, 0x30000, PROT_READ | PROT_WRITE,
				   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

	if (p != (void *)0x40000000)
	{
		printf("mmap failed\n");
		return 1;
	}

	TestClock();
	TestLptim();

	printf("%d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}
