/**-------------------------------------------------------------------------
@file	wba_timer_test.cpp

@brief	Host test of the STM32WBA LPTIM1 and LPTIM2 timer source.

Runs ARM/ST/STM32WBAxx/src/timer_lptim_stm32wbaxx.cpp with the ST
STM32WBA65 device header. The peripheral address ranges are mapped as plain
memory: the test sets the oscillator ready flags, the LPTIM write done flags,
status and counter itself, and calls the LPTIM interrupt handlers. Register
values only, no timing. The order of the LPTIM register writes is checked by
the instruction set emulator run of the TimerDemo image, see the README.

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

#include "stm32wbaxx.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"

uint32_t SystemCoreClock = 16000000;
McuOsc_t g_McuOsc = { { OSC_TYPE_RC, 16000000, 20 }, { OSC_TYPE_RC, 32000, 20 }, false };

extern "C" {
void LPTIM1_IRQHandler(void);
void LPTIM2_IRQHandler(void);
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

// Write done flags, kept set: plain memory does not clear them on ICR
#define OK_ALL		(LPTIM_ISR_DIEROK | LPTIM_ISR_ARROK | LPTIM_ISR_CMP1OK | LPTIM_ISR_CMP2OK)
#define DIER_ALL	(LPTIM_DIER_ARRMIE | LPTIM_DIER_CC1IE | LPTIM_DIER_CC2IE)
#define CR_RUN		(LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT)

static uint32_t s_Evt;
static int s_EvtCnt;
static int s_TrigCnt[2];
static int s_LastTrig;
static void *s_LastCtx;

static void EvtHandler(TimerDev_t * const pTimer, uint32_t Evt)
{
	s_Evt |= Evt;
	s_EvtCnt++;
}

static void TrigHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	if (TrigNo >= 0 && TrigNo < 2)
	{
		s_TrigCnt[TrigNo]++;
	}
	s_LastTrig = TrigNo;
	s_LastCtx = pContext;
}

static void LptReset(void)
{
	memset((void *)LPTIM1, 0, sizeof(LPTIM_TypeDef));
	memset((void *)LPTIM2, 0, sizeof(LPTIM_TypeDef));
	memset((void *)RCC, 0, sizeof(RCC_TypeDef));
	memset((void *)PWR, 0, sizeof(PWR_TypeDef));
	LPTIM1->ISR = OK_ALL;
	LPTIM2->ISR = OK_ALL;
	memset(s_NvicEn, 0, sizeof(s_NvicEn));
	memset(s_NvicPrio, 0, sizeof(s_NvicPrio));
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
}

// LPTIM status at an interrupt, the write done flags kept
static void LptIrq(LPTIM_TypeDef *pReg, uint32_t Flags, uint32_t Cnt)
{
	pReg->CNT = Cnt;
	pReg->ISR = Flags | OK_ALL;
	s_Evt = 0;
	if (pReg == LPTIM1)
	{
		LPTIM1_IRQHandler();
	}
	else
	{
		LPTIM2_IRQHandler();
	}
	pReg->ISR = OK_ALL;
}

static uint32_t Sel1(void)
{
	return (RCC->CCIPR3 & RCC_CCIPR3_LPTIM1SEL) >> RCC_CCIPR3_LPTIM1SEL_Pos;
}

static uint32_t Sel2(void)
{
	return (RCC->CCIPR1 & RCC_CCIPR1_LPTIM2SEL) >> RCC_CCIPR1_LPTIM2SEL_Pos;
}

static uint32_t Presc(LPTIM_TypeDef *pReg)
{
	return (pReg->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos;
}

static void TestInit(void)
{
	TimerDev_t t;
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler };

	memset(&t, 0, sizeof(t));
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY;

	// LPTIM1 and LPTIM2 only
	cfg.DevNo = 2;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.DevNo = -1;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(TimerInit(nullptr, &cfg) == false);
	CHECK(TimerInit(&t, nullptr) == false);
	CHECK(TimerGetLowFreqDevCount() == 2 && TimerGetHighFreqDevCount() == 0 && TimerGetHighFreqDevNo() == 2);
	CHECK(RCC->APB7ENR == 0 && RCC->APB1ENR2 == 0);
	cfg.DevNo = 0;

	// LPTIM1 from the LSI, 32 kHz
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel1() == 1);
	CHECK((RCC->APB7ENR & RCC_APB7ENR_LPTIM1EN) != 0 && (RCC->APB7RSTR & RCC_APB7RSTR_LPTIM1RST) == 0);
	CHECK(RCC->APB1ENR2 == 0 && RCC->CCIPR1 == 0);
	CHECK(t.Freq == 32000 && t.nsPeriod == 31250);
	CHECK(Presc(LPTIM1) == 0);
	CHECK(LPTIM1->DIER == DIER_ALL);
	CHECK(LPTIM1->CR == CR_RUN);
	CHECK(LPTIM1->ARR == 0xFFFF && LPTIM1->CCR1 == 0 && LPTIM1->CCR2 == 0);
	CHECK(s_NvicEn[LPTIM1_IRQn] && s_NvicPrio[LPTIM1_IRQn] == 5 && !s_NvicEn[LPTIM2_IRQn]);
	CHECK(t.Rollover == 0 && t.LastCount == 0);

	// LSI started with backup domain access, refused when it does not start
	LptReset();
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((RCC->AHB4ENR & RCC_AHB4ENR_PWREN) && (PWR->DBPR & PWR_DBPR_DBP));
	CHECK((RCC->BDCR1 & RCC_BDCR1_LSI1ON) != 0);
	CHECK(RCC->APB7ENR == 0);

	// LSI divided by 128
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY | RCC_BDCR1_LSI1PREDIV;
	cfg.Freq = 0;
	CHECK(TimerInit(&t, &cfg));
	CHECK(t.Freq == 250 && t.nsPeriod == 4000000);

	// LPTIM2 from HSI16 and the closest prescaler
	static const struct { uint32_t Req, Got, Presc; } hf[] = {
		{ 0, 16000000, 0 }, { 16000000, 16000000, 0 }, { 20000000, 16000000, 0 }, { 1000000, 1000000, 4 },
		{ 3000000, 4000000, 2 }, { 100, 125000, 7 } };
	for (auto &h : hf)
	{
		LptReset();
		RCC->CR = RCC_CR_HSIRDY;
		cfg.DevNo = 1;
		cfg.ClkSrc = TIMER_CLKSRC_HFRC;
		cfg.Freq = h.Req;
		CHECK(TimerInit(&t, &cfg));
		CHECK(Sel2() == 2 && Sel1() == 0);
		CHECK((RCC->APB1ENR2 & RCC_APB1ENR2_LPTIM2EN) != 0 && RCC->APB7ENR == 0);
		CHECK((RCC->CR & RCC_CR_HSION) != 0);
		CHECK(t.Freq == h.Got && Presc(LPTIM2) == h.Presc);
		CHECK(LPTIM2->CR == CR_RUN && LPTIM2->DIER == DIER_ALL && LPTIM1->CR == 0);
		CHECK(s_NvicEn[LPTIM2_IRQn] && !s_NvicEn[LPTIM1_IRQn]);
	}
	// HSI16 not ready
	LptReset();
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.DevNo = 0;

	// Default clock from g_McuOsc: LSE running, then LSI
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSEON | RCC_BDCR1_LSERDY | RCC_BDCR1_LSESYSEN | RCC_BDCR1_LSESYSRDY | RCC_BDCR1_LSI1RDY;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel1() == 3 && t.Freq == 32768);
	CHECK(PWR->DBPR == 0);
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel1() == 1 && t.Freq == 32000);

	// LSE crystal started with LSESYS, switched off again when it does not
	// start
	LptReset();
	cfg.ClkSrc = TIMER_CLKSRC_LFXTAL;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
	RCC->BDCR1 = RCC_BDCR1_LSEBYP;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((PWR->DBPR & PWR_DBPR_DBP) != 0);
	CHECK((RCC->BDCR1 & (RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN | RCC_BDCR1_LSEBYP)) == 0);
	CHECK(RCC->APB7ENR == 0);
	// External oscillator: bypass
	LptReset();
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_TCXO;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((RCC->BDCR1 & RCC_BDCR1_LSEBYP) != 0);
	// LSE on for the RTC only: LSESYS added, the LSE left on
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSEON | RCC_BDCR1_LSERDY;
	cfg.ClkSrc = TIMER_CLKSRC_LFXTAL;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((RCC->BDCR1 & (RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN)) == (RCC_BDCR1_LSEON | RCC_BDCR1_LSESYSEN));

	// An LPTIM write not taken: refused, clock off, timer disabled
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY;
	cfg.ClkSrc = TIMER_CLKSRC_LFRC;
	LPTIM1->ISR = OK_ALL & ~LPTIM_ISR_DIEROK;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(LPTIM1->CR == 0 && RCC->APB7ENR == 0 && !s_NvicEn[LPTIM1_IRQn]);
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY;
	LPTIM1->ISR = OK_ALL & ~LPTIM_ISR_CMP2OK;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(LPTIM1->CR == 0);
}

static void TestCount(void)
{
	TimerDev_t t;
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler };

	memset(&t, 0, sizeof(t));
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY;
	CHECK(TimerInit(&t, &cfg));

	// A read lower than the previous one is a wrap
	LPTIM1->CNT = 100;
	CHECK(t.GetTickCount(&t) == 100);
	LPTIM1->CNT = 50;
	CHECK(t.GetTickCount(&t) == 0x10032);
	CHECK(t.GetTickCount(&t) == 0x10032);

	// ARRM at 0xFFFF, the counter not wrapped yet: no wrap counted
	s_EvtCnt = 0;
	LptIrq(LPTIM1, LPTIM_ISR_ARRM, 0xFFFF);
	CHECK(s_EvtCnt == 1 && s_Evt == TIMER_EVT_COUNTER_OVR);
	CHECK(t.GetTickCount(&t) == 0x1FFFF);
	LPTIM1->CNT = 3;
	CHECK(t.GetTickCount(&t) == 0x20003);
	// ARRM handled after the wrap
	LPTIM1->CNT = 0xFFF0;
	CHECK(t.GetTickCount(&t) == 0x2FFF0);
	LptIrq(LPTIM1, LPTIM_ISR_ARRM, 2);
	CHECK(t.GetTickCount(&t) == 0x30002);
	CHECK(t.LastCount == 2);
	// Millisecond count: 0x30002 ticks at 32 kHz
	CHECK(TimerGetMilisecond(&t) == (uint32_t)(0x30002ULL * 1000 / 32000));
	// No flag: no event
	s_EvtCnt = 0;
	LptIrq(LPTIM1, 0, 10);
	CHECK(s_EvtCnt == 0);
}

static void TestTrigger(void)
{
	TimerDev_t t;
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler };
	int ctx;

	memset(&t, 0, sizeof(t));
	LptReset();
	RCC->BDCR1 = RCC_BDCR1_LSI1RDY;
	CHECK(TimerInit(&t, &cfg));

	// Two compare triggers, 10 ms is 320 counts, 25 ms 800
	CHECK(t.GetMaxTrigger(&t) == 2);
	CHECK(t.FindAvailTrigger(&t) == 0);
	LPTIM1->CNT = 1000;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &ctx) == 10000000ULL);
	CHECK(LPTIM1->CCR1 == 1320 && LPTIM1->ICR == LPTIM_ICR_CC1CF);
	CHECK(t.FindAvailTrigger(&t) == 1);
	CHECK(t.EnableTrigger(&t, 1, 25000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, nullptr) == 25000000ULL);
	CHECK(LPTIM1->CCR2 == 1800 && LPTIM1->ICR == LPTIM_ICR_CC2CF);
	CHECK(t.FindAvailTrigger(&t) == -1);

	// Trigger 0 on time: next compare one period after the previous one
	s_TrigCnt[0] = s_TrigCnt[1] = 0;
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF, 1321);
	CHECK(s_TrigCnt[0] == 1 && s_LastTrig == 0 && s_LastCtx == &ctx);
	CHECK(s_Evt == TIMER_EVT_TRIGGER(0));
	CHECK(LPTIM1->CCR1 == 1640 && LPTIM1->CCR2 == 1800);
	// 100 counts late: still from the previous compare value
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF, 1740);
	CHECK(LPTIM1->CCR1 == 1960);
	// Too late for the next one: one period from now
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF, 1960 + 319);
	CHECK(LPTIM1->CCR1 == 1960 + 319 + 320);
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF, 2599 + 400);
	CHECK(LPTIM1->CCR1 == 2999 + 320);
	CHECK(s_TrigCnt[0] == 4);

	// Trigger 1, then both in one interrupt
	LptIrq(LPTIM1, LPTIM_ISR_CC2IF, 1801);
	CHECK(s_TrigCnt[1] == 1 && s_LastTrig == 1 && s_Evt == TIMER_EVT_TRIGGER(1));
	CHECK(LPTIM1->CCR2 == 2600);
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF | LPTIM_ISR_CC2IF | LPTIM_ISR_ARRM, 2600);
	CHECK(s_TrigCnt[0] == 5 && s_TrigCnt[1] == 2);
	CHECK(s_Evt == (TIMER_EVT_TRIGGER(0) | TIMER_EVT_TRIGGER(1) | TIMER_EVT_COUNTER_OVR));
	CHECK(LPTIM1->CCR2 == 3400);

	// The compare value wraps at 16 bits
	LPTIM1->CNT = 0xFF00;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, nullptr) != 0);
	CHECK(LPTIM1->CCR1 == ((0xFF00 + 320) & 0xFFFF));
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF, 0x40);
	CHECK(LPTIM1->CCR1 == 0x40 + 320);

	// Disabled trigger: no callback, no trigger event, available again
	t.DisableTrigger(&t, 0);
	CHECK(t.FindAvailTrigger(&t) == 0);
	int n0 = s_TrigCnt[0];
	LptIrq(LPTIM1, LPTIM_ISR_CC1IF, 0x40 + 320);
	CHECK(s_TrigCnt[0] == n0 && (s_Evt & TIMER_EVT_TRIGGER(0)) == 0);

	// Single shot
	CHECK(t.EnableTrigger(&t, 1, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) != 0);
	int n1 = s_TrigCnt[1];
	uint32_t ccr2 = LPTIM1->CCR2;
	LptIrq(LPTIM1, LPTIM_ISR_CC2IF, ccr2);
	CHECK(s_TrigCnt[1] == n1 + 1 && LPTIM1->CCR2 == ccr2);
	LptIrq(LPTIM1, LPTIM_ISR_CC2IF, ccr2 + 320);
	CHECK(s_TrigCnt[1] == n1 + 1);
	CHECK(t.FindAvailTrigger(&t) == 0);

	// Out of the 16 bits compare range, wrong trigger number
	CHECK(t.EnableTrigger(&t, 0, 31250ULL * 2, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, 0, 31250ULL * 0x10000, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, 2, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, -1, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableExtTrigger(&t, 0, TIMER_EXTTRIG_SENSE_HIGH_TRANSITION) == false);
	// Compare write not taken
	LPTIM1->ISR = OK_ALL & ~LPTIM_ISR_CMP1OK;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.FindAvailTrigger(&t) == 0);
	LPTIM1->ISR = OK_ALL;

	// Trigger kept through Disable and Enable, counted from 0 again
	CHECK(t.EnableTrigger(&t, 1, 25000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, nullptr) != 0);
	t.Disable(&t);
	CHECK(LPTIM1->CR == 0 && (RCC->APB7ENR & RCC_APB7ENR_LPTIM1EN) == 0 && !s_NvicEn[LPTIM1_IRQn]);
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	LPTIM1->DIER = 0;
	CHECK(t.Enable(&t));
	CHECK(LPTIM1->CR == CR_RUN && (RCC->APB7ENR & RCC_APB7ENR_LPTIM1EN) && s_NvicEn[LPTIM1_IRQn]);
	CHECK(LPTIM1->DIER == DIER_ALL && LPTIM1->CCR1 == 0 && LPTIM1->CCR2 == 800);
	CHECK(t.Rollover == 0 && t.LastCount == 0);

	// Reset: counter from 0
	LPTIM1->CNT = 5000;
	CHECK(t.GetTickCount(&t) == 5000);
	t.Reset(&t);
	CHECK(t.Rollover == 0 && t.LastCount == 0 && LPTIM1->CR == CR_RUN && s_NvicEn[LPTIM1_IRQn]);

	// New frequency: counter restarted, triggers off
	CHECK(t.SetFrequency(&t, 8000) == 8000);
	CHECK(Presc(LPTIM1) == 2 && t.nsPeriod == 125000);
	CHECK(t.Rollover == 0 && LPTIM1->CR == CR_RUN && LPTIM1->CCR2 == 0);
	CHECK(t.FindAvailTrigger(&t) == 0);
	// Restart write not taken
	LPTIM1->ISR = OK_ALL & ~LPTIM_ISR_ARROK;
	CHECK(t.SetFrequency(&t, 16000) == 0);
	CHECK(LPTIM1->CR == 0);
	LPTIM1->ISR = OK_ALL;
	CHECK(t.Enable(&t));
	CHECK(LPTIM1->CR == CR_RUN && t.Freq == 16000);

	// LPTIM2 interrupt goes to the LPTIM2 timer
	TimerDev_t t2;
	memset(&t2, 0, sizeof(t2));
	cfg.DevNo = 1;
	CHECK(TimerInit(&t2, &cfg));
	LPTIM2->CNT = 10;
	CHECK(t2.EnableTrigger(&t2, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, nullptr) != 0);
	CHECK(LPTIM2->CCR1 == 330);
	n0 = s_TrigCnt[0];
	LptIrq(LPTIM2, LPTIM_ISR_CC1IF, 330);
	CHECK(s_TrigCnt[0] == n0 + 1 && LPTIM2->CCR1 == 650);
	CHECK(LPTIM1->CR == CR_RUN);
}

int main()
{
	static const struct { uintptr_t Base, Size; } map[] = { { 0x40000000, 0x40000 }, { 0x46000000, 0x30000 } };

	for (auto &m : map)
	{
		void *p = mmap((void *)m.Base, m.Size, PROT_READ | PROT_WRITE,
					   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

		if (p != (void *)m.Base)
		{
			printf("mmap failed\n");
			return 1;
		}
	}

	TestInit();
	TestCount();
	TestTrigger();

	printf("%d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}
