/**-------------------------------------------------------------------------
@file	timer_lptim_test.cpp

@brief	Host test of the STM32 LPTIM timer driver on L4 and G4.

		Runs ARM/ST/src/timer_lptim_stm32.cpp with TimerInit
		(timer_stm32.cpp) and the TIM driver, the ST device header of the
		part and the host core header of tests/stm32/shim. Peripheral
		registers are plain memory mapped at their addresses: a register
		keeps what the driver writes, the test sets the status flags. The
		LPTIM write done flags ARROK and CMPOK are kept set unless a test
		removes them.

		Checks for each LPTIM of the part: RCC clock enable, reset and
		kernel clock selection, LSI, LSE crystal and bypass, HSI16 and the
		prescaler, IER written with the timer disabled, ARR and compare
		values, count extended from the previous read, compare trigger
		rescheduled from its previous value, single shot, refused triggers,
		write not confirmed, SetFrequency, Disable, Enable, Reset, the timer
		taken by another TimerDev_t, released when its TimerDev_t moves to
		another timer, an LPTIM counting without the driver.

		The WBA LPTIM is tested by tests/stm32wba. Not a timing or hardware
		test.

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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

#include "stm32.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"
#include "timer_stm32.h"

#ifdef LPTIM_DIER_ARRMIE
#error "timer_lptim_test.cpp: L4 and G4 LPTIM only, the WBA LPTIM is tested by tests/stm32wba"
#endif

uint32_t SystemCoreClock = 16000000;
McuOsc_t g_McuOsc = { { OSC_TYPE_RC, 16000000, 20 }, { OSC_TYPE_RC, 32000, 20 }, false };

extern "C" uint32_t SystemPeriphClockGet(int Idx)
{
	return SystemCoreClock;
}

// Vector replaced by an application handler in the test built with
// TEST_VECTOR_OVERRIDE: the last LPTIM
#ifdef TEST_VECTOR_OVERRIDE
#ifdef LPTIM2
#define TEST_OVERRIDE_IRQ	LPTIM2_IRQn
#else
#define TEST_OVERRIDE_IRQ	LPTIM1_IRQn
#endif
#else
#define TEST_OVERRIDE_IRQ	(-100)
#endif

// The vector file function, called through Stm32TimerIRQHandler by the test
extern "C" void Stm32TimerVector(void)
{
}

static void TestAppHandler(void)
{
}

#define TEST_VEC(i)		((i) == 16 + (int)TEST_OVERRIDE_IRQ ? TestAppHandler : Stm32TimerVector)
#define TEST_VEC4(i)	TEST_VEC(i), TEST_VEC(i + 1), TEST_VEC(i + 2), TEST_VEC(i + 3)
#define TEST_VEC16(i)	TEST_VEC4(i), TEST_VEC4(i + 4), TEST_VEC4(i + 8), TEST_VEC4(i + 12)
#define TEST_VEC64(i)	TEST_VEC16(i), TEST_VEC16(i + 16), TEST_VEC16(i + 32), TEST_VEC16(i + 48)

// Vector table, the timer vectors aliased to Stm32TimerVector as in the
// vector file of the part
void (* const __Vectors[256])(void) = { TEST_VEC64(0), TEST_VEC64(64), TEST_VEC64(128), TEST_VEC64(192) };

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
#define OK_ALL		(LPTIM_ISR_ARROK | LPTIM_ISR_CMPOK)
#define IER_ALL		(LPTIM_IER_ARRMIE | LPTIM_IER_CMPMIE)
#define CR_RUN		(LPTIM_CR_ENABLE | LPTIM_CR_CNTSTRT)

// LPTIM of DevNo 0 and 1
typedef struct {
	LPTIM_TypeDef *pReg;
	IRQn_Type IrqNo;
	volatile uint32_t *pEnReg;
	uint32_t EnMask;
	uint32_t SelPos;
} TestLpt_t;

static const TestLpt_t s_Lpt[] = {
	{ LPTIM1, LPTIM1_IRQn, &RCC->APB1ENR1, RCC_APB1ENR1_LPTIM1EN, RCC_CCIPR_LPTIM1SEL_Pos },
#ifdef LPTIM2
	{ LPTIM2, LPTIM2_IRQn, &RCC->APB1ENR2, RCC_APB1ENR2_LPTIM2EN, RCC_CCIPR_LPTIM2SEL_Pos },
#endif
};

#define TEST_NB_LPT		((int)(sizeof(s_Lpt) / sizeof(TestLpt_t)))

static uint32_t s_Evt;
static int s_EvtCnt;
static int s_TrigCnt;
static TimerDev_t *s_LastTimer;

static void EvtHandler(TimerDev_t * const pTimer, uint32_t Evt)
{
	s_Evt |= Evt;
	s_EvtCnt++;
	s_LastTimer = pTimer;
}

static void TrigHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	CHECK(TrigNo == 0 && pContext == &s_TrigCnt);
	s_TrigCnt++;
}

static void LptReset(void)
{
	for (int i = 0; i < TEST_NB_LPT; i++)
	{
		memset((void *)s_Lpt[i].pReg, 0, sizeof(LPTIM_TypeDef));
		s_Lpt[i].pReg->ISR = OK_ALL;
	}
	memset((void *)RCC, 0, sizeof(RCC_TypeDef));
	memset((void *)PWR, 0, sizeof(PWR_TypeDef));
	memset(s_NvicEn, 0, sizeof(s_NvicEn));
	memset(s_NvicPrio, 0, sizeof(s_NvicPrio));
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_RC;
}

// LPTIM status at an interrupt, the write done flags kept
static void LptIrq(const TestLpt_t &l, uint32_t Flags, uint32_t Cnt)
{
	l.pReg->CNT = Cnt;
	l.pReg->ISR = Flags | OK_ALL;
	s_Evt = 0;
	// Stm32TimerVector of the vector file
	Stm32TimerIRQHandler(l.IrqNo);
	l.pReg->ISR = OK_ALL;
}

static uint32_t Sel(const TestLpt_t &l)
{
	return (RCC->CCIPR >> l.SelPos) & 3U;
}

static uint32_t Presc(const TestLpt_t &l)
{
	return (l.pReg->CFGR & LPTIM_CFGR_PRESC) >> LPTIM_CFGR_PRESC_Pos;
}

static void TestInit(int k)
{
	static TimerDev_t t;
	const TestLpt_t &l = s_Lpt[k];
	TimerCfg_t cfg = { k, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler, false };

	CHECK(TimerGetLowFreqDevCount() == TEST_NB_LPT && TimerGetHighFreqDevNo() == TEST_NB_LPT);

	// LSI 32 kHz
	memset(&t, 0, sizeof(t));
	LptReset();
	RCC->CCIPR = 0xFFFFFFFFU & ~(3U << l.SelPos);
	RCC->CSR = RCC_CSR_LSIRDY;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel(l) == 1);
	// Other selection fields kept
	CHECK((RCC->CCIPR | (3U << l.SelPos)) == 0xFFFFFFFFU);
	CHECK((*l.pEnReg & l.EnMask) != 0);
	CHECK(t.DevNo == k && t.Freq == 32000 && t.nsPeriod == 31250);
	CHECK(Presc(l) == 0);
	CHECK(l.pReg->IER == IER_ALL);
	CHECK(l.pReg->CR == CR_RUN);
	CHECK(l.pReg->ARR == 0xFFFF && l.pReg->CMP == 0);
	CHECK(s_NvicEn[l.IrqNo] && s_NvicPrio[l.IrqNo] == 5);

	// LSI started, never ready
	LptReset();
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((RCC->CSR & RCC_CSR_LSION) != 0);

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
		CHECK(Sel(l) == 2);
		CHECK(t.Freq == h.Got);
		CHECK(Presc(l) == h.Presc);
	}
	cfg.ClkSrc = TIMER_CLKSRC_HFXTAL;
	CHECK(TimerInit(&t, &cfg) && Sel(l) == 2);

	// HSI not ready
	LptReset();
	CHECK(TimerInit(&t, &cfg) == false);

	// Default clock from g_McuOsc: LSE crystal, then LSI
	LptReset();
	RCC->BDCR = RCC_BDCR_LSERDY;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_XTAL;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel(l) == 3 && t.Freq == 32768);
	// LSE already running: backup domain not touched
	CHECK((PWR->CR1 & PWR_CR1_DBP) == 0);
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	CHECK(TimerInit(&t, &cfg));
	CHECK(Sel(l) == 1 && t.Freq == 32000);

	// LSE crystal started with backup domain access, refused when it does
	// not start
	LptReset();
	cfg.ClkSrc = TIMER_CLKSRC_LFXTAL;
	RCC->BDCR = RCC_BDCR_LSEBYP;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((PWR->CR1 & PWR_CR1_DBP) != 0 && (RCC->APB1ENR1 & RCC_APB1ENR1_PWREN) != 0);
	CHECK((RCC->BDCR & (RCC_BDCR_LSEON | RCC_BDCR_LSEBYP)) == 0);

	// Bypass with an external oscillator
	LptReset();
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_TCXO;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK((RCC->BDCR & RCC_BDCR_LSEBYP) != 0 && (RCC->BDCR & RCC_BDCR_LSEON) == 0);

	// LSE on, not ready: its bypass and enable are not changed
	LptReset();
	RCC->BDCR = RCC_BDCR_LSEON;
	g_McuOsc.LowPwrOsc.Type = OSC_TYPE_TCXO;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(RCC->BDCR == RCC_BDCR_LSEON);

	// ARR write not confirmed: refused, clock disabled
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	cfg.ClkSrc = TIMER_CLKSRC_LFRC;
	l.pReg->ISR = LPTIM_ISR_CMPOK;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(l.pReg->CR == 0 && (*l.pEnReg & l.EnMask) == 0);
	// Compare write not confirmed
	l.pReg->ISR = LPTIM_ISR_ARROK;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(l.pReg->CR == 0);

	// Out of range DevNo and settings
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	cfg.DevNo = -1;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.DevNo = k;
	cfg.bTickInt = true;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.bTickInt = false;
	cfg.IntPrio = 16;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(Stm32LptimInit(&t, nullptr) == false);
	cfg.DevNo = TEST_NB_LPT;
	CHECK(Stm32LptimInit(&t, &cfg) == false);
}

static void TestRun(int k)
{
	static TimerDev_t t;
	const TestLpt_t &l = s_Lpt[k];
	TimerCfg_t cfg = { k, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler, false };

	memset(&t, 0, sizeof(t));
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	CHECK(TimerInit(&t, &cfg));

	// Count extended from the previous read
	l.pReg->CNT = 100;
	CHECK(t.GetTickCount(&t) == 100);
	s_EvtCnt = 0;
	LptIrq(l, LPTIM_ISR_ARRM, 5);
	CHECK(t.Rollover == 0x10000 && (s_Evt & TIMER_EVT_COUNTER_OVR) && s_EvtCnt == 1 && s_LastTimer == &t);
	CHECK(t.GetTickCount(&t) == 0x10005);
	// ARRM is set at 0xFFFF, a tick before the wrap
	LptIrq(l, LPTIM_ISR_ARRM, 0xFFFF);
	CHECK(t.Rollover == 0x10000 && t.GetTickCount(&t) == 0x1FFFF);
	l.pReg->CNT = 2;
	CHECK(t.GetTickCount(&t) == 0x20002);
	// Wrapped, the interrupt not taken yet
	l.pReg->CNT = 0x8000;
	CHECK(t.GetTickCount(&t) == 0x28000);
	l.pReg->CNT = 1;
	CHECK(t.GetTickCount(&t) == 0x30001);
	LptIrq(l, LPTIM_ISR_ARRM, 3);
	CHECK(t.GetTickCount(&t) == 0x30003);

	// Compare trigger, 10 ms at 32 kHz is 320 counts
	CHECK(t.GetMaxTrigger(&t) == 1 && t.FindAvailTrigger(&t) == 0);
	l.pReg->CNT = 1000;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &s_TrigCnt) == 10000000ULL);
	CHECK(l.pReg->CMP == 1320);
	CHECK(t.FindAvailTrigger(&t) == -1);
	s_TrigCnt = 0;
	// Interrupt a count late: the next is one period after the previous
	LptIrq(l, LPTIM_ISR_CMPM, 1321);
	CHECK(s_TrigCnt == 1 && (s_Evt & TIMER_EVT_TRIGGER(0)));
	CHECK(l.pReg->CMP == 1640);
	// The next compare value wraps at 16 bits
	l.pReg->CNT = 0xFF00;
	t.GetTickCount(&t);
	t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &s_TrigCnt);
	CHECK(l.pReg->CMP == ((0xFF00 + 320) & 0xFFFF));
	LptIrq(l, LPTIM_ISR_CMPM, 0x40);
	CHECK(l.pReg->CMP == 0x40 + 320);
	// Interrupt after the next compare value: one period from now
	LptIrq(l, LPTIM_ISR_CMPM, 0x40 + 320 + 400);
	CHECK(l.pReg->CMP == 0x40 + 320 + 400 + 320);
	// Interrupt 2 counts before the next compare value: one period from now
	uint32_t next = l.pReg->CMP + 320;

	LptIrq(l, LPTIM_ISR_CMPM, next - 2);
	CHECK(l.pReg->CMP == next - 2 + 320);
	CHECK(s_TrigCnt == 4);
	// Overflow and compare in one interrupt
	LptIrq(l, LPTIM_ISR_CMPM | LPTIM_ISR_ARRM, l.pReg->CMP + 1);
	CHECK(s_Evt == (TIMER_EVT_COUNTER_OVR | TIMER_EVT_TRIGGER(0)) && s_TrigCnt == 5);

	// Disabled trigger: no callback, no trigger event
	t.DisableTrigger(&t, 0);
	LptIrq(l, LPTIM_ISR_CMPM, 2000);
	CHECK(s_TrigCnt == 5 && (s_Evt & TIMER_EVT_TRIGGER(0)) == 0);

	// Single shot
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, &s_TrigCnt) != 0);
	LptIrq(l, LPTIM_ISR_CMPM, l.pReg->CMP);
	LptIrq(l, LPTIM_ISR_CMPM, l.pReg->CMP + 320);
	CHECK(s_TrigCnt == 6 && t.FindAvailTrigger(&t) == 0);

	// A compare match of the old value is not the new trigger
	l.pReg->ISR = OK_ALL | LPTIM_ISR_CMPM;
	t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, &s_TrigCnt);
	CHECK((l.pReg->ICR & LPTIM_ICR_CMPMCF) != 0);
	t.DisableTrigger(&t, 0);
	l.pReg->ISR = OK_ALL;

	// Out of the 16 bits compare range, wrong trigger number
	CHECK(t.EnableTrigger(&t, 0, 31250ULL * 2, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, 0, 31250ULL * 0x10000, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, 1, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableTrigger(&t, -1, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.EnableExtTrigger(&t, 0, TIMER_EXTTRIG_SENSE_HIGH_TRANSITION) == false);
	t.DisableExtTrigger(&t);

	// Compare write not confirmed
	l.pReg->ISR = LPTIM_ISR_ARROK;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	CHECK(t.FindAvailTrigger(&t) == 0);
	l.pReg->ISR = OK_ALL;

	// New frequency restarts the counter, triggers disabled
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, nullptr) != 0);
	l.pReg->IER = 0;
	CHECK(t.SetFrequency(&t, 8000) == 8000);
	CHECK(Presc(l) == 2 && t.Rollover == 0 && t.LastCount == 0 && t.nsPeriod == 125000);
	CHECK(l.pReg->CR == CR_RUN && l.pReg->IER == IER_ALL && l.pReg->CMP == 0);
	CHECK(t.FindAvailTrigger(&t) == 0);
	// Restart failing
	l.pReg->ISR = 0;
	CHECK(t.SetFrequency(&t, 32000) == 0);
	CHECK(l.pReg->CR == 0);
	l.pReg->ISR = OK_ALL;

	// Disable and enable
	CHECK(t.Enable(&t));
	CHECK(l.pReg->CR == CR_RUN);
	t.Disable(&t);
	CHECK(l.pReg->CR == 0 && (*l.pEnReg & l.EnMask) == 0 && !s_NvicEn[l.IrqNo]);
	// A trigger needs the timer enabled
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, TrigHandler, nullptr) == 0);
	l.pReg->IER = 0;
	CHECK(t.Enable(&t));
	CHECK(l.pReg->CR == CR_RUN && (*l.pEnReg & l.EnMask) != 0);
	CHECK(l.pReg->IER == IER_ALL && s_NvicEn[l.IrqNo]);
	// Enable of a running timer does not restart it
	l.pReg->CNT = 400;

	uint64_t before = t.GetTickCount(&t);

	l.pReg->CNT = 500;
	CHECK(t.Enable(&t) && t.GetTickCount(&t) == before + 100);
	// Enable failing to restart
	t.Disable(&t);
	l.pReg->ISR = 0;
	CHECK(t.Enable(&t) == false);
	l.pReg->ISR = OK_ALL;

	// Reset, the trigger one period from the counter start
	CHECK(t.Enable(&t));
	l.pReg->CNT = 0x1234;
	t.GetTickCount(&t);
	t.Rollover = 0x30000;
	CHECK(t.EnableTrigger(&t, 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &s_TrigCnt) != 0);
	CHECK(l.pReg->CMP == 0x1234 + 320);
	t.Reset(&t);
	CHECK(t.Rollover == 0 && t.LastCount == 0 && l.pReg->CR == CR_RUN && s_NvicEn[l.IrqNo]);
	CHECK(l.pReg->CMP == 320 && t.FindAvailTrigger(&t) == -1);
	// Enable after Disable: the same
	t.Disable(&t);
	CHECK(t.Enable(&t) && l.pReg->CMP == 320);
	t.DisableTrigger(&t, 0);
}

static void TestBinding(int k)
{
	static TimerDev_t t1, t2;
	const TestLpt_t &l = s_Lpt[k];
	TimerCfg_t cfg = { k, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler, false };

	memset(&t1, 0, sizeof(t1));
	memset(&t2, 0, sizeof(t2));
	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	CHECK(TimerInit(&t1, &cfg));
	CHECK(Stm32LptimFind(&t1) == k);

	// Taken by another TimerDev_t: the first one does nothing
	CHECK(TimerInit(&t2, &cfg));
	CHECK(Stm32LptimFind(&t1) == -1 && Stm32LptimFind(&t2) == k);
	l.pReg->CNT = 77;
	CHECK(t1.GetTickCount(&t1) == 0 && t1.GetMaxTrigger(&t1) == 0 && t1.FindAvailTrigger(&t1) == -1);
	CHECK(t1.EnableTrigger(&t1, 0, 10000000ULL, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
	CHECK(t1.SetFrequency(&t1, 1000) == 0 && t1.Enable(&t1) == false);
	t1.Disable(&t1);
	t1.Reset(&t1);
	t1.DisableTrigger(&t1, 0);
	CHECK(l.pReg->CR == CR_RUN && t2.GetTickCount(&t2) == 77);
	// Interrupts go to the new one
	s_LastTimer = nullptr;
	LptIrq(l, LPTIM_ISR_ARRM, 80);
	CHECK(s_LastTimer == &t2);

	// Moved to a TIM: the LPTIM is stopped and released
	cfg.DevNo = TEST_NB_LPT;
	CHECK(TimerInit(&t2, &cfg) == false);
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	CHECK(TimerInit(&t2, &cfg));
	CHECK(Stm32LptimFind(&t2) == -1 && Stm32TimFind(&t2) == TEST_NB_LPT);
	CHECK(l.pReg->CR == 0 && (*l.pEnReg & l.EnMask) == 0 && !s_NvicEn[l.IrqNo]);
	s_EvtCnt = 0;
	LptIrq(l, LPTIM_ISR_ARRM, 1);
	CHECK(s_EvtCnt == 0);
	Stm32TimRelease(TEST_NB_LPT);

	// Counting without the driver: not taken
	*l.pEnReg |= l.EnMask;
	l.pReg->CR = LPTIM_CR_ENABLE;
	cfg.DevNo = k;
	cfg.ClkSrc = TIMER_CLKSRC_LFRC;
	CHECK(TimerInit(&t1, &cfg) == false);
	CHECK(Stm32LptimFind(&t1) == -1);
	l.pReg->CR = 0;
	CHECK(TimerInit(&t1, &cfg));
	Stm32LptimRelease(k);
	CHECK(Stm32LptimFind(&t1) == -1 && l.pReg->CR == 0);
	Stm32LptimRelease(-1);
	Stm32LptimRelease(TEST_NB_LPT);
}

#if ST_LPTIM_CNT > 1
// Both LPTIM at once
static void TestTwo(void)
{
	static TimerDev_t t[2];
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler, false };

	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	RCC->CR = RCC_CR_HSIRDY;
	memset(t, 0, sizeof(t));
	CHECK(TimerInit(&t[0], &cfg));
	cfg.DevNo = 1;
	cfg.ClkSrc = TIMER_CLKSRC_HFRC;
	cfg.Freq = 1000000;
	cfg.IntPrio = 6;
	CHECK(TimerInit(&t[1], &cfg));
	CHECK(Sel(s_Lpt[0]) == 1 && Sel(s_Lpt[1]) == 2);
	CHECK(t[0].Freq == 32000 && t[1].Freq == 1000000);
	CHECK(s_NvicPrio[s_Lpt[0].IrqNo] == 5 && s_NvicPrio[s_Lpt[1].IrqNo] == 6);

	s_TrigCnt = 0;
	s_Lpt[0].pReg->CNT = 10;
	s_Lpt[1].pReg->CNT = 20;
	CHECK(t[0].EnableTrigger(&t[0], 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &s_TrigCnt));
	CHECK(t[1].EnableTrigger(&t[1], 0, 10000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &s_TrigCnt));
	CHECK(s_Lpt[0].pReg->CMP == 330 && s_Lpt[1].pReg->CMP == 10020);
	s_LastTimer = nullptr;
	LptIrq(s_Lpt[1], LPTIM_ISR_CMPM, 10020);
	CHECK(s_LastTimer == &t[1] && s_TrigCnt == 1);
	CHECK(s_Lpt[1].pReg->CMP == 20020 && s_Lpt[0].pReg->CMP == 330);
	LptIrq(s_Lpt[0], LPTIM_ISR_CMPM, 330);
	CHECK(s_LastTimer == &t[0] && s_TrigCnt == 2 && s_Lpt[0].pReg->CMP == 650);

	// Moved from LPTIM1 to LPTIM2: LPTIM1 released
	cfg.DevNo = 1;
	CHECK(TimerInit(&t[0], &cfg));
	CHECK(s_Lpt[0].pReg->CR == 0 && Stm32LptimFind(&t[0]) == 1 && Stm32LptimFind(&t[1]) == -1);
}
#endif

#ifdef TEST_VECTOR_OVERRIDE
// An application handler on the vector of the last LPTIM: TimerInit refuses
// it, before any register access
static void TestOverride(void)
{
	static TimerDev_t t[TEST_NB_LPT];
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_LFRC, 32000, 5, EvtHandler, false };

	LptReset();
	RCC->CSR = RCC_CSR_LSIRDY;
	memset(t, 0, sizeof(t));
	for (int k = 0; k < TEST_NB_LPT; k++)
	{
		bool own = s_Lpt[k].IrqNo != TEST_OVERRIDE_IRQ;

		cfg.DevNo = k;
		CHECK(TimerInit(&t[k], &cfg) == own);
		CHECK((*s_Lpt[k].pEnReg & s_Lpt[k].EnMask) == (own ? s_Lpt[k].EnMask : 0));
		CHECK(Stm32LptimFind(&t[k]) == (own ? k : -1));
	}
}
#endif

int main()
{
	void *p = mmap((void *)0x40000000, 0x30000, PROT_READ | PROT_WRITE,
				   MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0);

	if (p != (void *)0x40000000)
	{
		printf("mmap failed\n");
		return 1;
	}

#ifdef TEST_VECTOR_OVERRIDE
	TestOverride();
	printf("%d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
#endif

	for (int k = 0; k < TEST_NB_LPT; k++)
	{
		TestInit(k);
		TestRun(k);
		TestBinding(k);
	}
#if ST_LPTIM_CNT > 1
	TestTwo();
#endif

	printf("%d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}
