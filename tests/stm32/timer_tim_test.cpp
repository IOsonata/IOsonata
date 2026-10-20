/**-------------------------------------------------------------------------
@file	timer_tim_test.cpp

@brief	Host test of the STM32 TIM timer driver and of TimerInit.

		Runs ARM/ST/src/timer_stm32.cpp and ARM/ST/src/timer_tim_stm32.cpp
		with the ST device header of the part, the TIM and RCC registers
		replaced by the model below. The LPTIM driver is replaced by a
		record of its calls.

		TIM model : CNT counts up to ARR then wraps and sets UIF, a compare
		match sets CCxIF on the channels of the timer. SR bits are rc_w0. A
		vector runs when it is enabled, interrupts are not disabled, and it
		is pending or a timer on it has a flag with its interrupt enabled.
		On the advanced timers UIF goes to the update vector, CCxIF to the
		compare vector.

		Checks: DevNo order, TimerInit refusals, vectors of each timer,
		timer clock from PCLK, every timer with its triggers, single shot,
		continuous, rollover, Disable and Enable with a pending update,
		Reset, SetFrequency, deadlines longer than the counter, late
		interrupts, a callback disabling another trigger, two timers on one
		vector, a TimerDev_t moved to another timer or taken by another
		TimerDev_t, a timer counting without the driver.

		Not a timing or hardware test.

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
#include <initializer_list>

#include "stm32.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"
#include "timer_stm32.h"

static int s_Fail;
static int s_Checks;

#define CHECK(c)	do { s_Checks++; if (!(c)) { s_Fail++; printf("FAIL %s:%d %s\n", __FILE__, __LINE__, #c); } } while (0)

// ---- Register model -----------------------------------------------------

// rc_w0 status register: a 0 written clears the bit, a 1 keeps it
struct TestStatus {
	uint32_t Val;
	operator uint32_t() const { return Val; }
	TestStatus &operator=(uint32_t v) { Val &= v; return *this; }
};

struct TestTim;

// Counter. A read can be made to wrap the counter right after it returns.
struct TestCount {
	uint32_t Val;
	operator uint32_t() const;
	TestCount &operator=(uint32_t v) { Val = v; return *this; }
};

struct TestTim {
	uint32_t CR1, DIER, EGR;
	TestCount CNT;
	uint32_t PSC, ARR, CCR1, CCR2, CCR3, CCR4;
	TestStatus SR;
};

static TestTim *s_WrapTim;
static int s_WrapRead;

TestCount::operator uint32_t() const
{
	uint32_t v = Val;

	if (s_WrapRead > 0 && s_WrapTim && this == &s_WrapTim->CNT && --s_WrapRead == 0)
	{
		s_WrapTim->CNT.Val = 0;
		s_WrapTim->SR.Val |= TIM_SR_UIF;
	}

	return v;
}

struct TestRcc {
	uint32_t APB1ENR, APB1RSTR, APB1ENR1, APB1RSTR1, APB2ENR, APB2RSTR;
};

static TestTim g_Tim[21];
static TestRcc g_Rcc;

#define TIM_TypeDef		TestTim
#undef RCC
#define RCC				(&g_Rcc)

#ifdef TIM1
#undef TIM1
#define TIM1	(&g_Tim[1])
#endif
#ifdef TIM2
#undef TIM2
#define TIM2	(&g_Tim[2])
#endif
#ifdef TIM3
#undef TIM3
#define TIM3	(&g_Tim[3])
#endif
#ifdef TIM4
#undef TIM4
#define TIM4	(&g_Tim[4])
#endif
#ifdef TIM5
#undef TIM5
#define TIM5	(&g_Tim[5])
#endif
#ifdef TIM6
#undef TIM6
#define TIM6	(&g_Tim[6])
#endif
#ifdef TIM7
#undef TIM7
#define TIM7	(&g_Tim[7])
#endif
#ifdef TIM8
#undef TIM8
#define TIM8	(&g_Tim[8])
#endif
#ifdef TIM9
#undef TIM9
#define TIM9	(&g_Tim[9])
#endif
#ifdef TIM10
#undef TIM10
#define TIM10	(&g_Tim[10])
#endif
#ifdef TIM11
#undef TIM11
#define TIM11	(&g_Tim[11])
#endif
#ifdef TIM12
#undef TIM12
#define TIM12	(&g_Tim[12])
#endif
#ifdef TIM13
#undef TIM13
#define TIM13	(&g_Tim[13])
#endif
#ifdef TIM14
#undef TIM14
#define TIM14	(&g_Tim[14])
#endif
#ifdef TIM15
#undef TIM15
#define TIM15	(&g_Tim[15])
#endif
#ifdef TIM16
#undef TIM16
#define TIM16	(&g_Tim[16])
#endif
#ifdef TIM17
#undef TIM17
#define TIM17	(&g_Tim[17])
#endif
#ifdef TIM20
#undef TIM20
#define TIM20	(&g_Tim[20])
#endif

uint32_t g_TestIrqState;
uint32_t SystemCoreClock = 48000000;
static uint32_t s_Pclk[2] = { 48000000, 48000000 };

extern "C" uint32_t SystemPeriphClockGet(int Idx)
{
	return Idx >= 0 && Idx < 2 ? s_Pclk[Idx] : 0;
}

static bool s_NvicEn[256];
static bool s_NvicPend[256];
static uint32_t s_NvicPrio[256];

extern "C" void TestNvic(int Op, int IrqNo, uint32_t Prio)
{
	if (IrqNo < 0 || IrqNo >= 256)
	{
		return;
	}
	switch (Op)
	{
		case 0: s_NvicEn[IrqNo] = true; break;
		case 1: s_NvicEn[IrqNo] = false; break;
		case 2: s_NvicPend[IrqNo] = false; break;
		case 3: s_NvicPrio[IrqNo] = Prio; break;
		case 4: s_NvicPend[IrqNo] = true; break;
	}
}

// ---- LPTIM driver record ------------------------------------------------

#if ST_LPTIM_CNT > 0
static TimerDev_t *s_LptBound[ST_LPTIM_CNT];
static int s_LptInit = -1;
static int s_LptRelease = -1;

extern "C" bool Stm32LptimInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	s_LptInit = pCfg->DevNo;
	s_LptBound[pCfg->DevNo] = pTimer;
	pTimer->DevNo = pCfg->DevNo;

	return true;
}

extern "C" int Stm32LptimFind(TimerDev_t * const pTimer)
{
	for (int i = 0; i < ST_LPTIM_CNT; i++)
	{
		if (s_LptBound[i] == pTimer)
		{
			return i;
		}
	}

	return -1;
}

extern "C" void Stm32LptimRelease(int DevNo)
{
	s_LptRelease = DevNo;
	s_LptBound[DevNo] = nullptr;
}

static int s_LptIrq = -1;

extern "C" void Stm32LptimIRQHandler(int IrqNo)
{
	s_LptIrq = IrqNo;
}
#endif

#include "../../ARM/ST/src/timer_stm32.cpp"
#include "../../ARM/ST/src/timer_tim_stm32.cpp"

// ---- Expected DevNo order and vectors of each part ----------------------

// Timer vectors of the part: vector name and interrupt number
typedef struct {
	const char *Name;
	IRQn_Type IrqNo;
} TestVector_t;

#if defined(STM32F030x8)
static const int s_Order[] = { 6, 14, 16, 17, 15, 3, 1 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_UP_TRG_COM", TIM1_BRK_UP_TRG_COM_IRQn }, { "TIM1_CC", TIM1_CC_IRQn },
	{ "TIM3", TIM3_IRQn }, { "TIM6", TIM6_IRQn }, { "TIM14", TIM14_IRQn },
	{ "TIM15", TIM15_IRQn }, { "TIM16", TIM16_IRQn }, { "TIM17", TIM17_IRQn },
};
#elif defined(STM32F072xB)
static const int s_Order[] = { 6, 7, 14, 16, 17, 15, 2, 3, 1 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_UP_TRG_COM", TIM1_BRK_UP_TRG_COM_IRQn }, { "TIM1_CC", TIM1_CC_IRQn },
	{ "TIM2", TIM2_IRQn }, { "TIM3", TIM3_IRQn }, { "TIM6_DAC", TIM6_DAC_IRQn },
	{ "TIM7", TIM7_IRQn }, { "TIM14", TIM14_IRQn }, { "TIM15", TIM15_IRQn },
	{ "TIM16", TIM16_IRQn }, { "TIM17", TIM17_IRQn },
};
#elif defined(STM32F401xC)
static const int s_Order[] = { 10, 11, 9, 2, 3, 4, 5, 1 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_TIM9", TIM1_BRK_TIM9_IRQn }, { "TIM1_UP_TIM10", TIM1_UP_TIM10_IRQn },
	{ "TIM1_TRG_COM_TIM11", TIM1_TRG_COM_TIM11_IRQn }, { "TIM1_CC", TIM1_CC_IRQn },
	{ "TIM2", TIM2_IRQn }, { "TIM3", TIM3_IRQn }, { "TIM4", TIM4_IRQn },
	{ "TIM5", TIM5_IRQn },
};
#elif defined(STM32F446xx)
static const int s_Order[] = { 6, 7, 10, 11, 13, 14, 9, 12, 2, 3, 4, 5, 1, 8 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_TIM9", TIM1_BRK_TIM9_IRQn }, { "TIM1_UP_TIM10", TIM1_UP_TIM10_IRQn },
	{ "TIM1_TRG_COM_TIM11", TIM1_TRG_COM_TIM11_IRQn }, { "TIM1_CC", TIM1_CC_IRQn },
	{ "TIM2", TIM2_IRQn }, { "TIM3", TIM3_IRQn }, { "TIM4", TIM4_IRQn },
	{ "TIM5", TIM5_IRQn }, { "TIM6_DAC", TIM6_DAC_IRQn }, { "TIM7", TIM7_IRQn },
	{ "TIM8_BRK_TIM12", TIM8_BRK_TIM12_IRQn }, { "TIM8_UP_TIM13", TIM8_UP_TIM13_IRQn },
	{ "TIM8_TRG_COM_TIM14", TIM8_TRG_COM_TIM14_IRQn }, { "TIM8_CC", TIM8_CC_IRQn },
};
#elif defined(STM32L432xx)
static const int s_Order[] = { 6, 7, 16, 15, 2, 1 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_TIM15", TIM1_BRK_TIM15_IRQn }, { "TIM1_UP_TIM16", TIM1_UP_TIM16_IRQn },
	{ "TIM1_CC", TIM1_CC_IRQn }, { "TIM2", TIM2_IRQn },
	{ "TIM6_DAC", TIM6_DAC_IRQn }, { "TIM7", TIM7_IRQn },
};
#elif defined(STM32L476xx) || defined(STM32L496xx) || defined(STM32L4S9xx)
static const int s_Order[] = { 6, 7, 16, 17, 15, 2, 3, 4, 5, 1, 8 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_TIM15", TIM1_BRK_TIM15_IRQn }, { "TIM1_UP_TIM16", TIM1_UP_TIM16_IRQn },
	{ "TIM1_TRG_COM_TIM17", TIM1_TRG_COM_TIM17_IRQn }, { "TIM1_CC", TIM1_CC_IRQn },
	{ "TIM2", TIM2_IRQn }, { "TIM3", TIM3_IRQn }, { "TIM4", TIM4_IRQn },
	{ "TIM5", TIM5_IRQn }, { "TIM6_DAC", TIM6_DAC_IRQn }, { "TIM7", TIM7_IRQn },
	{ "TIM8_UP", TIM8_UP_IRQn }, { "TIM8_CC", TIM8_CC_IRQn },
};
#elif defined(STM32G474xx)
static const int s_Order[] = { 6, 7, 16, 17, 15, 2, 3, 4, 5, 1, 8, 20 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_BRK_TIM15", TIM1_BRK_TIM15_IRQn }, { "TIM1_UP_TIM16", TIM1_UP_TIM16_IRQn },
	{ "TIM1_TRG_COM_TIM17", TIM1_TRG_COM_TIM17_IRQn }, { "TIM1_CC", TIM1_CC_IRQn },
	{ "TIM2", TIM2_IRQn }, { "TIM3", TIM3_IRQn }, { "TIM4", TIM4_IRQn },
	{ "TIM5", TIM5_IRQn }, { "TIM6_DAC", TIM6_DAC_IRQn },
	{ "TIM7_DAC", TIM7_DAC_IRQn }, { "TIM8_UP", TIM8_UP_IRQn },
	{ "TIM8_CC", TIM8_CC_IRQn }, { "TIM20_UP", TIM20_UP_IRQn },
	{ "TIM20_CC", TIM20_CC_IRQn },
};
#elif defined(STM32WBA65xx)
static const int s_Order[] = { 16, 17, 2, 3, 4, 1 };
static const TestVector_t s_Vector[] = {
	{ "TIM1_UP", TIM1_UP_IRQn }, { "TIM1_CC", TIM1_CC_IRQn }, { "TIM2", TIM2_IRQn },
	{ "TIM3", TIM3_IRQn }, { "TIM4", TIM4_IRQn }, { "TIM16", TIM16_IRQn },
	{ "TIM17", TIM17_IRQn },
};
#else
#error "timer_tim_test.cpp: part not listed"
#endif

#define TEST_NB_TIM		((int)(sizeof(s_Order) / sizeof(int)))
#define TEST_NB_VECTOR	((int)(sizeof(s_Vector) / sizeof(TestVector_t)))

// Vector replaced by an application handler in the test built with
// TEST_VECTOR_OVERRIDE
#ifdef TEST_VECTOR_OVERRIDE
#if defined(STM32F030x8) || defined(STM32F072xB) || defined(STM32WBA65xx)
#define TEST_OVERRIDE_IRQ	TIM17_IRQn
#elif defined(STM32F401xC)
#define TEST_OVERRIDE_IRQ	TIM1_UP_TIM10_IRQn
#elif defined(STM32F446xx)
#define TEST_OVERRIDE_IRQ	TIM8_UP_TIM13_IRQn
#elif defined(STM32L432xx) || defined(STM32G474xx)
#define TEST_OVERRIDE_IRQ	TIM1_UP_TIM16_IRQn
#else
#define TEST_OVERRIDE_IRQ	TIM1_CC_IRQn
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

// ---- Hardware model -----------------------------------------------------

static TestTim &Reg(int Idx)
{
	return *s_StTimInfo[Idx].pReg;
}

// Interrupt request of a vector
static bool Requested(IRQn_Type IrqNo)
{
	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		const StTimInfo_t &info = s_StTimInfo[i];
		uint32_t flags = Reg(i).SR.Val & Reg(i).DIER;
		bool split = info.IrqNo != info.CcIrqNo;

		if (info.IrqNo == IrqNo && (split ? (flags & TIM_SR_UIF) : flags))
		{
			return true;
		}
		if (split && info.CcIrqNo == IrqNo && (flags & ~TIM_SR_UIF))
		{
			return true;
		}
	}

	return false;
}

static int s_VectorRuns[TEST_NB_VECTOR];

// Runs the vectors with an interrupt request
static void Service(void)
{
	for (int pass = 0; pass < 8; pass++)
	{
		bool run = false;

		for (int v = 0; v < TEST_NB_VECTOR; v++)
		{
			IRQn_Type irq = s_Vector[v].IrqNo;

			if (g_TestIrqState == 0 && s_NvicEn[irq] && (s_NvicPend[irq] || Requested(irq)))
			{
				s_NvicPend[irq] = false;
				s_VectorRuns[v]++;
				// Stm32TimerVector of the vector file
				Stm32TimerIRQHandler(irq);
				run = true;
			}
		}
		if (run == false)
		{
			return;
		}
	}
}

// One counter clock of every running timer, then the vectors
static void Advance(uint32_t Ticks, bool bService = true)
{
	for (uint32_t t = 0; t < Ticks; t++)
	{
		for (int i = 0; i < ST_TIM_CNT; i++)
		{
			TestTim &r = Reg(i);

			if ((r.CR1 & TIM_CR1_CEN) == 0)
			{
				continue;
			}
			if (++r.CNT.Val > r.ARR)
			{
				r.CNT.Val = 0;
				r.SR.Val |= TIM_SR_UIF;
			}

			uint32_t cc[4] = { r.CCR1, r.CCR2, r.CCR3, r.CCR4 };

			for (int n = 0; n < s_StTimInfo[i].NbChan; n++)
			{
				if (r.CNT.Val == cc[n])
				{
					r.SR.Val |= TIM_SR_CC1IF << n;
				}
			}
		}
		if (bService)
		{
			Service();
		}
	}
}

// ---- Events -------------------------------------------------------------

#define TEST_DEVNO_MAX		32

static unsigned s_Events[TEST_DEVNO_MAX][4];
static unsigned s_Ovr[TEST_DEVNO_MAX];
static unsigned s_Callbacks[TEST_DEVNO_MAX][4];
static TimerDev_t *s_CancelOnTrig0;

static void EvtHandler(TimerDev_t * const pTimer, uint32_t Evt)
{
	if (Evt & TIMER_EVT_COUNTER_OVR)
	{
		s_Ovr[pTimer->DevNo]++;
	}
	for (int n = 0; n < 4; n++)
	{
		if (Evt & TIMER_EVT_TRIGGER(n))
		{
			s_Events[pTimer->DevNo][n]++;
		}
	}
	if (s_CancelOnTrig0 == pTimer && (Evt & TIMER_EVT_TRIGGER(0)))
	{
		pTimer->DisableTrigger(pTimer, 1);
	}
}

static void TrigHandler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	CHECK(pContext == &s_Callbacks);
	s_Callbacks[pTimer->DevNo][TrigNo]++;
}

static int DevNo(int Idx)
{
	return Idx + ST_LPTIM_CNT;
}

static int TrigCnt(int Idx)
{
	return s_StTimInfo[Idx].NbChan > 0 ? s_StTimInfo[Idx].NbChan : 1;
}

// Timer clock of an entry with the PCLK of the test
static uint32_t TimClock(int Idx)
{
	uint32_t pclk = s_Pclk[s_StTimInfo[Idx].PclkIdx];

	return pclk < SystemCoreClock ? pclk * 2 : pclk;
}

// ---- Tests --------------------------------------------------------------

static void TestTable(void)
{
	CHECK(TimerGetLowFreqDevCount() == ST_LPTIM_CNT);
	CHECK(TimerGetHighFreqDevNo() == ST_LPTIM_CNT);
	CHECK(TimerGetHighFreqDevCount() == TEST_NB_TIM);
	CHECK(Stm32TimCount() == TEST_NB_TIM);

	for (int i = 0; i < TEST_NB_TIM && i < ST_TIM_CNT; i++)
	{
		const StTimInfo_t &info = s_StTimInfo[i];
		int n = s_Order[i];
		int chan = (n == 6 || n == 7) ? 0 : (n == 10 || n == 11 || n == 13 || n == 14 || n == 16 || n == 17) ? 1 :
				   (n == 9 || n == 12 || n == 15) ? 2 : 4;

		CHECK(info.pReg == &g_Tim[n]);
		CHECK(info.NbChan == chan);

		// Vectors of the timer are defined
		bool up = false, cc = false;

		for (int v = 0; v < TEST_NB_VECTOR; v++)
		{
			up = up || s_Vector[v].IrqNo == info.IrqNo;
			cc = cc || s_Vector[v].IrqNo == info.CcIrqNo;
		}
		CHECK(up && cc);
		CHECK((info.IrqNo != info.CcIrqNo) == (n == 1 || n == 8 || n == 20));

		// APB1: TIM2 to TIM7, TIM12 to TIM14
		bool apb1 = (n >= 2 && n <= 7) || (n >= 12 && n <= 14);

		CHECK(info.pEnReg == (apb1 ? &RCC->ST_RCC_APB1ENR : &RCC->APB2ENR));
		CHECK(info.pRstReg == (apb1 ? &RCC->ST_RCC_APB1RSTR : &RCC->APB2RSTR));
#ifdef STM32F0
		// One APB prescaler
		CHECK(info.PclkIdx == 0);
#else
		CHECK(info.PclkIdx == (apb1 ? 0 : 1));
#endif
	}

	// Each vector of the list is a timer vector
	for (int v = 0; v < TEST_NB_VECTOR; v++)
	{
		bool used = false;

		for (int i = 0; i < ST_TIM_CNT; i++)
		{
			used = used || s_StTimInfo[i].IrqNo == s_Vector[v].IrqNo || s_StTimInfo[i].CcIrqNo == s_Vector[v].IrqNo;
		}
		CHECK(used);
	}
}

static void TestInitRefusal(void)
{
	TimerDev_t t;
	TimerCfg_t cfg = { DevNo(0), TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };

	memset(&t, 0, sizeof(t));
	CHECK(TimerInit(nullptr, &cfg) == false);
	CHECK(TimerInit(&t, nullptr) == false);
	cfg.DevNo = -1;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.DevNo = ST_LPTIM_CNT + TEST_NB_TIM;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.DevNo = DevNo(0);
	cfg.bTickInt = true;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.bTickInt = false;
	cfg.IntPrio = -1;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.IntPrio = 1 << __NVIC_PRIO_BITS;
	CHECK(TimerInit(&t, &cfg) == false);
	cfg.IntPrio = 2;
	for (TIMER_CLKSRC c : { TIMER_CLKSRC_LFRC, TIMER_CLKSRC_HFRC, TIMER_CLKSRC_LFXTAL, TIMER_CLKSRC_HFXTAL, TIMER_CLKSRC_EXT })
	{
		cfg.ClkSrc = c;
		CHECK(TimerInit(&t, &cfg) == false);
	}
	CHECK(t.Enable == nullptr && s_StTimDev[0].pTimer == nullptr);
	CHECK((*s_StTimInfo[0].pEnReg & s_StTimInfo[0].Mask) == 0);

	// Counting without the driver
	*s_StTimInfo[0].pEnReg |= s_StTimInfo[0].Mask;
	Reg(0).CR1 = TIM_CR1_CEN;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	CHECK(TimerInit(&t, &cfg) == false);
	CHECK(s_StTimDev[0].pTimer == nullptr);
	Reg(0).CR1 = 0;
	*s_StTimInfo[0].pEnReg &= ~s_StTimInfo[0].Mask;

#if ST_LPTIM_CNT > 0
	// Low power timers go to the LPTIM driver
	for (int i = 0; i < ST_LPTIM_CNT; i++)
	{
		TimerDev_t l;

		memset(&l, 0, sizeof(l));
		cfg.DevNo = i;
		s_LptInit = -1;
		CHECK(TimerInit(&l, &cfg));
		CHECK(s_LptInit == i);
		s_LptBound[i] = nullptr;
	}
	// Timer interrupts go to the LPTIM driver too
	Stm32TimerIRQHandler(LPTIM1_IRQn);
	CHECK(s_LptIrq == LPTIM1_IRQn);
#endif
}

// Timer clock from PCLK, twice PCLK with an APB prescaler
static void TestClock(void)
{
	TimerDev_t t;
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };

	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		cfg.DevNo = DevNo(i);
		memset(&t, 0, sizeof(t));

		SystemCoreClock = 80000000;
		s_Pclk[0] = 80000000;
		s_Pclk[1] = 80000000;
		CHECK(TimerInit(&t, &cfg));
		CHECK(t.Freq == 1000000 && Reg(i).PSC == 79);

		// Prescaler 2 on the bus of the timer, 8 on the other one
		s_Pclk[s_StTimInfo[i].PclkIdx ^ 1] = 10000000;
		s_Pclk[s_StTimInfo[i].PclkIdx] = 40000000;
		CHECK(t.SetFrequency(&t, 1000000) == 1000000 && Reg(i).PSC == 79);
		s_Pclk[s_StTimInfo[i].PclkIdx] = 20000000;
		CHECK(t.SetFrequency(&t, 1000000) == 1000000 && Reg(i).PSC == 39);
		CHECK(t.SetFrequency(&t, 0) == 40000000 && Reg(i).PSC == 0);
		CHECK(t.SetFrequency(&t, 1) == 40000000 / 65536 && Reg(i).PSC == 65535);
		CHECK(t.SetFrequency(&t, 3000000) == 40000000 / 13 && Reg(i).PSC == 12);
		Stm32TimRelease(cfg.DevNo);
		CHECK(s_StTimDev[i].pTimer == nullptr && (*s_StTimInfo[i].pEnReg & s_StTimInfo[i].Mask) == 0);
	}
	SystemCoreClock = 48000000;
	s_Pclk[0] = 48000000;
	s_Pclk[1] = 48000000;
}

// Every timer: triggers, rollover, pause, reset
static void TestEach(void)
{
	static TimerDev_t dev[TEST_NB_TIM];
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };

	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		TimerDev_t &d = dev[i];
		TestTim &r = Reg(i);
		const StTimInfo_t &info = s_StTimInfo[i];
		int devno = DevNo(i);
		int trig = TrigCnt(i);

		cfg.DevNo = devno;
		memset(&d, 0, sizeof(d));
		CHECK(TimerInit(&d, &cfg));
		CHECK(d.DevNo == devno && d.Freq == 1000000 && r.PSC == 47 && r.ARR == 0xFFFF);
		CHECK((*info.pEnReg & info.Mask) != 0 && (*info.pRstReg & info.Mask) == 0);
		CHECK(r.CR1 == (TIM_CR1_URS | TIM_CR1_CEN) && r.DIER == TIM_DIER_UIE);
		CHECK(s_NvicEn[info.IrqNo] && s_NvicEn[info.CcIrqNo]);
		CHECK(s_NvicPrio[info.IrqNo] == 2 && s_NvicPrio[info.CcIrqNo] == 2);
		CHECK(d.GetMaxTrigger(&d) == trig && d.FindAvailTrigger(&d) == 0);
		CHECK(d.EnableExtTrigger(&d, 0, TIMER_EXTTRIG_SENSE_TOGGLE) == false);

		// All triggers, single shot, 1 ms
		for (int n = 0; n < trig; n++)
		{
			CHECK(d.EnableTrigger(&d, n, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 1000000);
		}
		CHECK(d.FindAvailTrigger(&d) == -1);

		unsigned saved[4];

		memcpy(saved, s_Events[devno], sizeof(saved));
		Advance(1000);
		for (int n = 0; n < trig; n++)
		{
			CHECK(s_Events[devno][n] == saved[n] + 1);
		}
		Advance(2000);
		for (int n = 0; n < trig; n++)
		{
			CHECK(s_Events[devno][n] == saved[n] + 1);
		}
		CHECK(d.FindAvailTrigger(&d) == 0);

		// Refused periods and trigger numbers
		d.Reset(&d);
		CHECK(d.EnableTrigger(&d, trig, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		CHECK(d.EnableTrigger(&d, -1, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		CHECK(d.EnableTrigger(&d, 0, UINT64_MAX, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		CHECK(d.EnableTrigger(&d, 0, 3000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		CHECK(d.EnableTrigger(&d, 0, 1000000, (TIMER_TRIG_TYPE)7, nullptr, nullptr) == 0);
		if (info.NbChan == 0)
		{
			// One counter period at most
			CHECK(d.EnableTrigger(&d, 0, 65537000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
			CHECK(d.EnableTrigger(&d, 0, 65536000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 65536000);
			d.DisableTrigger(&d, 0);
		}

		// Continuous, with the callback: callback and event both
		unsigned before = s_Events[devno][0];
		unsigned cb = s_Callbacks[devno][0];

		CHECK(d.EnableTrigger(&d, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, TrigHandler, &s_Callbacks) == 1000000);
		Advance(3000);
		CHECK(s_Events[devno][0] == before + 3 && s_Callbacks[devno][0] == cb + 3);
		CHECK(d.GetTickCount(&d) == 3000);

		// Disable keeps the count and the trigger
		d.Disable(&d);
		CHECK((r.CR1 & TIM_CR1_CEN) == 0 && r.DIER == 0);
		Advance(500);
		CHECK(d.GetTickCount(&d) == 3000);
		CHECK(d.Enable(&d));
		Advance(1000);
		CHECK(s_Events[devno][0] == before + 4);

		d.DisableTrigger(&d, 0);

		uint64_t count = d.GetTickCount(&d);

		Advance(17);
		CHECK(d.GetTickCount(&d) == count + 17);
		Advance(5000);
		CHECK(s_Events[devno][0] == before + 4);

		// Reset, single shot
		d.Reset(&d);
		CHECK(d.GetTickCount(&d) == 0);
		CHECK(d.EnableTrigger(&d, 0, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr));
		before = s_Events[devno][0];
		Advance(5000);
		CHECK(s_Events[devno][0] == before + 1 && d.GetTickCount(&d) == 5000);

		// Wrap seen before its interrupt
		unsigned ovr = s_Ovr[devno];

		d.Reset(&d);
		Advance(65530);
		Advance(10, false);
		CHECK(d.GetTickCount(&d) == 65540);
		CHECK(d.GetTickCount(&d) == 65540);
		Service();
		CHECK(d.GetTickCount(&d) == 65540 && d.Rollover == 65536 && s_Ovr[devno] == ovr + 1);

		// New frequency, the counter restarts
		CHECK(d.SetFrequency(&d, 2000000) == 2000000 && r.PSC == 23 && d.GetTickCount(&d) == 0);
		CHECK(d.nsPeriod == 500);
		if (info.NbChan == 0)
		{
			// A trigger the new frequency can not make refuses it
			CHECK(d.EnableTrigger(&d, 0, 30000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
			CHECK(d.SetFrequency(&d, 4000000) == 0 && d.Freq == 2000000 && r.PSC == 23);
			d.DisableTrigger(&d, 0);
		}

		// Pause with a pending update: Enable keeps it
		d.Reset(&d);
		Advance(65540, false);
		d.Disable(&d);
		CHECK(d.GetTickCount(&d) == 65540);
		CHECK(d.Enable(&d));
		Service();
		CHECK(d.GetTickCount(&d) == 65540);

		// Stop this one, the next one counts alone
		d.Disable(&d);
	}
}

// A four channel timer with its own vector
static int FindFour(void)
{
	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		if (s_StTimInfo[i].NbChan == 4 && s_StTimInfo[i].IrqNo == s_StTimInfo[i].CcIrqNo)
		{
			return i;
		}
	}

	return -1;
}

static void TestFour(void)
{
	static TimerDev_t t;
	int q = FindFour();

	CHECK(q >= 0);
	if (q < 0)
	{
		return;
	}

	TestTim &r = Reg(q);
	int devno = DevNo(q);
	TimerCfg_t cfg = { devno, TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };

	memset(&t, 0, sizeof(t));
	CHECK(TimerInit(&t, &cfg));

	// rc_w0: a flag set after the read is kept
	r.SR.Val = TIM_SR_CC1IF | TIM_SR_CC2IF;
	StTimClearFlags(&s_StTimInfo[q], TIM_SR_CC1IF);
	CHECK(r.SR.Val == TIM_SR_CC2IF);
	r.SR.Val = 0;

	// The counter wraps between the update flag read and the counter read
	t.Reset(&t);
	r.CNT.Val = 0xFFFF;
	s_WrapTim = &r;
	s_WrapRead = 1;
	CHECK(t.GetTickCount(&t) == 65536);
	CHECK(s_WrapRead == 0);
	s_WrapTim = nullptr;
	Service();
	CHECK(t.GetTickCount(&t) == 65536 && t.Rollover == 65536);
	t.Reset(&t);

	// Four continuous triggers, 1 to 4 ms
	unsigned ev[4], cb = s_Callbacks[devno][3];

	memcpy(ev, s_Events[devno], sizeof(ev));
	for (int n = 0; n < 4; n++)
	{
		CHECK(t.EnableTrigger(&t, n, 1000000ULL * (n + 1), TIMER_TRIG_TYPE_CONTINUOUS,
							  n == 3 ? TrigHandler : nullptr, n == 3 ? &s_Callbacks : nullptr));
	}
	Advance(12000);
	CHECK(s_Events[devno][0] == ev[0] + 12 && s_Events[devno][1] == ev[1] + 6);
	CHECK(s_Events[devno][2] == ev[2] + 4 && s_Events[devno][3] == ev[3] + 3);
	CHECK(s_Callbacks[devno][3] == cb + 3);
	for (int n = 0; n < 4; n++)
	{
		t.DisableTrigger(&t, n);
	}

	// A deadline longer than the counter does not fire on the earlier
	// compare match of its low 16 bits
	t.Reset(&t);
	ev[0] = s_Events[devno][0];
	CHECK(t.EnableTrigger(&t, 0, 200000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 200000000);
	Advance(199999);
	CHECK(s_Events[devno][0] == ev[0]);
	Advance(1);
	CHECK(s_Events[devno][0] == ev[0] + 1);
	Advance(200000);
	CHECK(s_Events[devno][0] == ev[0] + 1);

	// Late interrupt: missed periods are one event, the phase is kept
	t.Reset(&t);
	ev[0] = s_Events[devno][0];
	t.EnableTrigger(&t, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	Advance(3500, false);
	Service();
	CHECK(s_Events[devno][0] == ev[0] + 1 && r.CCR1 == 4000);
	t.Disable(&t);

	uint64_t paused = t.GetTickCount(&t);

	Advance(5000);
	CHECK(t.GetTickCount(&t) == paused);
	CHECK(t.Enable(&t));
	Advance(500);
	CHECK(s_Events[devno][0] == ev[0] + 2);

	// A callback disables a trigger due at the same count
	t.Reset(&t);
	t.EnableTrigger(&t, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	t.EnableTrigger(&t, 1, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	ev[1] = s_Events[devno][1];
	s_CancelOnTrig0 = &t;
	Advance(1000);
	CHECK(s_Events[devno][1] == ev[1]);
	s_CancelOnTrig0 = nullptr;

	// Triggers keep their period over a new frequency
	CHECK(t.SetFrequency(&t, 2000000) == 2000000 && r.PSC == 23 && r.CCR1 == 2000);
	CHECK(t.GetTickCount(&t) == 0);

	// A trigger enabled while paused is armed, its interrupt comes with Enable
	t.DisableTrigger(&t, 0);
	t.Disable(&t);
	CHECK(t.EnableTrigger(&t, 2, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr));
	CHECK(r.DIER == 0);
	CHECK(t.Enable(&t));
	CHECK(r.DIER == (TIM_DIER_UIE | TIM_DIER_CC3IE));
	ev[2] = s_Events[devno][2];
	Advance(2000);
	CHECK(s_Events[devno][2] == ev[2] + 1);
	t.Disable(&t);
}

// Advanced timer: compare on its own vector
static void TestAdvanced(void)
{
	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		const StTimInfo_t &info = s_StTimInfo[i];

		if (info.IrqNo == info.CcIrqNo)
		{
			continue;
		}

		static TimerDev_t t;
		TimerCfg_t cfg = { DevNo(i), TIMER_CLKSRC_DEFAULT, 1000000, 3, EvtHandler, false };
		int vu = -1, vc = -1;

		for (int v = 0; v < TEST_NB_VECTOR; v++)
		{
			vu = s_Vector[v].IrqNo == info.IrqNo ? v : vu;
			vc = s_Vector[v].IrqNo == info.CcIrqNo ? v : vc;
		}

		memset(&t, 0, sizeof(t));
		CHECK(TimerInit(&t, &cfg));
		CHECK(s_NvicPrio[info.CcIrqNo] == 3);

		int ru = s_VectorRuns[vu], rc = s_VectorRuns[vc];
		unsigned ev = s_Events[DevNo(i)][3];

		CHECK(t.EnableTrigger(&t, 3, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr));
		Advance(1000);
		CHECK(s_Events[DevNo(i)][3] == ev + 1);
		CHECK(s_VectorRuns[vc] == rc + 1 && s_VectorRuns[vu] == ru);
		Advance(65536);
		CHECK(s_VectorRuns[vu] == ru + 1);
		t.Disable(&t);
	}
}

// Two timers on one vector
static void TestShared(void)
{
	int found = 0;

	for (int a = 0; a < ST_TIM_CNT; a++)
	{
		for (int b = a + 1; b < ST_TIM_CNT; b++)
		{
			if (s_StTimInfo[a].IrqNo != s_StTimInfo[b].IrqNo)
			{
				continue;
			}

			static TimerDev_t ta, tb;
			TimerCfg_t cfg = { DevNo(a), TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };

			found++;
			memset(&ta, 0, sizeof(ta));
			memset(&tb, 0, sizeof(tb));
			CHECK(TimerInit(&ta, &cfg));
			cfg.DevNo = DevNo(b);
			CHECK(TimerInit(&tb, &cfg));

			unsigned ea = s_Events[DevNo(a)][0], eb = s_Events[DevNo(b)][0];

			CHECK(ta.EnableTrigger(&ta, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
			CHECK(tb.EnableTrigger(&tb, 0, 1500000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
			Advance(6000);
			CHECK(s_Events[DevNo(a)][0] == ea + 6 && s_Events[DevNo(b)][0] == eb + 4);

			// Disable of one leaves the vector to the other
			tb.Disable(&tb);
			CHECK(s_NvicEn[s_StTimInfo[a].IrqNo]);
			Advance(3000);
			CHECK(s_Events[DevNo(a)][0] == ea + 9 && s_Events[DevNo(b)][0] == eb + 4);

			// Release of one too
			Stm32TimRelease(DevNo(b));
			CHECK(s_NvicEn[s_StTimInfo[a].IrqNo]);
			Advance(1000);
			CHECK(s_Events[DevNo(a)][0] == ea + 10);
			Stm32TimRelease(DevNo(a));
			CHECK(s_NvicEn[s_StTimInfo[a].IrqNo] == false);
		}
	}
#if defined(STM32L4) || defined(STM32G4) || defined(STM32F4)
	CHECK(found > 0);
#endif
}

// TimerDev_t moved to another timer, timer taken by another TimerDev_t
static void TestBinding(void)
{
	static TimerDev_t t1, t2;
	TimerCfg_t cfg = { DevNo(0), TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };

	memset(&t1, 0, sizeof(t1));
	memset(&t2, 0, sizeof(t2));
	CHECK(TimerInit(&t1, &cfg));
	CHECK(t1.EnableTrigger(&t1, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
	CHECK(Stm32TimFind(&t1) == DevNo(0));

	// Init again, same TimerDev_t, same timer
	CHECK(TimerInit(&t1, &cfg));
	CHECK(t1.FindAvailTrigger(&t1) == 0);

	// Moved: the first timer is stopped and released
	cfg.DevNo = DevNo(1);
	CHECK(TimerInit(&t1, &cfg));
	CHECK(Stm32TimFind(&t1) == DevNo(1) && t1.DevNo == DevNo(1));
	CHECK(s_StTimDev[0].pTimer == nullptr);
	CHECK((Reg(0).CR1 & TIM_CR1_CEN) == 0 && Reg(0).DIER == 0);
	CHECK((*s_StTimInfo[0].pEnReg & s_StTimInfo[0].Mask) == 0);

	// Taken by another TimerDev_t: the first one does nothing
	CHECK(TimerInit(&t2, &cfg));
	CHECK(Stm32TimFind(&t1) == -1 && Stm32TimFind(&t2) == DevNo(1));
	Advance(100);
	CHECK(t1.GetTickCount(&t1) == 0 && t1.GetMaxTrigger(&t1) == 0 && t1.FindAvailTrigger(&t1) == -1);
	CHECK(t1.EnableTrigger(&t1, 0, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
	CHECK(t1.SetFrequency(&t1, 1000) == 0 && t1.Enable(&t1) == false);
	t1.Disable(&t1);
	t1.Reset(&t1);
	t1.DisableTrigger(&t1, 0);
	CHECK((Reg(1).CR1 & TIM_CR1_CEN) != 0 && t2.GetTickCount(&t2) == 100);

#if ST_LPTIM_CNT > 0
	// Moved to a low power timer and back
	cfg.DevNo = 0;
	s_LptRelease = -1;
	CHECK(TimerInit(&t2, &cfg));
	CHECK(s_StTimDev[1].pTimer == nullptr && (Reg(1).CR1 & TIM_CR1_CEN) == 0);
	cfg.DevNo = DevNo(1);
	CHECK(TimerInit(&t2, &cfg));
	CHECK(s_LptRelease == 0 && s_LptBound[0] == nullptr);
#endif
	Stm32TimRelease(DevNo(1));
}

#ifdef TEST_VECTOR_OVERRIDE
// An application handler on vector TEST_OVERRIDE_IRQ: TimerInit refuses the
// timers on it, before any register access
static void TestOverride(void)
{
	static TimerDev_t t[TEST_NB_TIM];
	TimerCfg_t cfg = { 0, TIMER_CLKSRC_DEFAULT, 1000000, 2, EvtHandler, false };
	int refused = 0;

	for (int i = 0; i < ST_TIM_CNT; i++)
	{
		bool own = s_StTimInfo[i].IrqNo != TEST_OVERRIDE_IRQ && s_StTimInfo[i].CcIrqNo != TEST_OVERRIDE_IRQ;

		cfg.DevNo = DevNo(i);
		memset(&t[i], 0, sizeof(t[i]));
		CHECK(TimerInit(&t[i], &cfg) == own);
		if (own == false)
		{
			refused++;
			CHECK(s_StTimDev[i].pTimer == nullptr && t[i].Enable == nullptr);
			CHECK((*s_StTimInfo[i].pEnReg & s_StTimInfo[i].Mask) == 0);
		}
	}
	CHECK(refused > 0);
}
#endif

int main(int argc, char **argv)
{
	// Timer vector names, for the check of the vector file
	if (argc > 1 && strcmp(argv[1], "--vectors") == 0)
	{
#ifdef ST_LPTIM1
		printf("LPTIM1\n");
#endif
#ifdef ST_LPTIM2
		printf("LPTIM2\n");
#endif
		for (int v = 0; v < TEST_NB_VECTOR; v++)
		{
			printf("%s\n", s_Vector[v].Name);
		}

		return 0;
	}

#ifdef TEST_VECTOR_OVERRIDE
	TestOverride();
#else
	TestTable();
	TestInitRefusal();
	TestClock();
	TestEach();
	TestFour();
	TestAdvanced();
	TestShared();
	TestBinding();
#endif

	CHECK(g_TestIrqState == 0);
	printf("%d checks, %d failures\n", s_Checks, s_Fail);

	return s_Fail != 0;
}
