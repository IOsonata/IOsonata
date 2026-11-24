/**-------------------------------------------------------------------------
@file	timer17_test.cpp

@brief	LPC17xx timer adapter regression using vendor CMSIS registers.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#include <cassert>
#include <climits>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include "timer_lpcxx.h"
#include "coredev/system_core_clock.h"

uint32_t g_IrqMasked;
uint32_t SystemCoreClock = 96000000;
static uint32_t s_Priority[5];
static uint32_t s_Enabled;
static unsigned s_Callbacks[4];
static const int s_Pclk[] = {1, 2, 22, 23};
static const unsigned s_Power[] = {1, 2, 22, 23};
static const unsigned s_Div[] = {4, 1, 2, 8};

extern "C" void TIMER0_IRQHandler(void);
extern "C" void TIMER1_IRQHandler(void);
extern "C" void TIMER2_IRQHandler(void);
extern "C" void TIMER3_IRQHandler(void);

uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t sel = Idx < 16 ? LPC_SC->PCLKSEL0 : LPC_SC->PCLKSEL1;

	assert(Idx == 1 || Idx == 2 || Idx == 22 || Idx == 23);
	return SystemCoreClock / s_Div[(sel >> (2U * (Idx & 15))) & 3U];
}

void NVIC_ClearPendingIRQ(IRQn_Type Irq)
{
	LPC_TIM_TypeDef *reg = g_LpcTimer[Irq - TIMER0_IRQn].pReg;

	// Apply reset side effects at the shared driver's reset synchronization point.
	if (reg->TCR & 2U)
	{
		reg->TC = 0;
		reg->PC = 0;
	}
	reg->IR = 0;
}

void NVIC_SetPendingIRQ(IRQn_Type Irq)
{
	(void)Irq;
}

void NVIC_SetPriority(IRQn_Type Irq, uint32_t Priority)
{
	s_Priority[Irq] = Priority;
}

void NVIC_EnableIRQ(IRQn_Type Irq)
{
	s_Enabled |= 1U << Irq;
}

void NVIC_DisableIRQ(IRQn_Type Irq)
{
	s_Enabled &= ~(1U << Irq);
}

static void Callback(TimerDev_t *pTimer, int TrigNo, void *pContext)
{
	assert(TrigNo >= 0 && TrigNo < 3);
	assert(pContext == &s_Callbacks[pTimer->DevNo]);
	s_Callbacks[pTimer->DevNo]++;
}

int main(void)
{
	Timer timers[4];
	TimerCfg_t cfg = {};
	void (* const handlers[])(void) = {
		TIMER0_IRQHandler, TIMER1_IRQHandler, TIMER2_IRQHandler, TIMER3_IRQHandler
	};

	assert(mmap((void *)LPC_SC_BASE, 0x1000, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == (void *)LPC_SC_BASE);
	assert(TimerGetHighFreqDevCount() == 4);
	assert(TimerGetLowFreqDevCount() == 0 && TimerGetHighFreqDevNo() == 0);
	for (int d = 0; d < 4; d++)
	{
		LPC_TIM_TypeDef *reg = g_LpcTimer[d].pReg;
		TimerDev_t *timer = timers[d];
		volatile uint32_t *pclk = d < 2 ? &LPC_SC->PCLKSEL0 : &LPC_SC->PCLKSEL1;
		unsigned shift = 2U * (s_Pclk[d] & 15);
		uint32_t mask = 3U << shift;
		TimerDev_t other = {};

		assert(mmap((void *)reg, 0x1000, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) == reg);
		assert(g_LpcTimer[d].IrqNo == TIMER0_IRQn + d);
		assert(g_LpcTimer[d].CounterMask == UINT32_MAX);
		assert(LpcTimerMatchReg(reg, 0) == &reg->MR0);
		assert(LpcTimerMatchReg(reg, 1) == &reg->MR1);
		assert(LpcTimerMatchReg(reg, 2) == &reg->MR2);
		assert(LpcTimerMatchReg(reg, 3) == &reg->MR3);
		cfg.DevNo = d;
		cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
		cfg.IntPrio = INT_MAX;
		for (unsigned sel = 0; sel < 4; sel++)
		{
			uint32_t before;
			uint32_t power = 0x12340001;

			*pclk = (0xA5A5A5A5U & ~mask) | (sel << shift);
			before = *pclk;
			LPC_SC->PCONP = power;
			memset((void *)reg, 0xA5, sizeof(*reg));
			// Sentinel at the PWMC offset on other LPC families: reserved here.
			*(volatile uint32_t *)((uintptr_t)reg + 0x74U) = 0x5A5A5A5A;
			assert(timers[d].Init(cfg));
			assert(*pclk == before);
			assert(LPC_SC->PCONP == (power | (1U << s_Power[d])));
			assert(timer->Freq == SystemPeriphClockGet(s_Pclk[d]));
			assert(reg->PR == 0 && reg->TC == 0 && reg->PC == 0);
			assert(reg->CTCR == 0 && reg->CCR == 0 && reg->EMR == 0);
			assert(reg->MR3 == 0x80000000U && reg->MCR == (1U << 9));
			assert(*(volatile uint32_t *)((uintptr_t)reg + 0x74U) == 0x5A5A5A5A);
			assert(s_Priority[d + 1] == 31);
			assert(!TimerInit(&other, &cfg));
		}
		cfg.IntPrio = INT_MIN;
		*pclk = (*pclk & ~mask) | (1U << shift);
		assert(timers[d].Init(cfg));
		assert(s_Priority[d + 1] == 0);
		assert(timer->SetFrequency(timer, 34000000) == 32000000 && reg->PR == 2);
		assert(timer->SetFrequency(timer, 1) == 1 && reg->PR == 95999999);
		assert(timer->SetFrequency(timer, UINT32_MAX) == SystemCoreClock);
		assert(timer->SetFrequency(timer, 1000000) == 1000000);
		assert(timer->GetMaxTrigger(timer) == 3);
		for (int trigger = 0; trigger < 3; trigger++)
		{
			assert(timer->EnableTrigger(timer, trigger, 1000000,
				TIMER_TRIG_TYPE_SINGLE, Callback, &s_Callbacks[d]) == 1000000);
		}
		assert(reg->MR0 == 1000 && reg->MR1 == 1000 && reg->MR2 == 1000);
		reg->TC = 1000;
		reg->IR = 7;
		handlers[d]();
		assert(s_Callbacks[d] == 3);
		assert(timer->FindAvailTrigger(timer) == 0);
		reg->IR = 0;
		handlers[d]();
		assert(s_Callbacks[d] == 3);
		timers[d].Reset();
		reg->TC = 0x80000000U;
		reg->IR = 8;
		handlers[d]();
		assert(reg->MR3 == 0);
		reg->TC = 0;
		reg->IR = 8;
		handlers[d]();
		assert(timer->GetTickCount(timer) == (1ULL << 32));
		timers[d].Disable();
		assert(reg->TCR == 0 && (s_Enabled & (1U << (d + 1))) == 0);
		assert(timers[d].Enable());
		assert(timer->GetTickCount(timer) == (1ULL << 32));
		assert(s_Priority[d + 1] == 0);
		cfg.bTickInt = true;
		assert(!timers[d].Init(cfg));
		cfg.bTickInt = false;
		assert(!timer->EnableExtTrigger(timer, 0, TIMER_EXTTRIG_SENSE_HIGH_TRANSITION));
	}
	puts("LPC17xx shared timer adapter: PASS");
}
