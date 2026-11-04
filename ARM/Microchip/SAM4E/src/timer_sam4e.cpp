/**-------------------------------------------------------------------------
@file	timer_sam4e.cpp

@brief	SAM4E Timer/Counter implementation for the generic IOsonata Timer.

Nine TC channels are indexed DevNo 0..8 (TC0..TC8).
All are high-frequency devices; their clock is derived from MCK.
A continuous 32-bit hardware counter is extended in software by sampling
on the mandatory overflow interrupt, even when no user event is installed.

@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "sam4e.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"

#define SAM4E_TIMER_COUNT		9
#define SAM4E_TIMER_TRIGGERS	3
#define SAM4E_TIMER_MIN_TICKS	4UL
#define SAM4E_TIMER_MAX_TICKS	0x7FFFFFFFUL

typedef struct {
	bool Active;
	uint32_t Ticks;
	uint64_t Deadline;
	TimerTrig_t Info;
} Sam4eTimerTrigger_t;

typedef struct {
	TimerDev_t *pTimer;
	TcChannel *pReg;
	IRQn_Type Irq;
	uint8_t ClockSelect;
	bool Running;
	uint32_t LastHwCount;
	uint64_t Elapsed;
	Sam4eTimerTrigger_t Trigger[SAM4E_TIMER_TRIGGERS];
} Sam4eTimerData_t;

static Sam4eTimerData_t s_Timer[SAM4E_TIMER_COUNT];

static TcChannel *Sam4eChannel(int DevNo)
{
	Sam4eTc * const pBlocks[] = { SAM4E_TC0, SAM4E_TC1, SAM4E_TC2 };

	return &pBlocks[DevNo / 3]->TC_CHANNEL[DevNo % 3];
}

static Sam4eTimerData_t *Sam4eData(TimerDev_t *pTimer)
{
	Sam4eTimerData_t *pData;

	if (pTimer == NULL || pTimer->DevNo < 0 || pTimer->DevNo >= SAM4E_TIMER_COUNT)
	{
		return NULL;
	}

	pData = &s_Timer[pTimer->DevNo];
	return pData->pTimer == pTimer ? pData : NULL;
}

static uint64_t Sam4eSampleCount(Sam4eTimerData_t *pData)
{
	uint32_t now;
	uint32_t delta;

	now = pData->pReg->TC_CV;
	delta = now - pData->LastHwCount;
	pData->Elapsed += delta;
	pData->LastHwCount = now;
	pData->pTimer->Rollover = pData->Elapsed & ~0xFFFFFFFFULL;
	pData->pTimer->LastCount = now;
	return pData->Elapsed;
}

static bool Sam4eSelectClock(uint32_t Request, uint8_t *pClock, uint32_t *pFreq)
{
	static const uint32_t dividers[] = { 2, 8, 32, 128 };
	uint32_t base;
	uint32_t target;
	uint32_t bestError;
	uint32_t freq;
	uint32_t error;
	int idx;

	base = SystemPeriphClockGet(0);
	if (base == 0)
	{
		return false;
	}

	target = Request ? Request : base / 2;
	bestError = UINT32_MAX;
	*pFreq = 0;
	*pClock = 0;
	for (idx = 0; idx < 5; idx++)
	{
		freq = idx == 4 ? 32768UL : base / dividers[idx];
		if (freq == 0)
		{
			continue;
		}
		error = freq > target ? freq - target : target - freq;
		if (error < bestError)
		{
			bestError = error;
			*pFreq = freq;
			*pClock = (uint8_t)idx;
		}
	}
	return *pFreq != 0;
}

static uint64_t Sam4eTicksForPeriod(uint32_t Freq, uint64_t nsPeriod,
									   uint32_t *pTicks)
{
	uint64_t ticks;

	if (Freq == 0 || nsPeriod == 0)
	{
		return 0;
	}

	if ((nsPeriod / 1000000000ULL) > SAM4E_TIMER_MAX_TICKS / Freq)
	{
		ticks = SAM4E_TIMER_MAX_TICKS;
	}
	else
	{
		ticks = (nsPeriod / 1000000000ULL) * Freq;
		ticks += ((nsPeriod % 1000000000ULL) * Freq + 500000000ULL)
				 / 1000000000ULL;
	}
	if (ticks < SAM4E_TIMER_MIN_TICKS)
	{
		ticks = SAM4E_TIMER_MIN_TICKS;
	}
	if (ticks > SAM4E_TIMER_MAX_TICKS)
	{
		ticks = SAM4E_TIMER_MAX_TICKS;
	}

	*pTicks = (uint32_t)ticks;
	return (ticks * 1000000000ULL + Freq / 2) / Freq;
}

static uint32_t Sam4eTriggerMask(int TrigNo)
{
	static const uint32_t s_Masks[] = { TC_IER_CPAS, TC_IER_CPBS, TC_IER_CPCS };

	return s_Masks[TrigNo];
}

static void Sam4eWriteCompare(Sam4eTimerData_t *pData, int TrigNo, uint32_t Tick)
{
	switch (TrigNo)
	{
		case 0:
			pData->pReg->TC_RA = Tick;
			break;
		case 1:
			pData->pReg->TC_RB = Tick;
			break;
		case 2:
			pData->pReg->TC_RC = Tick;
			break;
	}
}

static void Sam4eRun(Sam4eTimerData_t *pData, bool Enable)
{
	if (Enable)
	{
		(void)pData->pReg->TC_SR;
		NVIC_ClearPendingIRQ(pData->Irq);
		NVIC_EnableIRQ(pData->Irq);
		pData->pReg->TC_CCR = TC_CCR_CLKEN;
		pData->Running = true;
	}
	else
	{
		pData->pReg->TC_CCR = TC_CCR_CLKDIS;
		NVIC_DisableIRQ(pData->Irq);
		pData->Running = false;
	}
}

static void Sam4eDisable(TimerDev_t *pTimer)
{
	Sam4eTimerData_t *pData;
	uint32_t state;

	pData = Sam4eData(pTimer);
	if (pData == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	Sam4eRun(pData, false);
	EnableInterrupt(state);
}

static bool Sam4eEnable(TimerDev_t *pTimer)
{
	Sam4eTimerData_t *pData;
	uint32_t state;

	pData = Sam4eData(pTimer);
	if (pData == NULL)
	{
		return false;
	}
	state = DisableInterrupt();
	Sam4eRun(pData, true);
	EnableInterrupt(state);
	return true;
}

static void Sam4eReset(TimerDev_t *pTimer)
{
	Sam4eTimerData_t *pData;
	uint32_t state;
	bool running;
	int i;

	pData = Sam4eData(pTimer);
	if (pData == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	running = pData->Running;
	Sam4eRun(pData, false);
	pData->pReg->TC_CCR = TC_CCR_SWTRG;
	pData->LastHwCount = 0;
	pData->Elapsed = 0;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	(void)pData->pReg->TC_SR;
	for (i = 0; i < SAM4E_TIMER_TRIGGERS; i++)
	{
		if (pData->Trigger[i].Active)
		{
			pData->Trigger[i].Deadline = pData->Trigger[i].Ticks;
			Sam4eWriteCompare(pData, i, pData->Trigger[i].Ticks);
		}
	}
	if (running)
	{
		Sam4eRun(pData, true);
	}
	EnableInterrupt(state);
}

static uint64_t Sam4eCount(TimerDev_t *pTimer)
{
	Sam4eTimerData_t *pData;
	uint64_t result;
	uint32_t state;

	pData = Sam4eData(pTimer);
	if (pData == NULL)
	{
		return 0;
	}
	state = DisableInterrupt();
	result = Sam4eSampleCount(pData);
	EnableInterrupt(state);
	return result;
}

static uint32_t Sam4eSetFrequency(TimerDev_t *pTimer, uint32_t Request)
{
	Sam4eTimerData_t *pData;
	uint32_t state;
	uint32_t freq;
	uint32_t ticks[SAM4E_TIMER_TRIGGERS];
	uint8_t select;
	int i;
	bool running;

	pData = Sam4eData(pTimer);
	if (pData == NULL || !Sam4eSelectClock(Request, &select, &freq))
	{
		return 0;
	}

	for (i = 0; i < SAM4E_TIMER_TRIGGERS; i++)
	{
		if (pData->Trigger[i].Active)
		{
			if (!Sam4eTicksForPeriod(freq, pData->Trigger[i].Info.nsPeriod, &ticks[i]))
			{
				return 0;
			}
		}
	}

	state = DisableInterrupt();
	running = pData->Running;
	Sam4eRun(pData, false);
	pData->ClockSelect = select;
	pData->pReg->TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP | select;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + freq / 2) / freq;
	for (i = 0; i < SAM4E_TIMER_TRIGGERS; i++)
	{
		if (pData->Trigger[i].Active)
		{
			pData->Trigger[i].Ticks = ticks[i];
		}
	}
	pData->Running = true;
	Sam4eReset(pTimer);
	if (!running)
	{
		Sam4eRun(pData, false);
	}
	EnableInterrupt(state);
	return freq;
}

static int Sam4eMaxTrigger(TimerDev_t *pTimer)
{
	return Sam4eData(pTimer) != NULL ? SAM4E_TIMER_TRIGGERS : 0;
}

static int Sam4eFindTrigger(TimerDev_t *pTimer)
{
	Sam4eTimerData_t *pData;
	int i;

	pData = Sam4eData(pTimer);
	if (pData == NULL)
	{
		return -1;
	}
	for (i = 0; i < SAM4E_TIMER_TRIGGERS; i++)
	{
		if (!pData->Trigger[i].Active)
		{
			return i;
		}
	}
	return -1;
}

static void Sam4eDisableTrigger(TimerDev_t *pTimer, int TrigNo)
{
	Sam4eTimerData_t *pData;
	uint32_t state;

	pData = Sam4eData(pTimer);
	if (pData == NULL || TrigNo < 0 || TrigNo >= SAM4E_TIMER_TRIGGERS)
	{
		return;
	}
	state = DisableInterrupt();
	pData->Trigger[TrigNo].Active = false;
	pData->pReg->TC_IDR = Sam4eTriggerMask(TrigNo);
	EnableInterrupt(state);
}

static uint64_t Sam4eEnableTrigger(TimerDev_t *pTimer, int TrigNo, uint64_t nsPeriod,
									  TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t Handler,
									  void *pContext)
{
	Sam4eTimerData_t *pData;
	uint32_t state;
	uint32_t ticks;
	uint64_t actual;
	uint64_t count;

	pData = Sam4eData(pTimer);
	if (pData == NULL || TrigNo < 0 || TrigNo >= SAM4E_TIMER_TRIGGERS ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}
	actual = Sam4eTicksForPeriod(pTimer->Freq, nsPeriod, &ticks);
	if (actual == 0)
	{
		return 0;
	}
	state = DisableInterrupt();
	count = Sam4eSampleCount(pData);
	pData->Trigger[TrigNo].Ticks = ticks;
	pData->Trigger[TrigNo].Deadline = count + ticks;
	pData->Trigger[TrigNo].Info = { Type, actual, Handler, pContext };
	pData->Trigger[TrigNo].Active = true;
	Sam4eWriteCompare(pData, TrigNo, (uint32_t)pData->Trigger[TrigNo].Deadline);
	pData->pReg->TC_IER = Sam4eTriggerMask(TrigNo);
	EnableInterrupt(state);
	return actual;
}

static void Sam4eDisableExtTrigger(TimerDev_t *pTimer)
{
	(void)pTimer;
}

static bool Sam4eEnableExtTrigger(TimerDev_t *pTimer, int DevNo, TIMER_EXTTRIG_SENSE Sense)
{
	(void)pTimer;
	(void)DevNo;
	(void)Sense;
	return false;
}

static void Sam4eTimerIRQ(int DevNo)
{
	static const uint32_t s_Flags[] = { TC_SR_CPAS, TC_SR_CPBS, TC_SR_CPCS };
	Sam4eTimerData_t *pData;
	TimerTrig_t info;
	uint32_t status;
	uint64_t count;
	int i;

	pData = &s_Timer[DevNo];
	if (pData->pTimer == NULL || !pData->Running)
	{
		return;
	}
	status = pData->pReg->TC_SR;
	count = Sam4eSampleCount(pData);
	if ((status & TC_SR_COVFS) && pData->pTimer->EvtHandler != NULL)
	{
		pData->pTimer->EvtHandler(pData->pTimer, TIMER_EVT_COUNTER_OVR);
	}
	for (i = 0; i < SAM4E_TIMER_TRIGGERS; i++)
	{
		if ((status & s_Flags[i]) == 0 || !pData->Trigger[i].Active ||
			count < pData->Trigger[i].Deadline)
		{
			continue;
		}
		info = pData->Trigger[i].Info;
		if (info.Type == TIMER_TRIG_TYPE_SINGLE)
		{
			pData->Trigger[i].Active = false;
			pData->pReg->TC_IDR = Sam4eTriggerMask(i);
		}
		else
		{
			pData->Trigger[i].Deadline +=
				((count - pData->Trigger[i].Deadline) / pData->Trigger[i].Ticks + 1)
				* pData->Trigger[i].Ticks;
			Sam4eWriteCompare(pData, i, (uint32_t)pData->Trigger[i].Deadline);
		}
		if (info.Handler != NULL)
		{
			info.Handler(pData->pTimer, i, info.pContext);
		}
		else if (pData->pTimer->EvtHandler != NULL)
		{
			pData->pTimer->EvtHandler(pData->pTimer, TIMER_EVT_TRIGGER(i));
		}
	}
}

extern "C" void TC0_Handler(void) { Sam4eTimerIRQ(0); }
extern "C" void TC1_Handler(void) { Sam4eTimerIRQ(1); }
extern "C" void TC2_Handler(void) { Sam4eTimerIRQ(2); }
extern "C" void TC3_Handler(void) { Sam4eTimerIRQ(3); }
extern "C" void TC4_Handler(void) { Sam4eTimerIRQ(4); }
extern "C" void TC5_Handler(void) { Sam4eTimerIRQ(5); }
extern "C" void TC6_Handler(void) { Sam4eTimerIRQ(6); }
extern "C" void TC7_Handler(void) { Sam4eTimerIRQ(7); }
extern "C" void TC8_Handler(void) { Sam4eTimerIRQ(8); }

bool TimerInit(TimerDev_t *pTimer, const TimerCfg_t *pCfg)
{
	Sam4eTimerData_t *pData;
	uint32_t state;
	uint32_t frequency;
	uint8_t select;
	int i;

	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 ||
		pCfg->DevNo >= SAM4E_TIMER_COUNT || pCfg->bTickInt ||
		pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT ||
		pCfg->IntPrio < 0 || pCfg->IntPrio >= (1 << __NVIC_PRIO_BITS) ||
		!Sam4eSelectClock(pCfg->Freq, &select, &frequency))
	{
		return false;
	}
	state = DisableInterrupt();
	pData = &s_Timer[pCfg->DevNo];
	for (i = 0; i < SAM4E_TIMER_COUNT; i++)
	{
		if (s_Timer[i].pTimer == pTimer)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	if (pData->pTimer != NULL)
	{
		EnableInterrupt(state);
		return false;
	}
	pData->pTimer = pTimer;
	pData->pReg = Sam4eChannel(pCfg->DevNo);
	pData->Irq = (IRQn_Type)(TC0_IRQn + pCfg->DevNo);
	pData->ClockSelect = select;
	pData->Running = false;
	pData->LastHwCount = 0;
	pData->Elapsed = 0;

	pTimer->DevNo = pCfg->DevNo;
	pTimer->Freq = frequency;
	pTimer->nsPeriod = (1000000000ULL + frequency / 2) / frequency;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Disable = Sam4eDisable;
	pTimer->Enable = Sam4eEnable;
	pTimer->Reset = Sam4eReset;
	pTimer->GetTickCount = Sam4eCount;
	pTimer->SetFrequency = Sam4eSetFrequency;
	pTimer->GetMaxTrigger = Sam4eMaxTrigger;
	pTimer->FindAvailTrigger = Sam4eFindTrigger;
	pTimer->DisableTrigger = Sam4eDisableTrigger;
	pTimer->EnableTrigger = Sam4eEnableTrigger;
	pTimer->DisableExtTrigger = Sam4eDisableExtTrigger;
	pTimer->EnableExtTrigger = Sam4eEnableExtTrigger;

	// Every SAM4E TC channel uses one of peripheral IDs 21 through 29.
	SAM4E_PMC->PMC_PCER0 = 1UL << (ID_TC0 + pCfg->DevNo);
	pData->pReg->TC_CCR = TC_CCR_CLKDIS;
	pData->pReg->TC_IDR = 0xFFFFFFFFUL;
	pData->pReg->TC_CMR = TC_CMR_WAVE | TC_CMR_WAVSEL_UP | select;
	pData->pReg->TC_CCR = TC_CCR_SWTRG;
	(void)pData->pReg->TC_SR;
	pData->pReg->TC_IER = TC_IER_COVFS;
	NVIC_ClearPendingIRQ(pData->Irq);
	NVIC_SetPriority(pData->Irq, pCfg->IntPrio);
	Sam4eRun(pData, true);
	EnableInterrupt(state);
	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevCount(void)
{
	return SAM4E_TIMER_COUNT;
}

int TimerGetHighFreqDevNo(void)
{
	return 0;
}
