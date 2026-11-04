/**-------------------------------------------------------------------------
@file	timer_esp32c6.cpp

@brief	ESP32-C6 TG0/TG1 hardware timers for the generic Timer API.

Two 54-bit counters, one compare/alarm per Timer Group, no heap.
PCR selects PLL 80 MHz for the timer clock and enables it explicitly.
The CPU receives TG0/T0 through source 51 / INT 8 and TG1/T0
through source 54 / INT 9. C6 core-local interrupt slots 0,3,4,7
are deliberately avoided.

@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stddef.h>

#include "esp32xx.h"
#include "esp32xx_irq.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"

#define ESP32C6_TIMER_COUNT		2
#define ESP32C6_TIMER_APB_HZ		80000000UL
#define ESP32C6_TIMER_MIN_DIV		2UL
#define ESP32C6_TIMER_MAX_DIV		65535UL
#define ESP32C6_TIMER_MIN_TICKS	4ULL
#define ESP32C6_TIMER_MAX_TICKS	((1ULL << 54) - 1ULL)

#define ESP32C6_T0_CONFIG		0x000UL
#define ESP32C6_T0_LO			0x004UL
#define ESP32C6_T0_HI			0x008UL
#define ESP32C6_T0_UPDATE		0x00CUL
#define ESP32C6_T0_ALARM_LO		0x010UL
#define ESP32C6_T0_ALARM_HI		0x014UL
#define ESP32C6_T0_LOAD_LO		0x018UL
#define ESP32C6_T0_LOAD_HI		0x01CUL
#define ESP32C6_T0_LOAD		0x020UL
#define ESP32C6_T0_INT_ENA		0x070UL
#define ESP32C6_T0_INT_ST		0x078UL
#define ESP32C6_T0_INT_CLR		0x07CUL

#define ESP32C6_T0_ALARM_EN		(1UL << 10)
#define ESP32C6_T0_DIVCNT_RST	(1UL << 12)
#define ESP32C6_T0_DIV_SHIFT	13
#define ESP32C6_T0_DIV_MASK		(0xFFFFUL << ESP32C6_T0_DIV_SHIFT)
#define ESP32C6_T0_INCREASE		(1UL << 30)
#define ESP32C6_T0_ENABLE		(1UL << 31)
#define ESP32C6_T0_UPDATE_TRIG	(1UL << 31)
#define ESP32C6_T0_INT_BIT		1UL

// Verified against Espressif ESP32-C3 interrupt source enum (IDF 5.5):
// TG0/T0 = 32 and TG1/T0 = 34.
#define ESP32C6_TG0_T0_SOURCE	51UL
#define ESP32C6_TG1_T0_SOURCE	54UL
#define ESP32C6_TG0_T0_CPUINT	8UL
#define ESP32C6_TG1_T0_CPUINT	9UL

typedef struct {
	TimerDev_t *pTimer;
	uint32_t Base;
	uint32_t Divider;
	uint32_t IrqSource;
	uint32_t CpuInt;
	bool Running;
	bool TriggerActive;
	uint64_t TriggerTicks;
	uint64_t Deadline;
	TimerTrig_t TriggerInfo;
} Esp32c6TimerData_t;

static Esp32c6TimerData_t s_Timer[ESP32C6_TIMER_COUNT];

static volatile uint32_t &Esp32c6Reg(uint32_t Base, uint32_t Offset)
{
	return *(volatile uint32_t *)(uintptr_t)(Base + Offset);
}

// C6 PCR controls each Timer Group bus clock and a separate timer source.
// Choose PLL 80 MHz (CLK_SEL=1) before calculating period dividers.
static void Esp32c6EnableTimerClock(int DevNo)
{
	uint32_t cfgBase;
	uint32_t timerClock;
	uint32_t config;

	cfgBase = ESP32C6_PCR_BASE + (DevNo == 0 ? 0x3CUL : 0x48UL);
	timerClock = ESP32C6_PCR_BASE + (DevNo == 0 ? 0x40UL : 0x4CUL);
	Esp32c6Reg(cfgBase, 0) |= 1UL;
	config = Esp32c6Reg(timerClock, 0);
	config &= ~(3UL << 20);
	config |= (1UL << 20) | (1UL << 22);
	Esp32c6Reg(timerClock, 0) = config;
}


static Esp32c6TimerData_t *Esp32c6Data(TimerDev_t *pTimer)
{
	Esp32c6TimerData_t *pData;

	if (pTimer == NULL || pTimer->DevNo < 0 ||
		pTimer->DevNo >= ESP32C6_TIMER_COUNT)
	{
		return NULL;
	}
	pData = &s_Timer[pTimer->DevNo];
	return pData->pTimer == pTimer ? pData : NULL;
}

static uint64_t Esp32c6ReadCount(Esp32c6TimerData_t *pData)
{
	uint32_t high;
	uint32_t low;

	Esp32c6Reg(pData->Base, ESP32C6_T0_UPDATE) = ESP32C6_T0_UPDATE_TRIG;
	while (Esp32c6Reg(pData->Base, ESP32C6_T0_UPDATE) & ESP32C6_T0_UPDATE_TRIG)
	{
	}
	low = Esp32c6Reg(pData->Base, ESP32C6_T0_LO);
	high = Esp32c6Reg(pData->Base, ESP32C6_T0_HI) & 0x003FFFFFUL;
	pData->pTimer->LastCount = low;
	pData->pTimer->Rollover = (uint64_t)high << 32;
	return ((uint64_t)high << 32) | low;
}

static uint32_t Esp32c6ClockSelect(uint32_t Requested, uint32_t *pDivider)
{
	uint32_t div;
	uint32_t target;

	target = Requested == 0 ? ESP32C6_TIMER_APB_HZ / 2 : Requested;
	div = (ESP32C6_TIMER_APB_HZ + target / 2) / target;
	if (div < ESP32C6_TIMER_MIN_DIV)
	{
		div = ESP32C6_TIMER_MIN_DIV;
	}
	if (div > ESP32C6_TIMER_MAX_DIV)
	{
		div = ESP32C6_TIMER_MAX_DIV;
	}
	*pDivider = div;
	return ESP32C6_TIMER_APB_HZ / div;
}

static uint64_t Esp32c6PeriodToTicks(uint32_t Frequency, uint64_t nsPeriod,
										uint64_t *pTicks)
{
	uint64_t ticks;

	if (Frequency == 0 || nsPeriod == 0)
	{
		return 0;
	}
	if ((nsPeriod / 1000000000ULL) >
		ESP32C6_TIMER_MAX_TICKS / Frequency)
	{
		ticks = ESP32C6_TIMER_MAX_TICKS;
	}
	else
	{
		ticks = (nsPeriod / 1000000000ULL) * Frequency;
		ticks += ((nsPeriod % 1000000000ULL) * Frequency + 500000000ULL)
				 / 1000000000ULL;
	}
	if (ticks < ESP32C6_TIMER_MIN_TICKS)
	{
		ticks = ESP32C6_TIMER_MIN_TICKS;
	}
	if (ticks > ESP32C6_TIMER_MAX_TICKS)
	{
		ticks = ESP32C6_TIMER_MAX_TICKS;
	}
	*pTicks = ticks;
	return (ticks / Frequency) * 1000000000ULL +
		   ((ticks % Frequency) * 1000000000ULL + Frequency / 2) / Frequency;
}

static void Esp32c6ArmAlarm(Esp32c6TimerData_t *pData)
{
	uint32_t config;

	Esp32c6Reg(pData->Base, ESP32C6_T0_ALARM_LO) = (uint32_t)pData->Deadline;
	Esp32c6Reg(pData->Base, ESP32C6_T0_ALARM_HI) =
		(uint32_t)(pData->Deadline >> 32) & 0x003FFFFFUL;
	config = Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG);
	Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) =
		config | ESP32C6_T0_ALARM_EN;
}

static void Esp32c6Run(Esp32c6TimerData_t *pData, bool Enable)
{
	uint32_t config;

	config = Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG);
	if (Enable)
	{
		Esp32c6Reg(pData->Base, ESP32C6_T0_INT_CLR) = ESP32C6_T0_INT_BIT;
		Esp32c6Reg(pData->Base, ESP32C6_T0_INT_ENA) |=
			ESP32C6_T0_INT_BIT;
		Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) =
			config | ESP32C6_T0_ENABLE;
		pData->Running = true;
		if (pData->TriggerActive)
		{
			Esp32c6ArmAlarm(pData);
		}
	}
	else
	{
		Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) =
			config & ~(ESP32C6_T0_ENABLE | ESP32C6_T0_ALARM_EN);
		Esp32c6Reg(pData->Base, ESP32C6_T0_INT_ENA) &=
			~ESP32C6_T0_INT_BIT;
		pData->Running = false;
	}
}

static void Esp32c6Disable(TimerDev_t *pTimer)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	Esp32c6Run(pData, false);
	EnableInterrupt(state);
}

static bool Esp32c6Enable(TimerDev_t *pTimer)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL)
	{
		return false;
	}
	state = DisableInterrupt();
	Esp32c6Run(pData, true);
	EnableInterrupt(state);
	return true;
}

static void Esp32c6ResetCounter(Esp32c6TimerData_t *pData)
{
	bool running;

	running = pData->Running;
	Esp32c6Run(pData, false);
	Esp32c6Reg(pData->Base, ESP32C6_T0_LOAD_LO) = 0;
	Esp32c6Reg(pData->Base, ESP32C6_T0_LOAD_HI) = 0;
	Esp32c6Reg(pData->Base, ESP32C6_T0_LOAD) = 1;
	pData->pTimer->Rollover = 0;
	pData->pTimer->LastCount = 0;
	if (pData->TriggerActive)
	{
		pData->Deadline = pData->TriggerTicks;
	}
	if (running)
	{
		Esp32c6Run(pData, true);
	}
}

static void Esp32c6Reset(TimerDev_t *pTimer)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	Esp32c6ResetCounter(pData);
	EnableInterrupt(state);
}

static uint64_t Esp32c6Count(TimerDev_t *pTimer)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;
	uint64_t count;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL)
	{
		return 0;
	}
	state = DisableInterrupt();
	count = Esp32c6ReadCount(pData);
	EnableInterrupt(state);
	return count;
}

static uint32_t Esp32c6SetFrequency(TimerDev_t *pTimer, uint32_t Request)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;
	uint32_t config;
	uint32_t divider;
	uint32_t frequency;
	uint64_t ticks;
	bool running;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL)
	{
		return 0;
	}
	frequency = Esp32c6ClockSelect(Request, &divider);
	if (pData->TriggerActive)
	{
		if (Esp32c6PeriodToTicks(frequency, pData->TriggerInfo.nsPeriod,
										  &ticks) == 0)
		{
			return 0;
		}
	}
	state = DisableInterrupt();
	running = pData->Running;
	Esp32c6Run(pData, false);
	config = Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG);
	config &= ~(ESP32C6_T0_DIV_MASK | ESP32C6_T0_ALARM_EN);
	config |= divider << ESP32C6_T0_DIV_SHIFT;
	Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) = config;
	Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) =
		config | ESP32C6_T0_DIVCNT_RST;
	pData->Divider = divider;
	pTimer->Freq = frequency;
	pTimer->nsPeriod = (1000000000ULL + frequency / 2) / frequency;
	if (pData->TriggerActive)
	{
		pData->TriggerTicks = ticks;
	}
	pData->Running = true;
	Esp32c6ResetCounter(pData);
	if (!running)
	{
		Esp32c6Run(pData, false);
	}
	EnableInterrupt(state);
	return frequency;
}

static int Esp32c6MaxTrigger(TimerDev_t *pTimer)
{
	return Esp32c6Data(pTimer) != NULL ? 1 : 0;
}

static int Esp32c6FindTrigger(TimerDev_t *pTimer)
{
	Esp32c6TimerData_t *pData;

	pData = Esp32c6Data(pTimer);
	return pData != NULL && !pData->TriggerActive ? 0 : -1;
}

static void Esp32c6DisableTrigger(TimerDev_t *pTimer, int TrigNo)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;
	uint32_t config;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL || TrigNo != 0)
	{
		return;
	}
	state = DisableInterrupt();
	pData->TriggerActive = false;
	config = Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG);
	Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) =
		config & ~ESP32C6_T0_ALARM_EN;
	EnableInterrupt(state);
}

static uint64_t Esp32c6EnableTrigger(TimerDev_t *pTimer, int TrigNo,
									uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
									TimerTrigEvtHandler_t Handler, void *pContext)
{
	Esp32c6TimerData_t *pData;
	uint64_t ticks;
	uint64_t actual;
	uint32_t state;

	pData = Esp32c6Data(pTimer);
	if (pData == NULL || TrigNo != 0 ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}
	actual = Esp32c6PeriodToTicks(pTimer->Freq, nsPeriod, &ticks);
	if (actual == 0)
	{
		return 0;
	}
	state = DisableInterrupt();
	pData->TriggerTicks = ticks;
	pData->Deadline = Esp32c6ReadCount(pData) + ticks;
	pData->TriggerInfo = { Type, actual, Handler, pContext };
	pData->TriggerActive = true;
	Esp32c6ArmAlarm(pData);
	EnableInterrupt(state);
	return actual;
}

static void Esp32c6DisableExtTrigger(TimerDev_t *pTimer)
{
	(void)pTimer;
}

static bool Esp32c6EnableExtTrigger(TimerDev_t *pTimer, int DevNo,
									TIMER_EXTTRIG_SENSE Sense)
{
	(void)pTimer;
	(void)DevNo;
	(void)Sense;
	return false;
}

static void Esp32c6IRQ(int DevNo)
{
	Esp32c6TimerData_t *pData;
	TimerTrig_t info;
	uint64_t now;

	pData = &s_Timer[DevNo];
	Esp32c6Reg(pData->Base, ESP32C6_T0_INT_CLR) = ESP32C6_T0_INT_BIT;
	if (pData->pTimer == NULL || !pData->Running || !pData->TriggerActive)
	{
		return;
	}
	now = Esp32c6ReadCount(pData);
	if (now < pData->Deadline)
	{
		Esp32c6ArmAlarm(pData);
		return;
	}
	info = pData->TriggerInfo;
	if (info.Type == TIMER_TRIG_TYPE_SINGLE)
	{
		pData->TriggerActive = false;
	}
	else
	{
		pData->Deadline +=
			((now - pData->Deadline) / pData->TriggerTicks + 1) *
			pData->TriggerTicks;
		Esp32c6ArmAlarm(pData);
	}
	if (info.Handler != NULL)
	{
		info.Handler(pData->pTimer, 0, info.pContext);
	}
	else if (pData->pTimer->EvtHandler != NULL)
	{
		pData->pTimer->EvtHandler(pData->pTimer, TIMER_EVT_TRIGGER(0));
	}
}

extern "C" void Esp32c6Tg0Handler(void) { Esp32c6IRQ(0); }
extern "C" void Esp32c6Tg1Handler(void) { Esp32c6IRQ(1); }

bool TimerInit(TimerDev_t *pTimer, const TimerCfg_t *pCfg)
{
	Esp32c6TimerData_t *pData;
	uint32_t state;
	uint32_t divider;
	uint32_t frequency;
	uint32_t config;
	uint32_t source;
	uint32_t cpuint;
	uint32_t prio;
	int i;

	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 ||
		pCfg->DevNo >= ESP32C6_TIMER_COUNT || pCfg->bTickInt ||
		pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT)
	{
		return false;
	}
	frequency = Esp32c6ClockSelect(pCfg->Freq, &divider);
	state = DisableInterrupt();
	pData = &s_Timer[pCfg->DevNo];
	for (i = 0; i < ESP32C6_TIMER_COUNT; i++)
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

	pData->Base = pCfg->DevNo == 0 ? ESP32C6_TIMG0_BASE : ESP32C6_TIMG1_BASE;
	pData->pTimer = pTimer;
	pData->Divider = divider;
	pData->Running = false;
	source = pCfg->DevNo == 0 ? ESP32C6_TG0_T0_SOURCE : ESP32C6_TG1_T0_SOURCE;
	cpuint = pCfg->DevNo == 0 ? ESP32C6_TG0_T0_CPUINT : ESP32C6_TG1_T0_CPUINT;
	pData->IrqSource = source;
	pData->CpuInt = cpuint;
	prio = pCfg->IntPrio > 0 ? (uint32_t)pCfg->IntPrio : 1UL;
	if (prio > 15UL)
	{
		prio = 15UL;
	}

	pTimer->DevNo = pCfg->DevNo;
	pTimer->Freq = frequency;
	pTimer->nsPeriod = (1000000000ULL + frequency / 2) / frequency;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Disable = Esp32c6Disable;
	pTimer->Enable = Esp32c6Enable;
	pTimer->Reset = Esp32c6Reset;
	pTimer->GetTickCount = Esp32c6Count;
	pTimer->SetFrequency = Esp32c6SetFrequency;
	pTimer->GetMaxTrigger = Esp32c6MaxTrigger;
	pTimer->FindAvailTrigger = Esp32c6FindTrigger;
	pTimer->DisableTrigger = Esp32c6DisableTrigger;
	pTimer->EnableTrigger = Esp32c6EnableTrigger;
	pTimer->DisableExtTrigger = Esp32c6DisableExtTrigger;
	pTimer->EnableExtTrigger = Esp32c6EnableExtTrigger;

	Esp32c6EnableTimerClock(pCfg->DevNo);
	config = ESP32C6_T0_INCREASE |
		(divider << ESP32C6_T0_DIV_SHIFT);
	Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) = config;
	Esp32c6Reg(pData->Base, ESP32C6_T0_CONFIG) =
		config | ESP32C6_T0_DIVCNT_RST;
	Esp32c6Reg(pData->Base, ESP32C6_T0_LOAD_LO) = 0;
	Esp32c6Reg(pData->Base, ESP32C6_T0_LOAD_HI) = 0;
	Esp32c6Reg(pData->Base, ESP32C6_T0_LOAD) = 1;
	Esp32c6Reg(pData->Base, ESP32C6_T0_INT_CLR) = ESP32C6_T0_INT_BIT;
	Esp32c6Reg(pData->Base, ESP32C6_T0_INT_ENA) &=
		~ESP32C6_T0_INT_BIT;

	if (!Esp32EnableSourceIrq(source, cpuint, prio,
		pCfg->DevNo == 0 ? Esp32c6Tg0Handler : Esp32c6Tg1Handler))
	{
		pData->pTimer = NULL;
		EnableInterrupt(state);
		return false;
	}
	Esp32c6Run(pData, true);
	EnableInterrupt(state);
	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevCount(void)
{
	return ESP32C6_TIMER_COUNT;
}

int TimerGetHighFreqDevNo(void)
{
	return 0;
}
