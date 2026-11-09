/**-------------------------------------------------------------------------
@file	timer_esp32.cpp

@brief	ESP32 C-series Timer Group implementation for IOsonata Timer.

@author	IOsonata contributors
@date	Nov. 8, 2026

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
#include <stdint.h>
#include <stddef.h>

#include "esp32xx.h"
#include "esp32xx_irq.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"

// Timer Group source IDs and clock selectors: ESP-IDF v5.5.1 soc/hal headers.
#if defined(ESP32C2)
#define ESP32_TIMG_COUNT		1
#define ESP32_TIMER_APB_HZ		40000000UL
static const uint32_t s_TimerBases[] = { ESP32_TIMG0_BASE };
static const uint32_t s_TimerSources[] = { 23 };
#else
#define ESP32_TIMG_COUNT		2
#define ESP32_TIMER_APB_HZ		80000000UL
static const uint32_t s_TimerBases[] = { ESP32_TIMG0_BASE, ESP32_TIMG1_BASE };
#if defined(ESP32C3)
static const uint32_t s_TimerSources[] = { 32, 34 };
#elif defined(ESP32C5)
static const uint32_t s_TimerSources[] = { 57, 59 };
#define ESP32_TIMER_PCR_OFFSET	0x54UL
#define ESP32_TIMER_PCR_SEL		2UL
#elif defined(ESP32C6)
static const uint32_t s_TimerSources[] = { 51, 54 };
#define ESP32_TIMER_PCR_OFFSET	0x3CUL
#define ESP32_TIMER_PCR_SEL		1UL
#elif defined(ESP32C61)
static const uint32_t s_TimerSources[] = { 46, 49 };
#define ESP32_TIMER_PCR_OFFSET	0x40UL
#define ESP32_TIMER_PCR_SEL		2UL
#else
#error "Timer Group implementation requires ESP32C2/C3/C5/C6/C61"
#endif
#endif
#define ESP32_TIMER_MIN_DIV		2UL
#define ESP32_TIMER_MAX_DIV		65536UL
#define ESP32_TIMER_MIN_TICKS	4ULL
#define ESP32_TIMER_MAX_TICKS	((1ULL << 54) - 1ULL)

#define ESP32_T0_CONFIG		0x000UL
#define ESP32_T0_LO			0x004UL
#define ESP32_T0_HI			0x008UL
#define ESP32_T0_UPDATE		0x00CUL
#define ESP32_T0_ALARM_LO		0x010UL
#define ESP32_T0_ALARM_HI		0x014UL
#define ESP32_T0_LOAD_LO		0x018UL
#define ESP32_T0_LOAD_HI		0x01CUL
#define ESP32_T0_LOAD		0x020UL
#define ESP32_T0_INT_ENA		0x070UL
#define ESP32_T0_INT_ST		0x078UL
#define ESP32_T0_INT_CLR		0x07CUL

#define ESP32_T0_ALARM_EN		(1UL << 10)
#define ESP32_T0_DIVCNT_RST	(1UL << 12)
#define ESP32_T0_DIV_SHIFT	13
#define ESP32_T0_DIV_MASK		(0xFFFFUL << ESP32_T0_DIV_SHIFT)
#define ESP32_T0_INCREASE		(1UL << 30)
#define ESP32_T0_ENABLE		(1UL << 31)
#define ESP32_T0_UPDATE_TRIG	(1UL << 31)
#define ESP32_T0_INT_BIT		1UL

// CPU inputs 10/11 are available on all supported chips and do not overlap
// the UART or GPIO allocations. Input 6 is the disabled route on most chips.
#define ESP32_TIMER_CPUINT		10UL

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
} Esp32TimerData_t;

static Esp32TimerData_t s_Timer[ESP32_TIMG_COUNT];

#define Esp32Reg(Base, Offset) ESP32_REG32((Base) + (Offset))

static void Esp32EnableTimerClock(int DevNo)
{
#if defined(ESP32C2) || defined(ESP32C3)
	uint32_t mask;

	mask = 1UL << (DevNo == 0 ? 13 : 15);
	ESP32_REG32(ESP32_SYSTEM_BASE + 0x10UL) |= mask;
	ESP32_REG32(ESP32_SYSTEM_BASE + 0x18UL) &= ~mask;
	Esp32Reg(s_TimerBases[DevNo], 0xFCUL) |= 1UL << 30;
#else
	uint32_t base;
	uint32_t config;

	base = ESP32_PCR_BASE + ESP32_TIMER_PCR_OFFSET + (uint32_t)DevNo * 12UL;
	config = ESP32_REG32(base);
	ESP32_REG32(base) = (config | 1UL) & ~2UL;
	config = ESP32_REG32(base + 4UL);
	config &= ~(3UL << 20);
	ESP32_REG32(base + 4UL) = config | (ESP32_TIMER_PCR_SEL << 20) | (1UL << 22);
#endif
}

static Esp32TimerData_t *Esp32Data(TimerDev_t *pTimer)
{
	Esp32TimerData_t *pData;

	if (pTimer == NULL || pTimer->DevNo < 0 ||
		pTimer->DevNo >= ESP32_TIMG_COUNT)
	{
		return NULL;
	}
	pData = &s_Timer[pTimer->DevNo];
	return pData->pTimer == pTimer ? pData : NULL;
}

static uint64_t Esp32ReadCount(Esp32TimerData_t *pData)
{
	uint32_t high;
	uint32_t low;

	Esp32Reg(pData->Base, ESP32_T0_UPDATE) = ESP32_T0_UPDATE_TRIG;
	while (Esp32Reg(pData->Base, ESP32_T0_UPDATE) & ESP32_T0_UPDATE_TRIG)
	{
	}
	low = Esp32Reg(pData->Base, ESP32_T0_LO);
	high = Esp32Reg(pData->Base, ESP32_T0_HI) & 0x003FFFFFUL;
	pData->pTimer->LastCount = low;
	pData->pTimer->Rollover = (uint64_t)high << 32;
	return ((uint64_t)high << 32) | low;
}

static uint32_t Esp32ClockSelect(uint32_t Requested, uint32_t *pDivider)
{
	uint32_t div;
	uint32_t target;
	uint32_t upper;
	uint32_t lower;

	target = Requested == 0 ? ESP32_TIMER_APB_HZ / 2 : Requested;
	div = ESP32_TIMER_APB_HZ / target;
	if (div < ESP32_TIMER_MIN_DIV)
	{
		div = ESP32_TIMER_MIN_DIV;
	}
	if (div > ESP32_TIMER_MAX_DIV)
	{
		div = ESP32_TIMER_MAX_DIV;
	}
	if (div < ESP32_TIMER_MAX_DIV)
	{
		upper = ESP32_TIMER_APB_HZ / div;
		lower = ESP32_TIMER_APB_HZ / (div + 1);
		if (upper > target && target >= lower && upper - target > target - lower)
		{
			div++;
		}
	}
	*pDivider = div;
	return ESP32_TIMER_APB_HZ / div;
}

static uint64_t Esp32PeriodToTicks(uint32_t Frequency, uint64_t nsPeriod,
										uint64_t *pTicks)
{
	uint64_t ticks;
	uint64_t maximum;

	if (Frequency == 0 || nsPeriod == 0)
	{
		return 0;
	}
	maximum = (UINT64_MAX / 1000000000ULL) * Frequency;
	maximum += ((UINT64_MAX % 1000000000ULL) * Frequency) / 1000000000ULL;
	if (maximum > ESP32_TIMER_MAX_TICKS)
	{
		maximum = ESP32_TIMER_MAX_TICKS;
	}
	if ((nsPeriod / 1000000000ULL) >
		ESP32_TIMER_MAX_TICKS / Frequency)
	{
		ticks = ESP32_TIMER_MAX_TICKS;
	}
	else
	{
		ticks = (nsPeriod / 1000000000ULL) * Frequency;
		ticks += ((nsPeriod % 1000000000ULL) * Frequency + 500000000ULL)
				 / 1000000000ULL;
	}
	if (ticks < ESP32_TIMER_MIN_TICKS)
	{
		ticks = ESP32_TIMER_MIN_TICKS;
	}
	if (ticks > maximum)
	{
		ticks = maximum;
	}
	*pTicks = ticks;
	return (ticks / Frequency) * 1000000000ULL +
		   ((ticks % Frequency) * 1000000000ULL + Frequency / 2) / Frequency;
}

static void Esp32ArmAlarm(Esp32TimerData_t *pData)
{
	uint32_t config;

	Esp32Reg(pData->Base, ESP32_T0_ALARM_LO) = (uint32_t)pData->Deadline;
	Esp32Reg(pData->Base, ESP32_T0_ALARM_HI) =
		(uint32_t)(pData->Deadline >> 32) & 0x003FFFFFUL;
	config = Esp32Reg(pData->Base, ESP32_T0_CONFIG);
	Esp32Reg(pData->Base, ESP32_T0_CONFIG) =
		config | ESP32_T0_ALARM_EN;
}

static void Esp32Run(Esp32TimerData_t *pData, bool Enable)
{
	uint32_t config;

	config = Esp32Reg(pData->Base, ESP32_T0_CONFIG);
	if (Enable)
	{
		Esp32Reg(pData->Base, ESP32_T0_INT_CLR) = ESP32_T0_INT_BIT;
		Esp32Reg(pData->Base, ESP32_T0_INT_ENA) |=
			ESP32_T0_INT_BIT;
		Esp32Reg(pData->Base, ESP32_T0_CONFIG) =
			config | ESP32_T0_ENABLE;
		pData->Running = true;
		if (pData->TriggerActive)
		{
			Esp32ArmAlarm(pData);
		}
	}
	else
	{
		Esp32Reg(pData->Base, ESP32_T0_CONFIG) =
			config & ~(ESP32_T0_ENABLE | ESP32_T0_ALARM_EN);
		Esp32Reg(pData->Base, ESP32_T0_INT_ENA) &=
			~ESP32_T0_INT_BIT;
		pData->Running = false;
	}
}

static void Esp32Disable(TimerDev_t *pTimer)
{
	Esp32TimerData_t *pData;
	uint32_t state;

	pData = Esp32Data(pTimer);
	if (pData == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	Esp32Run(pData, false);
	EnableInterrupt(state);
}

static bool Esp32Enable(TimerDev_t *pTimer)
{
	Esp32TimerData_t *pData;
	uint32_t state;

	pData = Esp32Data(pTimer);
	if (pData == NULL)
	{
		return false;
	}
	state = DisableInterrupt();
	Esp32Run(pData, true);
	EnableInterrupt(state);
	return true;
}

static void Esp32ResetCounter(Esp32TimerData_t *pData)
{
	bool running;

	running = pData->Running;
	Esp32Run(pData, false);
	Esp32Reg(pData->Base, ESP32_T0_LOAD_LO) = 0;
	Esp32Reg(pData->Base, ESP32_T0_LOAD_HI) = 0;
	Esp32Reg(pData->Base, ESP32_T0_LOAD) = 1;
	pData->pTimer->Rollover = 0;
	pData->pTimer->LastCount = 0;
	if (pData->TriggerActive)
	{
		pData->Deadline = pData->TriggerTicks;
	}
	if (running)
	{
		Esp32Run(pData, true);
	}
}

static void Esp32Reset(TimerDev_t *pTimer)
{
	Esp32TimerData_t *pData;
	uint32_t state;

	pData = Esp32Data(pTimer);
	if (pData == NULL)
	{
		return;
	}
	state = DisableInterrupt();
	Esp32ResetCounter(pData);
	EnableInterrupt(state);
}

static uint64_t Esp32Count(TimerDev_t *pTimer)
{
	Esp32TimerData_t *pData;
	uint32_t state;
	uint64_t count;

	pData = Esp32Data(pTimer);
	if (pData == NULL)
	{
		return 0;
	}
	state = DisableInterrupt();
	count = Esp32ReadCount(pData);
	EnableInterrupt(state);
	return count;
}

static uint32_t Esp32SetFrequency(TimerDev_t *pTimer, uint32_t Request)
{
	Esp32TimerData_t *pData;
	uint32_t state;
	uint32_t config;
	uint32_t divider;
	uint32_t frequency;
	uint64_t ticks;
	bool running;

	pData = Esp32Data(pTimer);
	if (pData == NULL)
	{
		return 0;
	}
	frequency = Esp32ClockSelect(Request, &divider);
	if (pData->TriggerActive)
	{
		if (Esp32PeriodToTicks(frequency, pData->TriggerInfo.nsPeriod,
										  &ticks) == 0)
		{
			return 0;
		}
	}
	state = DisableInterrupt();
	running = pData->Running;
	Esp32Run(pData, false);
	config = Esp32Reg(pData->Base, ESP32_T0_CONFIG);
	config &= ~(ESP32_T0_DIV_MASK | ESP32_T0_ALARM_EN);
	config |= (divider & 0xFFFFUL) << ESP32_T0_DIV_SHIFT;
	Esp32Reg(pData->Base, ESP32_T0_CONFIG) = config;
	Esp32Reg(pData->Base, ESP32_T0_CONFIG) =
		config | ESP32_T0_DIVCNT_RST;
	pData->Divider = divider;
	pTimer->Freq = frequency;
	pTimer->nsPeriod = (1000000000ULL + frequency / 2) / frequency;
	if (pData->TriggerActive)
	{
		pData->TriggerTicks = ticks;
	}
	Esp32ResetCounter(pData);
	if (running)
	{
		Esp32Run(pData, true);
	}
	EnableInterrupt(state);
	return frequency;
}

static int Esp32MaxTrigger(TimerDev_t *pTimer)
{
	return Esp32Data(pTimer) != NULL ? 1 : 0;
}

static int Esp32FindTrigger(TimerDev_t *pTimer)
{
	Esp32TimerData_t *pData;

	pData = Esp32Data(pTimer);
	return pData != NULL && !pData->TriggerActive ? 0 : -1;
}

static void Esp32DisableTrigger(TimerDev_t *pTimer, int TrigNo)
{
	Esp32TimerData_t *pData;
	uint32_t state;
	uint32_t config;

	pData = Esp32Data(pTimer);
	if (pData == NULL || TrigNo != 0)
	{
		return;
	}
	state = DisableInterrupt();
	pData->TriggerActive = false;
	config = Esp32Reg(pData->Base, ESP32_T0_CONFIG);
	Esp32Reg(pData->Base, ESP32_T0_CONFIG) =
		config & ~ESP32_T0_ALARM_EN;
	EnableInterrupt(state);
}

static uint64_t Esp32EnableTrigger(TimerDev_t *pTimer, int TrigNo,
									uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
									TimerTrigEvtHandler_t Handler, void *pContext)
{
	Esp32TimerData_t *pData;
	uint64_t ticks;
	uint64_t actual;
	uint32_t state;

	pData = Esp32Data(pTimer);
	if (pData == NULL || TrigNo != 0 ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}
	actual = Esp32PeriodToTicks(pTimer->Freq, nsPeriod, &ticks);
	if (actual == 0)
	{
		return 0;
	}
	state = DisableInterrupt();
	pData->TriggerTicks = ticks;
	pData->Deadline = Esp32ReadCount(pData) + ticks;
	pData->TriggerInfo = { Type, actual, Handler, pContext };
	pData->TriggerActive = true;
	Esp32ArmAlarm(pData);
	EnableInterrupt(state);
	return actual;
}

static void Esp32DisableExtTrigger(TimerDev_t *pTimer)
{
	(void)pTimer;
}

static bool Esp32EnableExtTrigger(TimerDev_t *pTimer, int DevNo,
									TIMER_EXTTRIG_SENSE Sense)
{
	(void)pTimer;
	(void)DevNo;
	(void)Sense;
	return false;
}

static void Esp32IRQ(int DevNo)
{
	Esp32TimerData_t *pData;
	TimerTrig_t info;
	uint64_t now;

	pData = &s_Timer[DevNo];
	Esp32Reg(pData->Base, ESP32_T0_INT_CLR) = ESP32_T0_INT_BIT;
	if (pData->pTimer == NULL || !pData->Running || !pData->TriggerActive)
	{
		return;
	}
	now = Esp32ReadCount(pData);
	if (now < pData->Deadline)
	{
		Esp32ArmAlarm(pData);
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
		Esp32ArmAlarm(pData);
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

static void Esp32Tg0Handler(void)
{
	Esp32IRQ(0);
}

#if ESP32_TIMG_COUNT > 1
static void Esp32Tg1Handler(void)
{
	Esp32IRQ(1);
}
#endif

static const Esp32IrqHandler s_TimerHandlers[] = {
	Esp32Tg0Handler,
#if ESP32_TIMG_COUNT > 1
	Esp32Tg1Handler,
#endif
};

bool TimerInit(TimerDev_t *pTimer, const TimerCfg_t *pCfg)
{
	Esp32TimerData_t *pData;
	uint32_t state;
	uint32_t divider;
	uint32_t frequency;
	uint32_t config;
	uint32_t source;
	uint32_t cpuint;
	uint32_t prio;
	int i;

	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 ||
		pCfg->DevNo >= ESP32_TIMG_COUNT || pCfg->bTickInt ||
		pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT)
	{
		return false;
	}
	frequency = Esp32ClockSelect(pCfg->Freq, &divider);
	state = DisableInterrupt();
	pData = &s_Timer[pCfg->DevNo];
	for (i = 0; i < ESP32_TIMG_COUNT; i++)
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

	pData->Base = s_TimerBases[pCfg->DevNo];
	pData->pTimer = pTimer;
	pData->Divider = divider;
	pData->Running = false;
	source = s_TimerSources[pCfg->DevNo];
	cpuint = ESP32_TIMER_CPUINT + (uint32_t)pCfg->DevNo;
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
	pTimer->Disable = Esp32Disable;
	pTimer->Enable = Esp32Enable;
	pTimer->Reset = Esp32Reset;
	pTimer->GetTickCount = Esp32Count;
	pTimer->SetFrequency = Esp32SetFrequency;
	pTimer->GetMaxTrigger = Esp32MaxTrigger;
	pTimer->FindAvailTrigger = Esp32FindTrigger;
	pTimer->DisableTrigger = Esp32DisableTrigger;
	pTimer->EnableTrigger = Esp32EnableTrigger;
	pTimer->DisableExtTrigger = Esp32DisableExtTrigger;
	pTimer->EnableExtTrigger = Esp32EnableExtTrigger;
	pTimer->EnablePeiTrigger = NULL;
	pTimer->EnablePeiEvent = NULL;
	pTimer->DisablePeiEvent = NULL;
	pTimer->GetTaskEndpoint = NULL;
	pTimer->ExecuteTask = NULL;

	Esp32EnableTimerClock(pCfg->DevNo);
	config = ESP32_T0_INCREASE |
		((divider & 0xFFFFUL) << ESP32_T0_DIV_SHIFT);
	Esp32Reg(pData->Base, ESP32_T0_CONFIG) = config;
	Esp32Reg(pData->Base, ESP32_T0_CONFIG) =
		config | ESP32_T0_DIVCNT_RST;
	Esp32Reg(pData->Base, ESP32_T0_LOAD_LO) = 0;
	Esp32Reg(pData->Base, ESP32_T0_LOAD_HI) = 0;
	Esp32Reg(pData->Base, ESP32_T0_LOAD) = 1;
	Esp32Reg(pData->Base, ESP32_T0_INT_CLR) = ESP32_T0_INT_BIT;
	Esp32Reg(pData->Base, ESP32_T0_INT_ENA) &=
		~ESP32_T0_INT_BIT;

	if (!Esp32EnableSourceIrq(source, cpuint, prio,
		s_TimerHandlers[pCfg->DevNo]))
	{
		pData->pTimer = NULL;
		EnableInterrupt(state);
		return false;
	}
	Esp32Run(pData, true);
	EnableInterrupt(state);
	return true;
}

int TimerGetLowFreqDevCount(void)
{
	return 0;
}

int TimerGetHighFreqDevCount(void)
{
	return ESP32_TIMG_COUNT;
}

int TimerGetHighFreqDevNo(void)
{
	return 0;
}
