/**-------------------------------------------------------------------------
@file	timer_re01_gpt.cpp
@brief	RE01 GPT0/1 32-bit and GPT2..5 16-bit internal timers.

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#include <stddef.h>
#include "timer_re01.h"
#include "interrupt_re01.h"
#include "coredev/interrupt.h"

// GPT16 uses the same 32-bit register accesses/layout, with 16-bit counts.
static_assert(offsetof(GPT162_Type, GTCR) == offsetof(GPT320_Type, GTCR), "GPT control layout");
static_assert(offsetof(GPT162_Type, GTCNT) == offsetof(GPT320_Type, GTCNT), "GPT counter layout");
static_assert(offsetof(GPT162_Type, GTCCRD) == offsetof(GPT320_Type, GTCCRD), "GPT compare layout");
static_assert(offsetof(GPT162_Type, GTPR) == offsetof(GPT320_Type, GTPR), "GPT period layout");

static uint32_t Re01GptMask(const RE01_TimerData_t *dev)
{
	return dev->DevNo < RE01_TIMER_GPT_DEVNO + 2 ? UINT32_MAX : 0xFFFFUL;
}

static uint8_t Re01GptEvent(const RE01_TimerData_t *dev, int Offset)
{
	return RE01_EVTID_GPT0_CCMPA + 6 * (dev->DevNo - RE01_TIMER_GPT_DEVNO) + Offset;
}

static volatile uint32_t *Re01GptCompare(RE01_TimerData_t *dev, int TrigNo)
{
	// E is between C and D in the address map. Do not index A/B/C/D as an array.
	switch (TrigNo)
	{
		case 0: return &dev->pGptReg->GTCCRA;
		case 1: return &dev->pGptReg->GTCCRB;
		case 2: return &dev->pGptReg->GTCCRC;
		default: return &dev->pGptReg->GTCCRD;
	}
}

static void Re01GptDisableTrigger(TimerDev_t *pTimer, int TrigNo)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_TRIG_MAXCNT)
	{
		return;
	}
	uint32_t state = DisableInterrupt();
	// RE01 has no GPT compare interrupt-enable bits in GTINTAD. The ICU
	// route is the interrupt enable; releasing it also frees the trigger.
	Re01UnregisterIntHandler(dev->IrqMatch[TrigNo]);
	dev->IrqMatch[TrigNo] = (IRQn_Type)-1;
	dev->CC[TrigNo] = 0;
	dev->Trigger[TrigNo] = {};
	dev->pGptReg->GTST &= ~(1UL << TrigNo);
	EnableInterrupt(state);
}

static void Re01GptCompareIRQHandler(int IntNo, void *pCtx)
{
	RE01_TimerData_t *dev = (RE01_TimerData_t*)pCtx;
	for (int i = 0; i < RE01_TIMER_TRIG_MAXCNT; i++)
	{
		if (dev->IrqMatch[i] != (IRQn_Type)IntNo)
		{
			continue;
		}
		TimerTrig_t trig = dev->Trigger[i];
		dev->pGptReg->GTST &= ~(1UL << i);
		Re01TimerClearPending(dev->IrqMatch[i]);
		if (trig.Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			volatile uint32_t *cmp = Re01GptCompare(dev, i);
			*cmp = Re01TimerNextCompare(*cmp, dev->pGptReg->GTCNT,
				dev->CC[i], Re01GptMask(dev));
		}
		else
		{
			Re01GptDisableTrigger(dev->pTimer, i);
		}
		if (trig.Handler)
		{
			trig.Handler(dev->pTimer, i, trig.pContext);
		}
		else if (dev->pTimer->EvtHandler)
		{
			dev->pTimer->EvtHandler(dev->pTimer, TIMER_EVT_TRIGGER(i));
		}
		return;
	}
}

static void Re01GptOverflowIRQHandler(int IntNo, void *pCtx)
{
	RE01_TimerData_t *dev = (RE01_TimerData_t*)pCtx;
	uint32_t state = DisableInterrupt();
	dev->pGptReg->GTST &= ~GPT320_GTST_TCFPO_Msk;
	Re01TimerClearPending(dev->IrqOvr);
	dev->pTimer->Rollover += (uint64_t)Re01GptMask(dev) + 1;
	EnableInterrupt(state);
	if (dev->pTimer->EvtHandler)
	{
		dev->pTimer->EvtHandler(dev->pTimer, TIMER_EVT_COUNTER_OVR);
	}
}

static bool Re01GptEnable(TimerDev_t *pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL)
	{
		return false;
	}
	dev->pGptReg->GTCR |= GPT320_GTCR_CST_Msk;
	return true;
}

static void Re01GptDisable(TimerDev_t *pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev)
	{
		dev->pGptReg->GTCR &= ~GPT320_GTCR_CST_Msk;
	}
}

static void Re01GptReset(TimerDev_t *pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL)
	{
		return;
	}
	uint32_t state = DisableInterrupt();
	uint32_t cr = dev->pGptReg->GTCR;
	dev->pGptReg->GTCR = cr & ~GPT320_GTCR_CST_Msk;
	dev->pGptReg->GTCNT = 0;
	dev->pGptReg->GTST = 0;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	Re01TimerClearPending(dev->IrqOvr);
	for (int i = 0; i < RE01_TIMER_TRIG_MAXCNT; i++)
	{
		*Re01GptCompare(dev, i) = dev->CC[i];
		Re01TimerClearPending(dev->IrqMatch[i]);
	}
	dev->pGptReg->GTCR = cr;
	EnableInterrupt(state);
}

static uint32_t Re01GptSetFrequency(TimerDev_t *pTimer, uint32_t Freq)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || dev->BaseFreq == 0 || Freq > 64000000UL)
	{
		return 0;
	}
	// TPCS encodes /1, /4, /16, /64, /256, /1024. Select the closest rate.
	uint32_t code = 0;
	uint32_t freq = dev->BaseFreq;
	uint64_t error = Freq > freq ? Freq - freq : freq - Freq;
	if (Freq)
	{
		for (uint32_t i = 1; i <= 5; i++)
		{
			uint32_t candidate = dev->BaseFreq >> (2 * i);
			if (candidate == 0) break;
			uint64_t diff = Freq > candidate ? Freq - candidate : candidate - Freq;
			if (diff < error)
			{
				error = diff;
				freq = candidate;
				code = i;
			}
		}
	}
	uint32_t state = DisableInterrupt();
	uint32_t cc[RE01_TIMER_TRIG_MAXCNT];
	if (!Re01TimerNewPeriods(dev, freq, Re01GptMask(dev), cc))
	{
		EnableInterrupt(state);
		return 0;
	}
	dev->pGptReg->GTCR = code << GPT320_GTCR_TPCS_Pos;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + (freq >> 1)) / freq;
	Re01TimerSetPeriods(dev, cc);
	Re01GptReset(pTimer);
	Re01GptEnable(pTimer);
	EnableInterrupt(state);
	return freq;
}

static uint64_t Re01GptGetTickCount(TimerDev_t *pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return 0;
	uint32_t state = DisableInterrupt();
	uint32_t mask = Re01GptMask(dev);
	uint32_t count = dev->pGptReg->GTCNT & mask;
	uint64_t rollover = pTimer->Rollover;
	if ((dev->pGptReg->GTST & GPT320_GTST_TCFPO_Msk) ||
		(dev->IrqOvr != (IRQn_Type)-1 &&
		((&ICU->IELSR0)[dev->IrqOvr - IEL0_IRQn] & ICU_IELSR0_IR_Msk)))
	{
		// The ISR has not yet added this wrap. Resample after observing it.
		count = dev->pGptReg->GTCNT & mask;
		rollover += (uint64_t)mask + 1;
	}
	pTimer->LastCount = count;
	EnableInterrupt(state);
	return rollover + count;
}

static int Re01GptGetMaxTrigger(TimerDev_t *pTimer)
{
	return Re01TimerData(pTimer) ? RE01_TIMER_TRIG_MAXCNT : 0;
}

static int Re01GptFindAvailTrigger(TimerDev_t *pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev)
	{
		for (int i = 0; i < RE01_TIMER_TRIG_MAXCNT; i++)
		{
			if (dev->IrqMatch[i] == (IRQn_Type)-1) return i;
		}
	}
	return -1;
}

static uint64_t Re01GptEnableTrigger(TimerDev_t *pTimer, int TrigNo, uint64_t nsPeriod,
		TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t Handler, void *pContext)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_TRIG_MAXCNT ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}
	uint32_t state = DisableInterrupt();
	uint64_t cc = TimerNanosecondToTick(pTimer, nsPeriod);
	uint32_t mask = Re01GptMask(dev);
	if (cc <= 2 || cc > mask)
	{
		EnableInterrupt(state);
		return 0;
	}
	IRQn_Type irq = Re01RegisterIntHandler(Re01GptEvent(dev, TrigNo), dev->IntPrio,
		Re01GptCompareIRQHandler, dev);
	if (irq == (IRQn_Type)-1)
	{
		EnableInterrupt(state);
		return 0;
	}
	dev->IrqMatch[TrigNo] = irq;
	dev->CC[TrigNo] = (uint32_t)cc;
	dev->Trigger[TrigNo] = {Type, TimerTickToTime(pTimer, cc, 1000000000UL), Handler, pContext};
	dev->pGptReg->GTST &= ~(1UL << TrigNo);
	Re01TimerClearPending(irq);
	*Re01GptCompare(dev, TrigNo) = (dev->pGptReg->GTCNT + (uint32_t)cc) & mask;
	uint64_t period = dev->Trigger[TrigNo].nsPeriod;
	EnableInterrupt(state);
	return period;
}

static void Re01GptDisableExtTrigger(TimerDev_t *pTimer)
{
}

static bool Re01GptEnableExtTrigger(TimerDev_t *pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	return false;
}

bool Re01GptInit(RE01_TimerData_t * const dev, const TimerCfg_t * const pCfg)
{
	TimerDev_t *timer = dev->pTimer;
	timer->DevNo = pCfg->DevNo;
	timer->EvtHandler = pCfg->EvtHandler;
	timer->Enable = Re01GptEnable;
	timer->Disable = Re01GptDisable;
	timer->Reset = Re01GptReset;
	timer->GetTickCount = Re01GptGetTickCount;
	timer->SetFrequency = Re01GptSetFrequency;
	timer->GetMaxTrigger = Re01GptGetMaxTrigger;
	timer->FindAvailTrigger = Re01GptFindAvailTrigger;
	timer->EnableTrigger = Re01GptEnableTrigger;
	timer->DisableTrigger = Re01GptDisableTrigger;
	timer->EnableExtTrigger = Re01GptEnableExtTrigger;
	timer->DisableExtTrigger = Re01GptDisableExtTrigger;
	dev->IntPrio = pCfg->IntPrio;
	dev->BaseFreq = SystemPeriphClockGet(0); // PCLKA = ICLK; register bus uses PCLKB.
	if (dev->BaseFreq == 0) return false;

	uint32_t gate = dev->DevNo < RE01_TIMER_GPT_DEVNO + 2 ?
		MSTP_MSTPCRD_MSTPD5_Msk : MSTP_MSTPCRD_MSTPD6_Msk;
	MSTP->MSTPCRD &= ~gate;
	GPT320_Type *reg = dev->pGptReg;
	reg->GTWP = 0xA500UL;
	reg->GTCR = 0;
	reg->GTSSR = 0;
	reg->GTPSR = 0;
	reg->GTCSR = 0;
	reg->GTUPSR = 0;
	reg->GTDNSR = 0;
	reg->GTICASR = 0;
	reg->GTICBSR = 0;
	reg->GTUDDTYC = GPT320_GTUDDTYC_UD_Msk | GPT320_GTUDDTYC_UDF_Msk;
	reg->GTUDDTYC = GPT320_GTUDDTYC_UD_Msk;
	reg->GTIOR = 0;
	reg->GTINTAD = 0;
	reg->GTBER = GPT320_GTBER_BD_Msk; // Disable all buffering, including C/D.
	reg->GTDTCR = 0;
	reg->GTPR = Re01GptMask(dev);
	Re01GptReset(timer);
	dev->IrqOvr = Re01RegisterIntHandler(Re01GptEvent(dev, 4), pCfg->IntPrio,
		Re01GptOverflowIRQHandler, dev);
	if (dev->IrqOvr == (IRQn_Type)-1) return false;
	return Re01GptSetFrequency(timer, pCfg->Freq) != 0;
}
