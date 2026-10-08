/**-------------------------------------------------------------------------
@file	timer_re01_tmr.cpp

@brief	Renesas RE01 TMR timer class implementation

This timer is configured as 16bits counter using TMR0 & TMR1 in cascade

@author	Hoang Nguyen Hoan
@date	Feb. 3, 2022

@license

MIT License

Copyright (c) 2022 I-SYST inc. All rights reserved.

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
#include "re01xxx.h"

#include "timer_re01.h"
#include "interrupt_re01.h"
#include "coredev/interrupt.h"

extern RE01_TimerData_t g_Re01TimerData[RE01_TIMER_MAXCNT];

static void Re01TmrDisableTrigger(TimerDev_t * const pTimer, int TrigNo);

static void Re01TmrOvrIRQHandler(int IntNo, void *pCtx)
{
	RE01_TimerData_t *dev = (RE01_TimerData_t*)pCtx;
	uint32_t state = DisableInterrupt();
	// Clear the pending wrap before callbacks can read the extended count.
	Re01TimerClearPending(dev->IrqOvr);
	dev->pTimer->Rollover += 0x10000ULL;
	EnableInterrupt(state);
	if (dev->pTimer->EvtHandler)
	{
		dev->pTimer->EvtHandler(dev->pTimer, TIMER_EVT_COUNTER_OVR);
	}
}

static void Re01TmrCompareIRQHandler(RE01_TimerData_t *dev, int TrigNo)
{
	if (dev->IrqMatch[TrigNo] == (IRQn_Type)-1) return;
	TimerTrig_t trig = dev->Trigger[TrigNo];
	Re01TimerClearPending(dev->IrqMatch[TrigNo]);
	if (trig.Type == TIMER_TRIG_TYPE_CONTINUOUS)
	{
		volatile uint16_t *cmp = TrigNo ? &TMR01->TCORB : &TMR01->TCORA;
		*cmp = Re01TimerNextCompare(*cmp, TMR01->TCNT, dev->CC[TrigNo], 0xFFFFUL);
	}
	else
	{
		// Release before invoking the callback, which may rearm this trigger.
		Re01TmrDisableTrigger(dev->pTimer, TrigNo);
	}
	if (trig.Handler)
	{
		trig.Handler(dev->pTimer, TrigNo, trig.pContext);
	}
	else if (dev->pTimer->EvtHandler)
	{
		dev->pTimer->EvtHandler(dev->pTimer, TIMER_EVT_TRIGGER(TrigNo));
	}
}

static void Re01TmrTcmAIRQHandler(int IntNo, void *pCtx)
{
	Re01TmrCompareIRQHandler((RE01_TimerData_t*)pCtx, 0);
}

static void Re01TmrTcmBIRQHandler(int IntNo, void *pCtx)
{
	Re01TmrCompareIRQHandler((RE01_TimerData_t*)pCtx, 1);
}

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Re01TmrEnable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);

	if (dev == NULL) return false;
	// 16bits mode
	TMR0->TCCR = TMR0_TCCR_CSS_Msk;
	TMR1->TCCR = dev->TmrClock;

	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Re01TmrDisable(TimerDev_t * const pTimer)
{
	if (Re01TimerData(pTimer) == NULL) return;
	TMR0->TCCR = 0;
	TMR1->TCCR = 0;
}

/**
 * @brief   Reset timer.
 */
static void Re01TmrReset(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);

	if (dev == NULL) return;
	uint32_t state = DisableInterrupt();
	uint8_t clock = TMR1->TCCR;
	TMR1->TCCR = 0;
	TMR01->TCNT = 0;
	pTimer->LastCount = 0;
	pTimer->Rollover = 0;
	if (TMR0->TCR_b.CMIEA)
	{
		TMR01->TCORA = dev->CC[0];
	}
	if (TMR0->TCR_b.CMIEB)
	{
		TMR01->TCORB = dev->CC[1];
	}
	__IOM uint32_t *ielsr = &ICU->IELSR0;
	IRQn_Type irq[] = {dev->IrqOvr, dev->IrqMatch[0], dev->IrqMatch[1]};
	for (unsigned i = 0; i < sizeof(irq) / sizeof(irq[0]); i++)
	{
		if (irq[i] != (IRQn_Type)-1)
		{
			ielsr[irq[i] - IEL0_IRQn] &= ~ICU_IELSR0_IR_Msk;
			NVIC_ClearPendingIRQ(irq[i]);
		}
	}
	TMR1->TCCR = clock;
	EnableInterrupt(state);
}

/**
 * @brief	Set timer main frequency.
 *
 * This function allows dynamically changing the timer frequency.  Timer
 * will be reset and restarted with new frequency
 *
 * @param 	Freq : Frequency in Hz
 *
 * @return  Real frequency
 */
static uint32_t Re01TmrSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || dev->BaseFreq == 0 || Freq > 64000000UL)
	{
		return 0;
	}
	// CKS encodes /1, /2, /8, /32, /64, /1024, /8192.
	static const uint8_t shifts[] = {0, 1, 3, 5, 6, 10, 13};
	uint32_t code = 0;
	uint32_t freq = dev->BaseFreq;
	uint32_t error = Freq > freq ? Freq - freq : freq - Freq;
	if (Freq)
	{
		for (uint32_t i = 1; i < sizeof(shifts); i++)
		{
			uint32_t candidate = dev->BaseFreq >> shifts[i];
			if (candidate == 0) break;
			uint32_t diff = Freq > candidate ? Freq - candidate : candidate - Freq;
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
	if (!Re01TimerNewPeriods(dev, freq, 0xFFFFUL, cc))
	{
		EnableInterrupt(state);
		return 0;
	}
	Re01TmrDisable(pTimer);
	TMR0->TCCR = TMR0_TCCR_CSS_Msk;
	dev->TmrClock = (1U << TMR1_TCCR_CSS_Pos) | (code << TMR1_TCCR_CKS_Pos);
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + (freq >> 1)) / freq;
	Re01TimerSetPeriods(dev, cc);
	Re01TmrReset(pTimer);
	Re01TmrEnable(pTimer);
	EnableInterrupt(state);
	return freq;
}

static uint64_t Re01TmrGetTickCount(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return 0;
	uint32_t state = DisableInterrupt();
	uint16_t cnt = TMR01->TCNT;
	uint64_t rollover = pTimer->Rollover;
	if (dev->IrqOvr != (IRQn_Type)-1 &&
		((&ICU->IELSR0)[dev->IrqOvr - IEL0_IRQn] & ICU_IELSR0_IR_Msk))
	{
		cnt = TMR01->TCNT;
		rollover += 0x10000ULL;
	}
	pTimer->LastCount = cnt;
	EnableInterrupt(state);
	return rollover + cnt;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
static int Re01TmrGetMaxTrigger(TimerDev_t * const pTimer)
{
	return Re01TimerData(pTimer) ? RE01_TIMER_TMR_TRIG_MAXCNT : 0;
}

/**
 * @brief	Enable a specific nanosecond timer trigger event.
 *
 * @param   TrigNo : Trigger number to enable. Index value starting at 0
 * @param   nsPeriod : Trigger period in nsec.
 * @param   Type     : Trigger type single shot or continuous
 * @param	Handler	 : Optional Timer trigger user callback
 * @param   pContext : Optional pointer to user private data to be passed
 *                     to the callback. This could be a class or structure pointer.
 *
 * @return  real period in nsec based on clock calculation
 */
static uint64_t Re01TmrEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
							  TimerTrigEvtHandler_t const Handler, void * const pContext)
{
	if (pTimer == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_TMR_TRIG_MAXCNT)
		return 0;

	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || (Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS)) return 0;
	uint64_t cc = TimerNanosecondToTick(pTimer, nsPeriod);

	if (cc <= 2ULL || cc >= 0x10000ULL)
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();
	uint8_t evtid = TrigNo ? RE01_EVTID_TMR_CMIB0 : RE01_EVTID_TMR_CMIA0;
	Re01IRQHandler_t handler = TrigNo ? Re01TmrTcmBIRQHandler : Re01TmrTcmAIRQHandler;
	IRQn_Type irq = Re01RegisterIntHandler(evtid, dev->IntPrio, handler, dev);
	if (irq == (IRQn_Type)-1)
	{
		EnableInterrupt(state);
		return 0;
	}
	dev->IrqMatch[TrigNo] = irq;
	TMR0->TCR &= ~((TrigNo + 1) << TMR0_TCR_CMIEA_Pos);
	(&ICU->IELSR0)[irq - IEL0_IRQn] &= ~ICU_IELSR0_IR_Msk;
	NVIC_ClearPendingIRQ(irq);

	dev->Trigger[TrigNo].Type = Type;
	dev->CC[TrigNo] = cc & 0xFFFF;

	dev->Trigger[TrigNo].nsPeriod = TimerTickToTime(pTimer, cc, 1000000000UL);
	dev->Trigger[TrigNo].Handler = Handler;
	dev->Trigger[TrigNo].pContext = pContext;

	// TMR does not support periodic trigger.
	// Do it manually within interrupt handler
	if (TrigNo > 0)
	{
		TMR01->TCORB = (cc + TMR01->TCNT);
	}
	else
	{
		TMR01->TCORA = (cc + TMR01->TCNT);
	}

	TMR0->TCR |= (TrigNo + 1) << TMR0_TCR_CMIEA_Pos;

	uint64_t period = dev->Trigger[TrigNo].nsPeriod;
	EnableInterrupt(state);
	return period;
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static void Re01TmrDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	if (pTimer == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_TMR_TRIG_MAXCNT)
		return;

	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return;
	uint32_t state = DisableInterrupt();
	TMR0->TCR &= ~((TrigNo + 1) << TMR0_TCR_CMIEA_Pos);

	if (dev->IrqMatch[TrigNo] != -1)
	{
		Re01UnregisterIntHandler(dev->IrqMatch[TrigNo]);
		dev->IrqMatch[TrigNo] = (IRQn_Type)-1;
	}
	dev->CC[TrigNo] = 0;
	dev->Trigger[TrigNo] = {};
	if (TrigNo > 0)
	{
		TMR01->TCORB = 0;
	}
	else
	{
		TMR01->TCORA = 0;
	}
	EnableInterrupt(state);
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static void Re01TmrDisableExtTrigger(TimerDev_t * const pTimer)
{
}

/**
 * @brief	Enable external timer trigger event.
 *
 * @param   TrigDevNo : External trigger device number to enable. Index value starting at 0
 * @param   pContext : Optional pointer to user private data to be passed
 *                     to the callback. This could be a class or structure pointer.
 *
 * @return  true - Success
 */
static bool Re01TmrEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	switch (Sense)
	{
		case TIMER_EXTTRIG_SENSE_LOW_TRANSITION:
			break;
		case TIMER_EXTTRIG_SENSE_HIGH_TRANSITION:
			break;
		case TIMER_EXTTRIG_SENSE_TOGGLE:
			break;
		case TIMER_EXTTRIG_SENSE_DISABLE:
		default:
			break;
	}

	return false;
}

/**
 * @brief	Get first available timer trigger index.
 *
 * This function returns the first available timer trigger to be used to with
 * EnableTimerTrigger
 *
 * @return	success : Timer trigger index
 * 			fail : -1
 */
static int Re01TmrFindAvailTrigger(TimerDev_t * const pTimer)
{
	if (Re01TimerData(pTimer) == NULL) return -1;
	for (int i = 0; i < RE01_TIMER_TMR_TRIG_MAXCNT; i++)
	{
		if ((TMR0->TCR & ((i + 1) << TMR0_TCR_CMIEA_Pos)) == 0)
		{
			return i;
		}
	}
	return -1;
}

/**
 * @brief   Timer initialization.
 *
 * This is specific to each architecture.
 *
 * @param	Cfg	: Timer configuration data.
 *
 * @return
 * 			- true 	: Success
 * 			- false : Otherwise
 */
bool Re01TmrInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	if (pCfg->Freq > 64000000)
	{
		return false;
	}

	pTimerData->IntPrio = pCfg->IntPrio;
	pTimerData->pTimer->DevNo = pCfg->DevNo;
	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
	pTimerData->pTimer->Disable = Re01TmrDisable;
	pTimerData->pTimer->Enable = Re01TmrEnable;
	pTimerData->pTimer->Reset = Re01TmrReset;
	pTimerData->pTimer->GetTickCount = Re01TmrGetTickCount;
	pTimerData->pTimer->SetFrequency = Re01TmrSetFrequency;
	pTimerData->pTimer->GetMaxTrigger = Re01TmrGetMaxTrigger;
	pTimerData->pTimer->FindAvailTrigger = Re01TmrFindAvailTrigger;
	pTimerData->pTimer->DisableTrigger = Re01TmrDisableTrigger;
	pTimerData->pTimer->EnableTrigger = Re01TmrEnableTrigger;
	pTimerData->pTimer->DisableExtTrigger = Re01TmrDisableExtTrigger;
	pTimerData->pTimer->EnableExtTrigger = Re01TmrEnableExtTrigger;

	MSTP->MSTPCRD &= ~MSTP_MSTPCRD_MSTPD1_Msk;

	if (pCfg->ClkSrc == TIMER_CLKSRC_EXT)
	{
		return false;
	}
	else
	{
		// Only the PCLK is the TMR internal clock source
		pTimerData->BaseFreq = SystemPeriphClockGet(1);
	}
	TMR0->TCR = 0;
	TMR1->TCR = 0;

	uint32_t f = Re01TmrSetFrequency(pTimerData->pTimer, pCfg->Freq);
	if (f <= 0)
	{
		return false;
	}

	pTimerData->IrqOvr = Re01RegisterIntHandler(RE01_EVTID_TMR_OVF0, pCfg->IntPrio, Re01TmrOvrIRQHandler, pTimerData);
	if (pTimerData->IrqOvr == -1)
	{
		return false;
	}
	TMR0->TCR |= TMR0_TCR_OVIE_Msk;

	return true;
}


