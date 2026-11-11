/**-------------------------------------------------------------------------
@file	timer_re01_agt.cpp

@brief	Renesas RE01 AGT timer class implementation

NOTE: AGT timer seems to have a hardware bug when 2 comparator are used at the
same time. The TMCB got lockup when both comparator trigger at the same time.
This bug appears when TCMA is slower the TCMB. For more stability, TCMB must be
at lower or equal to the TCMA frequency.
There might be other issue with the TMCB interrupt.  It seems like it does not
wake the MCU from sleep.  Probably that is the reason off the instability when TCMB
is running faster than TMCA

NOTE: AGT1 does not support TCMB, only 1 comparator is avail.

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

// Stop acknowledgement crosses the selected timer clock domain.
// Keep the wait bounded, including when called from the compare ISR.
static bool Re01AgtStop(RE01_TimerData_t *dev)
{
	dev->pAgtReg->AGTCR &= ~AGT0_AGTCR_TSTART_Msk;
	for (uint32_t retry = 100000UL; retry > 0; retry--)
	{
		if ((dev->pAgtReg->AGTCR & AGT0_AGTCR_TCSTF_Msk) == 0)
		{
			return true;
		}
	}
	return false;
}

static void Re01AgtUnfIRQHandler(int IntNo, void *pCtx)
{
	RE01_TimerData_t *dev = (RE01_TimerData_t*)pCtx;

	if (dev->pAgtReg->AGTCR & AGT0_AGTCR_TUNDF_Msk)
	{
		uint32_t state = DisableInterrupt();
		dev->pAgtReg->AGTCR_b.TUNDF = 0;
		Re01TimerClearPending(dev->IrqOvr);
		dev->pTimer->Rollover += 0x10000;
		EnableInterrupt(state);
		if (dev->pTimer->EvtHandler)
		{
			dev->pTimer->EvtHandler(dev->pTimer, TIMER_EVT_COUNTER_OVR);
		}
	}
}

void Re01AgtDisableTrigger(TimerDev_t * const pTimer, int TrigNo);

// Counts the counter misses in a stop and a start, at timer frequency Freq
static uint32_t Re01AgtLost(uint32_t Freq)
{
	uint32_t core = SystemCoreClock;
	uint32_t cpu = 0;

	if (core)
	{
		cpu = (uint32_t)(((uint64_t)RE01_TIMER_AGT_STOP_CYCLES * Freq + core / 2) / core);
	}

	return RE01_TIMER_AGT_RESTART + cpu;
}

// Counter stopped at Count (up counts), tick count Off + Count once it runs
// again: writes the compare of trigger TrigNo for its deadline, the next count
// if the deadline is reached.
static void Re01AgtArm(RE01_TimerData_t *dev, int TrigNo, uint16_t Count, uint16_t Off)
{
	volatile uint16_t *cmp = TrigNo ? &dev->pAgtReg->AGTCMB : &dev->pAgtReg->AGTCMA;
	uint16_t up = (uint16_t)(dev->Deadline[TrigNo] - Off);

	if (Re01TimerReached((uint16_t)(Off + Count), dev->Deadline[TrigNo], dev->CC[TrigNo], 0xFFFFUL))
	{
		up = Count + 1;
	}
	*cmp = 0xFFFFU - up;
}

// Service both flags together. The compare values are written with the
// counter stopped. The counts it misses are added back to the tick count, the
// compare values written for the deadlines in that count.
static void Re01AgtTcmIRQHandler(int IntNo, void *pCtx)
{
	RE01_TimerData_t *dev = (RE01_TimerData_t*)pCtx;

	// Flag cleared first: a match while the handler runs calls it again
	Re01AcknowledgeInt((IRQn_Type)IntNo);

	if ((dev->pAgtReg->AGTCR & (AGT0_AGTCR_TCMAF_Msk | AGT0_AGTCR_TCMBF_Msk)) == 0)
	{
		return;
	}

	TimerDev_t *timer = dev->pTimer;
	TimerEvtHandler_t evtHandler = timer->EvtHandler;
	TimerTrig_t trig[RE01_TIMER_AGT_TRIG_MAXCNT] = {};
	uint8_t fired = 0;
	int maxtrig = dev->DevNo == 0 ? RE01_TIMER_AGT_TRIG_MAXCNT : 1;
	bool running = (dev->pAgtReg->AGTCR & AGT0_AGTCR_TSTART_Msk) != 0;

	if (!Re01AgtStop(dev))
	{
		Re01AgtDisableTrigger(timer, 0);
		if (dev->DevNo == 0) Re01AgtDisableTrigger(timer, 1);
		return;
	}

	uint32_t state = DisableInterrupt();

	// Counter stopped, no match until it runs: the flags have all of them.
	// Short, the counts missed depend on it. Tick count once the counter runs
	// again: off + count.
	uint8_t cr = dev->pAgtReg->AGTCR & (AGT0_AGTCR_TCMAF_Msk | AGT0_AGTCR_TCMBF_Msk);
	uint16_t count = 0xFFFFU - dev->pAgtReg->AGT;
	uint8_t cmsr = dev->pAgtReg->AGTCMSR;
	uint16_t off = (uint16_t)timer->Rollover;
	uint16_t lost = running ? (uint16_t)dev->Lost : 0;

	dev->pAgtReg->AGTCR &= ~cr;
	for (int i = 0; i < maxtrig; i++)
	{
		if ((cmsr & (1U << (i << 2))) == 0)
		{
			continue;
		}
		if ((cr & (AGT0_AGTCR_TCMAF_Msk << i)) && dev->IrqMatch[i] != (IRQn_Type)-1)
		{
			fired |= 1U << i;
			if (dev->Trigger[i].Type != TIMER_TRIG_TYPE_CONTINUOUS)
			{
				// Released once the counter runs
				*(i ? &dev->pAgtReg->AGTCMB : &dev->pAgtReg->AGTCMA) = 0xFFFF;
				continue;
			}
			dev->Deadline[i] = Re01TimerNextCompare(dev->Deadline[i], (uint16_t)(off + count),
				dev->CC[i], 0xFFFFUL);
		}
		Re01AgtArm(dev, i, count, off + lost);
	}
	if (running)
	{
		dev->pAgtReg->AGTCR |= AGT0_AGTCR_TSTART_Msk;
	}

	// Counts missed while stopped
	timer->Rollover += lost;
	for (int i = 0; i < maxtrig; i++)
	{
		if (fired & (1U << i))
		{
			trig[i] = dev->Trigger[i];
		}
	}
	EnableInterrupt(state);

	for (int i = 0; i < maxtrig; i++)
	{
		if ((fired & (1U << i)) && trig[i].Type != TIMER_TRIG_TYPE_CONTINUOUS)
		{
			// Released before invoking the callback, which may rearm this trigger
			Re01AgtDisableTrigger(timer, i);
		}
	}
	// Snapshot both callbacks before either can reconfigure the timer.
	for (int i = 0; i < maxtrig; i++)
	{
		if ((fired & (1U << i)) == 0) continue;
		// Trigger handler first, then the event handler, as the other timer ports
		if (trig[i].Handler)
		{
			trig[i].Handler(timer, i, trig[i].pContext);
		}
		if (evtHandler)
		{
			evtHandler(timer, TIMER_EVT_TRIGGER(i));
		}
	}
}

#define Re01AgtTcmAIRQHandler	Re01AgtTcmIRQHandler
#define Re01AgtTcmBIRQHandler	Re01AgtTcmIRQHandler


/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 */
static bool Re01AgtEnable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);

	if (dev == NULL) return false;
	dev->pAgtReg->AGTCR |= AGT0_AGTCR_TSTART_Msk;

	return true;
}

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 */
static void Re01AgtDisable(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);

	if (dev) dev->pAgtReg->AGTCR &= ~AGT0_AGTCR_TSTART_Msk;
}

/**
 * @brief   Reset timer.
 */
static void Re01AgtReset(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return;
	uint32_t state = DisableInterrupt();
	bool running = (dev->pAgtReg->AGTCR & AGT0_AGTCR_TSTART_Msk) != 0;
	if (!Re01AgtStop(dev))
	{
		EnableInterrupt(state);
		return;
	}
	dev->pAgtReg->AGT = 0xFFFF;
	dev->pAgtReg->AGTCR &= ~(AGT0_AGTCR_TUNDF_Msk | AGT0_AGTCR_TCMAF_Msk | AGT0_AGTCR_TCMBF_Msk);
	pTimer->LastCount = 0;
	pTimer->Rollover = 0;
	dev->Deadline[0] = dev->CC[0];
	dev->Deadline[1] = dev->CC[1];
	if (dev->pAgtReg->AGTCMSR & AGT0_AGTCMSR_TCMEA_Msk)
	{
		dev->pAgtReg->AGTCMA = 0xFFFF - dev->CC[0];
	}
	if (dev->DevNo == 0 && (dev->pAgtReg->AGTCMSR & AGT0_AGTCMSR_TCMEB_Msk))
	{
		dev->pAgtReg->AGTCMB = 0xFFFF - dev->CC[1];
	}
	IRQn_Type irq[] = {dev->IrqOvr, dev->IrqMatch[0], dev->IrqMatch[1]};
	for (unsigned i = 0; i < sizeof(irq) / sizeof(irq[0]); i++)
	{
		if (irq[i] != (IRQn_Type)-1)
		{
			(&ICU->IELSR0)[irq[i] - IEL0_IRQn] &= ~ICU_IELSR0_IR_Msk;
			NVIC_ClearPendingIRQ(irq[i]);
		}
	}
	if (running)
	{
		dev->pAgtReg->AGTCR |= AGT0_AGTCR_TSTART_Msk;
	}
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
static uint32_t Re01AgtSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || dev->BaseFreq == 0) return 0;
	uint32_t cks = 0;
	uint32_t freq = dev->BaseFreq;
	uint32_t error = Freq > freq ? Freq - freq : freq - Freq;
	if (Freq)
	{
		for (uint32_t i = 1; i <= 7; i++)
		{
			uint32_t candidate = dev->BaseFreq >> i;
			if (candidate == 0) break;
			uint32_t diff = Freq > candidate ? Freq - candidate : candidate - Freq;
			if (diff < error)
			{
				error = diff;
				freq = candidate;
				cks = i;
			}
		}
	}
	uint32_t state = DisableInterrupt();
	uint32_t cc[RE01_TIMER_TRIG_MAXCNT];
	if (!Re01TimerNewPeriods(dev, freq, 0xFFFFUL, cc) || !Re01AgtStop(dev))
	{
		EnableInterrupt(state);
		return 0;
	}
	dev->pAgtReg->AGTMR2 = (dev->pAgtReg->AGTMR2 & ~AGT0_AGTMR2_CKS_Msk) | cks;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + (freq >> 1)) / freq;
	dev->Lost = Re01AgtLost(freq);
	Re01TimerSetPeriods(dev, cc);
	Re01AgtReset(pTimer);
	Re01AgtEnable(pTimer);
	EnableInterrupt(state);
	return freq;
}

static uint64_t Re01AgtGetTickCount(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return 0;
	uint32_t state = DisableInterrupt();
	uint16_t cnt = dev->pAgtReg->AGT;
	uint64_t rollover = pTimer->Rollover;
	if (dev->pAgtReg->AGTCR & AGT0_AGTCR_TUNDF_Msk)
	{
		cnt = dev->pAgtReg->AGT;
		rollover += 0x10000ULL;
	}
	pTimer->LastCount = 0xFFFFU - cnt;
	uint64_t ticks = rollover + pTimer->LastCount;
	EnableInterrupt(state);
	return ticks;
}

/**
 * @brief	Get maximum available timer trigger event for the timer.
 *
 * @return	count
 */
int Re01AgtGetMaxTrigger(TimerDev_t * const pTimer)
{
	if (Re01TimerData(pTimer) == NULL) return 0;
	return pTimer->DevNo == 0 ? RE01_TIMER_AGT_TRIG_MAXCNT : 1;
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
uint64_t Re01AgtEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
								 TimerTrigEvtHandler_t const Handler, void * const pContext)
{
	if (pTimer == NULL || TrigNo < 0 || TrigNo >= RE01_TIMER_AGT_TRIG_MAXCNT)
	{
		return 0;
	}

	if (pTimer->DevNo == 1 && TrigNo > 0)
	{
		// AGT1 does not support compare B event
		return 0;
	}

	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL || (Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS)) return 0;
	if (nsPeriod == 0)
	{
		return 0;
	}
	uint64_t cc = Re01TimerClosestTicks(TimerNanosecondToTick(pTimer, nsPeriod), 0xFFFFUL);

	uint8_t evtid = 0;
	Re01IRQHandler_t hndlr = 0;


	if (TrigNo > 0)
	{
		evtid = RE01_EVTID_AGT0_AGTCMBI;
		hndlr = Re01AgtTcmBIRQHandler;
	}
	else
	{
		evtid = pTimer->DevNo > 0? RE01_EVTID_AGT1_AGTCMAI : RE01_EVTID_AGT0_AGTCMAI;
		hndlr = Re01AgtTcmAIRQHandler;
	}

	uint32_t state = DisableInterrupt();
	IRQn_Type irq = Re01RegisterIntHandler(evtid, dev->IntPrio, hndlr, dev);
	if (irq == (IRQn_Type)-1)
	{
		EnableInterrupt(state);
		return 0;
	}
	bool running = (dev->pAgtReg->AGTCR & AGT0_AGTCR_TSTART_Msk) != 0;

	// Compare disabled while it is set up, with the counter running
	uint8_t cmsr = dev->pAgtReg->AGTCMSR;

	dev->pAgtReg->AGTCMSR = cmsr & ~(1U << (TrigNo << 2));
	dev->pAgtReg->AGTCR &= ~(AGT0_AGTCR_TCMAF_Msk << TrigNo);
	(&ICU->IELSR0)[irq - IEL0_IRQn] &= ~ICU_IELSR0_IR_Msk;
	NVIC_ClearPendingIRQ(irq);

	// The period starts now, the counter running
	uint16_t start = (uint16_t)pTimer->Rollover + (uint16_t)(0xFFFFU - dev->pAgtReg->AGT);

	if (!Re01AgtStop(dev))
	{
		if (dev->IrqMatch[TrigNo] == (IRQn_Type)-1)
		{
			Re01UnregisterIntHandler(irq);
		}
		dev->pAgtReg->AGTCMSR = cmsr;
		EnableInterrupt(state);
		return 0;
	}

	dev->IrqMatch[TrigNo] = irq;
	dev->Trigger[TrigNo].Type = Type;
	dev->CC[TrigNo] = cc;
	dev->Deadline[TrigNo] = (uint16_t)(start + cc);
	dev->Trigger[TrigNo].Handler = Handler;
	dev->Trigger[TrigNo].pContext = pContext;

	// Counter stopped. Short, the counts missed depend on it: those are added
	// back to the tick count, the compare values written for it. Tick count
	// once the counter runs again: off + count.
	uint16_t count = 0xFFFFU - dev->pAgtReg->AGT;
	uint16_t lost = running ? (uint16_t)dev->Lost : 0;
	uint16_t off = (uint16_t)pTimer->Rollover + lost;
	int other = TrigNo ^ 1;

	Re01AgtArm(dev, TrigNo, count, off);
	if (dev->DevNo == 0 && (dev->pAgtReg->AGTCMSR & (1U << (other << 2))))
	{
		Re01AgtArm(dev, other, count, off);
	}
	dev->pAgtReg->AGTCMSR |= (1 << (TrigNo << 2));

	if (running)
	{
		dev->pAgtReg->AGTCR |= AGT0_AGTCR_TSTART_Msk;
	}
	pTimer->Rollover += lost;

	// Period of the counts, computed once the period has started
	uint64_t period = TimerTickToTime(pTimer, cc, 1000000000UL);

	dev->Trigger[TrigNo].nsPeriod = period;
	EnableInterrupt(state);
	return period;
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Re01AgtDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	if (pTimer == NULL || TrigNo < 0 || TrigNo >= Re01AgtGetMaxTrigger(pTimer))
	{
		return;
	}
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return;
	uint32_t state = DisableInterrupt();
	dev->pAgtReg->AGTCMSR &= ~(1U << (TrigNo << 2));
	Re01UnregisterIntHandler(dev->IrqMatch[TrigNo]);
	dev->IrqMatch[TrigNo] = (IRQn_Type)-1;
	dev->CC[TrigNo] = 0;
	dev->Trigger[TrigNo] = {};
	// Do not write the compare register while the counter is running.
	EnableInterrupt(state);
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
void Re01AgtDisableExtTrigger(TimerDev_t * const pTimer)
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
bool Re01AgtEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
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
static int Re01AgtFindAvailTrigger(TimerDev_t * const pTimer)
{
	RE01_TimerData_t *dev = Re01TimerData(pTimer);
	if (dev == NULL) return -1;
	for (int i = 0; i < Re01AgtGetMaxTrigger(pTimer); i++)
	{
		if ((dev->pAgtReg->AGTCMSR & (1U << (i << 2))) == 0)
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
bool Re01AgtInit(RE01_TimerData_t * const pTimerData, const TimerCfg_t * const pCfg)
{
	pTimerData->IntPrio = pCfg->IntPrio;
	pTimerData->pTimer->DevNo = pCfg->DevNo;
	pTimerData->pTimer->EvtHandler = pCfg->EvtHandler;
	pTimerData->pTimer->Disable = Re01AgtDisable;
	pTimerData->pTimer->Enable = Re01AgtEnable;
	pTimerData->pTimer->Reset = Re01AgtReset;
	pTimerData->pTimer->GetTickCount = Re01AgtGetTickCount;
	pTimerData->pTimer->SetFrequency = Re01AgtSetFrequency;
	pTimerData->pTimer->GetMaxTrigger = Re01AgtGetMaxTrigger;
	pTimerData->pTimer->FindAvailTrigger = Re01AgtFindAvailTrigger;
	pTimerData->pTimer->DisableTrigger = Re01AgtDisableTrigger;
	pTimerData->pTimer->EnableTrigger = Re01AgtEnableTrigger;
	pTimerData->pTimer->DisableExtTrigger = Re01AgtDisableExtTrigger;
	pTimerData->pTimer->EnableExtTrigger = Re01AgtEnableExtTrigger;


	TIMER_CLKSRC clksrc = pCfg->ClkSrc;

	if (pCfg->ClkSrc == TIMER_CLKSRC_DEFAULT)
	{
		if (GetLowFreqOscType() == OSC_TYPE_RC)
		{
			clksrc = TIMER_CLKSRC_LFRC;
		}
		else
		{
			clksrc = TIMER_CLKSRC_LFXTAL;
		}
	}

	MSTP->MSTPCRD &= ~(0x8 >> pCfg->DevNo) ;

	if (!Re01AgtStop(pTimerData))
	{
		return false;
	}
	pTimerData->pAgtReg->AGTCMSR = 0;
	pTimerData->pAgtReg->AGTMR1 = 0;
	pTimerData->pAgtReg->AGTMR2 = 0;

	switch (clksrc)
	{
		case TIMER_CLKSRC_LFRC:
			pTimerData->BaseFreq = 32768;
			pTimerData->pAgtReg->AGTMR1_b.TCK = 4;
			break;

		case TIMER_CLKSRC_LFXTAL:
			pTimerData->BaseFreq = 32768;
			pTimerData->pAgtReg->AGTMR1_b.TCK = 6;
			break;
		case TIMER_CLKSRC_HFRC:
		case TIMER_CLKSRC_HFXTAL:
		default:
			pTimerData->BaseFreq = SystemPeriphClockGet(1);
			break;
		case TIMER_CLKSRC_EXT:
			return false;
	}

	uint32_t f = Re01AgtSetFrequency(pTimerData->pTimer, pCfg->Freq);
	if (f <= 0)
	{
		return false;
	}

	uint8_t evtid = 0;

	if (pTimerData->pTimer->DevNo > 0)
	{
		evtid = RE01_EVTID_AGT1_AGTI;

	}
	else
	{
		evtid = RE01_EVTID_AGT0_AGTI;

	}

	pTimerData->IrqOvr = Re01RegisterIntHandler(evtid, pCfg->IntPrio, Re01AgtUnfIRQHandler, pTimerData);
	if (pTimerData->IrqOvr == -1)
	{
		return false;
	}

	if (!Re01AgtStop(pTimerData))
	{
		return false;
	}
	pTimerData->pAgtReg->AGTCMA = 0xFFFF;
	if (pCfg->DevNo == 0)
	{
		pTimerData->pAgtReg->AGTCMB = 0xFFFF;
		ICU->WUPEN_b.AGT0CAWUPEN = 1;
	}
	else
	{
		ICU->WUPEN_b.AGT1CAWUPEN = 1;
	}
	Re01AgtReset(pTimerData->pTimer);
	return Re01AgtEnable(pTimerData->pTimer);
}


