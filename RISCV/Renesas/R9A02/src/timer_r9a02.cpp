/**-------------------------------------------------------------------------
@file	timer_r9a02.cpp

@brief	R9A02G021 timers: machine timer and timer array unit

coredev/timer.h implementation. Device numbers, from low frequency to high
frequency, are in timer_r9a02.h:

DevNo 0, machine timer (R9A02G021 User's Manual R01UH1036EJ0110, 2.5.4).
64-bit mtime counting MTCLK, the LOCO at 32768 Hz. The frequency is fixed.
One trigger on mtimecmp. A continuous trigger moves mtimecmp on from the
previous deadline, missed periods are merged into one event. The count does
not overflow. TimerDisable stops the MTCLK supply (MACTCR.ENABLE).

DevNo 1 to 8, TAU channels 0 to 7 (manual chapter 18). 16-bit down counter
in interval timer mode on CK00 or CK01, PCLKB / 2^n with n = 0 to 15. The
two prescalers are shared by the 8 channels: a channel gets the closest
frequency of a free prescaler, or of one already set by a running channel.
The interval interrupt (TAU0_ENDIn) extends the count to 64 bits, it must
be serviced at least once per period: 65536 ticks without a trigger. The
trigger is the interval period, 2 to 65536 ticks. Enabling, changing or
disabling the trigger restarts the period, which can lose one count clock.
Without a trigger the interval event is TIMER_EVT_COUNTER_OVR.

A trigger handler given to EnableTrigger takes precedence, without one the
timer event handler gets TIMER_EVT_TRIGGER(0). bTickInt and external
triggers are not supported.

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
#include <stdint.h>
#include <stddef.h>

#include "r9a02g021.h"
#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "interrupt_r9a02.h"
#include "system_r9a02.h"
#include "timer_r9a02.h"

#define R9A02_TAU_PERIOD_MAX		65536U		//!< TDR = 0xFFFF
#define R9A02_TIMER_TICKS_MIN		2U			//!< Shortest trigger period
#define R9A02_MTIMER_TICKS_MAX		(UINT64_MAX >> 2)

typedef struct __R9a02_Timer_Dev {
	TimerDev_t *pTimer;
	int IntNo;						// TAU: IELSR number, -1 when not routed
	uint8_t Chan;					// TAU channel
	bool bRunning;
	bool bTrigEn;
	TIMER_TRIG_TYPE TrigType;
	uint64_t TrigNs;				// Trigger period asked for
	uint64_t TrigTicks;				// Trigger period in ticks
	TimerTrigEvtHandler_t TrigHandler;
	void *pTrigCtx;
	uint64_t Base;					// Machine timer: mtime at count 0. TAU: count at the
									// start of the running period.
	uint64_t Deadline;				// Machine timer: mtime of the next trigger
	uint64_t Last;					// TAU: last count returned
	uint32_t Reload;				// TAU: ticks of the running period
	uint32_t NextReload;			// TAU: ticks of the next period, TDR + 1
} R9a02TimerDev_t;

static R9a02TimerDev_t s_R9a02TimerDev[R9A02_TIMER_DEV_CNT];

static void R9a02TimerEvent(R9a02TimerDev_t * const pDev, uint32_t Evt, TimerTrigEvtHandler_t Handler, void *pCtx)
{
	TimerDev_t *timer = pDev->pTimer;

	if ((Evt & TIMER_EVT_TRIGGER(0)) && Handler != NULL)
	{
		Handler(timer, 0, pCtx);
	}
	else if (timer->EvtHandler)
	{
		timer->EvtHandler(timer, Evt);
	}
}

static uint64_t R9a02MtimeRead(void)
{
	uint32_t hi;
	uint32_t lo;

	do {
		hi = R9A02_IMT->MTIME_HI;
		lo = R9A02_IMT->MTIME_LO;
	} while (hi != R9A02_IMT->MTIME_HI);

	return ((uint64_t)hi << 32) | lo;
}

// Low word to all ones first, so that the compare value never goes below
// the old and new values while the halves change
static void R9a02MtimecmpWrite(uint64_t Val)
{
	R9A02_IMT->MTIMECMP_LO = UINT32_MAX;
	R9A02_IMT->MTIMECMP_HI = (uint32_t)(Val >> 32);
	R9A02_IMT->MTIMECMP_LO = (uint32_t)Val;
}

static void R9a02MtimerIRQHandler(int IntNo, void *pCtx)
{
	R9a02TimerDev_t *dev = (R9a02TimerDev_t *)pCtx;
	TimerTrigEvtHandler_t handler = NULL;
	void *ctx = NULL;
	bool trig = false;
	uintptr_t state = DisableInterrupt();
	uint64_t now = R9a02MtimeRead();

	if (dev->bTrigEn && now >= dev->Deadline)
	{
		trig = true;
		handler = dev->TrigHandler;
		ctx = dev->pTrigCtx;
		if (dev->TrigType == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			// From the previous deadline, missed periods merged
			dev->Deadline += ((now - dev->Deadline) / dev->TrigTicks + 1U) * dev->TrigTicks;
			R9a02MtimecmpWrite(dev->Deadline);
		}
		else
		{
			dev->bTrigEn = false;
			R9a02MtimecmpWrite(UINT64_MAX);
		}
	}

	EnableInterrupt(state);

	if (trig)
	{
		R9a02TimerEvent(dev, TIMER_EVT_TRIGGER(0), handler, ctx);
	}
}

static void R9a02MtimerDisable(TimerDev_t * const pTimer)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();

	R9A02_CPU_AUX_MACTCR = 0U;
	dev->bRunning = false;

	EnableInterrupt(state);
}

static bool R9a02MtimerEnable(TimerDev_t * const pTimer)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();

	R9A02_CPU_AUX_MACTCR = R9A02_MACTCR_ENABLE;
	dev->bRunning = true;

	EnableInterrupt(state);

	return true;
}

static void R9a02MtimerReset(TimerDev_t * const pTimer)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();
	uint64_t now = R9a02MtimeRead();

	dev->Base = now;
	if (dev->bTrigEn)
	{
		dev->Deadline = now + dev->TrigTicks;
		R9a02MtimecmpWrite(dev->Deadline);
	}

	EnableInterrupt(state);
}

static uint64_t R9a02MtimerGetTickCount(TimerDev_t * const pTimer)
{
	return R9a02MtimeRead() - s_R9a02TimerDev[pTimer->DevNo].Base;
}

// MTCLK is the LOCO, the only frequency
static uint32_t R9a02MtimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	pTimer->Freq = R9A02_LOCO_FREQ;
	pTimer->nsPeriod = (1000000000ULL + R9A02_LOCO_FREQ / 2U) / R9A02_LOCO_FREQ;
	R9a02MtimerReset(pTimer);

	return pTimer->Freq;
}

static int R9a02TimerGetMaxTrigger(TimerDev_t * const pTimer)
{
	return 1;
}

static int R9a02TimerFindAvailTrigger(TimerDev_t * const pTimer)
{
	return s_R9a02TimerDev[pTimer->DevNo].bTrigEn ? -1 : 0;
}

static void R9a02MtimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	if (TrigNo != 0)
	{
		return;
	}

	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();

	dev->bTrigEn = false;
	R9a02MtimecmpWrite(UINT64_MAX);

	EnableInterrupt(state);
}

static uint64_t R9a02MtimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										 TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
										 void * const pContext)
{
	if (TrigNo != 0 || (Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}

	uint64_t ticks = TimerNanosecondToTick(pTimer, nsPeriod);

	if (ticks < R9A02_TIMER_TICKS_MIN || ticks > R9A02_MTIMER_TICKS_MAX)
	{
		return 0;
	}

	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();

	dev->TrigType = Type;
	dev->TrigNs = nsPeriod;
	dev->TrigTicks = ticks;
	dev->TrigHandler = Handler;
	dev->pTrigCtx = pContext;
	dev->Deadline = R9a02MtimeRead() + ticks;
	dev->bTrigEn = true;
	R9a02MtimecmpWrite(dev->Deadline);

	EnableInterrupt(state);

	return TimerTickToNanosecond(pTimer, ticks);
}

static void R9a02TimerDisableExtTrigger(TimerDev_t * const pTimer)
{
}

// No external trigger
static bool R9a02TimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	return false;
}

// Count with the interval period that may have ended but not been serviced.
// Called with interrupts disabled.
static uint64_t R9a02TauCount(R9a02TimerDev_t * const pDev)
{
	uint32_t c = R9A02_TAU->TCR[pDev->Chan];
	bool pend = pDev->IntNo >= 0 && (R9A02_ICU->IELSR[pDev->IntNo] & R9A02_IELSR_IR);
	uint64_t base = pDev->Base;
	uint32_t reload = pDev->Reload;

	if (pend)
	{
		// TCR was reloaded, read it again in the new period
		c = R9A02_TAU->TCR[pDev->Chan];
		base += reload;
		reload = pDev->NextReload;
	}

	// TCR is 0xFFFF before the first count clock of a start (table 18.3)
	uint64_t cnt = base + (c < reload ? reload - 1U - c : 0U);

	// A higher level interrupt can come between the dispatcher clearing IR
	// and this channel handler moving Base on
	if (cnt < pDev->Last)
	{
		cnt = pDev->Last;
	}
	pDev->Last = cnt;

	return cnt;
}

// Channel stopped, its count kept in Base. Called with interrupts disabled.
static void R9a02TauStop(R9a02TimerDev_t * const pDev)
{
	if (!pDev->bRunning)
	{
		return;
	}

	// TCR holds its value once stopped (table 18.3)
	R9A02_TAU->TT = (uint16_t)(1U << pDev->Chan);

	uint64_t cnt = R9a02TauCount(pDev);

	R9a02ClearPendingInt(pDev->IntNo);
	pDev->Base = cnt;
	pDev->Last = cnt;
	pDev->bRunning = false;
}

// Channel started from Base with an interval of Ticks. Called with
// interrupts disabled.
static void R9a02TauStart(R9a02TimerDev_t * const pDev, uint32_t Ticks)
{
	R9A02_TAU->TDR[pDev->Chan] = (uint16_t)(Ticks - 1U);
	pDev->Reload = Ticks;
	pDev->NextReload = Ticks;
	R9A02_TAU->TS = (uint16_t)(1U << pDev->Chan);
	pDev->bRunning = true;
}

static uint32_t R9a02TauPeriod(R9a02TimerDev_t * const pDev)
{
	return pDev->bTrigEn ? (uint32_t)pDev->TrigTicks : R9A02_TAU_PERIOD_MAX;
}

static void R9a02TauIRQHandler(int IntNo, void *pCtx)
{
	R9a02TimerDev_t *dev = (R9a02TimerDev_t *)pCtx;
	TimerTrigEvtHandler_t handler = NULL;
	void *ctx = NULL;
	uint32_t evt = TIMER_EVT_COUNTER_OVR;
	uintptr_t state = DisableInterrupt();

	if (!dev->bRunning)
	{
		EnableInterrupt(state);

		return;
	}

	dev->Base += dev->Reload;
	dev->Reload = dev->NextReload;

	if (dev->bTrigEn)
	{
		evt = TIMER_EVT_TRIGGER(0);
		handler = dev->TrigHandler;
		ctx = dev->pTrigCtx;
		if (dev->TrigType == TIMER_TRIG_TYPE_SINGLE)
		{
			// Back to the full period after the one just started, TDR is used
			// from the next reload (18.7.1)
			dev->bTrigEn = false;
			R9A02_TAU->TDR[dev->Chan] = (uint16_t)(R9A02_TAU_PERIOD_MAX - 1U);
			dev->NextReload = R9A02_TAU_PERIOD_MAX;
		}
	}

	EnableInterrupt(state);

	R9a02TimerEvent(dev, evt, handler, ctx);
}

static void R9a02TauDisable(TimerDev_t * const pTimer)
{
	uintptr_t state = DisableInterrupt();

	R9a02TauStop(&s_R9a02TimerDev[pTimer->DevNo]);

	EnableInterrupt(state);
}

static bool R9a02TauEnable(TimerDev_t * const pTimer)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();

	if (!dev->bRunning)
	{
		R9a02TauStart(dev, R9a02TauPeriod(dev));
	}

	EnableInterrupt(state);

	return true;
}

static void R9a02TauReset(TimerDev_t * const pTimer)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();
	bool running = dev->bRunning;

	R9a02TauStop(dev);
	dev->Base = 0;
	dev->Last = 0;
	if (running)
	{
		R9a02TauStart(dev, R9a02TauPeriod(dev));
	}

	EnableInterrupt(state);
}

static uint64_t R9a02TauGetTickCount(TimerDev_t * const pTimer)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();
	uint64_t cnt = dev->bRunning ? R9a02TauCount(dev) : dev->Base;

	EnableInterrupt(state);

	return cnt;
}

// Operation clock for a channel, the closest to Freq (0 : PCLKB) of CK00 and
// CK01. A prescaler used by another running channel keeps its setting
// (18.2.3). Returns the frequency, 0 when PCLKB is not known.
static uint32_t R9a02TauFindClock(uint32_t Chan, uint32_t Freq, uint16_t *pCks, uint32_t *pPrs)
{
	uint32_t pclk = SystemPeriphClockGet(0);

	if (pclk == 0U)
	{
		return 0;
	}
	if (Freq == 0U)
	{
		Freq = pclk;
	}

	uint16_t tps = R9A02_TAU->TPS;
	uint32_t best = 0;
	uint32_t besterr = UINT32_MAX;

	for (uint32_t ck = 0; ck < 2U; ck++)
	{
		uint16_t cks = ck ? R9A02_TAU_TMR_CKS_CK01 : R9A02_TAU_TMR_CKS_CK00;
		uint32_t pos = ck ? R9A02_TAU_TPS_PRS1_Pos : R9A02_TAU_TPS_PRS0_Pos;
		uint32_t cur = (tps >> pos) & R9A02_TAU_TPS_PRS_Msk;
		bool used = false;

		for (uint32_t k = 0; k < R9A02_TAU_CHAN_COUNT; k++)
		{
			if (k != Chan && (R9A02_TAU->TE & (1U << k)) && (R9A02_TAU->TMR[k] & R9A02_TAU_TMR_CKS_Msk) == cks)
			{
				used = true;
			}
		}

		for (uint32_t prs = 0; prs <= R9A02_TAU_TPS_PRS_Msk; prs++)
		{
			if (used && prs != cur)
			{
				continue;
			}

			uint32_t f = pclk >> prs;
			uint32_t err = f > Freq ? f - Freq : Freq - f;

			if (err < besterr)
			{
				besterr = err;
				best = f;
				*pCks = cks;
				*pPrs = prs;
			}
		}
	}

	return best;
}

// Operation clock written, the channel is stopped
static void R9a02TauSetClock(uint32_t Chan, uint16_t Cks, uint32_t Prs)
{
	uint32_t pos = Cks == R9A02_TAU_TMR_CKS_CK01 ? R9A02_TAU_TPS_PRS1_Pos : R9A02_TAU_TPS_PRS0_Pos;
	uint16_t tps = R9A02_TAU->TPS;

	if (((tps >> pos) & R9A02_TAU_TPS_PRS_Msk) != Prs)
	{
		R9A02_TAU->TPS = (uint16_t)((tps & ~(R9A02_TAU_TPS_PRS_Msk << pos)) | (Prs << pos));
	}

	// Interval timer mode, software start, no interrupt at start, 16-bit
	R9A02_TAU->TMR[Chan] = (uint16_t)(Cks | R9A02_TAU_TMR_MD_INTERVAL);
}

static uint32_t R9a02TauSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uint16_t cks = R9A02_TAU_TMR_CKS_CK00;
	uint32_t prs = 0;
	uintptr_t state = DisableInterrupt();
	uint32_t f = R9a02TauFindClock(dev->Chan, Freq, &cks, &prs);
	uint64_t ticks = 0;

	if (f != 0U && dev->bTrigEn)
	{
		// The active trigger must fit the new frequency
		uint64_t s = dev->TrigNs / 1000000000ULL;
		uint64_t r = dev->TrigNs % 1000000000ULL;

		ticks = s * f + (r * f + 500000000ULL) / 1000000000ULL;
		if (ticks < R9A02_TIMER_TICKS_MIN || ticks > R9A02_TAU_PERIOD_MAX)
		{
			f = 0;
		}
	}

	if (f == 0U)
	{
		EnableInterrupt(state);

		return 0;
	}

	bool running = dev->bRunning;

	R9a02TauStop(dev);
	R9a02TauSetClock(dev->Chan, cks, prs);
	pTimer->Freq = f;
	pTimer->nsPeriod = (1000000000ULL + f / 2U) / f;
	if (dev->bTrigEn)
	{
		dev->TrigTicks = ticks;
	}
	dev->Base = 0;
	dev->Last = 0;
	if (running)
	{
		R9a02TauStart(dev, R9a02TauPeriod(dev));
	}

	EnableInterrupt(state);

	return f;
}

static void R9a02TauDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];

	if (TrigNo != 0 || !dev->bTrigEn)
	{
		return;
	}

	uintptr_t state = DisableInterrupt();
	bool running = dev->bRunning;

	R9a02TauStop(dev);
	dev->bTrigEn = false;
	if (running)
	{
		R9a02TauStart(dev, R9A02_TAU_PERIOD_MAX);
	}

	EnableInterrupt(state);
}

static uint64_t R9a02TauEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
									  TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
									  void * const pContext)
{
	if (TrigNo != 0 || (Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}

	uint64_t ticks = TimerNanosecondToTick(pTimer, nsPeriod);

	if (ticks < R9A02_TIMER_TICKS_MIN || ticks > R9A02_TAU_PERIOD_MAX)
	{
		return 0;
	}

	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pTimer->DevNo];
	uintptr_t state = DisableInterrupt();
	bool running = dev->bRunning;

	R9a02TauStop(dev);
	dev->TrigType = Type;
	dev->TrigNs = nsPeriod;
	dev->TrigTicks = ticks;
	dev->TrigHandler = Handler;
	dev->pTrigCtx = pContext;
	dev->bTrigEn = true;
	if (running)
	{
		R9a02TauStart(dev, (uint32_t)ticks);
	}

	EnableInterrupt(state);

	return TimerTickToNanosecond(pTimer, ticks);
}

static bool R9a02MtimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[R9A02_TIMER_MTIMER_DEVNO];

	// MTCLK is the LOCO (8.5.9)
	if ((pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT && pCfg->ClkSrc != TIMER_CLKSRC_LFRC) ||
		(R9A02_SYSC_LOCOCR & R9A02_LOCOCR_LCSTP))
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	R9a02UnregisterCoreIntHandler(R9A02_CLIC_ID_MTIP);
	dev->bTrigEn = false;
	R9a02MtimecmpWrite(UINT64_MAX);
	R9A02_CPU_AUX_MACTCR = R9A02_MACTCR_ENABLE;

	pTimer->DevNo = R9A02_TIMER_MTIMER_DEVNO;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = R9a02MtimerDisable;
	pTimer->Enable = R9a02MtimerEnable;
	pTimer->Reset = R9a02MtimerReset;
	pTimer->GetTickCount = R9a02MtimerGetTickCount;
	pTimer->SetFrequency = R9a02MtimerSetFrequency;
	pTimer->GetMaxTrigger = R9a02TimerGetMaxTrigger;
	pTimer->FindAvailTrigger = R9a02TimerFindAvailTrigger;
	pTimer->DisableTrigger = R9a02MtimerDisableTrigger;
	pTimer->EnableTrigger = R9a02MtimerEnableTrigger;
	pTimer->DisableExtTrigger = R9a02TimerDisableExtTrigger;
	pTimer->EnableExtTrigger = R9a02TimerEnableExtTrigger;

	dev->pTimer = pTimer;
	dev->IntNo = -1;
	dev->bRunning = true;
	R9a02MtimerSetFrequency(pTimer, pCfg->Freq);

	bool ok = R9a02RegisterCoreIntHandler(R9A02_CLIC_ID_MTIP, pCfg->IntPrio, R9a02MtimerIRQHandler, dev);

	if (!ok)
	{
		R9A02_CPU_AUX_MACTCR = 0U;
		dev->bRunning = false;
		dev->pTimer = NULL;
	}

	EnableInterrupt(state);

	return ok;
}

static bool R9a02TauInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	R9a02TimerDev_t *dev = &s_R9a02TimerDev[pCfg->DevNo];
	uint32_t chan = (uint32_t)pCfg->DevNo - R9A02_TIMER_TAU_DEVNO(0);
	uint8_t cksel = R9A02_SYSC_SCKSCR & R9A02_SCKSCR_CKSEL_Msk;

	// The count clock comes from PCLKB, the clock source is the core one
	switch (pCfg->ClkSrc)
	{
		case TIMER_CLKSRC_DEFAULT:
			break;
		case TIMER_CLKSRC_HFRC:
			if (cksel != R9A02_SCKSCR_HOCO && cksel != R9A02_SCKSCR_MOCO)
			{
				return false;
			}
			break;
		case TIMER_CLKSRC_EXT:
			if (cksel != R9A02_SCKSCR_EXTAL)
			{
				return false;
			}
			break;
		default:
			return false;
	}

	uintptr_t state = DisableInterrupt();

	R9a02ModuleStart(R9A02_MSTPCR_D, R9A02_MSTPCRD_TAU);

	// A channel running for something else is not taken over
	if ((R9A02_TAU->TE & (1U << chan)) && dev->pTimer == NULL)
	{
		EnableInterrupt(state);

		return false;
	}

	// IntNo is valid once the device was initialized
	if (dev->pTimer != NULL)
	{
		R9a02UnregisterIntHandler(dev->IntNo);
	}
	dev->IntNo = -1;
	dev->bRunning = false;
	R9A02_TAU->TT = (uint16_t)(1U << chan);
	R9A02_TAU->TOE = (uint16_t)(R9A02_TAU->TOE & ~(1U << chan));

	uint16_t cks = R9A02_TAU_TMR_CKS_CK00;
	uint32_t prs = 0;
	uint32_t f = R9a02TauFindClock(chan, pCfg->Freq, &cks, &prs);

	if (f == 0U)
	{
		EnableInterrupt(state);

		return false;
	}

	R9a02TauSetClock(chan, cks, prs);

	pTimer->DevNo = pCfg->DevNo;
	pTimer->Freq = f;
	pTimer->nsPeriod = (1000000000ULL + f / 2U) / f;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->Disable = R9a02TauDisable;
	pTimer->Enable = R9a02TauEnable;
	pTimer->Reset = R9a02TauReset;
	pTimer->GetTickCount = R9a02TauGetTickCount;
	pTimer->SetFrequency = R9a02TauSetFrequency;
	pTimer->GetMaxTrigger = R9a02TimerGetMaxTrigger;
	pTimer->FindAvailTrigger = R9a02TimerFindAvailTrigger;
	pTimer->DisableTrigger = R9a02TauDisableTrigger;
	pTimer->EnableTrigger = R9a02TauEnableTrigger;
	pTimer->DisableExtTrigger = R9a02TimerDisableExtTrigger;
	pTimer->EnableExtTrigger = R9a02TimerEnableExtTrigger;

	dev->pTimer = pTimer;
	dev->Chan = (uint8_t)chan;
	dev->bRunning = false;
	dev->bTrigEn = false;
	dev->Base = 0;
	dev->Last = 0;
	dev->IntNo = R9a02RegisterIntHandler((uint8_t)(R9A02_EVT_TAU0_ENDI0 + chan), pCfg->IntPrio,
										 R9a02TauIRQHandler, dev);
	if (dev->IntNo < 0)
	{
		dev->pTimer = NULL;
		EnableInterrupt(state);

		return false;
	}

	R9a02TauStart(dev, R9A02_TAU_PERIOD_MAX);

	EnableInterrupt(state);

	return true;
}

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= (int)R9A02_TIMER_DEV_CNT ||
		pCfg->bTickInt || pCfg->IntPrio < 0 || pCfg->IntPrio > (int)R9A02_CLIC_PRIO_MAX)
	{
		return false;
	}

	// One timer object per device
	TimerDev_t *owner = s_R9a02TimerDev[pCfg->DevNo].pTimer;

	if (owner != NULL && owner != pTimer)
	{
		return false;
	}

	if (pCfg->DevNo == R9A02_TIMER_MTIMER_DEVNO)
	{
		return R9a02MtimerInit(pTimer, pCfg);
	}

	return R9a02TauInit(pTimer, pCfg);
}

int TimerGetLowFreqDevCount(void)
{
	return R9A02_TIMER_LOWFREQ_CNT;
}

int TimerGetHighFreqDevCount(void)
{
	return R9A02_TIMER_HIGHFREQ_CNT;
}

int TimerGetHighFreqDevNo(void)
{
	return R9A02_TIMER_TAU_DEVNO(0);
}
