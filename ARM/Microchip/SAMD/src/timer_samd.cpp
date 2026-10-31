/**-------------------------------------------------------------------------
@file	timer_samd.cpp

@brief	SAM D11/D21 timer implementation

		DevNo 0 is the RTC, 32-bit counter on the 32.768 kHz generator 1,
		1 trigger. Then the TC, 16-bit counters on the main clock, 2
		triggers each: TC1, TC2 on SAM D11, TC3 to TC7 on SAM D21. Then the
		TCC on the main clock, a trigger per compare channel: TCC0 (24-bit,
		4) on SAM D11; TCC0 (24-bit, 4), TCC1 (24-bit, 2) and TCC2 (16-bit,
		2) on SAM D21.

		Frequencies: RTC 32768 Hz divided by 1 to 1024 in powers of 2,
		default 32768 Hz. TC and TCC main clock divided by 1, 2, 4, 8, 16,
		64, 256 or 1024, default main clock / 16 (3 MHz at 48 MHz).

		The counters are extended to 64 bits by the overflow interrupt, which
		must be served within half a counter period. Trigger deadlines are 64
		bits, the compare holds their low bits, a match that is not yet the
		deadline is ignored. A trigger period is at least 4 counts.

		The RTC count is read continuously synchronized, late by a few counts
		of 32768 Hz. A compare match within that distance of the deadline
		counts as the deadline. An RTC register write takes up to 6 periods
		of 32.768 kHz, about 200 us, to synchronize: the functions wait for
		it with the interrupts as the caller has them, and disable the
		interrupts only for the register accesses. A call on the RTC timer
		can still take a few hundred microseconds, the TC is the timer for
		short latency.

		The trigger handler is called first, then EvtHandler with
		TIMER_EVT_TRIGGER(n). EvtHandler also gets TIMER_EVT_COUNTER_OVR.

		Tick interrupt (bTickInt), RTC only: EvtHandler gets TIMER_EVT_TICK at
		each count, from the RTC periodic event at the count frequency
		through a resynchronized event channel and the EVSYS interrupt. The
		periodic events start at 32768 Hz / 8: with the tick the RTC
		frequency is 4096 Hz at most, the default too. TimerInit fails with
		bTickInt on a TC or TCC.

		External trigger, TC and TCC: TrigDevNo is an EIC line, its edges go
		through an asynchronous event channel to the capture input of the
		last compare channel (CC1 of a TC, CC3 of TCC0, CC1 of TCC1 and
		TCC2), which is then not a trigger. The application sets the pin of
		the line to the EIC function A (IOPINOP_FUNC0). EvtHandler gets
		TIMER_EVT_EXTTRIG with the count captured at the edge in LastCount
		(low 32 bits). A capture is extended from the count of the interrupt,
		it must be served within a counter turn. A second edge before the
		capture is read is lost. The RTC has no capture.

@author	Hoang Nguyen Hoan
@date	Oct. 29, 2026

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
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "samdxxx.h"

#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "clock_samd.h"
#include "evsys_samd.h"
#include "timer_samd.h"

#define SAMD_TIMER_MIN_TICKS		4U
#define SAMD_TIMER_LOCK_TRIES		16U
#define SAMD_TIMER_ARM_TRIES		100000U
#define SAMD_RTC_PRESC_MAX			10U		// 32768 Hz / 1024
#define SAMD_TC_PRESC_MAX			7U
#define SAMD_TC_PRESC_DEFAULT		4U		// Main clock / 16
#define SAMD_RTC_TICK_PRESC_MIN		3U		// 4096 Hz, periodic event 0

// TC prescaler divisions, PRESCALER field values 0 to 7
static const uint16_t s_SamdTcDiv[SAMD_TC_PRESC_MAX + 1] = { 1, 2, 4, 8, 16, 64, 256, 1024 };

static const SamdTimerOps_t s_SamdRtcOps = {
	SamdRtcSetup, SamdRtcRun, SamdRtcReset, SamdRtcRead, SamdRtcOverflow, SamdRtcArm, SamdRtcMatched,
	NULL, NULL,
};

static const SamdTimerOps_t s_SamdTcOps = {
	SamdTcSetup, SamdTcRun, SamdTcReset, SamdTcRead, SamdTcOverflow, SamdTcArm, SamdTcMatched,
	SamdTcCapture, SamdTcCaptured,
};

static const SamdTimerOps_t s_SamdTccOps = {
	SamdTccSetup, SamdTccRun, SamdTccReset, SamdTccRead, SamdTccOverflow, SamdTccArm, SamdTccMatched,
	SamdTccCapture, SamdTccCaptured,
};

// Hardware of each DevNo, copied to its data by TimerInit
typedef struct {
	const SamdTimerOps_t *pOps;
	void *pReg;
	IRQn_Type IrqNo;
	uint32_t ApbMask;
	uint8_t GclkId;
	uint8_t NbTrig;
	uint8_t CntBits;
	uint8_t TrigIdx;			//!< First trigger in s_SamdTimerTrig
	int8_t EvUser;				//!< Event user of the capture channel, -1 none
} SamdTimerHw_t;

#define SAMD_TC_HW(n, Gclk, Trig, User)	{ &s_SamdTcOps, &TC##n##_REGS->COUNT16, TC##n##_IRQn, \
										  PM_APBCMASK_TC##n##_Msk, Gclk, SAMD_TC_TIMER_TRIG_MAXCNT, 16, Trig, User }
#define SAMD_TCC_HW(n, Gclk, Trig, User)	{ &s_SamdTccOps, TCC##n##_REGS, TCC##n##_IRQn, PM_APBCMASK_TCC##n##_Msk, \
										  Gclk, TCC##n##_CC_NUM, TCC##n##_SIZE, Trig, User + TCC##n##_CC_NUM - 1 }

static const SamdTimerHw_t s_SamdTimerHw[SAMD_TIMER_MAXCNT] = {
	{ &s_SamdRtcOps, NULL, RTC_IRQn, 0, GCLK_CLKCTRL_ID_RTC_Val, SAMD_RTC_TIMER_TRIG_MAXCNT, 32, 0, -1 },
#if defined(TC3_REGS)
	SAMD_TC_HW(3, GCLK_CLKCTRL_ID_TCC2_TC3_Val, 1, SAMD_EVSYS_USER_TC3_EVU),
	SAMD_TC_HW(4, GCLK_CLKCTRL_ID_TC4_TC5_Val, 3, SAMD_EVSYS_USER_TC3_EVU + 1),
	SAMD_TC_HW(5, GCLK_CLKCTRL_ID_TC4_TC5_Val, 5, SAMD_EVSYS_USER_TC3_EVU + 2),
#if SAMD_TC_TIMER_MAXCNT > 3
	SAMD_TC_HW(6, GCLK_CLKCTRL_ID_TC6_TC7_Val, 7, SAMD_EVSYS_USER_TC3_EVU + 3),
	SAMD_TC_HW(7, GCLK_CLKCTRL_ID_TC6_TC7_Val, 9, SAMD_EVSYS_USER_TC3_EVU + 4),
#endif
	SAMD_TCC_HW(0, GCLK_CLKCTRL_ID_TCC0_TCC1_Val, 1 + 2 * SAMD_TC_TIMER_MAXCNT, SAMD_EVSYS_USER_TCC0_MC0),
	SAMD_TCC_HW(1, GCLK_CLKCTRL_ID_TCC0_TCC1_Val, 1 + 2 * SAMD_TC_TIMER_MAXCNT + TCC0_CC_NUM,
				SAMD_EVSYS_USER_TCC1_MC0),
	SAMD_TCC_HW(2, GCLK_CLKCTRL_ID_TCC2_TC3_Val, 1 + 2 * SAMD_TC_TIMER_MAXCNT + TCC0_CC_NUM + TCC1_CC_NUM,
				SAMD_EVSYS_USER_TCC2_MC0),
#else
	SAMD_TC_HW(1, GCLK_CLKCTRL_ID_TC1_TC2_Val, 1, SAMD_EVSYS_USER_TC1_EVU),
	SAMD_TC_HW(2, GCLK_CLKCTRL_ID_TC1_TC2_Val, 3, SAMD_EVSYS_USER_TC1_EVU + 1),
	SAMD_TCC_HW(0, GCLK_CLKCTRL_ID_TCC0_Val, 5, SAMD_EVSYS_USER_TCC0_MC0),
#endif
};

SamdTimerData_t g_SamdTimerData[SAMD_TIMER_MAXCNT];

static bool SamdTimerTickOpen(SamdTimerData_t *pData, int IntPrio);

static SamdTimerTrig_t s_SamdTimerTrig[SAMD_TIMER_TRIG_CNT];

static inline bool SamdTimerIsRtc(const SamdTimerData_t *pData)
{
	return pData->pReg == NULL;
}

// Compare channel of the external trigger capture, -1 none
static inline int SamdTimerCaptChan(const SamdTimerData_t *pData)
{
	return pData->bCapture ? pData->NbTrig - 1 : -1;
}

// Device data of a timer, NULL if the timer is not bound to it
static SamdTimerData_t *SamdTimerGet(TimerDev_t * const pTimer)
{
	if (pTimer == NULL || pTimer->DevNo < 0 || pTimer->DevNo >= SAMD_TIMER_MAXCNT)
	{
		return NULL;
	}

	SamdTimerData_t *d = &g_SamdTimerData[pTimer->DevNo];

	return d->pTimer == pTimer ? d : NULL;
}

static inline int SamdTimerTrigCnt(const SamdTimerData_t *pData)
{
	return pData->NbTrig;
}

static inline uint64_t SamdTimerSpan(const SamdTimerData_t *pData)
{
	return 1ULL << pData->CntBits;
}

// RTC counts from a compare write to a compare in effect, seen from a count
// read: the count is late by up to a lag, the compare set up to a lag after
// the write
static inline uint32_t SamdTimerMargin(SamdTimerData_t *pData)
{
	return 2U * SamdRtcLag(pData) + 1U;
}

static inline bool SamdTimerRead(SamdTimerData_t *pData, uint32_t *pCount)
{
	return pData->pOps->Read(pData, pCount);
}

// Count extended to 64 bits, never lower than the last one. Interrupts
// disabled. False when the count cannot be read now: the RTC is
// synchronizing, or the count was read before an overflow just taken.
static bool SamdTimerNow(SamdTimerData_t *pData, uint64_t *pNow)
{
	TimerDev_t *timer = pData->pTimer;
	uint64_t span = SamdTimerSpan(pData);
	uint32_t count;

	if (!SamdTimerRead(pData, &count))
	{
		return false;
	}

	// The RTC count is readable once the reset is in effect: an overflow
	// flag from before it is dropped
	if (pData->bResetOvf)
	{
		(void)SamdRtcOverflow(pData);
		pData->bResetOvf = false;
	}

	if (pData->pOps->Overflow(pData))
	{
		timer->Rollover += span;
		pData->bOverflow = true;

		// The interrupt reports the event
		NVIC_SetPendingIRQ(pData->IrqNo);

		// A count near the top was read before the overflow. The RTC count
		// is not readable after the flag write until the next SamdRtcWait.
		if (count >= span / 2 && !SamdTimerRead(pData, &count))
		{
			return false;
		}
	}

	uint64_t res = timer->Rollover + count;

	if (res < pData->Floor)
	{
		res = pData->Floor;
	}
	pData->Floor = res;
	timer->LastCount = (uint32_t)res;
	*pNow = res;

	return true;
}

// Interrupts disabled and the count read, before a change of the timer
// state. The RTC synchronizations are waited for before, with the interrupts
// as the caller has them. Fresh: the count is read after the synchronization
// in progress, a register write before has taken effect. False if the count
// could not be read, the interrupts are as they were.
static bool SamdTimerLock(SamdTimerData_t *pData, uint32_t *pState, uint64_t *pNow, bool bFresh)
{
	for (uint32_t i = 0; i < SAMD_TIMER_LOCK_TRIES; i++)
	{
		// Again after a failed read: fresh, the count may be older than the
		// flags
		if (SamdTimerIsRtc(pData) && !SamdRtcWait(pData, bFresh || i > 0))
		{
			break;
		}

		*pState = DisableInterrupt();
		if (SamdTimerNow(pData, pNow))
		{
			return true;
		}
		EnableInterrupt(*pState);

		// The TC and TCC reads fail only on a synchronization timeout
		if (!SamdTimerIsRtc(pData))
		{
			break;
		}
	}

	return false;
}

// End of a SamdTimerLock, the RTC continuous read started again after the
// register writes
static void SamdTimerUnlock(SamdTimerData_t *pData, uint32_t State)
{
	if (SamdTimerIsRtc(pData))
	{
		SamdRtcResume(pData);
	}
	EnableInterrupt(State);
}

static void SamdTimerNvic(SamdTimerData_t *pData, bool bEnable)
{
	if (bEnable)
	{
		NVIC_EnableIRQ(pData->IrqNo);
	}
	else
	{
		NVIC_DisableIRQ(pData->IrqNo);
	}
}

// Counter started or stopped, with its interrupt. A failed stop still turns
// the interrupt off.
static bool SamdTimerRun(SamdTimerData_t *pData, bool bRun)
{
	uint32_t state;
	uint64_t now;
	bool res = false;

	if (SamdTimerLock(pData, &state, &now, false))
	{
		res = pData->pOps->Run(pData, bRun);
	}
	else
	{
		state = DisableInterrupt();
	}

	pData->bRunning = bRun && res;
	SamdTimerNvic(pData, pData->bRunning);
	SamdTimerUnlock(pData, state);

	// SYNCRDY is not set at the end of an enable synchronization: the
	// continuous read is started here
	if (SamdTimerIsRtc(pData))
	{
		(void)SamdRtcWait(pData, false);
	}

	return res;
}

// Compare of a trigger set from its deadline, or cleared. Interrupts
// disabled, Count from SamdTimerLock. A TC or TCC deadline already passed when the
// compare is set would wait for the next counter turn: the trigger is due
// now. An RTC compare takes effect a synchronization later: pCheck is set
// when the deadline is close enough to pass before, SamdTimerCheck is then
// called after the interrupts are enabled.
static bool SamdTimerArm(SamdTimerData_t *pData, int TrigNo, bool bEnable, uint64_t Count, bool *pCheck)
{
	SamdTimerTrig_t *trig = &pData->pTrig[TrigNo];
	bool res = pData->pOps->Arm(pData, TrigNo, bEnable);
	uint64_t now;

	trig->bDue = false;
	trig->Seq++;
	*pCheck = false;

	if (!res || !bEnable)
	{
		return res;
	}

	if (SamdTimerIsRtc(pData))
	{
		pData->Compare = (uint32_t)trig->Deadline;
		*pCheck = trig->Deadline <= Count + SamdTimerMargin(pData);
	}
	else if (SamdTimerNow(pData, &now) && now >= trig->Deadline)
	{
		trig->bDue = true;
		NVIC_SetPendingIRQ(pData->IrqNo);
	}

	return res;
}

// RTC trigger with a close deadline, once its compare is in effect: a
// deadline already passed is due now. Seq tells the compare is still the
// one set.
static void SamdTimerCheck(SamdTimerData_t *pData, int TrigNo, uint32_t Seq)
{
	SamdTimerTrig_t *trig = &pData->pTrig[TrigNo];
	uint32_t state;
	uint64_t now;

	if (!SamdTimerLock(pData, &state, &now, true))
	{
		return;
	}

	if (trig->Seq == Seq && trig->bActive && !trig->bDue && now >= trig->Deadline)
	{
		trig->bDue = true;
		NVIC_SetPendingIRQ(pData->IrqNo);
	}
	SamdTimerUnlock(pData, state);
}

// SamdTimerLock before a new RTC compare. The old compare still matches
// until the new one is in effect, after its flag is cleared: an old value
// just ahead of the count would be taken for the new deadline. The new
// compare waits for the count to pass it, at most the margin, about 520 us
// at 32.768 kHz. The interrupt sets a compare after its deadline, already
// passed, without this.
static bool SamdTimerLockArm(SamdTimerData_t *pData, uint32_t *pState, uint64_t *pNow)
{
	for (uint32_t i = 0; i < SAMD_TIMER_ARM_TRIES; i++)
	{
		if (!SamdTimerLock(pData, pState, pNow, false))
		{
			return false;
		}
		if (!SamdTimerIsRtc(pData) || !pData->bRunning ||
			pData->Compare - (uint32_t)*pNow > SamdTimerMargin(pData))
		{
			return true;
		}
		SamdTimerUnlock(pData, *pState);
	}

	return SamdTimerLock(pData, pState, pNow, false);
}

// Prescaler and frequency the closest to the request, 0 for the default
static bool SamdTimerPlan(SamdTimerData_t *pData, uint32_t Request, uint32_t *pPresc, uint32_t *pFreq)
{
	uint32_t best = UINT32_MAX;
	uint32_t nb = SamdTimerIsRtc(pData) ? SAMD_RTC_PRESC_MAX + 1 : SAMD_TC_PRESC_MAX + 1;

	if (pData->BaseFreq == 0)
	{
		return false;
	}

	// The RTC tick is the periodic event from prescaler 3 on
	uint32_t first = SamdTimerIsRtc(pData) && pData->bTick ? SAMD_RTC_TICK_PRESC_MIN : 0U;

	if (Request == 0)
	{
		if (SamdTimerIsRtc(pData))
		{
			*pPresc = first;
			*pFreq = pData->BaseFreq >> first;
		}
		else
		{
			*pPresc = SAMD_TC_PRESC_DEFAULT;
			*pFreq = pData->BaseFreq / s_SamdTcDiv[SAMD_TC_PRESC_DEFAULT];
		}

		return true;
	}

	*pFreq = 0;
	for (uint32_t p = first; p < nb; p++)
	{
		uint32_t f = SamdTimerIsRtc(pData) ? pData->BaseFreq >> p : pData->BaseFreq / s_SamdTcDiv[p];
		uint32_t diff = f > Request ? f - Request : Request - f;

		if (f != 0 && diff < best)
		{
			best = diff;
			*pPresc = p;
			*pFreq = f;
		}
	}

	return *pFreq != 0;
}

// Counts of a period, rounded, the closest supported value
static bool SamdTimerPeriod(uint32_t Freq, uint64_t nsPeriod, uint32_t *pTicks, uint64_t *pActual)
{
	uint64_t ticks = UINT32_MAX;

	if (Freq == 0 || nsPeriod == 0)
	{
		return false;
	}

	// Whole seconds and the rest apart, the product would overflow otherwise
	if (nsPeriod / 1000000000ULL <= UINT32_MAX / Freq)
	{
		ticks = nsPeriod / 1000000000ULL * Freq + (nsPeriod % 1000000000ULL * Freq + 500000000ULL) / 1000000000ULL;
	}
	if (ticks > UINT32_MAX)
	{
		ticks = UINT32_MAX;
	}
	if (ticks < SAMD_TIMER_MIN_TICKS)
	{
		ticks = SAMD_TIMER_MIN_TICKS;
	}

	*pTicks = (uint32_t)ticks;
	*pActual = (ticks * 1000000000ULL + Freq / 2) / Freq;

	return true;
}

// Counter at zero, the active triggers set again from zero. The counter
// keeps running, or stays stopped.
static bool SamdTimerResetCounter(SamdTimerData_t *pData)
{
	TimerDev_t *timer = pData->pTimer;
	uint32_t state;
	uint64_t count;
	bool check;

	if (!SamdTimerLock(pData, &state, &count, false))
	{
		return false;
	}

	bool res = pData->pOps->Reset(pData);

	timer->LastCount = 0;
	timer->Rollover = 0;
	pData->Floor = 0;
	pData->bOverflow = false;
	pData->bResetOvf = SamdTimerIsRtc(pData);
	pData->Epoch++;
	NVIC_ClearPendingIRQ(pData->IrqNo);

	uint32_t epoch = pData->Epoch;

	for (int i = 0; i < SamdTimerTrigCnt(pData); i++)
	{
		pData->pTrig[i].Deadline = pData->pTrig[i].Ticks;
		if (i != SamdTimerCaptChan(pData))
		{
			(void)SamdTimerArm(pData, i, false, 0, &check);
		}
	}
	SamdTimerUnlock(pData, state);

	// The compares are set from the count after the reset, one at a time
	for (int i = 0; res && i < SamdTimerTrigCnt(pData); i++)
	{
		uint32_t seq = 0;

		check = false;
		if (!pData->pTrig[i].bActive)
		{
			continue;
		}
		if (!SamdTimerLockArm(pData, &state, &count))
		{
			return false;
		}
		if (epoch == pData->Epoch && pData->pTrig[i].bActive)
		{
			res = SamdTimerArm(pData, i, true, count, &check);
			seq = pData->pTrig[i].Seq;
		}
		SamdTimerUnlock(pData, state);

		if (check)
		{
			SamdTimerCheck(pData, i, seq);
		}
	}

	return res;
}

static void SamdTimerDisable(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);

	if (d != NULL)
	{
		(void)SamdTimerRun(d, false);
	}
}

static bool SamdTimerEnable(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);

	return d != NULL && SamdTimerRun(d, true);
}

static void SamdTimerReset(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);

	if (d != NULL)
	{
		(void)SamdTimerResetCounter(d);
	}
}

static uint64_t SamdTimerGetTickCount(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);
	uint32_t state;
	uint64_t count;

	if (d == NULL)
	{
		return 0;
	}

	if (!SamdTimerLock(d, &state, &count, false))
	{
		// The last count
		state = DisableInterrupt();
		count = d->Floor;
	}
	SamdTimerUnlock(d, state);

	return count;
}

// The capture channel of the external trigger is not a trigger
static int SamdTimerGetMaxTrigger(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);

	if (d == NULL)
	{
		return 0;
	}

	return SamdTimerTrigCnt(d) - (d->bCapture ? 1 : 0);
}

// New prescaler: the counter restarts from zero, the trigger periods are
// converted to the new frequency
static uint32_t SamdTimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);
	uint32_t presc;
	uint32_t freq;

	if (d == NULL || !SamdTimerPlan(d, Freq, &presc, &freq))
	{
		return 0;
	}

	// The prescaler is written with the counter stopped
	if (!SamdTimerRun(d, false))
	{
		return 0;
	}

	uint32_t state = DisableInterrupt();

	d->Prescaler = presc;
	d->Compare = 0;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + freq / 2) / freq;

	for (int i = 0; i < SamdTimerTrigCnt(d); i++)
	{
		SamdTimerTrig_t *trig = &d->pTrig[i];

		if (trig->bActive)
		{
			(void)SamdTimerPeriod(freq, trig->Info.nsPeriod, &trig->Ticks, &trig->Info.nsPeriod);
		}
	}
	EnableInterrupt(state);

	bool res = d->pOps->Setup(d) && SamdTimerTickOpen(d, (int)NVIC_GetPriority(d->IrqNo)) &&
			   SamdTimerResetCounter(d) && SamdTimerRun(d, true);

	return res ? freq : 0;
}

static int SamdTimerFindAvailTrigger(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);
	int res = -1;

	if (d == NULL)
	{
		return -1;
	}

	uint32_t state = DisableInterrupt();
	for (int i = 0; i < SamdTimerTrigCnt(d); i++)
	{
		if (!d->pTrig[i].bActive && i != SamdTimerCaptChan(d))
		{
			res = i;
			break;
		}
	}
	EnableInterrupt(state);

	return res;
}

static void SamdTimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);
	bool check;

	if (d == NULL || TrigNo < 0 || TrigNo >= SamdTimerTrigCnt(d) || TrigNo == SamdTimerCaptChan(d))
	{
		return;
	}

	// No synchronized register written, no wait
	uint32_t state = DisableInterrupt();
	d->pTrig[TrigNo].bActive = false;
	(void)SamdTimerArm(d, TrigNo, false, 0, &check);
	SamdTimerUnlock(d, state);
}

static uint64_t SamdTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
									   TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
									   void * const pContext)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);
	uint32_t ticks;
	uint64_t actual;
	uint32_t state;
	uint64_t count;
	bool check = false;
	uint32_t seq = 0;

	if (d == NULL || TrigNo < 0 || TrigNo >= SamdTimerTrigCnt(d) || TrigNo == SamdTimerCaptChan(d) ||
		(Type != TIMER_TRIG_TYPE_SINGLE && Type != TIMER_TRIG_TYPE_CONTINUOUS) ||
		!SamdTimerPeriod(pTimer->Freq, nsPeriod, &ticks, &actual))
	{
		return 0;
	}

	if (!SamdTimerLockArm(d, &state, &count))
	{
		return 0;
	}

	SamdTimerTrig_t *trig = &d->pTrig[TrigNo];

	trig->Info.Type = Type;
	trig->Info.nsPeriod = actual;
	trig->Info.Handler = Handler;
	trig->Info.pContext = pContext;
	trig->Ticks = ticks;
	trig->Deadline = count + ticks;
	trig->bActive = true;

	if (SamdTimerArm(d, TrigNo, true, count, &check))
	{
		seq = trig->Seq;
	}
	else
	{
		trig->bActive = false;
		actual = 0;
	}
	SamdTimerUnlock(d, state);

	if (check)
	{
		SamdTimerCheck(d, TrigNo, seq);
	}

	return actual;
}

// Capture of the last compare channel on or off, with the counter state
// kept. Interrupts disabled for the register accesses.
static bool SamdTimerCapture(SamdTimerData_t *pData, bool bEnable)
{
	uint32_t state = DisableInterrupt();

	pData->bCapture = bEnable;
	bool res = pData->pOps->Capture(pData, bEnable);

	EnableInterrupt(state);

	return res;
}

// External trigger off: no event from the EIC line, no capture, channel
// released
static void SamdTimerExtOff(SamdTimerData_t *pData)
{
	if (!pData->bExtOn)
	{
		return;
	}

	(void)SamdEicEvent(pData->ExtLine, IOPINSENSE_DISABLE);
	(void)SamdTimerCapture(pData, false);
	SamdEvsysClose(pData->ExtChan, pData->EvUser);
	pData->bExtOn = false;
}

static void SamdTimerDisableExtTrigger(TimerDev_t * const pTimer)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);

	if (d != NULL)
	{
		SamdTimerExtOff(d);
	}
}

// The EIC line TrigDevNo goes through an asynchronous event channel to the
// capture input of the last compare channel, which is then not a trigger.
// The RTC has no capture.
static bool SamdTimerEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	SamdTimerData_t *d = SamdTimerGet(pTimer);
	IOPINSENSE sense;

	if (d == NULL)
	{
		return false;
	}

	switch (Sense)
	{
		case TIMER_EXTTRIG_SENSE_DISABLE:
			SamdTimerExtOff(d);
			return true;
		case TIMER_EXTTRIG_SENSE_LOW_TRANSITION:
			sense = IOPINSENSE_LOW_TRANSITION;
			break;
		case TIMER_EXTTRIG_SENSE_HIGH_TRANSITION:
			sense = IOPINSENSE_HIGH_TRANSITION;
			break;
		case TIMER_EXTTRIG_SENSE_TOGGLE:
			sense = IOPINSENSE_TOGGLE;
			break;
		default:
			return false;
	}

	if (d->pOps->Capture == NULL || d->EvUser < 0 || TrigDevNo < 0 || TrigDevNo >= EIC_EXTINT_NUM ||
		d->pTrig[d->NbTrig - 1].bActive)
	{
		return false;
	}

	SamdTimerExtOff(d);

	int chan = SamdEvsysOpen(SAMD_EVSYS_GEN_EIC_EXTINT(TrigDevNo), SAMD_EVSYS_PATH_ASYNC, SAMD_EVSYS_EDGE_NONE,
							 d->EvUser, NULL, NULL, 0);

	if (chan < 0)
	{
		return false;
	}

	d->ExtChan = (int8_t)chan;
	d->ExtLine = (int8_t)TrigDevNo;
	d->bExtOn = true;

	// The capture before the events
	if (!SamdTimerCapture(d, true) || !SamdEicEvent(TrigDevNo, sense))
	{
		SamdTimerExtOff(d);

		return false;
	}

	return true;
}

// Tick interrupt of the RTC, from its periodic event channel
static void SamdTimerTick(int Chan, void *pCtx)
{
	SamdTimerData_t *d = (SamdTimerData_t *)pCtx;
	TimerDev_t *timer = d->pTimer;

	(void)Chan;

	if (timer != NULL && d->bRunning && timer->EvtHandler != NULL)
	{
		timer->EvtHandler(timer, TIMER_EVT_TICK);
	}
}

// Event channel of the RTC tick, for the periodic event of the prescaler
static bool SamdTimerTickOpen(SamdTimerData_t *pData, int IntPrio)
{
	if (pData->bTickOn)
	{
		SamdEvsysClose(pData->TickChan, SAMD_EVSYS_USER_NONE);
		pData->bTickOn = false;
	}

	uint32_t gen = SamdTimerIsRtc(pData) ? SamdRtcTickGen(pData) : 0U;

	if (gen == 0U)
	{
		return !pData->bTick;
	}

	// The RTC is on another clock generator: resynchronized path
	int chan = SamdEvsysOpen((int)gen, SAMD_EVSYS_PATH_RESYNC, SAMD_EVSYS_EDGE_RISE, SAMD_EVSYS_USER_NONE,
							 SamdTimerTick, pData, IntPrio);

	pData->TickChan = (int8_t)chan;
	pData->bTickOn = chan >= 0;

	return chan >= 0;
}

// Interrupt of a timer device: overflow event, then the triggers whose
// deadline is reached, all from one count. The callbacks run with
// interrupts enabled.
void SamdTimerIrq(int DevNo)
{
	SamdTimerData_t *d = &g_SamdTimerData[DevNo];
	TimerDev_t *timer = d->pTimer;
	TimerTrig_t info[SAMD_TIMER_TRIG_MAXCNT];
	bool fired[SAMD_TIMER_TRIG_MAXCNT];
	bool check[SAMD_TIMER_TRIG_MAXCNT];
	uint32_t seq[SAMD_TIMER_TRIG_MAXCNT];
	uint32_t state;
	uint64_t count;

	if (timer == NULL || !d->bRunning)
	{
		return;
	}

	// RTC: the SYNCRDY interrupt alone needs no count
	if (SamdTimerIsRtc(d))
	{
		state = DisableInterrupt();

		bool evt = SamdRtcEvent(d) || d->bOverflow;

		for (int i = 0; i < SamdTimerTrigCnt(d); i++)
		{
			evt = evt || (d->pTrig[i].bActive && d->pTrig[i].bDue);
		}
		EnableInterrupt(state);

		if (!evt)
		{
			return;
		}
	}

	if (!SamdTimerLock(d, &state, &count, false))
	{
		// No count: the flags are cleared, the matched triggers due at the
		// next interrupt
		state = DisableInterrupt();
		for (int i = 0; i < SamdTimerTrigCnt(d); i++)
		{
			if (i != SamdTimerCaptChan(d) && d->pOps->Matched(d, i) && d->pTrig[i].bActive)
			{
				d->pTrig[i].bDue = true;
			}
		}
		if (d->pOps->Overflow(d))
		{
			timer->Rollover += SamdTimerSpan(d);
			d->bOverflow = true;
		}
		SamdTimerUnlock(d, state);

		return;
	}

	uint32_t epoch = d->Epoch;
	bool matched[SAMD_TIMER_TRIG_MAXCNT];

	for (int i = 0; i < SamdTimerTrigCnt(d); i++)
	{
		matched[i] = i != SamdTimerCaptChan(d) && d->pOps->Matched(d, i);
	}

	// External trigger captured before the count read again below
	uint32_t capt = 0;
	bool ext = d->bCapture && d->pOps->Captured(d, &capt);

	// A TC or TCC compare can match after the count was read, the count is
	// read again after the flags. The RTC count is not readable after the
	// flag write: a match that close is in the lag.
	if (!SamdTimerIsRtc(d))
	{
		(void)SamdTimerNow(d, &count);
	}

	if (ext)
	{
		// The capture is less than a counter turn before the count
		uint64_t back = ((uint64_t)count - capt) & (SamdTimerSpan(d) - 1U);

		d->ExtCount = count >= back ? count - back : 0;
	}

	for (int i = 0; i < SamdTimerTrigCnt(d); i++)
	{
		SamdTimerTrig_t *trig = &d->pTrig[i];
		uint64_t now = count;

		fired[i] = false;
		check[i] = false;
		seq[i] = 0;

		if (!trig->bActive)
		{
			continue;
		}

		if (!trig->bDue && now < trig->Deadline)
		{
			// The RTC count is late, a compare match that close is the
			// deadline
			if (!matched[i] || !SamdTimerIsRtc(d) || trig->Deadline - now > SamdRtcLag(d))
			{
				continue;
			}
			now = trig->Deadline;
			d->Floor = now;
		}

		fired[i] = true;
		info[i] = trig->Info;

		if (info[i].Type == TIMER_TRIG_TYPE_SINGLE)
		{
			trig->bActive = false;
			(void)SamdTimerArm(d, i, false, count, &check[i]);
			seq[i] = trig->Seq;
		}
		else
		{
			// Keeps the phase, missed periods are one event
			if (now >= trig->Deadline)
			{
				trig->Deadline += ((now - trig->Deadline) / trig->Ticks + 1) * trig->Ticks;
			}
			else
			{
				trig->Deadline += trig->Ticks;
			}
			(void)SamdTimerArm(d, i, true, count, &check[i]);
			seq[i] = trig->Seq;
		}
	}

	// Also an overflow found by the TC reads of SamdTimerArm
	bool overflow = d->bOverflow;

	d->bOverflow = false;
	SamdTimerUnlock(d, state);

	if (overflow && timer->EvtHandler)
	{
		timer->EvtHandler(timer, TIMER_EVT_COUNTER_OVR);
	}

	// The count captured at the external trigger in LastCount
	if (ext && timer->EvtHandler && epoch == d->Epoch && d->bRunning)
	{
		timer->LastCount = (uint32_t)d->ExtCount;
		timer->EvtHandler(timer, TIMER_EVT_EXTTRIG);
	}

	for (int i = 0; i < SamdTimerTrigCnt(d); i++)
	{
		if (check[i])
		{
			SamdTimerCheck(d, i, seq[i]);
		}
	}

	for (int i = 0; i < SamdTimerTrigCnt(d); i++)
	{
		// A callback reset, reconfigured or stopped the timer
		if (epoch != d->Epoch || !d->bRunning)
		{
			return;
		}

		// Not when a callback before changed this trigger
		if (fired[i] && d->pTrig[i].Seq == seq[i])
		{
			if (info[i].Handler)
			{
				info[i].Handler(timer, i, info[i].pContext);
			}
			if (timer->EvtHandler)
			{
				timer->EvtHandler(timer, TIMER_EVT_TRIGGER(i));
			}
		}
	}
}

// A timer device not bound to any timer, its interrupt off
static void SamdTimerRelease(SamdTimerData_t *pData)
{
	SamdTimerExtOff(pData);
	if (pData->bTickOn)
	{
		SamdEvsysClose(pData->TickChan, SAMD_EVSYS_USER_NONE);
		pData->bTickOn = false;
	}

	(void)SamdTimerRun(pData, false);

	uint32_t state = DisableInterrupt();

	for (int i = 0; i < pData->NbTrig; i++)
	{
		pData->pTrig[i].bActive = false;
		pData->pTrig[i].Seq++;
	}
	pData->pTimer = NULL;
	EnableInterrupt(state);
}

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	// Tick interrupt on the RTC only
	if (pTimer == NULL || pCfg == NULL || pCfg->DevNo < 0 || pCfg->DevNo >= SAMD_TIMER_MAXCNT ||
		(pCfg->bTickInt && pCfg->DevNo >= SAMD_RTC_TIMER_MAXCNT) || pCfg->IntPrio < 0)
	{
		return false;
	}

	SamdTimerData_t *d = &g_SamdTimerData[pCfg->DevNo];

	const SamdTimerHw_t *hw = &s_SamdTimerHw[pCfg->DevNo];

	// Constants of the DevNo, the same if the device is bound
	d->pOps = hw->pOps;
	d->pReg = hw->pReg;
	d->pTrig = &s_SamdTimerTrig[hw->TrigIdx];
	d->IrqNo = hw->IrqNo;
	d->ApbMask = hw->ApbMask;
	d->GclkId = hw->GclkId;
	d->NbTrig = hw->NbTrig;
	d->CntBits = hw->CntBits;
	d->EvUser = hw->EvUser;

	// Clock source: the RTC runs on generator 1 from the low frequency
	// oscillator of g_McuOsc, the TC and TCC on the main clock
	if (SamdTimerIsRtc(d))
	{
		static const TIMER_CLKSRC s_LfSrc[] = { TIMER_CLKSRC_LFRC, TIMER_CLKSRC_LFXTAL, TIMER_CLKSRC_EXT };
		TIMER_CLKSRC lf = s_LfSrc[GetLowFreqOscType()];

		if (pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT && pCfg->ClkSrc != lf)
		{
			return false;
		}
		d->BaseFreq = SystemPeriphClockGet(SAMD_GCLK_LF);
	}
	else
	{
		if (pCfg->ClkSrc != TIMER_CLKSRC_DEFAULT)
		{
			return false;
		}
		d->BaseFreq = SystemPeriphClockGet(SAMD_GCLK_MAIN);
	}

	uint32_t presc;
	uint32_t freq;

	// The interrupts of a device bound to another DevNo would update this
	// timer, it is released. A device bound to another timer is taken.
	for (int i = 0; i < SAMD_TIMER_MAXCNT; i++)
	{
		if (g_SamdTimerData[i].pTimer == pTimer || (i == pCfg->DevNo && g_SamdTimerData[i].pTimer != NULL))
		{
			SamdTimerRelease(&g_SamdTimerData[i]);
		}
	}

	d->bCapture = false;
	d->bTick = pCfg->bTickInt;

	if (!SamdTimerPlan(d, pCfg->Freq, &presc, &freq))
	{
		return false;
	}

	SamdTimerNvic(d, false);
	NVIC_SetPriority(d->IrqNo, SamdIrqPrio(pCfg->IntPrio));

	d->Prescaler = presc;
	d->Compare = 0;
	d->bRunning = false;
	d->bOverflow = false;
	d->bResetOvf = false;

	pTimer->DevNo = pCfg->DevNo;
	pTimer->Freq = freq;
	pTimer->nsPeriod = (1000000000ULL + freq / 2) / freq;
	pTimer->Rollover = 0;
	pTimer->LastCount = 0;
	pTimer->EvtHandler = pCfg->EvtHandler;
	pTimer->Disable = SamdTimerDisable;
	pTimer->Enable = SamdTimerEnable;
	pTimer->Reset = SamdTimerReset;
	pTimer->GetTickCount = SamdTimerGetTickCount;
	pTimer->SetFrequency = SamdTimerSetFrequency;
	pTimer->GetMaxTrigger = SamdTimerGetMaxTrigger;
	pTimer->FindAvailTrigger = SamdTimerFindAvailTrigger;
	pTimer->DisableTrigger = SamdTimerDisableTrigger;
	pTimer->EnableTrigger = SamdTimerEnableTrigger;
	pTimer->DisableExtTrigger = SamdTimerDisableExtTrigger;
	pTimer->EnableExtTrigger = SamdTimerEnableExtTrigger;

	d->pTimer = pTimer;

	bool res = d->pOps->Setup(d) && SamdTimerTickOpen(d, pCfg->IntPrio) &&
			   SamdTimerResetCounter(d) && SamdTimerRun(d, true);

	if (!res)
	{
		SamdTimerRelease(d);
	}

	return res;
}

int TimerGetLowFreqDevCount(void)
{
	return SAMD_RTC_TIMER_MAXCNT;
}

int TimerGetHighFreqDevCount(void)
{
	return SAMD_TC_TIMER_MAXCNT + SAMD_TCC_TIMER_MAXCNT;
}

int TimerGetHighFreqDevNo(void)
{
	return SAMD_RTC_TIMER_MAXCNT;
}
