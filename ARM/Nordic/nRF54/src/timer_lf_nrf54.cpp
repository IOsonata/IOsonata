/**-------------------------------------------------------------------------
@file	timer_lf_nrf54.cpp

@brief	timer class implementation on Nordic nRF54 series using the GRTC (real time counter)

@author	Hoang Nguyen Hoan
@date	Jan. 25, 2025

@license

MIT License

Copyright (c) 2025, I-SYST inc., all rights reserved

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

#include "nrf.h"

#include "timer_nrfx.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"

// GRTC IRQ_3 and CC 7-11 are reserved by MPSL
#define MPSL_RESERVED_NB_CC		5	// 7-11 are reserved for MPSL

#define NRF_CLOCK			NRF_CLOCK_S
#define NRF_RTC0			NRF_RTC10_S
#define NRF_RTC1			NRF_RTC30_S
#define GRTC0_CC_NUM		GRTC_CC_NUM_SIZE
#define GRTC1_CC_NUM		RTC30_CC_NUM_SIZE

#pragma pack(push, 4)
typedef struct {
	int DevNo;
    int MaxNbTrigEvt;		//!< Number of trigger is not the same for all timers.
    int StartCCIdx;			//!< CC channel start
    uint64_t CC[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];	//!< Trigger period in ticks
    uint64_t Compare[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];	//!< Compare value written to the CC, GRTC reads are slow
    TimerTrig_t Trigger[TIMER_NRFX_RTC_MAX_TRIGGER_EVT];
    TimerDev_t *pTimer;
    volatile uint32_t *pInt;	//!< INTEN, INTENSET, INTENCLR, INTPEND of interrupt group DevNo
    volatile NRF_GRTC_SYSCOUNTER_Type *pSysCnt;	//!< SYSCOUNTER[DevNo]
} nRFGrtcData_t;
#pragma pack(pop)

//static uint32_t s_GrtcMaxFreq = TIMER_NRFX_RTC_BASE_FREQ;

// There are a total of GRTC_NCC_SIZE (12) CC sharing 4 IRQ.  We are splitting into 4 devices
alignas(4) static nRFGrtcData_t s_nRfxGrtcData[TIMER_NRFX_RTC_MAX] = {
	{
		.DevNo = 0,
		.MaxNbTrigEvt = 2,	// CC 0..1
		.StartCCIdx = 0,
	},
	{
		.DevNo = 1,
		.MaxNbTrigEvt = 2,	// CC 2..3
		.StartCCIdx = 2,
	},
	{
		.DevNo = 2,
		.MaxNbTrigEvt = 3,	// CC 4..6
		.StartCCIdx = 4,
	},
	{
		// This one is reserved by MPSL
		.DevNo = 3,
		.MaxNbTrigEvt = 5,	// CC 7..11
		.StartCCIdx = 7,
	},
};

// Timers enabled. The first Enable starts the SYSCOUNTER when it is not
// running, Disable never stops it: the radio stack (MPSL, SoftDevice) uses it
// too and can start it after this driver did.
static bool s_bGrtcEnabled[TIMER_NRFX_RTC_MAX];
// Reservation persists when a generic Timer handle is rebound to another
// disabled GRTC group; pTimer in each group is only its active dispatch handle.
static TimerDev_t *s_pGrtcOwner[TIMER_NRFX_RTC_MAX];

// Reject stale handles after PowerOff or reinitialization.
// Ownership must be checked before accessing any per-group GRTC state.
static nRFGrtcData_t *nRFxGrtcOwned(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;
	int devno;

	if (pTimer == nullptr)
	{
		return nullptr;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX)
	{
		return nullptr;
	}
	pRtc = &s_nRfxGrtcData[devno];

	return pRtc->pTimer == pTimer ? pRtc : nullptr;
}

// Interrupt of a timer: GRTC_0_IRQn to GRTC_3_IRQn are consecutive, one per
// INTEN register group, the DevNo
static inline IRQn_Type nRFxGrtcIrqNo(int DevNo)
{
	return (IRQn_Type)(GRTC_0_IRQn + DevNo);
}

// Compare channels of a timer, as INTEN bits
static inline uint32_t nRFxGrtcCCMask(const nRFGrtcData_t &Rtc)
{
	return ((1U << Rtc.MaxNbTrigEvt) - 1U) << Rtc.StartCCIdx;
}

static bool nRFxGrtcEnable(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;
	uint32_t state;
	uint32_t clkCfg;
	uint32_t clkSel;
	int devno;

	if (pTimer == nullptr)
	{
		return false;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	state = DisableInterrupt();
	pRtc = &s_nRfxGrtcData[devno];
	if (pRtc->pTimer != pTimer)
	{
		EnableInterrupt(state);
		return false;
	}

	if ((NRF_GRTC->MODE & GRTC_MODE_SYSCOUNTEREN_Msk) == 0U)
	{
		// GRTC clock selection is latched at the first START and cannot
		// be changed while running. Use the board's oscillator choice.
		// An existing GRTC owner is never reconfigured by this driver.
		switch (GetLowFreqOscType())
		{
			case OSC_TYPE_XTAL:
				clkSel = GRTC_CLKCFG_CLKSEL_LFXO;
				break;
			case OSC_TYPE_RC:
				clkSel = GRTC_CLKCFG_CLKSEL_LFLPRC;
				break;
			default:
				EnableInterrupt(state);
				return false;
		}
		clkCfg = NRF_GRTC->CLKCFG & ~GRTC_CLKCFG_CLKSEL_Msk;
		clkCfg |= clkSel << GRTC_CLKCFG_CLKSEL_Pos;
		NRF_GRTC->CLKCFG = clkCfg;
		NRF_GRTC->MODE |= GRTC_MODE_SYSCOUNTEREN_Msk |
			GRTC_MODE_AUTOEN_CpuActive;
		NRF_GRTC->TASKS_START = 1U;
	}

	s_bGrtcEnabled[devno] = true;

	// Interrupt on while a trigger is armed, see nRFxGrtcEnableTrigger. Off
	// otherwise: the SoftDevice owns GRTC_3_IRQn and wants it off when it
	// starts, after the Init of timer 3 (bt_app_bm.cpp).
	if (pRtc->pInt[0] & nRFxGrtcCCMask(*pRtc))
	{
		NVIC_EnableIRQ(nRFxGrtcIrqNo(devno));
	}

	EnableInterrupt(state);
	return true;
}

// The triggers are kept, their interrupt off until Enable. The SYSCOUNTER
// keeps running, the tick count too.
static void nRFxGrtcDisable(TimerDev_t * const pTimer)
{
	uint32_t state;
	nRFGrtcData_t *pRtc;
	int devno;

	if (pTimer == nullptr)
	{
		return;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX)
	{
		return;
	}
	state = DisableInterrupt();
	pRtc = &s_nRfxGrtcData[devno];
	if (pRtc->pTimer != pTimer)
	{
		EnableInterrupt(state);
		return;
	}
	s_bGrtcEnabled[devno] = false;
	NVIC_DisableIRQ(nRFxGrtcIrqNo(devno));
	NVIC_ClearPendingIRQ(nRFxGrtcIrqNo(devno));

	EnableInterrupt(state);
}

// Power off this logical GRTC group without disturbing the shared
// SYSCOUNTER or compare channels owned by other clients (including MPSL).
static bool nRFxGrtcPowerOff(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;
	uint32_t state;
	int devno;
	int i;
	int idx;

	if (pTimer == nullptr)
	{
		return false;
	}
	devno = pTimer->DevNo;
	if (devno < 0 || devno >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	state = DisableInterrupt();
	pRtc = &s_nRfxGrtcData[devno];
	if (pRtc->pTimer != pTimer || s_pGrtcOwner[devno] != pTimer)
	{
		EnableInterrupt(state);
		return false;
	}
	nRFxGrtcDisable(pTimer);
	pRtc->pInt[2] = nRFxGrtcCCMask(*pRtc);
	for (i = 0; i < pRtc->MaxNbTrigEvt; i++)
	{
		idx = pRtc->StartCCIdx + i;
		NRF_GRTC->CC[idx].CCEN = 0U;
		NRF_GRTC->EVENTS_COMPARE[idx] = 0U;
		pRtc->CC[i] = 0U;
		pRtc->Compare[i] = 0U;
		pRtc->Trigger[i].Type = TIMER_TRIG_TYPE_SINGLE;
		pRtc->Trigger[i].nsPeriod = 0ULL;
		pRtc->Trigger[i].Handler = nullptr;
		pRtc->Trigger[i].pContext = nullptr;
	}
	NVIC_DisableIRQ(nRFxGrtcIrqNo(devno));
	NVIC_ClearPendingIRQ(nRFxGrtcIrqNo(devno));
	pRtc->pTimer = nullptr;
	s_pGrtcOwner[devno] = nullptr;
	s_bGrtcEnabled[devno] = false;
	pTimer->PowerOff = nullptr;
	pTimer->DevNo = -1;
	pTimer->Freq = 0U;
	EnableInterrupt(state);
	return true;
}

// SYSCOUNTER, SYSCOUNTERL read before SYSCOUNTERH. Read again while BUSY
// (value not valid) or OVERFLOW (SYSCOUNTERL overflowed after it was read, the
// two halves are not from the same count), as nrfy_grtc_sys_counter_get does.
// OVERFLOW is set once in 71 minutes, the product specification takes
// SYSCOUNTERH - 1 for it.
static inline __attribute__((always_inline)) uint64_t nRFxGrtcSysCounter(volatile NRF_GRTC_SYSCOUNTER_Type *pSysCnt)
{
	uint32_t cl;
	uint32_t ch;

	do {
		cl = pSysCnt->SYSCOUNTERL;
		ch = pSysCnt->SYSCOUNTERH;
	} while (ch & (GRTC_SYSCOUNTER_SYSCOUNTERH_BUSY_Msk | GRTC_SYSCOUNTER_SYSCOUNTERH_OVERFLOW_Msk));

	return ((uint64_t)(ch & GRTC_SYSCOUNTER_SYSCOUNTERH_VALUE_Msk) << 32) | cl;
}

// The SYSCOUNTER is shared by the GRTC timers and the radio stack: Reset does
// not clear it. The tick count restarts from Reset, kept as an offset in
// Rollover, and the armed triggers restart their period from it.
static void nRFxGrtcReset(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;
	uint32_t state;
	uint32_t inten;
	uint64_t now;
	int i;
	int idx;

	state = DisableInterrupt();
	pRtc = nRFxGrtcOwned(pTimer);
	if (pRtc == nullptr)
	{
		EnableInterrupt(state);
		return;
	}
	inten = pRtc->pInt[0];
	now = nRFxGrtcSysCounter(pRtc->pSysCnt);

	pTimer->Rollover = 0ULL - now;

	for (i = 0; i < pRtc->MaxNbTrigEvt; i++)
	{
		idx = pRtc->StartCCIdx + i;

		if (pRtc->Trigger[i].nsPeriod == 0U ||
			(inten & (1U << idx)) == 0U)
		{
			continue;
		}
		pRtc->Compare[i] = now + pRtc->CC[i];
		NRF_GRTC->CC[idx].CCL = (uint32_t)pRtc->Compare[i];
		NRF_GRTC->EVENTS_COMPARE[idx] = 0U;
		NRF_GRTC->CC[idx].CCH = (uint32_t)(pRtc->Compare[i] >> 32);
	}

	EnableInterrupt(state);
}

// GRTC is fixed at 1MHz.
static uint32_t nRFxGrtcSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
	if (nRFxGrtcOwned(pTimer) == nullptr)
	{
		return 0U;
	}
	(void)Freq;
	pTimer->Freq = TIMER_NRFX_RTC_BASE_FREQ;

	// Pre-calculate periods for faster timer counter to time conversion.
	pTimer->nsPeriod = (1000000000ULL + (pTimer->Freq >> 1)) /
		(uint64_t)pTimer->Freq;

	return pTimer->Freq;
}

// SYSCOUNTER (52 bits at 1 MHz, 142 years), counted from Reset once called
static uint64_t nRFxGrtcGetTickCount(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;

	pRtc = nRFxGrtcOwned(pTimer);
	if (pRtc == nullptr)
	{
		return 0ULL;
	}
	return nRFxGrtcSysCounter(pRtc->pSysCnt) + pTimer->Rollover;
}

static int nRFxGrtcGetMaxTrigger(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;

	pRtc = nRFxGrtcOwned(pTimer);
	return pRtc != nullptr ? pRtc->MaxNbTrigEvt : 0;
}

static uint64_t nRFxGrtcEnableTrigger(TimerDev_t * const pTimer, int TrigNo,
	uint64_t nsPeriod, TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t Handler,
	void *pContext)
{
	nRFGrtcData_t *pRtc;
	TimerTrig_t *pTrig;
	uint64_t start;
	uint64_t cc;
	uint64_t period;
	uint32_t state;
	int idx;

	if (nsPeriod == 0U)
	{
		return 0ULL;
	}
	// GRTC tick is 1 us. Use a 32-bit divide where possible.
	if (nsPeriod <= 0xFFFFFFFFULL - 500U)
	{
		cc = ((uint32_t)nsPeriod + 500U) / 1000U;
	}
	else
	{
		cc = (nsPeriod + 500U) / 1000U;
	}
	if (cc == 0U)
	{
		cc = 1U;
	}

	state = DisableInterrupt();
	pRtc = nRFxGrtcOwned(pTimer);
	if (pRtc == nullptr || TrigNo < 0 || TrigNo >= pRtc->MaxNbTrigEvt)
	{
		EnableInterrupt(state);
		return 0ULL;
	}
	pTrig = &pRtc->Trigger[TrigNo];
	// Never replace a compare belonging to an already armed trigger.
	if (pTrig->nsPeriod != 0U)
	{
		EnableInterrupt(state);
		return 0ULL;
	}

	start = nRFxGrtcSysCounter(pRtc->pSysCnt) + cc;
	idx = pRtc->StartCCIdx + TrigNo;
	pRtc->Compare[TrigNo] = start;
	// Writing CCL disables the channel; writing CCH arms it again.
	NRF_GRTC->CC[idx].CCL = (uint32_t)start;
	NRF_GRTC->EVENTS_COMPARE[idx] = 0U;
	NRF_GRTC->CC[idx].CCH = (uint32_t)(start >> 32);

	period = pTimer->nsPeriod * cc;
	pTrig->Type = Type;
	pTrig->nsPeriod = period;
	pTrig->Handler = Handler;
	pTrig->pContext = pContext;
	pRtc->CC[TrigNo] = cc;
	pRtc->pInt[1] = 1U << idx;
	if (s_bGrtcEnabled[pRtc->DevNo])
	{
		NVIC_EnableIRQ(nRFxGrtcIrqNo(pRtc->DevNo));
	}
	EnableInterrupt(state);

	return period;
}

static void nRFxGrtcDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
	nRFGrtcData_t *pRtc;
	uint32_t state;
	int idx;

	state = DisableInterrupt();
	pRtc = nRFxGrtcOwned(pTimer);
	if (pRtc == nullptr || TrigNo < 0 || TrigNo >= pRtc->MaxNbTrigEvt)
	{
		EnableInterrupt(state);
		return;
	}
	idx = pRtc->StartCCIdx + TrigNo;

	pRtc->pInt[2] = 1U << idx;
	if ((pRtc->pInt[0] & nRFxGrtcCCMask(*pRtc)) == 0U)
	{
		NVIC_DisableIRQ(nRFxGrtcIrqNo(pRtc->DevNo));
		NVIC_ClearPendingIRQ(nRFxGrtcIrqNo(pRtc->DevNo));
	}
	NRF_GRTC->CC[idx].CCEN = 0U;
	NRF_GRTC->EVENTS_COMPARE[idx] = 0U;

	pRtc->CC[TrigNo] = 0U;
	pRtc->Compare[TrigNo] = 0U;
	pRtc->Trigger[TrigNo].Type = TIMER_TRIG_TYPE_SINGLE;
	pRtc->Trigger[TrigNo].Handler = nullptr;
	pRtc->Trigger[TrigNo].pContext = nullptr;
	pRtc->Trigger[TrigNo].nsPeriod = 0U;
	EnableInterrupt(state);
}

static int nRFxGrtcFindAvailTrigger(TimerDev_t * const pTimer)
{
	nRFGrtcData_t *pRtc;
	uint32_t state;
	int i;

	state = DisableInterrupt();
	pRtc = nRFxGrtcOwned(pTimer);
	if (pRtc == nullptr)
	{
		EnableInterrupt(state);
		return -1;
	}
	for (i = 0; i < pRtc->MaxNbTrigEvt; i++)
	{
		if (pRtc->Trigger[i].nsPeriod == 0U)
		{
			EnableInterrupt(state);
			return i;
		}
	}
	EnableInterrupt(state);
	return -1;
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static inline void nRFxGrtcDisableExtTrigger(TimerDev_t * const pTimer) {
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
bool nRFxGrtcEnableExtTrigger(TimerDev_t * const pTimer, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
	// No support for external trigger
	return false;
}

bool nRFxGrtcInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
	TimerCfg_t selected;
	int devno;
	int candidate;
	uint32_t state;
	bool automatic;

	if (pTimer == nullptr || pCfg == nullptr ||
		pCfg->DevNo < -1 || pCfg->DevNo >= TIMER_NRFX_RTC_MAX)
	{
		return false;
	}
	// Reserve the selected group atomically. Never assign MPSL's
	// final group (CC7..11) on an automatic timer request.
	state = DisableInterrupt();
	automatic = pCfg->DevNo == -1;
	selected = *pCfg;
	if (automatic)
	{
		selected.DevNo = -1;
		for (candidate = TIMER_NRFX_RTC_MAX - 2; candidate >= 0;
			candidate--)
		{
			if (s_pGrtcOwner[candidate] == nullptr &&
				!s_bGrtcEnabled[candidate])
			{
				selected.DevNo = candidate;
				break;
			}
		}
		if (selected.DevNo < 0)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	devno = selected.DevNo;
	// Generic Timer clients may reuse one handle sequentially after
	// disabling the previous device. Never detach a live GRTC or one
	// with armed compare events when moving that handle.
	for (candidate = 0; candidate < TIMER_NRFX_RTC_MAX; candidate++)
	{
		if (candidate != devno &&
			s_nRfxGrtcData[candidate].pTimer == pTimer)
		{
			if (s_bGrtcEnabled[candidate])
			{
				EnableInterrupt(state);
				return false;
			}
			for (int i = 0; i < s_nRfxGrtcData[candidate].MaxNbTrigEvt; i++)
			{
				if (s_nRfxGrtcData[candidate].Trigger[i].nsPeriod != 0U)
				{
					EnableInterrupt(state);
					return false;
				}
			}
		}
	}
	if (s_bGrtcEnabled[devno])
	{
		EnableInterrupt(state);
		return false;
	}
	// A timer held by another Timer object stays with it until its PowerOff
	if (s_nRfxGrtcData[devno].pTimer != nullptr &&
		s_nRfxGrtcData[devno].pTimer != pTimer)
	{
		EnableInterrupt(state);
		return false;
	}
	// Retire the old disabled group before rebinding this Timer handle.
	// Otherwise the old interrupt group keeps a pointer whose DevNo now
	// names the new hardware and can dispatch an event to the wrong timer.
	for (candidate = 0; candidate < TIMER_NRFX_RTC_MAX; candidate++)
	{
		if (candidate != devno &&
			s_nRfxGrtcData[candidate].pTimer == pTimer)
		{
			s_nRfxGrtcData[candidate].pInt[2] =
				nRFxGrtcCCMask(s_nRfxGrtcData[candidate]);
			s_nRfxGrtcData[candidate].pTimer = nullptr;
		}
	}
	// Install ownership before restoring interrupts, so another user
	// cannot select the same group while initialization proceeds.
	s_nRfxGrtcData[devno].pTimer = pTimer;
	s_pGrtcOwner[devno] = pTimer;
	EnableInterrupt(state);
    pTimer->DevNo = devno;
    pTimer->EvtHandler = pCfg->EvtHandler;
	nRFGrtcData_t &rtc = s_nRfxGrtcData[pTimer->DevNo];
//	NRF_GRTC_Type *reg = s_nRfxGrtcData[pTimer->DevNo].pReg;

	rtc.pInt = &NRF_GRTC->INTEN0 + devno * 4;
	rtc.pSysCnt = &NRF_GRTC->SYSCOUNTER[devno];
    rtc.pInt[2] = nRFxGrtcCCMask(rtc);
	for (int i = 0; i < rtc.MaxNbTrigEvt; i++)
	{
		NRF_GRTC->CC[rtc.StartCCIdx + i].CCEN = 0U;
		NRF_GRTC->EVENTS_COMPARE[rtc.StartCCIdx + i] = 0U;
	}
	memset(rtc.Trigger, 0, sizeof(rtc.Trigger));


	pTimer->Rollover = 0;
	pTimer->LastCount = 0;


    pTimer->Disable = nRFxGrtcDisable;
    pTimer->PowerOff = nRFxGrtcPowerOff;
    pTimer->Enable = nRFxGrtcEnable;
    pTimer->Reset = nRFxGrtcReset;
    pTimer->GetTickCount = nRFxGrtcGetTickCount;
    pTimer->SetFrequency = nRFxGrtcSetFrequency;
    pTimer->GetMaxTrigger = nRFxGrtcGetMaxTrigger;
    pTimer->FindAvailTrigger = nRFxGrtcFindAvailTrigger;
    pTimer->DisableTrigger = nRFxGrtcDisableTrigger;
    pTimer->EnableTrigger = nRFxGrtcEnableTrigger;
    pTimer->DisableExtTrigger = nRFxGrtcDisableExtTrigger;
    pTimer->EnableExtTrigger = nRFxGrtcEnableExtTrigger;
	// No task endpoint on the GRTC timer
	pTimer->GetTaskEndpoint = nullptr;
	pTimer->ExecuteTask = nullptr;

	// Interrupt priority, the interrupt enabled with a trigger
	NVIC_SetPriority(nRFxGrtcIrqNo(devno), pCfg->IntPrio);

    nRFxGrtcSetFrequency(pTimer, pCfg->Freq);

    return nRFxGrtcEnable(pTimer);
}

// Next compare value of a continuous trigger when Next is not after Now: the
// periods already passed are skipped, the handler is called once for them
static __attribute__((noinline)) uint64_t nRFxGrtcSkip(uint64_t Next, uint64_t Now, uint64_t Period)
{
	return Next + ((Now - Next) / Period + 1U) * Period;
}

// Next compare of continuous trigger TrigNo on CC[Idx], one period after the
// compare that matched: the period does not depend on the interrupt latency.
// The match disabled the channel and its event is cleared, CCL then CCH
// enable it again. A value passed by the time it is written matches at once.
static inline __attribute__((always_inline)) void nRFxGrtcNextCompare(nRFGrtcData_t &Rtc, int TrigNo, int Idx, uint64_t Period)
{
	uint64_t next = Rtc.Compare[TrigNo] + Period;
	uint64_t now = nRFxGrtcSysCounter(Rtc.pSysCnt);

	if (next <= now)
	{
		next = nRFxGrtcSkip(next, now, Period);
	}

	Rtc.Compare[TrigNo] = next;
	NRF_GRTC->CC[Idx].CCL = (uint32_t)next;
	NRF_GRTC->CC[Idx].CCH = (uint32_t)(next >> 32);
}

static void GrtcIRQHandler(nRFGrtcData_t &rtc)
{
	TimerDev_t *timer = rtc.pTimer;

	if (timer == nullptr)
	{
		return;
	}

    uint32_t evt = 0;

	// Compares of the armed triggers with their event set, INTPEND is the
	// events with their interrupt set
	uint32_t pend = (rtc.pInt[3] >> rtc.StartCCIdx) & ((1U << rtc.MaxNbTrigEvt) - 1U);

	while (pend != 0U)
	{
		int i = __builtin_ctz(pend);
		int idx = rtc.StartCCIdx + i;
		TimerTrig_t &trig = rtc.Trigger[i];
		TimerTrigEvtHandler_t handler;
		void *ctx;

		pend &= pend - 1U;

		NRF_GRTC->EVENTS_COMPARE[idx] = 0;

		if (trig.Type == TIMER_TRIG_TYPE_CONTINUOUS)
		{
			// Next compare first, then the handler. No period: disabled by an
			// interrupt of higher priority.
			uint64_t period = rtc.CC[i];

			if (period != 0U)
			{
				nRFxGrtcNextCompare(rtc, i, idx, period);
			}
			handler = trig.Handler;
			ctx = trig.pContext;
		}
		else
		{
			// Single shot done, the match disabled the channel. The trigger is
			// free before its handler, which can set it again: interrupt off
			// and no period, see nRFxGrtcFindAvailTrigger.
			handler = trig.Handler;
			ctx = trig.pContext;
			rtc.pInt[2] = 1U << idx;
			trig.nsPeriod = 0;
		}

		evt |= TIMER_EVT_TRIGGER(i);
		if (handler)
		{
			handler(timer, i, ctx);
		}
	}

    if (timer->EvtHandler)
    {
        timer->EvtHandler(timer, evt);
    }
}

extern "C" {

#if defined(NRF54L_SERIES) || defined(NRF71_SERIES)
__WEAK void GRTC_0_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[0]);
}

__WEAK void GRTC_1_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[1]);
}

__WEAK void GRTC_2_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[2]);
}

__WEAK void GRTC_3_IRQHandler(void)
{
	GrtcIRQHandler(s_nRfxGrtcData[3]);
}

#endif

}

