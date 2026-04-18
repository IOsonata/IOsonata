/**-------------------------------------------------------------------------
@file	timer_systick.cpp

@brief	ARM Cortex-M SysTick timer — IOsonata TimerDev_t MCU port (Roots layer)

Implements the TimerDev_t interface on top of the systick.h Land-layer
primitives. Follows the same structure as all IOsonata MCU port timer
implementations (timer_lf_nrfx.cpp, timer_lptim_stm32l4xx.cpp, etc.):

  - Static private data  (SysTick_TimerData_t)
  - Static functions     (one per TimerDev_t function pointer)
  - TimerInit()          wires all function pointers
  - SysTick_Handler()    ISR — dispatches to registered EvtHandler

No SysTick register addresses in this file — all hardware access goes
through systick.h (Land layer). No CMSIS dependency beyond cmsis_compiler.h
(already pulled in via systick.h).

@author	Hoang Nguyen Hoan
@date	Mar. 2026

@license

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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "systick.h"              // Land layer — register primitives
#include "coredev/timer.h"        // IOsonata timer interface
#include "timer_systick.h"        // this port's header

// ── Private data ─────────────────────────────────────────────────────────────
// One SysTick per Cortex-M core — no indexing needed.

static SysTick_TimerData_t s_SysTickData;

// ── Forward declarations ──────────────────────────────────────────────────────

static bool     SystickEnable        (TimerDev_t * const pTimer);
static void     SystickDisable       (TimerDev_t * const pTimer);
static void     SystickReset         (TimerDev_t * const pTimer);
static uint64_t SystickGetTickCount  (TimerDev_t * const pTimer);
static uint32_t SystickSetFrequency  (TimerDev_t * const pTimer, uint32_t Freq);
static int      SystickGetMaxTrigger (TimerDev_t * const pTimer);
static int      SystickFindAvailTrig (TimerDev_t * const pTimer);
static void     SystickDisableTrigger(TimerDev_t * const pTimer, int TrigNo);
static uint64_t SystickEnableTrigger (TimerDev_t * const pTimer, int TrigNo,
                                       uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                       TimerTrigEvtHandler_t const Handler,
                                       void * const pContext);
static void     SystickDisableExtTrig(TimerDev_t * const pTimer);
static bool     SystickEnableExtTrig (TimerDev_t * const pTimer,
                                       int TrigDevNo, TIMER_EXTTRIG_SENSE Sense);

// ── Enable ────────────────────────────────────────────────────────────────────

static bool SystickEnable(TimerDev_t * const pTimer)
{
    SysTickStart();
    return true;
}

// ── Disable ───────────────────────────────────────────────────────────────────

static void SystickDisable(TimerDev_t * const pTimer)
{
    SysTickStop();
}

// ── Reset ─────────────────────────────────────────────────────────────────────

static void SystickReset(TimerDev_t * const pTimer)
{
    SysTickSetReload(SysTickGetLoad());   // rewrite LOAD to reset VAL → 0
    pTimer->Rollover  = 0u;
    pTimer->LastCount = 0u;
}

// ── SetFrequency ─────────────────────────────────────────────────────────────

static uint32_t SystickSetFrequency(TimerDev_t * const pTimer, uint32_t Freq)
{
    uint32_t achieved = SysTickSetFrequency(s_SysTickData.CoreClkHz, Freq);

    if (achieved == 0u) {
        return pTimer->Freq;
    }

    pTimer->Freq     = achieved;
    pTimer->nsPeriod = (1000000000ULL + (achieved >> 1u)) / achieved;

    return achieved;
}

// ── GetTickCount ──────────────────────────────────────────────────────────────
// Monotonic 64-bit count built from the 24-bit hardware counter + software
// rollover accumulator. SysTick counts DOWN; elapsed = LOAD - VAL.
//
// COUNTFLAG check: SysTickGetCountFlag() reads CTRL, which clears COUNTFLAG.
// This is atomic w.r.t. the ISR because the ISR increments Rollover and we
// read CTRL+VAL in the same interrupt-disabled window via PRIMASK in callers.

static uint64_t SystickGetTickCount(TimerDev_t * const pTimer)
{
    if (SysTickGetCountFlag()) {
        pTimer->Rollover += (uint64_t)(SysTickGetLoad() + 1u);
    }

    uint32_t elapsed = SysTickGetLoad() - SysTickGetVal();
    return pTimer->Rollover + (uint64_t)elapsed;
}

// ── GetMaxTrigger ─────────────────────────────────────────────────────────────

static int SystickGetMaxTrigger(TimerDev_t * const pTimer)
{
    return SYSTICK_TRIG_MAXCNT;
}

// ── FindAvailTrigger ─────────────────────────────────────────────────────────

static int SystickFindAvailTrig(TimerDev_t * const pTimer)
{
    return (s_SysTickData.Trigger[0].Handler == nullptr) ? 0 : -1;
}

// ── EnableTrigger ─────────────────────────────────────────────────────────────
// Configures the tick interrupt. TrigNo must be 0 (one SysTick trigger slot).
// SINGLE type fires once; CONTINUOUS reloads automatically.

static uint64_t SystickEnableTrigger(TimerDev_t * const pTimer, int TrigNo,
                                       uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
                                       TimerTrigEvtHandler_t const Handler,
                                       void * const pContext)
{
    if (TrigNo != 0) {
        return 0u;
    }

    uint64_t reload_cycles =
        (nsPeriod + (pTimer->nsPeriod >> 1u)) / pTimer->nsPeriod;

    if (reload_cycles == 0u || reload_cycles > 0x01000000u) {
        return 0u;
    }

    s_SysTickData.Trigger[0].Type     = Type;
    s_SysTickData.Trigger[0].nsPeriod = pTimer->nsPeriod * reload_cycles;
    s_SysTickData.Trigger[0].Handler  = Handler;
    s_SysTickData.Trigger[0].pContext = pContext;

    SysTickSetReload((uint32_t)(reload_cycles - 1u));
    SysTickEnableInt();
    SysTickStart();

    return s_SysTickData.Trigger[0].nsPeriod;
}

// ── DisableTrigger ────────────────────────────────────────────────────────────

static void SystickDisableTrigger(TimerDev_t * const pTimer, int TrigNo)
{
    if (TrigNo != 0) {
        return;
    }

    SysTickDisableInt();
    s_SysTickData.Trigger[0].Handler  = nullptr;
    s_SysTickData.Trigger[0].pContext = nullptr;
}

// ── ExtTrigger stubs (SysTick has no external trigger input) ─────────────────

static void SystickDisableExtTrig(TimerDev_t * const pTimer) {}

static bool SystickEnableExtTrig(TimerDev_t * const pTimer,
                                   int TrigDevNo, TIMER_EXTTRIG_SENSE Sense)
{
    return false;
}

// ── TimerInit ─────────────────────────────────────────────────────────────────
// Public entry point called by the application MCU port setup.
//
// pCfg->DevNo   : repurposed as CoreClkHz (always 0 for SysTick otherwise)
// pCfg->Freq    : desired tick rate in Hz
// pCfg->IntPrio : SysTick exception priority (0xFF = lowest, recommended)

bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg)
{
    if (pTimer == nullptr || pCfg == nullptr) {
        return false;
    }

    memset(&s_SysTickData, 0, sizeof(s_SysTickData));

    s_SysTickData.DevNo     = 0;
    s_SysTickData.CoreClkHz = (uint32_t)pCfg->DevNo;
    s_SysTickData.pTimer    = pTimer;

    pTimer->DevNo      = 0;
    pTimer->Rollover   = 0u;
    pTimer->LastCount  = 0u;
    pTimer->EvtHandler = pCfg->EvtHandler;
    pTimer->pObj       = &s_SysTickData;

    // Wire function pointers — same pattern as all IOsonata MCU port timers.
    pTimer->Disable           = SystickDisable;
    pTimer->Enable            = SystickEnable;
    pTimer->Reset             = SystickReset;
    pTimer->GetTickCount      = SystickGetTickCount;
    pTimer->SetFrequency      = SystickSetFrequency;
    pTimer->GetMaxTrigger     = SystickGetMaxTrigger;
    pTimer->FindAvailTrigger  = SystickFindAvailTrig;
    pTimer->DisableTrigger    = SystickDisableTrigger;
    pTimer->EnableTrigger     = SystickEnableTrigger;
    pTimer->DisableExtTrigger = SystickDisableExtTrig;
    pTimer->EnableExtTrigger  = SystickEnableExtTrig;

    // Stop and configure SysTick.
    SysTickStop();
    SysTickSetPriority(pCfg->IntPrio);

    // Apply initial frequency — writes LOAD, computes Freq + nsPeriod.
    SystickSetFrequency(pTimer, pCfg->Freq);

    // Start with processor clock source; TICKINT armed by EnableTrigger().
    SysTickStart();

    return true;
}

// ── SysTick_Handler ──────────────────────────────────────────────────────────
// Fires at the tick rate set by EnableTrigger / TimerInit.
// Increments the 64-bit rollover accumulator and dispatches to the
// registered EvtHandler — which is TaktOS's tick handler in RTOS use,
// or a user callback in bare-metal use.

extern "C" void SysTick_Handler(void)
{
    TimerDev_t * const pTimer = s_SysTickData.pTimer;

    // Accumulate one full LOAD+1 cycle worth of ticks into the rollover.
    // GetTickCount() reads LOAD-VAL for sub-tick resolution on top of this.
    pTimer->Rollover += (uint64_t)(SysTickGetLoad() + 1u);

    // Single-shot: disable interrupt after first fire.
    if (s_SysTickData.Trigger[0].Type == TIMER_TRIG_TYPE_SINGLE) {
        SysTickDisableInt();
    }

    if (pTimer->EvtHandler != nullptr) {
        pTimer->EvtHandler(pTimer, TIMER_EVT_TICK);
    }
}
