/**-------------------------------------------------------------------------
@file timer_ra4m1.cpp
@brief RA4M1 AGT and GPT implementation of the IOsonata Timer interface.

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
#include "ra4m1xxx.h"
#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "interrupt_ra4m1.h"
#include "timer_ra4m1.h"


struct Ra4m1TimerTrigger {
    TimerTrig_t Info;
    uint64_t Deadline;
    uint32_t Ticks, Generation;
    IRQn_Type Irq;
    uint8_t DevNo, No;
    bool Active;
};
struct Ra4m1TimerData {
    TimerDev_t *Timer;
    Ra4m1TimerTrigger *Trigger;
    R_AGTX0_Type *AgtReg;
    R_GPT0_Type *GptReg;
    uint64_t Cycle; // 2^32 is representable; AGT cycle can be shortened.
    uint32_t Mask, BaseFreq, Divider, SyncCycles, Epoch;
    IRQn_Type Irq;
    TIMER_CLKSRC Source;
    uint8_t No, ClockBits, MaxTrigger, Priority;
    bool Agt, Running, Fault;
};
static R_AGTX0_Type * const s_AgtRegs[2] = {R_AGT0, R_AGT1};
static R_GPT0_Type * const s_GptRegs[8] = {
    R_GPT0, R_GPT1, R_GPT2, R_GPT3,
    R_GPT4, R_GPT5, R_GPT6, R_GPT7
};
static const uint8_t s_GptCompareIndex[6] = {0, 1, 2, 4, 3, 5};
static Ra4m1TimerData s_Timer[RA4M1_TIMER_COUNT];
static Ra4m1TimerTrigger s_AgtTrigger[2];
static Ra4m1TimerTrigger s_GptTrigger[8][6];
volatile uint32_t g_Ra4m1TimerError[RA4M1_TIMER_COUNT];

static uint32_t ModuleBit(const Ra4m1TimerData &d)
{
    return 1UL << (d.Agt ? 3U-d.No : (d.No < 4 ? 5U : 6U));
}
static bool ModuleWrite(uint32_t Value)
{
    const uint16_t prcr = R_SYSTEM->PRCR & 15U;
    R_SYSTEM->PRCR =  0xA500U | prcr | 0x02U;
    R_MSTP->MSTPCRD =  Value;
    const bool ok = R_MSTP->MSTPCRD == Value;
    R_SYSTEM->PRCR =  0xA500U | prcr;
    return ok;
}
/* AGT's LF counting continues with MSTPD set. Section 23.4.10 requires its
 * bus gate to be closed outside register access. Nesting is excluded by the
 * CPU critical section; callbacks always execute after this scope ends.
 */
class TimerAccess {
    Ra4m1TimerData &d;
    uint32_t State, Wp;
public:
    bool Ok;
    explicit TimerAccess(Ra4m1TimerData &Dev) : d(Dev), State(DisableInterrupt()), Wp(0), Ok(false)
    {
        uint32_t mask = R_MSTP->MSTPCRD;
        if (d.Agt) Ok = ModuleWrite(mask & ~ModuleBit(d));
        else Ok = !(mask & ModuleBit(d));
        if (Ok && !d.Agt) {
            Wp = d.GptReg->GTWP & 1U;
            d.GptReg->GTWP =  0xA500UL;
            Ok = !(d.GptReg->GTWP & 1U);
        }
        if (!Ok) g_Ra4m1TimerError[d.No] = RA4M1_TIMER_ACCESS_ERROR;
    }
    ~TimerAccess()
    {
        if (Ok && !d.Agt) d.GptReg->GTWP =  0xA500UL | Wp;
        if (d.Agt && !ModuleWrite(R_MSTP->MSTPCRD | ModuleBit(d)))
            g_Ra4m1TimerError[d.No] = RA4M1_TIMER_ACCESS_ERROR;
        EnableInterrupt(State);
    }
};
static Ra4m1TimerData *Data(TimerDev_t *t)
{
    if (!t || (unsigned)t->DevNo >= RA4M1_TIMER_COUNT) return nullptr;
    return s_Timer[t->DevNo].Timer == t ? &s_Timer[t->DevNo] : nullptr;
}
static void Sync(const Ra4m1TimerData &d)
{
    __DSB();
    for (uint32_t i=0; i<d.SyncCycles; ++i) __NOP();
}
static bool IrqPending(IRQn_Type irq)
{
    return irq >= 0 && (R_ICU->IELSR[(unsigned)irq] & (1UL << 16));
}
static bool Overflow(const Ra4m1TimerData &d)
{
    const bool flag = d.Agt ? (d.AgtReg->AGT16.CTRL.AGTCR & 0x20U) :
        (d.GptReg->GTST & (1UL << 6));
    return flag || IrqPending(d.Irq);
}
static uint32_t Count(const Ra4m1TimerData &d)
{
    return d.Agt ? (uint32_t)(d.Cycle-1U)-d.AgtReg->AGT16.AGT :
        d.GptReg->GTCNT & d.Mask;
}
/* Caller excludes ISR updates, not the hardware clock. If a wrap races the
 * sample, reread the counter from the new cycle. Do not consume the IRQ here.
 * The overflow IRQ must be serviced at least once per hardware cycle.
 */
static uint64_t Ticks(const Ra4m1TimerData &d)
{
    bool before = Overflow(d);
    uint32_t count = Count(d);
    bool after = Overflow(d);
    if (after && !before) count = Count(d);
    return d.Timer->Rollover + count + (after ? d.Cycle : 0U);
}
static void ClearOverflow(Ra4m1TimerData &d)
{
    /* These status flags only permit writing zero. GPT compare IRQ latches
     * are independent: never use their sticky GTST bits as dispatch causes.
     */
    if (d.Agt) d.AgtReg->AGT16.CTRL.AGTCR =  d.Running ? 0x01U : 0U;
    else d.GptReg->GTST =  0U;
    Sync(d);
    if (d.Irq >= 0) Ra4m1AcknowledgeInt(d.Irq);
}
static void IrqsEnable(Ra4m1TimerData &d, bool Enable)
{
    if (d.Irq >= 0) {
        if (Enable) NVIC_EnableIRQ(d.Irq); else NVIC_DisableIRQ(d.Irq);
    }
    if (!d.Agt) for (unsigned i=0; i<d.MaxTrigger; ++i) {
        Ra4m1TimerTrigger &tr = d.Trigger[i];
        if (tr.Irq >= 0) {
            if (Enable && tr.Active) NVIC_EnableIRQ(tr.Irq); else NVIC_DisableIRQ(tr.Irq);
        }
    }
}
static bool Stop(Ra4m1TimerData &d)
{
    if (d.Agt) {
        /* Do not copy W0 status ones back. A previously latched underflow
         * remains in the ICU; settle the pulse before the control write.
         */
        Sync(d);
        d.AgtReg->AGT16.CTRL.AGTCR =  0U;
        for (uint32_t i=0; i<1000000UL; ++i)
            if (!(d.AgtReg->AGT16.CTRL.AGTCR & 0x02U)) {
                d.Running=false; return true;
            }
    } else {
        d.GptReg->GTCR =  (uint32_t)d.ClockBits << 24;
        /* CST readback precedes the final prescaled counter edge (22.9.4).
         * Wait at least two count-clock cycles before reading the stopped CNT.
         */
        for (uint32_t i=0; i<d.SyncCycles*d.Divider; ++i) __NOP();
        if (!(d.GptReg->GTCR&1U)) { d.Running=false; return true; }
    }
    d.Fault=true; IrqsEnable(d,false);
    g_Ra4m1TimerError[d.No]=RA4M1_TIMER_STOP_TIMEOUT;
    return false;
}
static bool Start(Ra4m1TimerData &d)
{
    if (d.Fault) return false;
    if (d.Agt) {
        ClearOverflow(d);
        NVIC_ClearPendingIRQ(d.Irq);
        d.AgtReg->AGT16.CTRL.AGTCR =  0x01U;
        for (uint32_t i=0; i<1000000UL; ++i)
            if (d.AgtReg->AGT16.CTRL.AGTCR&0x02U) {
                d.Running=true; IrqsEnable(d,true); return true;
            }
    } else {
        d.GptReg->GTCR =  ((uint32_t)d.ClockBits<<24)|1U;
        if (d.GptReg->GTCR&1U) {
            d.Running=true; IrqsEnable(d,true); return true;
        }
    }
    d.Fault=true; IrqsEnable(d,false);
    g_Ra4m1TimerError[d.No]=RA4M1_TIMER_START_TIMEOUT;
    return false;
}
struct TimerPlan { uint32_t Base, Divider, Frequency, SyncCycles; uint8_t Bits; TIMER_CLKSRC Source; };
static bool Plan(bool Agt, TIMER_CLKSRC Source, uint32_t Request, TimerPlan &p)
{
    if (Agt && Source == TIMER_CLKSRC_DEFAULT)
        Source=g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC ? TIMER_CLKSRC_LFRC : TIMER_CLKSRC_LFXTAL;
    uint32_t base=0;
    if (Agt) {
        if (Source == TIMER_CLKSRC_LFRC && !(R_SYSTEM->LOCOCR&1U)) base=32768U;
        if (Source == TIMER_CLKSRC_LFXTAL && g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL &&
            g_McuOsc.LowPwrOsc.Freq == 32768U && !(R_SYSTEM->SOSCCR&1U)) base=32768U;
    } else {
        uint8_t src=R_SYSTEM->SCKSCR&7U;
        if (Source == TIMER_CLKSRC_DEFAULT ||
            (Source == TIMER_CLKSRC_HFRC && (src==0U || src==1U)) ||
            (Source == TIMER_CLKSRC_HFXTAL && (src==3U || src==5U) &&
             g_McuOsc.CoreOsc.Type==OSC_TYPE_XTAL)) base=SystemPeriphClockGet(3);
    }
    uint32_t bus=SystemPeriphClockGet(Agt ? 1 : 3), cpu=SystemCoreClockGet();
    if (!base || !bus || !cpu || bus>cpu) return false;
    uint32_t best=UINT32_MAX;
    for (unsigned i=0; i<(Agt ? 8U : 6U); ++i) {
        uint32_t div=1UL << (Agt ? i : 2U*i), f=base/div;
        if (!f) continue;
        uint32_t diff=Request ? (f>Request ? f-Request : Request-f) : base-f;
        if (diff<best) {best=diff; p.Bits=(uint8_t)i; p.Divider=div; p.Frequency=f;}
    }
    p.Base=base; p.Source=Source; p.SyncCycles=2U*((cpu+bus-1U)/bus)+2U;
    return best!=UINT32_MAX;
}
/* Split seconds before multiplying. Reject huge requests before overflow. */
static bool Period(uint32_t Freq, uint64_t Ns, uint64_t Max, uint32_t &CountOut, uint64_t &Actual)
{
    if (!Freq || !Ns || Ns/1000000000ULL > Max/Freq) return false;
    uint64_t count=(Ns/1000000000ULL)*Freq +
        ((Ns%1000000000ULL)*Freq+500000000ULL)/1000000000ULL;
    if (count<4U || count>Max || count>UINT32_MAX) return false;
    CountOut=(uint32_t)count;
    Actual=(count*1000000000ULL+Freq/2U)/Freq;
    return true;
}
static bool CompareWrite(Ra4m1TimerData &d, unsigned No, uint64_t Deadline)
{

    // Rollover may hold a non-aligned origin after pause/reconfiguration.
    uint32_t value=(uint32_t)(Deadline-d.Timer->Rollover) & d.Mask;
    d.GptReg->GTCCR[s_GptCompareIndex[No]] = value;
    return d.GptReg->GTCCR[s_GptCompareIndex[No]] == value;
}
static void CompareIrq(int IrqNo, void *Ctx);
static bool ArmCompare(Ra4m1TimerData &d, Ra4m1TimerTrigger &tr, bool Periodic)
{
    for (unsigned attempt=0; attempt<4; ++attempt) {
        uint64_t now=Ticks(d);
        if (Periodic && tr.Deadline<=now) {
            uint64_t steps=(now-tr.Deadline)/tr.Ticks+1U;
            if (steps>(UINT64_MAX-tr.Deadline)/tr.Ticks) break;
            tr.Deadline+=steps*tr.Ticks;
        }
        if (!CompareWrite(d,tr.No,tr.Deadline)) break;
        // Retire an old-target pulse arriving during the rewrite. If the new
        // target also passed during synchronization, the check below reports
        // a missed single shot or advances the periodic phase, never a wrap-long wait.
        Sync(d); Ra4m1AcknowledgeInt(tr.Irq);
        if (!d.Running || Ticks(d)<tr.Deadline || IrqPending(tr.Irq)) return true;
        /* Programming missed the target without an event. Never wait a whole
         * counter wrap for it. A periodic alarm advances to its next phase.
         */
        if (!Periodic) break;
    }
    tr.Active=false;
    if (tr.Irq>=0) NVIC_DisableIRQ(tr.Irq);
    g_Ra4m1TimerError[d.No]=RA4M1_TIMER_COMPARE_ERROR;
    return false;
}
static void TriggerCall(Ra4m1TimerData &d, const TimerTrig_t &info, unsigned No)
{
    if (info.Handler) info.Handler(d.Timer,(int)No,info.pContext);
    else if (d.Timer->EvtHandler) d.Timer->EvtHandler(d.Timer,TIMER_EVT_TRIGGER(No));
}
static void OverflowIrq(int IrqNo, void *Ctx)
{
    Ra4m1TimerData &d=*(Ra4m1TimerData*)Ctx;
    TimerTrig_t info={}; bool call=false; uint32_t epoch=0, generation=0;
    TimerEvtHandler_t handler=nullptr;
    {
        TimerAccess access(d);
        if (!access.Ok || d.Irq!=IrqNo || !d.Timer) return;
        if (d.Running && !d.Fault && Overflow(d)) {
            d.Timer->Rollover+=d.Cycle;
            handler=d.Timer->EvtHandler;
            if (d.Agt && d.Trigger[0].Active) {
                Ra4m1TimerTrigger &tr=d.Trigger[0]; info=tr.Info;
                if (info.Type==TIMER_TRIG_TYPE_SINGLE) tr.Active=false;
                generation=tr.Generation; call=true;
            }
        }
        ClearOverflow(d); epoch=d.Epoch;
    }
    if (handler) handler(d.Timer,TIMER_EVT_COUNTER_OVR);
    if (call) {
        uint32_t state=DisableInterrupt();
        call=d.Epoch==epoch && d.Running && d.Trigger[0].Generation==generation;
        EnableInterrupt(state);
        if (call) TriggerCall(d,info,0);
    }
}
static void CompareIrq(int IrqNo, void *Ctx)
{
    Ra4m1TimerTrigger &tr=*(Ra4m1TimerTrigger*)Ctx;
    Ra4m1TimerData &d=s_Timer[tr.DevNo]; TimerTrig_t info={}; bool call=false;
    {
        TimerAccess access(d);
        if (!access.Ok || tr.Irq!=IrqNo) return;
        /* GPT event requests are pulses, independent of sticky GTST flags.
         * Acknowledge before rearming/callback; never clear another route.
         */
        Sync(d); Ra4m1AcknowledgeInt(tr.Irq);
        if (tr.Active && d.Running && !d.Fault) {
            info=tr.Info; call=true;
            if (info.Type==TIMER_TRIG_TYPE_SINGLE) {
                tr.Active=false; NVIC_DisableIRQ(tr.Irq);
            } else {
                if (tr.Deadline>UINT64_MAX-tr.Ticks) {
                    tr.Active=false; NVIC_DisableIRQ(tr.Irq);
                } else {
                    tr.Deadline+=tr.Ticks; ArmCompare(d,tr,true);
                }
            }
        }
    }
    if (call) TriggerCall(d,info,tr.No);
}
static uint64_t GetCount(TimerDev_t *t)
{
    Ra4m1TimerData *d=Data(t); if (!d) return 0;
    TimerAccess access(*d); if (!access.Ok || d->Fault) return 0;
    uint64_t value=Ticks(*d); t->LastCount=(uint32_t)value; return value;
}
/* Stop, settle and fold the old hardware cycle into the software origin. */
static bool Rebase(Ra4m1TimerData &d, uint64_t Cycle, bool Zero)
{
    IrqsEnable(d,false);
    bool pending=Overflow(d);
    if (!Stop(d)) return false;
    uint64_t elapsed=d.Timer->Rollover+Count(d)+((pending||Overflow(d)) ? d.Cycle : 0U);
    ClearOverflow(d);
    d.Timer->Rollover=Zero ? 0 : elapsed; d.Timer->LastCount=0;
    d.Cycle=Cycle;
    if (d.Agt) d.AgtReg->AGT16.AGT = (uint16_t)(Cycle-1U);
    else d.GptReg->GTCNT = 0U;
    for (unsigned i=0; i<d.MaxTrigger && !d.Agt; ++i) {
        Ra4m1TimerTrigger &tr=d.Trigger[i];
        if (tr.Irq>=0) { Ra4m1AcknowledgeInt(tr.Irq); NVIC_ClearPendingIRQ(tr.Irq); }
    }
    NVIC_ClearPendingIRQ(d.Irq);
    ++d.Epoch;
    bool ok=d.Agt ? d.AgtReg->AGT16.AGT==Cycle-1U : d.GptReg->GTCNT==0;
    if (!ok) { d.Fault=true; g_Ra4m1TimerError[d.No]=RA4M1_TIMER_ACCESS_ERROR; }
    return ok;
}
static bool RestartTriggers(Ra4m1TimerData &d)
{
    if (!d.Agt) for (unsigned i=0; i<d.MaxTrigger; ++i) {
        Ra4m1TimerTrigger &tr=d.Trigger[i];
        if (tr.Active) { tr.Deadline=d.Timer->Rollover+tr.Ticks; if (!ArmCompare(d,tr,false)) return false; }
    }
    return true;
}
static void Disable(TimerDev_t *t)
{
    Ra4m1TimerData *d=Data(t); if (!d) return;
    TimerAccess access(*d); if (!access.Ok) return;
    Rebase(*d,d->Cycle,false); // Pause count; restart active trigger periods on Enable.
}
static bool Enable(TimerDev_t *t)
{
    Ra4m1TimerData *d=Data(t); if (!d) return false;
    TimerAccess access(*d); if (!access.Ok || d->Fault) return false;
    return d->Running || (RestartTriggers(*d) && Start(*d));
}
static void Reset(TimerDev_t *t)
{
    Ra4m1TimerData *d=Data(t); if (!d) return;
    TimerAccess access(*d); if (!access.Ok) return;
    bool running=d->Running;
    if (Rebase(*d,d->Cycle,true)) {
        d->Fault=false; g_Ra4m1TimerError[d->No]=RA4M1_TIMER_OK;
        if (RestartTriggers(*d) && running) Start(*d);
    }
}
static uint32_t Frequency(TimerDev_t *t, uint32_t Request)
{
    Ra4m1TimerData *d=Data(t); if (!d) return 0;
    TimerPlan plan={};
    if (!Plan(d->Agt,d->Source,Request,plan)) {
        g_Ra4m1TimerError[d->No]=RA4M1_TIMER_CLOCK_ERROR; return 0;
    }
    TimerAccess access(*d); if (!access.Ok || d->Fault) return 0;
    uint32_t count[6]={}; uint64_t ns[6]={};
    for (unsigned i=0; i<d->MaxTrigger; ++i)
        if (d->Trigger[i].Active && !Period(plan.Frequency,d->Trigger[i].Info.nsPeriod,
            d->Agt ? 65536U : d->Mask,count[i],ns[i])) return 0;
    uint64_t cycle=d->Agt && count[0] ? count[0] : (uint64_t)d->Mask+1U;
    if (!Rebase(*d,cycle,true)) return 0;
    d->BaseFreq=plan.Base; d->Divider=plan.Divider; d->ClockBits=plan.Bits; d->SyncCycles=plan.SyncCycles;
    if (d->Agt) d->AgtReg->AGT16.CTRL.AGTMR2 = d->ClockBits;
    else d->GptReg->GTCR = (uint32_t)d->ClockBits<<24;
    bool programmed=d->Agt ? d->AgtReg->AGT16.CTRL.AGTMR2==d->ClockBits :
        d->GptReg->GTCR==((uint32_t)d->ClockBits<<24);
    if (!programmed) {
        d->Fault=true; t->Freq=0; t->nsPeriod=0;
        g_Ra4m1TimerError[d->No]=RA4M1_TIMER_ACCESS_ERROR; return 0;
    }
    t->Freq=plan.Frequency; t->nsPeriod=(1000000000ULL+t->Freq/2U)/t->Freq;
    for (unsigned i=0; i<d->MaxTrigger; ++i) if (d->Trigger[i].Active) {
        d->Trigger[i].Ticks=count[i]; d->Trigger[i].Info.nsPeriod=ns[i];
    }
    // Changing AGT mode can leave status flags undefined; clear before start.
    ClearOverflow(*d);
    return RestartTriggers(*d) && Start(*d) ? t->Freq : 0;
}
static int MaxTrigger(TimerDev_t *t) { Ra4m1TimerData *d=Data(t); return d ? d->MaxTrigger : 0; }
static int FindTrigger(TimerDev_t *t)
{
    Ra4m1TimerData *d=Data(t); if (!d) return -1;
    uint32_t state=DisableInterrupt(); int result=-1;
    for (unsigned i=0; i<d->MaxTrigger; ++i) if (!d->Trigger[i].Active) {result=(int)i;break;}
    EnableInterrupt(state); return result;
}
static uint64_t TriggerEnable(TimerDev_t *t, int No, uint64_t Ns, TIMER_TRIG_TYPE Type,
    TimerTrigEvtHandler_t Handler, void *Ctx)
{
    Ra4m1TimerData *d=Data(t);
    if (!d || (unsigned)No>=d->MaxTrigger ||
        (Type!=TIMER_TRIG_TYPE_SINGLE && Type!=TIMER_TRIG_TYPE_CONTINUOUS)) return 0;
    uint32_t count=0; uint64_t actual=0;
    if (!Period(t->Freq,Ns,d->Agt ? 65536U : d->Mask,count,actual)) return 0;
    TimerAccess access(*d); if (!access.Ok || d->Fault) return 0;
    Ra4m1TimerTrigger &tr=d->Trigger[No];
    bool running=d->Running;
    if (d->Agt) {
        if (!Rebase(*d,count,false)) return 0;
    } else {
        if (tr.Irq<0) {
            tr.Irq=Ra4m1RegisterIntHandler((uint8_t)(RA4M1_EVTID_GPT0_CCMPA+8U*(d->No-2U)+No),
                (int)d->Priority,CompareIrq,&tr);
            if (tr.Irq<0) {g_Ra4m1TimerError[d->No]=RA4M1_TIMER_IRQ_ERROR;return 0;}
        }
        NVIC_DisableIRQ(tr.Irq); Sync(*d); Ra4m1AcknowledgeInt(tr.Irq); NVIC_ClearPendingIRQ(tr.Irq);
    }
    tr.Info.Type=Type; tr.Info.nsPeriod=actual; tr.Info.Handler=Handler; tr.Info.pContext=Ctx;
    tr.Ticks=count; tr.Active=true; ++tr.Generation;
    if (d->Agt) {
        if (running && !Start(*d)) {tr.Active=false;return 0;}
    } else {
        uint64_t now=Ticks(*d);
        if (now>UINT64_MAX-count) {tr.Active=false;return 0;}
        tr.Deadline=now+count;
        if (!ArmCompare(*d,tr,false)) return 0;
        if (running) NVIC_EnableIRQ(tr.Irq);
    }
    return actual;
}
static void TriggerDisable(TimerDev_t *t, int No)
{
    Ra4m1TimerData *d=Data(t); if (!d || (unsigned)No>=d->MaxTrigger) return;
    TimerAccess access(*d); if (!access.Ok) return;
    Ra4m1TimerTrigger &tr=d->Trigger[No]; tr.Active=false; ++tr.Generation;
    if (d->Agt) {
        bool running=d->Running;
        if (Rebase(*d,65536U,false) && running) Start(*d);
    } else if (tr.Irq>=0) {
        NVIC_DisableIRQ(tr.Irq); Ra4m1UnregisterIntHandler(tr.Irq); tr.Irq=(IRQn_Type)-1;
    }
}
static void ExtDisable(TimerDev_t *) {}
static bool ExtEnable(TimerDev_t *, int, TIMER_EXTTRIG_SENSE) { return false; }

bool TimerInit(TimerDev_t *t, const TimerCfg_t *cfg)
{
    if (!t || !cfg || (unsigned)cfg->DevNo>=RA4M1_TIMER_COUNT || cfg->bTickInt ||
        (unsigned)cfg->IntPrio >= (1UL<<__NVIC_PRIO_BITS)) return false;
    TimerPlan plan={}; bool agt=cfg->DevNo<2;
    if (!Plan(agt,cfg->ClkSrc,cfg->Freq,plan)) return false;
    uint32_t state=DisableInterrupt();
    for (unsigned i=0;i<RA4M1_TIMER_COUNT;++i) if (s_Timer[i].Timer==t) {EnableInterrupt(state);return false;}
    Ra4m1TimerData &d=s_Timer[cfg->DevNo];
    if (d.Timer || d.Fault) {EnableInterrupt(state);return false;}
    // A stopped channel may still belong to a CPU/DTC/DMAC client.
    uint8_t first=agt ? (uint8_t)(RA4M1_EVTID_AGT0_AGTI+3U*cfg->DevNo) :
        (uint8_t)(RA4M1_EVTID_GPT0_CCMPA+8U*(cfg->DevNo-2));
    uint8_t last=(uint8_t)(first+(agt ? 2U : 7U));
    for (unsigned i=0;i<RA4M1_IELS_CNT+4U;++i) {
        uint32_t event = i < RA4M1_IELS_CNT ?
            R_ICU->IELSR[i] & 0xFFU : R_ICU->DELSR[i-RA4M1_IELS_CNT] & 0xFFU;
        if (event>=first && event<=last) {EnableInterrupt(state);return false;}
    }
    d.No=(uint8_t)cfg->DevNo; d.Agt=agt; d.AgtReg = agt ? s_AgtRegs[d.No] : nullptr;
    d.GptReg = agt ? nullptr : s_GptRegs[d.No-2U];
    d.Trigger=agt ? &s_AgtTrigger[d.No] : s_GptTrigger[d.No-2U];
    d.MaxTrigger=agt ? 1U : 6U; d.Mask=(!agt && d.No<4) ? UINT32_MAX : 65535U;
    d.Cycle=(uint64_t)d.Mask+1U; d.BaseFreq=plan.Base; d.Divider=plan.Divider;
    d.ClockBits=plan.Bits; d.Source=plan.Source; d.SyncCycles=plan.SyncCycles;
    d.Irq=(IRQn_Type)-1; d.Fault=false; d.Running=false; d.Priority=(uint8_t)cfg->IntPrio;
    uint32_t previous=R_MSTP->MSTPCRD;
    if (!agt && !ModuleWrite(previous & ~ModuleBit(d))) {EnableInterrupt(state);return false;}
    bool ok=false, startAttempted=false;
    {
        TimerAccess access(d);
        if (access.Ok) {
            /* Do not overwrite a running GPT/AGT owned outside this driver. */
            bool busy = false;
            if (agt)
            {
                uint8_t status = d.AgtReg->AGT16.CTRL.AGTCR;
                if ((status & 0x01U) != 0U)
                {
                    busy = true;
                }
                else
                {
                    for (uint32_t poll = 0U; poll < 1000000UL; ++poll)
                    {
                        status = d.AgtReg->AGT16.CTRL.AGTCR;
                        if ((status & 0x02U) == 0U)
                        {
                            break;
                        }
                    }
                    busy = (status & 0x03U) != 0U;
                }
            }
            else
            {
                busy = (d.GptReg->GTCR & 1U) != 0U;
            }
            if (!busy) {
                if (agt) {
                    d.AgtReg->AGT16.CTRL.AGTCMSR = 0U;
                    d.AgtReg->AGT16.CTRL.AGTMR1 = plan.Source==TIMER_CLKSRC_LFRC ? 0x40U : 0x60U;
                    d.AgtReg->AGT16.CTRL.AGTMR2 = plan.Bits;
                    d.AgtReg->AGT16.CTRL.AGTIOC = 0U; d.AgtReg->AGT16.CTRL.AGTISR = 0U;
                    d.AgtReg->AGT16.CTRL.AGTIOSEL = 0U;
                    d.AgtReg->AGT16.AGTCMA = 65535U; d.AgtReg->AGT16.AGTCMB = 65535U;
                    d.AgtReg->AGT16.AGT = 65535U; d.AgtReg->AGT16.CTRL.AGTCR = 0U;
                    ok=d.AgtReg->AGT16.AGT==65535U &&
                        d.AgtReg->AGT16.CTRL.AGTMR1==(plan.Source==TIMER_CLKSRC_LFRC ? 0x40U : 0x60U) &&
                        d.AgtReg->AGT16.CTRL.AGTMR2==plan.Bits;
                } else {
                    d.GptReg->GTSSR = 0U;
                    d.GptReg->GTPSR = 0U;
                    d.GptReg->GTCSR = 0U;
                    d.GptReg->GTUPSR = 0U;
                    d.GptReg->GTDNSR = 0U;
                    d.GptReg->GTICASR = 0U;
                    d.GptReg->GTICBSR = 0U;
                    d.GptReg->GTCR = (uint32_t)plan.Bits<<24;
                    d.GptReg->GTUDDTYC = 3U; // UDF forces up-count direction.
                    d.GptReg->GTUDDTYC = 1U;
                    d.GptReg->GTIOR = 0U; d.GptReg->GTINTAD = 0U;
                    d.GptReg->GTBER = 0U; d.GptReg->GTDTCR = 0U;
                    d.GptReg->GTDVU = 0U; d.GptReg->GTPR = d.Mask;
                    d.GptReg->GTPBR = d.Mask; d.GptReg->GTCNT = 0U;
                    d.GptReg->GTST = 0U;
                    for (unsigned i=0;i<6;++i) d.GptReg->GTCCR[s_GptCompareIndex[i]] = d.Mask;
                    ok=d.GptReg->GTPR==d.Mask &&
                        d.GptReg->GTCR==((uint32_t)plan.Bits<<24);
                }
                if (ok) {
                    for (unsigned i=0;i<d.MaxTrigger;++i) {
                        d.Trigger[i]={}; d.Trigger[i].Irq=(IRQn_Type)-1;
                        d.Trigger[i].DevNo=d.No; d.Trigger[i].No=(uint8_t)i;
                    }
                    uint8_t event=agt ? (uint8_t)(RA4M1_EVTID_AGT0_AGTI+3U*d.No) :
                        (uint8_t)(RA4M1_EVTID_GPT0_OVF+8U*(d.No-2U));
                    d.Irq=Ra4m1RegisterIntHandler(event,cfg->IntPrio,OverflowIrq,&d);
                    ok=d.Irq>=0;
                    if (!ok) g_Ra4m1TimerError[d.No]=RA4M1_TIMER_IRQ_ERROR;
                    if (ok) {
                        d.Timer=t; t->DevNo=d.No; t->Freq=plan.Frequency;
                        t->nsPeriod=(1000000000ULL+t->Freq/2U)/t->Freq;
                        t->Rollover=0; t->LastCount=0; t->EvtHandler=cfg->EvtHandler;
                        t->Disable=Disable; t->Enable=Enable; t->Reset=Reset; t->GetTickCount=GetCount;
                        t->SetFrequency=Frequency; t->GetMaxTrigger=MaxTrigger; t->FindAvailTrigger=FindTrigger;
                        t->DisableTrigger=TriggerDisable; t->EnableTrigger=TriggerEnable;
                        t->DisableExtTrigger=ExtDisable; t->EnableExtTrigger=ExtEnable;
                        g_Ra4m1TimerError[d.No]=RA4M1_TIMER_OK;
                        startAttempted=true; ok=Start(d);
                    }
                }
            }
        }
        if (!ok) {
            // No caller buffer or IRQ callback may survive failed Init.
            // Quarantine an unconfirmed stop until reset, rather than reusing
            // a channel that may still be running after a failed handshake.
            if (startAttempted) d.Fault=!Stop(d);
            if (d.Irq>=0) {Ra4m1UnregisterIntHandler(d.Irq);d.Irq=(IRQn_Type)-1;}
            d.Timer=nullptr;
        }
    }
    /* Restore only this group bit on failure. Never gate a neighbouring GPT
     * that was already clocked before this initialization attempt.
     */
    if (!ok && !agt && (previous&ModuleBit(d)))
        ModuleWrite(R_MSTP->MSTPCRD|ModuleBit(d));
    EnableInterrupt(state); return ok;
}
int TimerGetLowFreqDevCount(void) {return RA4M1_TIMER_AGT_COUNT;}
int TimerGetHighFreqDevCount(void) {return RA4M1_TIMER_GPT_COUNT;}
int TimerGetHighFreqDevNo(void) {return RA4M1_TIMER_AGT_COUNT;}
