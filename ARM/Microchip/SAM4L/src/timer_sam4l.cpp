/**-------------------------------------------------------------------------
@file	timer_sam4l.cpp

@brief	timer implementation on SAM4Lxx series

@author	Hoang Nguyen Hoan
@date	Aug. 24, 2021

@license

MIT License

Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#include "timer_sam4l.h"

volatile uint32_t g_Sam4lTimerInitStage;

Sam4l_TimerData_t g_Sam4lTimerData[SAM4L_TIMER_MAXCNT] = {
	{nullptr, nullptr, AST_ALARM_IRQn, PM_PBDMASK_AST},
	{nullptr, &SAM4L_TC0->TC_CHANNEL[0], TC00_IRQn, PM_PBAMASK_TC0},
	{nullptr, &SAM4L_TC0->TC_CHANNEL[1], TC01_IRQn, PM_PBAMASK_TC0},
	{nullptr, &SAM4L_TC0->TC_CHANNEL[2], TC02_IRQn, PM_PBAMASK_TC0},
	{nullptr, &SAM4L_TC1->TC_CHANNEL[0], TC10_IRQn, PM_PBAMASK_TC1},
	{nullptr, &SAM4L_TC1->TC_CHANNEL[1], TC11_IRQn, PM_PBAMASK_TC1},
	{nullptr, &SAM4L_TC1->TC_CHANNEL[2], TC12_IRQn, PM_PBAMASK_TC1},
};
void Sam4lTimerPmWrite(volatile uint32_t *reg, uint32_t value)
{
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAA) | PM_UNLOCK_ADDR((uintptr_t)reg - (uintptr_t)SAM4L_PM);
	*reg = value;
}
static Sam4l_TimerData_t *Data(TimerDev_t *t)
{
	if (!t || (unsigned)t->DevNo >= SAM4L_TIMER_MAXCNT) return nullptr;
	auto &d = g_Sam4lTimerData[t->DevNo];
	return d.Timer == t ? &d : nullptr;
}
static void Irq(Sam4l_TimerData_t &d, bool enable)
{
	if (enable) {
		NVIC_EnableIRQ(d.Irq);
		if (!d.TcReg) NVIC_EnableIRQ(AST_OVF_IRQn);
	} else {
		NVIC_DisableIRQ(d.Irq);
		if (!d.TcReg) NVIC_DisableIRQ(AST_OVF_IRQn);
	}
}
static bool Run(Sam4l_TimerData_t &d, bool run)
{
	if (d.TcReg) Sam4lTcRun(d, run);
	else if (!Sam4lAstRun(d, run)) d.Healthy = false;
	d.Running = run && d.Healthy;
	Irq(d, d.Running);
	return d.Healthy;
}
static uint64_t Count(Sam4l_TimerData_t &d)
{
	uint64_t count = d.TcReg ? Sam4lTcCount(d) : Sam4lAstCount(d);
	d.Timer->LastCount = (uint32_t)count;
	if (!d.Healthy) { d.Running = false; Irq(d, false); }
	return count;
}
static int MaxTrigger(TimerDev_t *t) { auto d = Data(t); return d ? (d->TcReg ? 3 : 1) : 0; }
static bool Arm(Sam4l_TimerData_t &d, int n, bool enable)
{
	if (d.TcReg) Sam4lTcArm(d, n, enable);
	else if (!Sam4lAstArm(d, enable)) d.Healthy = false;
	if (enable && Count(d) >= d.Trigger[n].Deadline) NVIC_SetPendingIRQ(d.Irq);
	if (!d.Healthy) { d.Running = false; Irq(d, false); }
	return d.Healthy;
}
static bool Plan(Sam4l_TimerData_t &d, uint32_t request, uint32_t &select, uint32_t &freq)
{
	if (!d.BaseFreq) return false;
	if (!request) request = d.TcReg ? d.BaseFreq / 128 : 1024;
	uint32_t best = UINT32_MAX;
	freq = 0;
	for (uint32_t n = (d.TcReg ? 1 : 0); n <= (d.TcReg ? 4 : 14); ++n) {
		uint32_t shift = d.TcReg ? 2 * n - 1 : n + 1;
		// AST CV reads must not outrun its synchronization (19.5.3).
		if (!d.TcReg && d.BaseFreq >= SystemPeriphClockGet(3) && n < 3) continue;
		uint32_t f = d.BaseFreq >> shift;
		if (!f) continue;
		uint32_t diff = f > request ? f - request : request - f;
		if (diff < best) { best = diff; select = n; freq = f; }
	}
	return freq != 0;
}
static bool Period(uint32_t freq, uint64_t ns, uint32_t &ticks, uint64_t &actual)
{
	if (!freq || !ns) return false;
	// Bound the whole-second product before multiplication.
	uint64_t v = UINT32_MAX;
	if (ns / 1000000000ULL <= UINT32_MAX / freq) {
		v = ns / 1000000000ULL * freq +
			(ns % 1000000000ULL * freq + 500000000ULL) / 1000000000ULL;
	}
	if (v > UINT32_MAX) v = UINT32_MAX;
	// Return the closest supported period; the application decides if it fits.
	if (v < 4) v = 4;
	ticks = (uint32_t)v;
	actual = (v * 1000000000ULL + freq / 2) / freq;
	return true;
}
static bool ResetCounter(Sam4l_TimerData_t &d)
{
	Run(d, false);
	if (!d.Healthy) return false;
	if (d.TcReg) Sam4lTcReset(d);
	else if (!Sam4lAstReset(d)) { d.Healthy = false; return false; }
	d.Timer->Rollover = 0; d.Timer->LastCount = 0; d.Overflow = false;
	++d.Epoch;
	NVIC_ClearPendingIRQ(d.Irq);
	if (!d.TcReg) NVIC_ClearPendingIRQ(AST_OVF_IRQn);
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) {
		d.Trigger[n].Deadline = d.Trigger[n].Ticks;
		if (!Arm(d, n, d.Trigger[n].Active)) return false;
	}
	return true;
}
static void Disable(TimerDev_t *t)
{
	auto d = Data(t); if (!d) return;
	uint32_t state = DisableInterrupt(); Run(*d, false); EnableInterrupt(state);
}
static bool Enable(TimerDev_t *t)
{
	auto d = Data(t); if (!d || !d->Healthy) return false;
	uint32_t state = DisableInterrupt(); bool ok = Run(*d, true); EnableInterrupt(state); return ok;
}
static void Reset(TimerDev_t *t)
{
	auto d = Data(t); if (!d || !d->Healthy) return;
	uint32_t state = DisableInterrupt();
	bool running = d->Running;
	if (ResetCounter(*d) && running) Run(*d, true);
	EnableInterrupt(state);
}
static uint64_t GetCount(TimerDev_t *t)
{
	auto d = Data(t); if (!d) return 0;
	uint32_t state = DisableInterrupt(); uint64_t value = Count(*d); EnableInterrupt(state); return value;
}
static uint32_t Frequency(TimerDev_t *t, uint32_t request)
{
	auto d = Data(t); if (!d || !d->Healthy) return 0;
	uint32_t state = DisableInterrupt();
	uint32_t select, freq, ticks[3] = {};
	uint64_t actual[3] = {};
	if (!Plan(*d, request, select, freq)) { EnableInterrupt(state); return 0; }
	for (int n = 0; n < MaxTrigger(t); ++n) if (d->Trigger[n].Active &&
		!Period(freq, d->Trigger[n].Info.nsPeriod, ticks[n], actual[n])) {
		EnableInterrupt(state); return 0;
	}
	if (!Run(*d, false)) { EnableInterrupt(state); return 0; }
	d->Select = select;
	if (d->TcReg) Sam4lTcSetup(*d);
	t->Freq = freq; t->nsPeriod = (1000000000ULL + freq / 2) / freq;
	for (int n = 0; n < MaxTrigger(t); ++n) if (d->Trigger[n].Active) {
		d->Trigger[n].Ticks = ticks[n]; d->Trigger[n].Info.nsPeriod = actual[n];
	}
	bool ok = ResetCounter(*d) && Run(*d, true);
	EnableInterrupt(state); return ok ? freq : 0;
}
static int FindTrigger(TimerDev_t *t)
{
	auto d = Data(t); if (!d || !d->Healthy) return -1;
	uint32_t state = DisableInterrupt(); int result = -1;
	for (int n = 0; n < MaxTrigger(t); ++n) if (!d->Trigger[n].Active) { result = n; break; }
	EnableInterrupt(state); return result;
}
static void TriggerDisable(TimerDev_t *t, int n)
{
	auto d = Data(t); if (!d || (unsigned)n >= (unsigned)MaxTrigger(t)) return;
	uint32_t state = DisableInterrupt();
	d->Trigger[n].Active = false; Arm(*d, n, false);
	EnableInterrupt(state);
}
static uint64_t TriggerEnable(TimerDev_t *t, int n, uint64_t ns, TIMER_TRIG_TYPE type,
	TimerTrigEvtHandler_t handler, void *ctx)
{
	auto d = Data(t);
	if (!d || !d->Healthy || (unsigned)n >= (unsigned)MaxTrigger(t) ||
		(type != TIMER_TRIG_TYPE_SINGLE && type != TIMER_TRIG_TYPE_CONTINUOUS)) return 0;
	uint32_t state = DisableInterrupt(), ticks;
	uint64_t actual;
	if (!Period(t->Freq, ns, ticks, actual)) { EnableInterrupt(state); return 0; }
	auto &tr = d->Trigger[n];
	tr.Info = {type, actual, handler, ctx}; tr.Ticks = ticks;
	tr.Deadline = Count(*d) + ticks; tr.Active = true;
	bool ok = Arm(*d, n, true);
	if (!ok) tr.Active = false;
	EnableInterrupt(state); return ok ? actual : 0;
}
static void ExtDisable(TimerDev_t *) {}
static bool ExtEnable(TimerDev_t *, int, TIMER_EXTTRIG_SENSE) { return false; }
void Sam4lTimerIRQ(int devno)
{
	auto &d = g_Sam4lTimerData[devno];
	if (!d.Timer || !d.Running || !d.Healthy) return;
	uint32_t state = DisableInterrupt();
	Count(d);
	uint32_t epoch = d.Epoch;
	bool overflow = d.Overflow; d.Overflow = false;
	EnableInterrupt(state);
	if (overflow && d.Timer->EvtHandler) d.Timer->EvtHandler(d.Timer, TIMER_EVT_COUNTER_OVR);
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) {
		state = DisableInterrupt();
		if (epoch != d.Epoch || !d.Running || !d.Healthy) { EnableInterrupt(state); return; }
		uint64_t now = Count(d);
		auto &tr = d.Trigger[n];
		if (!d.Healthy || !tr.Active || now < tr.Deadline) { EnableInterrupt(state); continue; }
		TimerTrig_t info = tr.Info;
		if (info.Type == TIMER_TRIG_TYPE_SINGLE) { tr.Active = false; Arm(d, n, false); }
		else {
			// Preserve phase; coalesce late periods into one callback.
			tr.Deadline += ((now - tr.Deadline) / tr.Ticks + 1) * tr.Ticks;
			Arm(d, n, true);
		}
		EnableInterrupt(state);
		if (info.Handler) info.Handler(d.Timer, n, info.pContext);
		else if (d.Timer->EvtHandler) d.Timer->EvtHandler(d.Timer, TIMER_EVT_TRIGGER(n));
	}
}
bool TimerInit(TimerDev_t *t, const TimerCfg_t *cfg)
{
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_CONFIG;
	if (!t || !cfg || (unsigned)cfg->DevNo >= SAM4L_TIMER_MAXCNT || cfg->bTickInt ||
		cfg->IntPrio < 0 || cfg->IntPrio >= (1 << __NVIC_PRIO_BITS)) return false;
	uint32_t state = DisableInterrupt();
	auto &d = g_Sam4lTimerData[cfg->DevNo];
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_OWNER;
	// Reject a handle that is already assigned to a timer.
	for (auto &entry : g_Sam4lTimerData) if (entry.Timer == t) { EnableInterrupt(state); return false; }
	if (d.Timer) { EnableInterrupt(state); return false; }
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_SOURCE;
	if (d.TcReg) {
		if (cfg->ClkSrc != TIMER_CLKSRC_DEFAULT) { EnableInterrupt(state); return false; }
		d.BaseFreq = SystemPeriphClockGet(0);
	} else {
		bool rc = (SAM4L_BPM->BPM_PMCON & BPM_PMCON_CK32S) != 0;
		if ((cfg->ClkSrc != TIMER_CLKSRC_DEFAULT && cfg->ClkSrc != (rc ? TIMER_CLKSRC_LFRC : TIMER_CLKSRC_LFXTAL)) ||
			(cfg->ClkSrc == TIMER_CLKSRC_LFXTAL && GetLowFreqOscType() != OSC_TYPE_XTAL) ||
			!(SAM4L_BSCIF->BSCIF_PCLKSR & (rc ? BSCIF_PCLKSR_RC32KRDY : BSCIF_PCLKSR_OSC32RDY)) ||
			!(rc ? (SAM4L_BSCIF->BSCIF_RC32KCR & BSCIF_RC32KCR_EN32K) : (SAM4L_BSCIF->BSCIF_OSCCTRL32 & BSCIF_OSCCTRL32_EN32K))) {
			EnableInterrupt(state); return false;
		}
		d.BaseFreq = rc ? 32768 : GetLowFreqOscFreq();
	}
	uint32_t freq;
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_FREQUENCY;
	if (!Plan(d, cfg->Freq, d.Select, freq)) { EnableInterrupt(state); return false; }
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_SYNC;
	if (!d.TcReg) {
		Sam4lTimerPmWrite(&SAM4L_PM->PM_PBDMASK, SAM4L_PM->PM_PBDMASK | PM_PBDMASK_AST);
		if (!Sam4lAstWait(AST_SR_BUSY | AST_SR_CLKBUSY)) { EnableInterrupt(state); return false; }
	}
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_IRQ;
	// AST survives every reset except POR (42023H table 10-12). CR.EN
	// alone does not mean another Timer object is using AST. If no object
	// uses it and its NVIC interrupts are disabled, reset its old state.
	// TC is in the core reset domain; a running channel remains a conflict.
	if ((NVIC->ISER[(unsigned)d.Irq / 32] & (1UL << ((unsigned)d.Irq % 32))) ||
		(!d.TcReg &&
		(NVIC->ISER[(unsigned)AST_OVF_IRQn / 32] & (1UL << ((unsigned)AST_OVF_IRQn % 32)))) ||
		(d.TcReg && (SAM4L_PM->PM_PBAMASK & d.ClockMask) && (d.TcReg->TC_SR & TC_SR_CLKSTA))) {
		EnableInterrupt(state); return false;
	}
	d.Timer = t; d.Healthy = true;
	t->DevNo = cfg->DevNo; t->Freq = freq; t->nsPeriod = (1000000000ULL + freq / 2) / freq;
	t->EvtHandler = cfg->EvtHandler;
	t->Disable = Disable; t->Enable = Enable; t->Reset = Reset; t->GetTickCount = GetCount;
	t->SetFrequency = Frequency; t->GetMaxTrigger = MaxTrigger; t->FindAvailTrigger = FindTrigger;
	t->DisableTrigger = TriggerDisable; t->EnableTrigger = TriggerEnable;
	t->DisableExtTrigger = ExtDisable; t->EnableExtTrigger = ExtEnable;
	Irq(d, false);
	g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_SETUP;
	if (d.TcReg) Sam4lTcSetup(d);
	else d.Healthy = Sam4lAstSetup(d);
	bool ok = d.Healthy;
	if (ok) {
		g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_RESET;
		ok = ResetCounter(d);
	}
	NVIC_SetPriority(d.Irq, cfg->IntPrio);
	if (!d.TcReg) NVIC_SetPriority(AST_OVF_IRQn, cfg->IntPrio);
	if (ok) {
		g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_START;
		ok = Run(d, true);
	}
	if (ok) g_Sam4lTimerInitStage = SAM4L_TIMER_INIT_OK;
	if (!ok) { d.Healthy = false; d.Running = false; Irq(d, false); d.Timer = nullptr; }
	EnableInterrupt(state); return ok;
}
int TimerGetLowFreqDevCount(void) { return SAM4L_AST_TIMER_MAXCNT; }
int TimerGetHighFreqDevCount(void) { return SAM4L_TC_TIMER_MAXCNT; }
int TimerGetHighFreqDevNo(void) { return SAM4L_AST_TIMER_MAXCNT; }
