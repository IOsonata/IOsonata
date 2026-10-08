/**-------------------------------------------------------------------------
@file timer_stm32f030x8.cpp
@brief STM32F030x8 peripheral timers. SysTick remains application-owned.

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
#include "stm32f0xx.h"
#include "coredev/timer.h"
#include "coredev/interrupt.h"
#include "coredev/system_core_clock.h"

#if defined(STM32F030x8)
extern "C" {
void TIM6_IRQHandler(void);
void STM32F030_TIM6_IRQHandler(void);
void TIM14_IRQHandler(void);
void STM32F030_TIM14_IRQHandler(void);
void TIM16_IRQHandler(void);
void STM32F030_TIM16_IRQHandler(void);
void TIM17_IRQHandler(void);
void STM32F030_TIM17_IRQHandler(void);
void TIM15_IRQHandler(void);
void STM32F030_TIM15_IRQHandler(void);
void TIM3_IRQHandler(void);
void STM32F030_TIM3_IRQHandler(void);
void TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void STM32F030_TIM1_BRK_UP_TRG_COM_IRQHandler(void);
void TIM1_CC_IRQHandler(void);
void STM32F030_TIM1_CC_IRQHandler(void);
}
struct Stm32f0Trigger {
	TimerTrig_t Info;
	uint64_t Deadline;
	uint32_t Ticks;
	bool Active;
};
struct Stm32f0Timer {
	TIM_TypeDef *Reg;
	IRQn_Type Irq, CompareIrq;
	uint32_t ClockMask;
	bool Apb2;
	int Channels;
	TimerDev_t *Timer;
	Stm32f0Trigger *Trigger;
	uint32_t Epoch;
	uint32_t Cycle;
};
// Virtual order: basic, one-channel, two-channel, four-channel, advanced.
// All use the APB timer clock; none is a low-frequency timer backend.
static Stm32f0Trigger s_Triggers[14];
static Stm32f0Timer s_Devices[] = {
	{TIM6, TIM6_IRQn, TIM6_IRQn, RCC_APB1ENR_TIM6EN, false, 0, nullptr, &s_Triggers[0]},
	{TIM14, TIM14_IRQn, TIM14_IRQn, RCC_APB1ENR_TIM14EN, false, 1, nullptr, &s_Triggers[1]},
	{TIM16, TIM16_IRQn, TIM16_IRQn, RCC_APB2ENR_TIM16EN, true, 1, nullptr, &s_Triggers[2]},
	{TIM17, TIM17_IRQn, TIM17_IRQn, RCC_APB2ENR_TIM17EN, true, 1, nullptr, &s_Triggers[3]},
	{TIM15, TIM15_IRQn, TIM15_IRQn, RCC_APB2ENR_TIM15EN, true, 2, nullptr, &s_Triggers[4]},
	{TIM3, TIM3_IRQn, TIM3_IRQn, RCC_APB1ENR_TIM3EN, false, 4, nullptr, &s_Triggers[6]},
	{TIM1, TIM1_BRK_UP_TRG_COM_IRQn, TIM1_CC_IRQn, RCC_APB2ENR_TIM1EN, true, 4, nullptr, &s_Triggers[10]},
};
static bool OwnHandler(int devno)
{
	using Handler = void (*)(void);
	static const Handler actual[] = {TIM6_IRQHandler,TIM14_IRQHandler,TIM16_IRQHandler,TIM17_IRQHandler,TIM15_IRQHandler,TIM3_IRQHandler,TIM1_BRK_UP_TRG_COM_IRQHandler,TIM1_CC_IRQHandler};
	static const Handler expected[] = {STM32F030_TIM6_IRQHandler,STM32F030_TIM14_IRQHandler,STM32F030_TIM16_IRQHandler,STM32F030_TIM17_IRQHandler,STM32F030_TIM15_IRQHandler,STM32F030_TIM3_IRQHandler,STM32F030_TIM1_BRK_UP_TRG_COM_IRQHandler,STM32F030_TIM1_CC_IRQHandler};
	return actual[devno] == expected[devno] && (devno != 6 || actual[7] == expected[7]);
}
static Stm32f0Timer *Data(TimerDev_t *t)
{
	if (!t || (unsigned)t->DevNo >= 7) return nullptr;
	return s_Devices[t->DevNo].Timer == t ? &s_Devices[t->DevNo] : nullptr;
}
static void IrqEnable(Stm32f0Timer &d, bool enable)
{
	if (enable) { NVIC_EnableIRQ(d.Irq); NVIC_EnableIRQ(d.CompareIrq); }
	else { NVIC_DisableIRQ(d.Irq); NVIC_DisableIRQ(d.CompareIrq); }
}
static int MaxTrigger(TimerDev_t *t);
static void ClearFlags(Stm32f0Timer &d, uint32_t flags)
{
	// RM0360 14.4.5: rc_w0. Preserve flags arriving after the snapshot.
	d.Reg->SR = ~flags & ((1UL << (d.Channels + 1)) - 1);
}
static void Compare(Stm32f0Timer &d, int n, uint32_t value)
{
	switch (n) {
		case 0: d.Reg->CCR1 = value; break;
		case 1: d.Reg->CCR2 = value; break;
		case 2: d.Reg->CCR3 = value; break;
		case 3: d.Reg->CCR4 = value; break;
	}
}
// Caller excludes software writers. A hardware rollover may race the read.
// Interrupts must be serviced at least once per counter cycle.
static uint64_t Count(Stm32f0Timer &d)
{
	uint32_t before = d.Reg->SR & TIM_SR_UIF;
	uint32_t low = d.Reg->CNT & 0xFFFFU;
	uint32_t after = d.Reg->SR & TIM_SR_UIF;
	if (after && !before) low = d.Reg->CNT & 0xFFFFU;
	return d.Timer->Rollover + low + (after ? (uint64_t)d.Cycle : 0);
}
static uint64_t GetCount(TimerDev_t *t)
{
	Stm32f0Timer *data = Data(t); if (!data) return 0;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	uint64_t count = Count(d);
	t->LastCount = (uint32_t)count;
	EnableInterrupt(state);
	return count;
}
static bool Plan(uint32_t request, uint32_t &divider, uint32_t &freq)
{
	uint32_t base = SystemPeriphClockGet(0);
	if ((RCC->CFGR & RCC_CFGR_PPRE_Msk) >= RCC_CFGR_PPRE_DIV2) base *= 2;
	if (!base) return false;
	divider = request ? (uint32_t)(((uint64_t)base + request / 2) / request) : 1;
	if (!divider) divider = 1;
	if (divider > 65536) divider = 65536;
	freq = base / divider;
	return freq != 0;
}
static bool Period(uint32_t freq, uint64_t ns, uint32_t &ticks, uint64_t &actual)
{
	if (!freq || !ns || ns / 1000000000ULL > UINT32_MAX / freq) return false;
	uint64_t value = (ns / 1000000000ULL) * freq +
		((ns % 1000000000ULL) * freq + 500000000ULL) / 1000000000ULL;
	if (value < 4 || value > UINT32_MAX) return false;
	ticks = (uint32_t)value;
	actual = (value * 1000000000ULL + freq / 2) / freq;
	return true;
}
static void Arm(Stm32f0Timer &d, int n)
{
	Stm32f0Trigger &tr = d.Trigger[n];
	if (!d.Channels) return;
	uint32_t bit = TIM_DIER_CC1IE << n;
	d.Reg->DIER &= ~bit;
	ClearFlags(d, bit);
	Compare(d, n, (uint32_t)tr.Deadline & 0xFFFFU);
	d.Reg->DIER |= bit;
	// A short deadline may pass while CCR is being programmed. Service it
	// through the same IRQ path, rather than waiting for another full wrap.
	if ((d.Reg->CR1 & TIM_CR1_CEN) && Count(d) >= tr.Deadline)
		NVIC_SetPendingIRQ(d.Irq);
}
static void BasicCycle(Stm32f0Timer &d, uint32_t cycle)
{
	bool running = d.Reg->CR1 & TIM_CR1_CEN;
	d.Reg->CR1 = TIM_CR1_URS;
	(void)d.Reg->CR1;
	uint64_t count = Count(d);
	d.Cycle = cycle; d.Reg->ARR = cycle - 1;
	d.Reg->EGR = TIM_EGR_UG; d.Reg->CNT = 0; d.Reg->SR = 0;
	d.Timer->Rollover = count;
	NVIC_ClearPendingIRQ(d.Irq);
	if (running) d.Reg->CR1 |= TIM_CR1_CEN;
}
static void Disable(TimerDev_t *t)
{
	Stm32f0Timer *data = Data(t); if (!data) return;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	d.Reg->CR1 &= ~TIM_CR1_CEN;
	(void)d.Reg->CR1;
	IrqEnable(d, false);
	// Preserve counter, compare deadlines and any pending update on pause.
	EnableInterrupt(state);
}
static bool Enable(TimerDev_t *t)
{
	Stm32f0Timer *data = Data(t); if (!data) return false;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	d.Reg->CR1 |= TIM_CR1_CEN;
	IrqEnable(d, true);
	EnableInterrupt(state);
	return true;
}
static void ResetCounter(Stm32f0Timer &d)
{
	d.Reg->CR1 = TIM_CR1_URS;
	if (!d.Channels) {
		d.Cycle = d.Trigger[0].Active ? d.Trigger[0].Ticks : 65536;
		d.Reg->ARR = d.Cycle - 1;
	}
	d.Reg->EGR = TIM_EGR_UG; // Load PSC and reset prescaler/counter (14.4.6).
	d.Reg->CNT = 0;
	d.Reg->SR = 0;
	NVIC_ClearPendingIRQ(d.Irq);
	NVIC_ClearPendingIRQ(d.CompareIrq);
	d.Timer->Rollover = 0;
	d.Timer->LastCount = 0;
	++d.Epoch;
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) if (d.Trigger[n].Active) {
		d.Trigger[n].Deadline = d.Trigger[n].Ticks;
		Arm(d, n);
	}
}
static void Reset(TimerDev_t *t)
{
	Stm32f0Timer *data = Data(t); if (!data) return;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	bool running = d.Reg->CR1 & TIM_CR1_CEN;
	ResetCounter(d);
	if (running) d.Reg->CR1 |= TIM_CR1_CEN;
	EnableInterrupt(state);
}
static uint32_t Frequency(TimerDev_t *t, uint32_t request)
{
	Stm32f0Timer *data = Data(t); if (!data) return 0;
	Stm32f0Timer &d = *data;
	uint32_t divider, freq;
	if (!Plan(request, divider, freq)) return 0;
	uint32_t state = DisableInterrupt();
	uint32_t ticks[4] = {};
	uint64_t actual[4] = {};
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) if (d.Trigger[n].Active &&
		(!Period(freq, d.Trigger[n].Info.nsPeriod, ticks[n], actual[n]) || (!d.Channels && ticks[n] > 65536))) {
		EnableInterrupt(state);
		return 0;
	}
	d.Reg->CR1 = TIM_CR1_URS;
	d.Reg->PSC = divider - 1;
	t->Freq = freq;
	t->nsPeriod = (1000000000ULL + freq / 2) / freq;
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) if (d.Trigger[n].Active) {
		d.Trigger[n].Ticks = ticks[n];
		d.Trigger[n].Info.nsPeriod = actual[n];
	}
	ResetCounter(d);
	d.Reg->CR1 |= TIM_CR1_CEN;
	IrqEnable(d, true);
	EnableInterrupt(state);
	return freq;
}
static int MaxTrigger(TimerDev_t *t) { auto d = Data(t); return d ? (d->Channels ? d->Channels : 1) : 0; }
static int FindTrigger(TimerDev_t *t)
{
	Stm32f0Timer *data = Data(t); if (!data) return -1;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	int result = -1;
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) if (!d.Trigger[n].Active) { result = n; break; }
	EnableInterrupt(state);
	return result;
}
static void TriggerDisable(TimerDev_t *t, int n)
{
	Stm32f0Timer *data = Data(t);
	if (!data || (unsigned)n >= (unsigned)MaxTrigger(t)) return;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	d.Trigger[n].Active = false;
	if (!d.Channels) { BasicCycle(d, 65536); EnableInterrupt(state); return; }
	d.Reg->DIER &= ~(TIM_DIER_CC1IE << n);
	ClearFlags(d, TIM_SR_CC1IF << n);
	EnableInterrupt(state);
}
static uint64_t TriggerEnable(TimerDev_t *t, int n, uint64_t ns, TIMER_TRIG_TYPE type,
	TimerTrigEvtHandler_t handler, void *ctx)
{
	Stm32f0Timer *data = Data(t);
	if (!data || (unsigned)n >= (unsigned)MaxTrigger(t) ||
		(type != TIMER_TRIG_TYPE_SINGLE && type != TIMER_TRIG_TYPE_CONTINUOUS)) return 0;
	Stm32f0Timer &d = *data;
	uint32_t state = DisableInterrupt();
	uint32_t ticks;
	uint64_t actual;
	if ((!Period(t->Freq, ns, ticks, actual) || (!d.Channels && ticks > 65536))) { EnableInterrupt(state); return 0; }
	Stm32f0Trigger &tr = d.Trigger[n];
	tr.Info = { type, actual, handler, ctx };
	tr.Ticks = ticks;
	if (!d.Channels) BasicCycle(d, ticks);
	tr.Deadline = (d.Channels ? Count(d) : d.Timer->Rollover) + ticks;
	tr.Active = true;
	Arm(d, n);
	EnableInterrupt(state);
	return actual;
}
static void ExtDisable(TimerDev_t *) {}
static bool ExtEnable(TimerDev_t *, int, TIMER_EXTTRIG_SENSE) { return false; }

extern "C" void STM32F030TimerIRQHandler(int devno)
{
	if ((unsigned)devno >= 7) return;
	Stm32f0Timer &d = s_Devices[devno];
	if (!d.Timer || !(d.Reg->CR1 & TIM_CR1_CEN)) return;
	uint32_t state = DisableInterrupt();
	uint32_t flags = d.Reg->SR & d.Reg->DIER;
	ClearFlags(d, flags);
	if (flags & TIM_SR_UIF) d.Timer->Rollover += d.Cycle;
	uint32_t epoch = d.Epoch;
	EnableInterrupt(state);
	if ((flags & TIM_SR_UIF) && d.Timer->EvtHandler)
		d.Timer->EvtHandler(d.Timer, TIMER_EVT_COUNTER_OVR);
	for (int n = 0; n < MaxTrigger(d.Timer); ++n) {
		state = DisableInterrupt();
		if (epoch != d.Epoch || !(d.Reg->CR1 & TIM_CR1_CEN)) {
			EnableInterrupt(state); return;
		}
		Stm32f0Trigger &tr = d.Trigger[n];
		uint64_t now = Count(d);
		if (!tr.Active || now < tr.Deadline) { EnableInterrupt(state); continue; }
		TimerTrig_t info = tr.Info;
		if (info.Type == TIMER_TRIG_TYPE_SINGLE) TriggerDisable(d.Timer, n);
		else {
			// Keep phase; coalesce missed periods into one notification.
			tr.Deadline += ((now - tr.Deadline) / tr.Ticks + 1) * tr.Ticks;
			Arm(d, n);
		}
		EnableInterrupt(state);
		if (info.Handler) info.Handler(d.Timer, n, info.pContext);
		else if (d.Timer->EvtHandler) d.Timer->EvtHandler(d.Timer, TIMER_EVT_TRIGGER(n));
	}
}


bool TimerInit(TimerDev_t *t, const TimerCfg_t *cfg)
{
	if (!t || !cfg || (unsigned)cfg->DevNo >= 7 || !OwnHandler(cfg->DevNo) || cfg->ClkSrc != TIMER_CLKSRC_DEFAULT ||
		cfg->bTickInt || cfg->IntPrio < 0 || cfg->IntPrio > 3) return false;
	uint32_t divider, freq;
	if (!Plan(cfg->Freq, divider, freq)) return false;
	uint32_t state = DisableInterrupt();
	Stm32f0Timer &d = s_Devices[cfg->DevNo];
	for (auto &entry : s_Devices) if (entry.Timer == t) { EnableInterrupt(state); return false; }
	volatile uint32_t &en = d.Apb2 ? RCC->APB2ENR : RCC->APB1ENR;
	volatile uint32_t &rst = d.Apb2 ? RCC->APB2RSTR : RCC->APB1RSTR;
	if (d.Timer || (NVIC->ISER[0] & ((1UL << d.Irq) | (1UL << d.CompareIrq))) ||
		((en & d.ClockMask) && (d.Reg->CR1 & TIM_CR1_CEN))) {
		EnableInterrupt(state); return false;
	}
	en |= d.ClockMask;
	uint32_t enabled = en; (void)enabled;
	rst |= d.ClockMask; rst &= ~d.ClockMask;
	d.Reg->CR1 = TIM_CR1_URS;
	if (cfg->DevNo != 1) d.Reg->CR2 = 0;
	d.Reg->DIER = TIM_DIER_UIE;
	if (d.Channels) {
		d.Reg->CCMR1 = 0; d.Reg->CCER = 0;
		if (d.Channels == 4) d.Reg->CCMR2 = 0;
		if (cfg->DevNo >= 4) d.Reg->SMCR = 0;
		if (d.Apb2) { d.Reg->RCR = 0; d.Reg->BDTR = 0; }
	}
	d.Reg->PSC = divider - 1; d.Cycle = 65536; d.Reg->ARR = d.Cycle - 1;
	d.Timer = t;
	t->DevNo = cfg->DevNo; t->Freq = freq; t->nsPeriod = (1000000000ULL + freq / 2) / freq;
	t->EvtHandler = cfg->EvtHandler;
	t->Disable = Disable; t->Enable = Enable; t->Reset = Reset;
	t->GetTickCount = GetCount; t->SetFrequency = Frequency;
	t->GetMaxTrigger = MaxTrigger; t->FindAvailTrigger = FindTrigger;
	t->DisableTrigger = TriggerDisable; t->EnableTrigger = TriggerEnable;
	t->DisableExtTrigger = ExtDisable; t->EnableExtTrigger = ExtEnable;
	ResetCounter(d);
	NVIC_SetPriority(d.Irq, cfg->IntPrio);
	NVIC_SetPriority(d.CompareIrq, cfg->IntPrio);
	IrqEnable(d, true);
	d.Reg->CR1 |= TIM_CR1_CEN;
	EnableInterrupt(state);
	return true;
}
int TimerGetLowFreqDevCount(void) { return 0; }
int TimerGetHighFreqDevCount(void) { return 7; }
int TimerGetHighFreqDevNo(void) { return 0; }
#endif
