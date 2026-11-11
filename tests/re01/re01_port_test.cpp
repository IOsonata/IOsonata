// Register-level regression checks; this is not a hardware peripheral emulator.
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <cstdio>
#include <cstring>
#include <sys/mman.h>
#include "re01xxx.h"
#include "interrupt_re01.h"
#include "timer_re01.h"
#include "coredev/iopincfg.h"
#include "coredev/uart.h"
#include "coredev/i2c.h"
#include "coredev/spi.h"

extern "C" void IEL0_IRQHandler();
extern "C" void IEL1_IRQHandler();
extern "C" void IEL2_IRQHandler();
extern "C" void IEL3_IRQHandler();
extern "C" void IEL4_IRQHandler();
extern "C" void IEL5_IRQHandler();
extern "C" void IEL6_IRQHandler();
extern "C" void IEL7_IRQHandler();
extern "C" void IEL8_IRQHandler();
extern "C" void IEL9_IRQHandler();
extern "C" void IEL10_IRQHandler();
extern "C" void IEL11_IRQHandler();
extern "C" void IEL12_IRQHandler();
extern "C" void IEL13_IRQHandler();
extern "C" void IEL14_IRQHandler();
extern "C" void IEL15_IRQHandler();
extern "C" void IEL16_IRQHandler();
extern "C" void IEL17_IRQHandler();
extern "C" void IEL18_IRQHandler();
extern "C" void IEL19_IRQHandler();
extern "C" void IEL20_IRQHandler();
extern "C" void IEL21_IRQHandler();
extern "C" void IEL22_IRQHandler();
extern "C" void IEL23_IRQHandler();
extern "C" void IEL24_IRQHandler();
extern "C" void IEL25_IRQHandler();
extern "C" void IEL26_IRQHandler();
extern "C" void IEL27_IRQHandler();
extern "C" void IEL28_IRQHandler();
extern "C" void IEL29_IRQHandler();
extern "C" void IEL30_IRQHandler();
extern "C" void IEL31_IRQHandler();
static void (* const handlers[])() = {
	IEL0_IRQHandler,
	IEL1_IRQHandler,
	IEL2_IRQHandler,
	IEL3_IRQHandler,
	IEL4_IRQHandler,
	IEL5_IRQHandler,
	IEL6_IRQHandler,
	IEL7_IRQHandler,
	IEL8_IRQHandler,
	IEL9_IRQHandler,
	IEL10_IRQHandler,
	IEL11_IRQHandler,
	IEL12_IRQHandler,
	IEL13_IRQHandler,
	IEL14_IRQHandler,
	IEL15_IRQHandler,
	IEL16_IRQHandler,
	IEL17_IRQHandler,
	IEL18_IRQHandler,
	IEL19_IRQHandler,
	IEL20_IRQHandler,
	IEL21_IRQHandler,
	IEL22_IRQHandler,
	IEL23_IRQHandler,
	IEL24_IRQHandler,
	IEL25_IRQHandler,
	IEL26_IRQHandler,
	IEL27_IRQHandler,
	IEL28_IRQHandler,
	IEL29_IRQHandler,
	IEL30_IRQHandler,
	IEL31_IRQHandler,
};

static volatile uint32_t * const routes = &ICU->IELSR0;
static uint32_t backup[32];
static void irq(int Group, int Value)
{
	for (int i = Group; i < 32; i += 8)
	{
		if ((routes[i] & ICU_IELSR0_IELS_Msk) == (uint32_t)Value)
		{
			routes[i] |= ICU_IELSR0_IR_Msk;
			handlers[i]();
			assert((routes[i] & ICU_IELSR0_IR_Msk) == 0);
			return;
		}
	}
	assert(false && "expected event is not routed");
}

static void exhaust(int Group = -1)
{
	for (int i = 0; i < 32; i++)
	{
		backup[i] = routes[i];
		if ((Group < 0 || (i & 7) == Group) && routes[i] == 0)
			routes[i] = ICU_IELSR0_IELS_Msk;
	}
}
static void restore()
{
	for (int i = 0; i < 32; i++)
		routes[i] = backup[i];
}
static void release()
{
	for (int i = 0; i < 32; i++)
		Re01UnregisterIntHandler((IRQn_Type)i);
}
static int countRoutes()
{
	int n = 0;
	for (int i = 0; i < 32; i++) n += routes[i] != 0;
	return n;
}
static void hook(int, void *Ctx) { (*(int*)Ctx)++; }

// An event while the handler runs, at its first call. Ack: the handler clears
// the flag first.
struct RaiseCtx { int Count; bool Ack; };
static void raiseHook(int IntNo, void *Ctx)
{
	RaiseCtx *c = (RaiseCtx*)Ctx;

	if (c->Ack)
	{
		Re01AcknowledgeInt((IRQn_Type)IntNo);
	}
	if (c->Count++ == 0)
	{
		routes[IntNo] |= ICU_IELSR0_IR_Msk;
	}
}

// Edge on interrupt pin IRQ8 while the callback runs, at its first call
static int s_PinEdge;
static void pinEdgeHook(int, void *)
{
	if (s_PinEdge++ == 0)
	{
		for (int i = 0; i < 32; i += 8)
		{
			if ((routes[i] & ICU_IELSR0_IELS_Msk) == 0x1E)
			{
				routes[i] |= ICU_IELSR0_IR_Msk;
			}
		}
	}
}

static void clocks()
{
	SYSTEM->SCKSCR = 0;
	SYSTEM->HOCOMCR = 3;
	SYSTEM->SCKDIVCR = (2UL << SYSTEM_SCKDIVCR_ICK_Pos) | (3UL << SYSTEM_SCKDIVCR_PCKB_Pos);
	FLASH->FLWT = 0x5A;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 16000000 && SystemPeriphClockGet(1) == 8000000);
	assert(FLASH->FLWT == 0x5A);
	assert(SYSTEM->SCKDIVCR == ((2UL << SYSTEM_SCKDIVCR_ICK_Pos) | (3UL << SYSTEM_SCKDIVCR_PCKB_Pos)));
	SYSTEM->PRCR = 0xA502;
	assert(SystemPeriphClockSet(0, 64000000) == 64000000);
	assert(SystemPeriphClockSet(1, 64000000) == 32000000);
	assert((SYSTEM->PRCR & 0xF) == 2);
	assert(SystemPeriphClockSet(0, 16000000) == 16000000);
	assert(SystemPeriphClockGet(1) <= 16000000);
	uint32_t div = SYSTEM->SCKDIVCR;
	assert(SystemPeriphClockSet(-1, 1000) == 0);
	assert(SystemPeriphClockSet(2, 1000) == 0);
	assert(SystemPeriphClockSet(0, 0) == 0);
	assert(SystemPeriphClockSet(1, 1) == 0);
	assert(SYSTEM->SCKDIVCR == div);
	SYSTEM->SCKSCR = 0;
	SYSTEM->HOCOMCR = 2;
	SYSTEM->SCKDIVCR = 1UL << SYSTEM_SCKDIVCR_PCKB_Pos;
	SystemCoreClockUpdate();
	assert(SystemCoreClock == 48000000 && SystemPeriphClockGet(1) == 24000000);
}

static void icuAndGpio()
{
	int a = 0, b = 0;
	assert(Re01RegisterIntHandler(0xA5, 1, hook, &a) == -1);
	assert(Re01RegisterIntHandler(0xFF, 1, hook, &a) == -1);
	assert(Re01RegisterIntHandler(RE01_EVTID_PORT_IRQ0, 1, nullptr, &a) == -1);
	IRQn_Type n = Re01RegisterIntHandler(RE01_EVTID_PORT_IRQ0, 1, hook, &a);
	assert(n >= 0);
	routes[n] |= ICU_IELSR0_IR_Msk;
	assert(Re01RegisterIntHandler(RE01_EVTID_PORT_IRQ0, 1, hook, &a) == n);
	assert(Re01RegisterIntHandler(RE01_EVTID_PORT_IRQ0, 1, hook, &b) == -1);
	handlers[n](); assert(a == 1 && b == 0);
	Re01UnregisterIntHandler(n);
	n = Re01RegisterIntHandler(RE01_EVTID_PORT_IRQ8, 1, hook, &a);
	assert((n & 7) == 0 && (routes[n] & ICU_IELSR0_IELS_Msk) == 0x1E);
	Re01UnregisterIntHandler(n);
	n = Re01RegisterIntHandler(RE01_EVTID_PORT_IRQ9, 1, hook, &a);
	assert((n & 7) == 2 && (routes[n] & ICU_IELSR0_IELS_Msk) == 0x1F);
	Re01UnregisterIntHandler(n);

	// The flag is cleared after the handler returns, an event while it runs
	// with it. A handler that clears it first keeps that event: called again.
	RaiseCtx lost = {0, false}, kept = {0, true};
	n = Re01RegisterIntHandler(RE01_EVTID_DMAC0_INT, 1, raiseHook, &lost);
	assert(n >= 0);
	routes[n] |= ICU_IELSR0_IR_Msk;
	handlers[n](); assert(lost.Count == 1 && (routes[n] & ICU_IELSR0_IR_Msk) == 0);
	Re01UnregisterIntHandler(n);
	n = Re01RegisterIntHandler(RE01_EVTID_DMAC0_INT, 1, raiseHook, &kept);
	assert(n >= 0);
	routes[n] |= ICU_IELSR0_IR_Msk;
	handlers[n](); assert(kept.Count == 1 && (routes[n] & ICU_IELSR0_IR_Msk));
	handlers[n](); assert(kept.Count == 2 && (routes[n] & ICU_IELSR0_IR_Msk) == 0);
	handlers[n](); assert(kept.Count == 2); // no flag, no call
	assert(!Re01AcknowledgeInt((IRQn_Type)32) && !Re01AcknowledgeInt((IRQn_Type)-1));
	Re01UnregisterIntHandler(n);

	uint32_t p0 = *(volatile uint32_t*)PFS_BASE;
	IOPinDisableInterrupt(0); // never allocated: must not touch P0.0 or IEL0
	assert(*(volatile uint32_t*)PFS_BASE == p0);
	IOPinConfig(9, 0, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(0, 16, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinConfig(-2, 0, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	IOPinDisable(0, -2);
	SYSTEM->PRCR = 0xA500;
	IOPinConfig(0, 9, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	assert(SYSTEM->PRCR == 0xA500);
	IOPinConfig(2, 8, IOPINOP_GPIO, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	assert(SYSTEM->PRCR == 0xA500);
	IOPinConfig(0, 9, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_OPENDRAIN);
	assert((*(volatile uint32_t*)(PFS_BASE + 9 * 4) & 0xF0U) == 0x40U);
	IOPinConfig(0, 9, IOPINOP_FUNC15, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN);
	assert((*(volatile uint32_t*)(PFS_BASE + 9 * 4) & 0xF0U) == 0x40U);
	IOPinConfig(0, 9, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	assert((*(volatile uint32_t*)(PFS_BASE + 9 * 4) & 0xC0U) == 0);
	PORT0->PODR |= 1U << 9;
	IOPinConfig(0, 9, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
	assert(*(volatile uint32_t*)(PFS_BASE + 9 * 4) & 1U);
	PORT0->PODR &= ~(1U << 9);
	IOPinConfig(0, 9, IOPINOP_FUNC15, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN);
	assert((*(volatile uint32_t*)(PFS_BASE + 9 * 4) & 1U) == 0);
	assert(!IOPinEnableInterrupt(0, 1, 9, 0, IOPINSENSE_TOGGLE, hook, &a));
	assert(!IOPinEnableInterrupt(0, 1, 0, 0, IOPINSENSE_TOGGLE, hook, &a));
	assert(IOPinEnableInterrupt(8, 1, 1, 5, IOPINSENSE_TOGGLE, hook, &a));
	irq(0, 0x1E); assert(a == 2);
	assert(!IOPinEnableInterrupt(8, 1, 2, 5, IOPINSENSE_TOGGLE, hook, &b));
	IOPinDisableInterrupt(8);
	assert((ICU->WUPEN & (1UL << 8)) == 0);
	assert(IOPinEnableInterrupt(9, 1, 1, 4, IOPINSENSE_HIGH_TRANSITION, hook, &a));
	irq(2, 0x1F); assert(a == 3);
	IOPinDisableInterrupt(9);
	assert((ICU->WUPEN & (1UL << 9)) == 0);
	// An edge while the callback runs calls it again
	assert(IOPinEnableInterrupt(8, 1, 1, 5, IOPINSENSE_TOGGLE, pinEdgeHook, nullptr));
	for (int i = 0; i < 32; i += 8)
	{
		if ((routes[i] & ICU_IELSR0_IELS_Msk) == 0x1E)
		{
			routes[i] |= ICU_IELSR0_IR_Msk;
			handlers[i](); assert(s_PinEdge == 1 && (routes[i] & ICU_IELSR0_IR_Msk));
			handlers[i](); assert(s_PinEdge == 2 && (routes[i] & ICU_IELSR0_IR_Msk) == 0);
		}
	}
	assert(s_PinEdge == 2);
	IOPinDisableInterrupt(8);
	exhaust();
	uint32_t pfs = *(volatile uint32_t*)(PFS_BASE + ((1 * 16 + 6) * 4));
	uint32_t wake = ICU->WUPEN;
	assert(!IOPinEnableInterrupt(0, 1, 1, 6, IOPINSENSE_TOGGLE, hook, &a));
	assert(*(volatile uint32_t*)(PFS_BASE + ((1 * 16 + 6) * 4)) == pfs && ICU->WUPEN == wake);
	restore();
	assert(countRoutes() == 0);
}

static uint32_t timerEvents[RE01_TIMER_MAXCNT];
static uint64_t overflowReads[RE01_TIMER_MAXCNT];
static void timerEvent(TimerDev_t *Timer, uint32_t Event)
{
	timerEvents[Timer->DevNo] |= Event;
	if (Event == TIMER_EVT_COUNTER_OVR)
		overflowReads[Timer->DevNo] = Timer->GetTickCount(Timer);
}

static void timerIrq(IRQn_Type Irq)
{
	assert(Irq >= IEL0_IRQn && Irq <= IEL31_IRQn);
	assert(routes[Irq - IEL0_IRQn] & ICU_IELSR0_IELS_Msk);
	routes[Irq - IEL0_IRQn] |= ICU_IELSR0_IR_Msk;
	handlers[Irq - IEL0_IRQn]();
	assert((routes[Irq - IEL0_IRQn] & ICU_IELSR0_IR_Msk) == 0);
}

static void triggerCount(TimerDev_t *, int, void *Context)
{
	(*(unsigned*)Context)++;
}

static void triggerRearm(TimerDev_t *Timer, int TrigNo, void *Context)
{
	(*(unsigned*)Context)++;
	assert(Timer->FindAvailTrigger(Timer) == TrigNo);
	assert(Timer->EnableTrigger(Timer, TrigNo, 10000, TIMER_TRIG_TYPE_SINGLE,
		triggerCount, Context) != 0);
}

static void triggerFailReinit(TimerDev_t *Timer, int, void *Context)
{
	AGT0->AGTCR |= AGT0_AGTCR_TCSTF_Msk;
	assert(!TimerInit(Timer, (const TimerCfg_t*)Context));
}

static void timers()
{
	assert(TimerGetLowFreqDevCount() == 2 && TimerGetHighFreqDevCount() == 7);
	assert(TimerGetHighFreqDevNo() == 2 && RE01_TIMER_MAXCNT == 9);
	static TimerDev_t t = {};
	TimerCfg_t cfg = {.DevNo = 2, .ClkSrc = TIMER_CLKSRC_DEFAULT, .Freq = 64000000,
		.IntPrio = 1, .EvtHandler = timerEvent};
	exhaust();
	assert(!TimerInit(&t, &cfg) && TMR1->TCCR == 0);
	restore();
	assert(TimerInit(&t, &cfg));
	assert(t.Freq == 24000000);
	assert(t.SetFrequency(&t, 100000000) == 24000000 && t.Freq == 24000000);
	assert(t.GetMaxTrigger(&t) == 2);
	TimerDev_t other = {};
	assert(!TimerInit(&other, &cfg));
	TimerCfg_t moved = cfg; moved.DevNo = 8;
	assert(!TimerInit(&t, &moved) && t.DevNo == 2);
	TimerCfg_t tick = cfg; tick.bTickInt = true;
	assert(!TimerInit(&t, &tick) && t.Freq == 24000000);
	uint8_t clk = TMR1->TCCR;
	t.Disable(&t); assert(TMR1->TCCR == 0);
	assert(t.Enable(&t) && TMR1->TCCR == clk);
	assert(t.FindAvailTrigger(&t) == 0);
	assert(t.EnableTrigger(&t, 0, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 10000);
	assert(t.FindAvailTrigger(&t) == 1);
	assert(t.EnableTrigger(&t, 1, 20000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 20000);
	assert(t.FindAvailTrigger(&t) == -1);
	// The counter at the deadline of single A: called and released
	TMR01->TCNT = g_Re01TimerData[2].Deadline[0];
	irq(2, 0x14);
	assert(t.FindAvailTrigger(&t) == 0);
	assert(countRoutes() == 2); // overflow and continuous B; single A was released
	assert(t.EnableTrigger(&t, 0, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 10000);
	TMR01->TCNT = 123;
	t.Rollover = 0x10000;
	t.Reset(&t);
	assert(TMR01->TCNT == 0 && t.Rollover == 0);
	assert(t.SetFrequency(&t, 12000000) == 12000000);
	assert(TMR01->TCORA == 120 && TMR01->TCORB == 240);
	TMR01->TCNT = 257; timerIrq(g_Re01TimerData[2].IrqMatch[1]);
	assert(TMR01->TCORB == 480); // original phase, independent of ISR delay
	TMR01->TCNT = 1021; timerIrq(g_Re01TimerData[2].IrqMatch[1]);
	assert(TMR01->TCORB == 1200); // skip missed periods without a catch-up loop
	TMR01->TCNT = 0xFFFC; assert(t.GetTickCount(&t) == 0xFFFC);
	TMR01->TCNT = 4; routes[g_Re01TimerData[2].IrqOvr] |= ICU_IELSR0_IR_Msk;
	assert(t.GetTickCount(&t) == 0x10004 && t.Rollover == 0);
	timerIrq(g_Re01TimerData[2].IrqOvr);
	assert(overflowReads[2] == 0x10004 && t.GetTickCount(&t) == 0x10004);
	t.Disable(&t); t.Reset(&t);
	assert(TMR1->TCCR == 0 && t.GetTickCount(&t) == 0);
	assert(t.Enable(&t));
	// Active periods take the closest one at the lowest rate
	uint32_t low = t.SetFrequency(&t, 1);
	assert(low != 0 && low < 12000000 && t.Freq == low);
	assert(t.SetFrequency(&t, 12000000) == 12000000);
	assert(t.SetFrequency(&t, 65000000) == 24000000 && t.Freq == 24000000);
	assert(t.SetFrequency(&t, 12000000) == 12000000);
	assert(t.EnableTrigger(&t, 0, UINT64_MAX, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) ==
		TimerTickToTime(&t, 0xFFFF, 1000000000UL) && g_Re01TimerData[2].CC[0] == 0xFFFF);
	assert(t.EnableTrigger(&t, 0, 0, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
	assert(t.EnableTrigger(&t, 0, 10000, (TIMER_TRIG_TYPE)2, nullptr, nullptr) == 0);
	t.DisableTrigger(&t, 0); t.DisableTrigger(&t, 1);
	unsigned rearmed = 0;
	assert(t.EnableTrigger(&t, 0, 10000, TIMER_TRIG_TYPE_SINGLE, triggerRearm, &rearmed));
	// A compare interrupt before the deadline is not a call
	timerIrq(g_Re01TimerData[2].IrqMatch[0]); assert(rearmed == 0);
	TMR01->TCNT = g_Re01TimerData[2].Deadline[0];
	timerIrq(g_Re01TimerData[2].IrqMatch[0]); assert(rearmed == 1);
	assert(TMR01->TCORA == ((TMR01->TCNT + 120) & 0xFFFF));
	TMR01->TCNT = g_Re01TimerData[2].Deadline[0];
	timerIrq(g_Re01TimerData[2].IrqMatch[0]); assert(rearmed == 2 && countRoutes() == 1);
	assert(!t.EnableExtTrigger(&t, 0, TIMER_EXTTRIG_SENSE_TOGGLE));
	exhaust();
	uint16_t cmp = TMR01->TCORA;
	assert(t.EnableTrigger(&t, 0, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
	assert(TMR01->TCORA == cmp && t.FindAvailTrigger(&t) == 0);
	restore();
	assert(TimerInit(&t, &cfg)); // reinitialization releases compare routes
	assert(countRoutes() == 1);
	t.Disable(&t);

	static TimerDev_t a = {};
	cfg.DevNo = 0; cfg.ClkSrc = TIMER_CLKSRC_LFRC; cfg.Freq = 1000000;
	exhaust();
	assert(!TimerInit(&a, &cfg) && (AGT0->AGTCR & AGT0_AGTCR_TSTART_Msk) == 0);
	restore();
	assert(TimerInit(&a, &cfg) && a.Freq == 32768);
	assert(AGT0->AGTMR2_b.CKS == 0);
	// Above the range: the closest, the highest
	assert(a.SetFrequency(&a, UINT32_MAX) == 32768 && AGT0->AGTMR2_b.CKS == 0);
	cfg.Freq = 100000000;
	assert(TimerInit(&a, &cfg) && a.Freq == 32768);
	cfg.Freq = 1000000;
	assert(a.SetFrequency(&a, 1) == 256 && AGT0->AGTMR2_b.CKS == 7);
	AGT0->AGT = 123; a.Rollover = 0x10000;
	a.Reset(&a);
	assert(AGT0->AGT == 0xFFFF && a.Rollover == 0 && a.GetTickCount(&a) == 0);
	cfg.ClkSrc = TIMER_CLKSRC_HFRC; cfg.Freq = 24000000;
	assert(TimerInit(&a, &cfg));
	assert(AGT0->AGTMR1_b.TCK == 0 && a.Freq == 24000000);
	assert(a.EnableTrigger(&a, 0, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 10000);
	assert(a.FindAvailTrigger(&a) == 1);
	assert(a.EnableTrigger(&a, 1, 20000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr) == 20000);
	assert(a.FindAvailTrigger(&a) == -1);
	AGT0->AGTCR |= AGT0_AGTCR_TCMAF_Msk;
	irq(2, 4); assert(a.FindAvailTrigger(&a) == 0);
	assert(countRoutes() == 3); // TMR overflow, AGT overflow and continuous B
	assert(a.SetFrequency(&a, 12000000) == 12000000);
	assert(AGT0->AGTCMB == 0xFFFF - 240);
	// The counts the counter misses while stopped for the compare write: added
	// to the tick count, the compare moved earlier by as many
	uint32_t lost = g_Re01TimerData[0].Lost;
	assert(lost == RE01_TIMER_AGT_RESTART +
		(RE01_TIMER_AGT_STOP_CYCLES * 12000000ULL + SystemCoreClock / 2) / SystemCoreClock);
	AGT0->AGT = 0xFFFF - 257; AGT0->AGTCR |= AGT0_AGTCR_TCMBF_Msk;
	timerIrq(g_Re01TimerData[0].IrqMatch[1]); assert(AGT0->AGTCMB == 0xFFFF - 480 + lost);
	assert((AGT0->AGTCR & AGT0_AGTCR_TSTART_Msk) && a.Rollover == lost);
	AGT0->AGT = 3; assert(a.GetTickCount(&a) == 0xFFFC + lost);
	AGT0->AGT = 0xFFFF - 4; AGT0->AGTCR |= AGT0_AGTCR_TUNDF_Msk;
	assert(a.GetTickCount(&a) == 0x10004 + lost && a.Rollover == lost);
	timerIrq(g_Re01TimerData[0].IrqOvr);
	assert(overflowReads[0] == 0x10004 + lost && a.GetTickCount(&a) == 0x10004 + lost);
	// Next deadline 720 (480 + 240), reached once the counts missed are added:
	// the next count, the deadline kept
	uint16_t at = 720 - 2 * lost + 1;
	assert(lost < 120);
	AGT0->AGT = 0xFFFF - at; AGT0->AGTCR |= AGT0_AGTCR_TCMBF_Msk;
	timerIrq(g_Re01TimerData[0].IrqMatch[1]);
	assert(AGT0->AGTCMB == 0xFFFF - (at + 1) && a.Rollover == 0x10000 + 2 * lost);
	assert(g_Re01TimerData[0].Deadline[1] == 720);
	a.Disable(&a); a.Reset(&a);
	assert((AGT0->AGTCR & AGT0_AGTCR_TSTART_Msk) == 0 && a.GetTickCount(&a) == 0);
	assert(a.Enable(&a));
	unsigned simultaneous[2] = {};
	assert(a.EnableTrigger(&a, 0, 10000, TIMER_TRIG_TYPE_SINGLE, triggerCount, &simultaneous[0]));
	assert(a.EnableTrigger(&a, 1, 10000, TIMER_TRIG_TYPE_SINGLE, triggerCount, &simultaneous[1]));
	AGT0->AGTCR |= AGT0_AGTCR_TCMAF_Msk | AGT0_AGTCR_TCMBF_Msk;
	timerIrq(g_Re01TimerData[0].IrqMatch[0]);
	assert(simultaneous[0] == 1 && simultaneous[1] == 1 && countRoutes() == 2);
	timerEvents[0] &= ~TIMER_EVT_TRIGGER(1);
	assert(a.EnableTrigger(&a, 0, 10000, TIMER_TRIG_TYPE_SINGLE, triggerFailReinit, &cfg));
	assert(a.EnableTrigger(&a, 1, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr));
	AGT0->AGTCR |= AGT0_AGTCR_TCMAF_Msk | AGT0_AGTCR_TCMBF_Msk;
	timerIrq(g_Re01TimerData[0].IrqMatch[0]);
	assert(a.Freq == 0 && !a.Enable(&a) && (timerEvents[0] & TIMER_EVT_TRIGGER(1)));
	AGT0->AGTCR &= ~AGT0_AGTCR_TCSTF_Msk;
	assert(TimerInit(&a, &cfg)); // the second callback survived first-callback failure
	assert(a.EnableTrigger(&a, 0, 10000, TIMER_TRIG_TYPE_SINGLE, triggerRearm, &rearmed));
	AGT0->AGTCR |= AGT0_AGTCR_TCMAF_Msk; timerIrq(g_Re01TimerData[0].IrqMatch[0]);
	assert(rearmed == 3 && a.FindAvailTrigger(&a) == 1);
	AGT0->AGTCR |= AGT0_AGTCR_TCMAF_Msk; timerIrq(g_Re01TimerData[0].IrqMatch[0]);
	assert(rearmed == 4 && a.FindAvailTrigger(&a) == 0);
	assert(a.EnableTrigger(&a, 0, UINT64_MAX, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) ==
		TimerTickToTime(&a, 0xFFFF, 1000000000UL) && g_Re01TimerData[0].CC[0] == 0xFFFF);
	assert(a.EnableTrigger(&a, 0, 0, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
	assert(a.EnableTrigger(&a, 0, 10000, (TIMER_TRIG_TYPE)2, nullptr, nullptr) == 0);
	assert(a.EnableTrigger(&a, 0, 10000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
	AGT0->AGTCR |= AGT0_AGTCR_TCSTF_Msk | AGT0_AGTCR_TCMAF_Msk;
	timerIrq(g_Re01TimerData[0].IrqMatch[0]); // bounded failed stop releases compare IRQ
	assert(countRoutes() == 2 && a.FindAvailTrigger(&a) == 0);
	AGT0->AGTCR &= ~AGT0_AGTCR_TCSTF_Msk;
	a.DisableTrigger(&a, 0); a.DisableTrigger(&a, 1);
	a.Disable(&a);

	static TimerDev_t b = {};
	cfg.DevNo = 1;
	assert(TimerInit(&b, &cfg));
	assert(b.GetMaxTrigger(&b) == 1);
	assert(b.EnableTrigger(&b, 1, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
	assert(b.EnableTrigger(&b, 0, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 10000);
	assert(b.FindAvailTrigger(&b) == -1);
	b.DisableTrigger(&b, 0); b.Disable(&b);
	b.Reset(&b);
	assert((AGT1->AGTCR & AGT0_AGTCR_TSTART_Msk) == 0 && b.GetTickCount(&b) == 0);
	AGT1->AGT = 0xFFFF - 7; AGT1->AGTCR |= AGT0_AGTCR_TUNDF_Msk;
	assert(b.GetTickCount(&b) == 0x10007);
	timerIrq(g_Re01TimerData[1].IrqOvr);
	assert(overflowReads[1] == 0x10007 && b.GetTickCount(&b) == 0x10007);
	cfg.ClkSrc = TIMER_CLKSRC_EXT;
	assert(!TimerInit(&b, &cfg));
	release();
}

static void gptTimers()
{
	static TimerDev_t timers[RE01_TIMER_GPT_CNT] = {};
	// Independent expected ICU group/value choices, including the alternate
	// group for GPT0/2. Tests dispatch through the real generic IRQ handlers.
	static const uint8_t groups[6][5] = {
		{0, 1, 2, 3, 0}, {0, 1, 2, 3, 4}, {0, 1, 2, 3, 2},
		{4, 5, 6, 7, 0}, {0, 1, 2, 3, 6}, {4, 5, 6, 7, 2},
	};
	static const uint8_t values[6][5] = {
		{0xC, 0xD, 0xC, 0xD, 0xD}, {0x18, 0x17, 0x15, 0x17, 0x17},
		{0xE, 0xF, 0xD, 0xE, 0xE}, {0x18, 0x17, 0x16, 0x16, 0x19},
		{0x1A, 0x19, 0x16, 0x18, 0x17}, {0x19, 0x18, 0x18, 0x18, 0x17},
	};
	MSTP->MSTPCRD |= MSTP_MSTPCRD_MSTPD5_Msk | MSTP_MSTPCRD_MSTPD6_Msk;
	for (int channel = 0; channel < RE01_TIMER_GPT_CNT; channel++)
	{
		TimerDev_t &timer = timers[channel];
		int no = RE01_TIMER_GPT_DEVNO + channel;
		TimerCfg_t cfg = {.DevNo = no, .ClkSrc = TIMER_CLKSRC_DEFAULT, .Freq = 0,
			.IntPrio = 1, .EvtHandler = timerEvent};
		RE01_TimerData_t &dev = g_Re01TimerData[no];
		GPT320_Type *reg = dev.pGptReg;
		if (channel == 0)
		{
			exhaust();
			assert(!TimerInit(&timer, &cfg) && !(reg->GTCR & GPT320_GTCR_CST_Msk));
			assert(timer.Freq == 0 && !timer.Enable(&timer) && timer.GetTickCount(&timer) == 0);
			restore();
		}
		assert(TimerInit(&timer, &cfg));
		assert(timer.Freq == 48000000 && timer.Freq == SystemPeriphClockGet(0));
		assert(timer.SetFrequency(&timer, UINT32_MAX) == 48000000 && timer.Freq == 48000000);
		assert(timer.Freq != SystemPeriphClockGet(1));
		assert(timer.GetMaxTrigger(&timer) == 4 && timer.FindAvailTrigger(&timer) == 0);
		uint32_t mask = channel < 2 ? UINT32_MAX : 0xFFFFUL;
		assert(reg->GTPR == mask && reg->GTBER == 3 && reg->GTINTAD == 0);
		assert(reg->GTUDDTYC == 1 && reg->GTWP == 0xA500);
		assert(reg->GTIOR == 0 && reg->GTICASR == 0 && reg->GTICBSR == 0);
		assert(((unsigned)dev.IrqOvr & 7) == groups[channel][4] ||
			((channel == 0 || channel == 2) && ((unsigned)dev.IrqOvr & 7) == groups[channel][4] + 4U));
		assert((routes[dev.IrqOvr] & ICU_IELSR0_IELS_Msk) == values[channel][4]);
		assert(!(MSTP->MSTPCRD & (channel < 2 ? MSTP_MSTPCRD_MSTPD5_Msk : MSTP_MSTPCRD_MSTPD6_Msk)));
		for (uint32_t code = 0; code <= 5; code++)
		{
			uint32_t rate = 48000000UL >> (2 * code);
			assert(timer.SetFrequency(&timer, rate) == rate && reg->GTCR_b.TPCS == code);
		}
		assert(timer.SetFrequency(&timer, 0) == 48000000);
		assert(timer.SetFrequency(&timer, 65000000) == 48000000 && timer.Freq == 48000000);
		assert(timer.SetFrequency(&timer, 3000000) == 3000000);
		unsigned hits[4] = {};
		reg->GTCCRE = 0xA5A5; reg->GTCCRF = 0x5A5A;
		volatile uint32_t *cmp[] = {&reg->GTCCRA, &reg->GTCCRB, &reg->GTCCRC, &reg->GTCCRD};
		for (int i = 0; i < 4; i++)
		{
			assert(timer.EnableTrigger(&timer, i, 10000ULL * (i + 1), TIMER_TRIG_TYPE_SINGLE,
				i == 2 ? nullptr : triggerCount, &hits[i]) == 10000ULL * (i + 1));
			assert(*cmp[i] == 30U * (i + 1));
			unsigned group = (unsigned)dev.IrqMatch[i] & 7;
			assert(group == groups[channel][i] || ((channel == 0 || channel == 2) && group == groups[channel][i] + 4U));
			assert((routes[dev.IrqMatch[i]] & ICU_IELSR0_IELS_Msk) == values[channel][i]);
		}
		assert(timer.FindAvailTrigger(&timer) == -1 && reg->GTCCRE == 0xA5A5 && reg->GTCCRF == 0x5A5A);
		for (int i = 0; i < 4; i++)
		{
			int before = countRoutes();
			reg->GTCNT = *cmp[i]; reg->GTST |= 1UL << i;
			timerEvents[no] &= ~TIMER_EVT_TRIGGER(i);
			timerIrq(dev.IrqMatch[i]);
			assert(countRoutes() == before - 1 && dev.IrqMatch[i] == (IRQn_Type)-1);
			// Event handler after the trigger handler too
			assert((timerEvents[no] & TIMER_EVT_TRIGGER(i)) != 0 && (i == 2 || hits[i] == 1));
		}
		assert(timer.FindAvailTrigger(&timer) == 0);
		unsigned rearmed = 0;
		assert(timer.EnableTrigger(&timer, 0, 10000, TIMER_TRIG_TYPE_SINGLE, triggerRearm, &rearmed));
		// A compare interrupt before the deadline is not a call
		reg->GTST |= 1; timerIrq(dev.IrqMatch[0]); assert(rearmed == 0);
		reg->GTCNT = dev.Deadline[0];
		reg->GTST |= 1; timerIrq(dev.IrqMatch[0]); assert(rearmed == 1);
		reg->GTCNT = dev.Deadline[0];
		reg->GTST |= 1; timerIrq(dev.IrqMatch[0]); assert(rearmed == 2 && timer.FindAvailTrigger(&timer) == 0);
		reg->GTCNT = mask - 10;
		assert(timer.EnableTrigger(&timer, 0, 10000, TIMER_TRIG_TYPE_CONTINUOUS, triggerCount, &hits[0]));
		assert(reg->GTCCRA == 19);
		reg->GTCNT = 24; reg->GTST |= 1; timerIrq(dev.IrqMatch[0]);
		assert(reg->GTCCRA == 49); // phase preserved through counter wrap
		reg->GTCNT = 109; reg->GTST |= 1; timerIrq(dev.IrqMatch[0]);
		assert(reg->GTCCRA == 139 && dev.Deadline[0] == 139); // skip missed periods
		// A deadline closer than the margin when the interrupt returns: the
		// counter does not run here, the wait ends and the compare is after
		// the deadline, the deadline kept
		reg->GTCNT = 130; reg->GTST |= 1; timerIrq(dev.IrqMatch[0]);
		assert(dev.Margin > 9 && reg->GTCCRA == 130 + dev.Margin + 1 && dev.Deadline[0] == 139);
		reg->GTCNT = mask - 1; assert(timer.GetTickCount(&timer) == mask - 1);
		reg->GTCNT = 5; reg->GTST |= GPT320_GTST_TCFPO_Msk;
		uint64_t wrapped = (uint64_t)mask + 6;
		assert(timer.GetTickCount(&timer) == wrapped && timer.Rollover == 0);
		timerIrq(dev.IrqOvr);
		assert(overflowReads[no] == wrapped && timer.GetTickCount(&timer) == wrapped);
		routes[dev.IrqOvr] |= ICU_IELSR0_IR_Msk;
		routes[dev.IrqMatch[0]] |= ICU_IELSR0_IR_Msk;
		reg->GTST |= GPT320_GTST_TCFPO_Msk | 1;
		timer.Disable(&timer); timer.Reset(&timer);
		assert(!(reg->GTCR & GPT320_GTCR_CST_Msk) && timer.GetTickCount(&timer) == 0);
		assert(!(routes[dev.IrqOvr] & ICU_IELSR0_IR_Msk) && !(routes[dev.IrqMatch[0]] & ICU_IELSR0_IR_Msk));
		assert(reg->GTCCRA == 30 && timer.Enable(&timer));
		assert(timer.SetFrequency(&timer, 750000) == 750000 && reg->GTCCRA == 8);
		uint32_t cr = reg->GTCR;
		// The active period takes the closest one, 3 ticks, at the lowest rate
		assert(timer.SetFrequency(&timer, 1) == 46875 && reg->GTCR != cr && reg->GTCCRA == 3);
		timer.DisableTrigger(&timer, 0);
		assert(timer.SetFrequency(&timer, 3000000) == 3000000);
		assert(timer.EnableTrigger(&timer, -1, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		assert(timer.EnableTrigger(&timer, 4, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		assert(timer.EnableTrigger(&timer, 0, 0, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		assert(timer.EnableTrigger(&timer, 0, 1, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 1000 && dev.CC[0] == 3);
		assert(timer.EnableTrigger(&timer, 0, UINT64_MAX, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) ==
			TimerTickToTime(&timer, mask, 1000000000UL) && dev.CC[0] == mask);
		timer.DisableTrigger(&timer, 0);
		assert(timer.EnableTrigger(&timer, 0, 10000, (TIMER_TRIG_TYPE)2, nullptr, nullptr) == 0);
		uint64_t tooLong = ((uint64_t)mask + 1) * 1000000000ULL / timer.Freq;
		assert(timer.EnableTrigger(&timer, 0, tooLong, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) &&
			dev.CC[0] == mask);
		timer.DisableTrigger(&timer, 0);
		uint64_t longest = (uint64_t)mask * 1000000000ULL / timer.Freq;
		assert(timer.EnableTrigger(&timer, 0, longest, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr));
		assert(dev.CC[0] == mask);
		cr = reg->GTCR;
		// The active period takes the longest one at the higher rate
		assert(timer.SetFrequency(&timer, 48000000) == 48000000 && reg->GTCR != cr && dev.CC[0] == mask);
		assert(timer.SetFrequency(&timer, 3000000) == 3000000);
		timer.DisableTrigger(&timer, 0);
		assert(!timer.EnableExtTrigger(&timer, 0, TIMER_EXTTRIG_SENSE_TOGGLE));
		exhaust();
		uint32_t deadline = reg->GTCCRA;
		assert(timer.EnableTrigger(&timer, 0, 10000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr) == 0);
		assert(timer.FindAvailTrigger(&timer) == 0 && reg->GTCCRA == deadline);
		restore();
		assert(timer.EnableTrigger(&timer, 3, 10000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
		assert(TimerInit(&timer, &cfg) && timer.FindAvailTrigger(&timer) == 0);
		for (int i = 0; i < 4; i++) assert(dev.IrqMatch[i] == (IRQn_Type)-1 && *cmp[i] == 0);
		timer.Disable(&timer);
	}
	assert(countRoutes() == 6);
	assert(!(MSTP->MSTPCRD & (MSTP_MSTPCRD_MSTPD5_Msk | MSTP_MSTPCRD_MSTPD6_Msk)));
	// Starting/stopping/resetting one channel must not stop its sibling.
	assert(timers[0].Enable(&timers[0]) && timers[1].Enable(&timers[1]));
	GPT321->GTCNT = 42; timers[0].Disable(&timers[0]); timers[0].Reset(&timers[0]);
	assert(GPT321->GTCR_b.CST && GPT321->GTCNT == 42);
	timers[1].Disable(&timers[1]);
	release();
}

static uint32_t actualBaud(SCI2_Type *reg, uint32_t Clock)
{
	uint32_t div = reg->SEMR_b.ABCSE ? 6 : reg->SEMR_b.BGDM ? (reg->SEMR_b.ABCS ? 8 : 16) : 32;
	uint32_t m = reg->SEMR_b.BRME ? reg->MDDR : 256;
	double baud = (double)Clock * m / (div * (1UL << (reg->SMR_b.CKS * 2)) * 256.0 * (reg->BRR + 1));
	return (uint32_t)std::floor(baud + 0.5);
}

// Independent exhaustive oracle: search every BRR and round the optimal MDDR.
static uint32_t bestError(uint32_t Clock, uint32_t Rate)
{
	uint32_t best = UINT32_MAX;
	for (uint32_t div : {32U, 16U, 8U, 6U})
		for (unsigned cks = 0; cks < 4; cks++)
			for (unsigned brr = 0; brr < 256; brr++)
			{
				double base = (double)Clock / (div * (1UL << (cks * 2)) * (brr + 1));
				int ideal = (int)std::floor(Rate / base * 256);
				// 128 is the closest M below the range
				for (int m : {256, ideal, ideal + 1, 128})
				{
					if (m < 128 || m > 256) continue;
					uint32_t actual = (uint32_t)std::floor(base * m / 256 + 0.5);
					uint32_t e = actual > Rate ? actual - Rate : Rate - actual;
					if (e < best) best = e;
				}
			}
	return best;
}

// A byte received while the UART handler runs, at the first call
static int s_UartRxIrq = -1, s_UartRxCalls;
static int uartRaise(UARTDev_t * const, UART_EVT Evt, uint8_t *, int)
{
	if (Evt == UART_EVT_RXDATA && s_UartRxCalls++ == 0)
	{
		routes[s_UartRxIrq] |= ICU_IELSR0_IR_Msk;
	}
	return 0;
}

static void uarts()
{
	const IOPinCfg_t pins[] = {{8, 13, IOPINOP_FUNC3, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{8, 12, IOPINOP_FUNC3, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}};
	UARTDev_t u = {}, fifo = {};
	UARTCfg_t cfg = {.DevNo = 6, .pIOPinMap = pins, .NbIOPins = 2, .Rate = 115200, .DataBits = 8,
		.Parity = UART_PARITY_NONE, .StopBits = 1, .FlowControl = UART_FLWCTRL_NONE,
		.bIntMode = false, .IntPrio = 1, .bFifoBlocking = true};
	MSTP->MSTPCRB = UINT32_MAX;
	assert(UARTInit(&u, &cfg));
	assert(MSTP->MSTPCRB == (UINT32_MAX & ~MSTP_MSTPCRB_MSTPB22_Msk));
	assert(countRoutes() == 0 && (SCI9->SCR & (SCI2_SCR_TIE_Msk | SCI2_SCR_RIE_Msk | SCI2_SCR_TEIE_Msk)) == 0);
	for (uint32_t clock : {24000000U, 16000000U, 32000000U})
	{
		SYSTEM->HOCOMCR = clock == 24000000 ? 2 : 3;
		SYSTEM->SCKDIVCR = (clock == 16000000 ? 2UL : 1UL) << SYSTEM_SCKDIVCR_PCKB_Pos;
		SystemCoreClockUpdate();
		// Rates out of the range take the closest one
		for (uint32_t rate : {1U, 31U, 50U, 300U, 9600U, 115200U, 250000U, 1000000U, 2000000U,
			4000000U, 100000000U})
		{
			uint32_t actual = UARTSetRate(&u, rate);
			assert(actual == actualBaud(SCI9, clock));
			uint32_t err = actual > rate ? actual - rate : rate - actual;
			assert(err == bestError(clock, rate));
			assert(UARTGetRate(&u) == (int)actual);
		}
	}
	// Lowest: PCLK * 128 / (32 * 64 * 256 * 256), highest: PCLK / 6
	assert(UARTSetRate(&u, 1) == 31 && UARTSetRate(&u, 100000000) == 5333333);
	uint8_t smr = SCI9->SMR, semr = SCI9->SEMR, brr = SCI9->BRR;
	assert(UARTSetRate(&u, 0) == 0);
	assert(SCI9->SMR == smr && SCI9->SEMR == semr && SCI9->BRR == brr);
	uint8_t data[] = {0x12, 0x34};
	SCI9->SSR = SCI2_SSR_TDRE_Msk;
	assert(u.DevIntrf.TxData(&u.DevIntrf, data, 1) == 1 && SCI9->TDR == data[0]);
	SCI9->SSR = 0;
	assert(u.DevIntrf.TxData(&u.DevIntrf, data, 1) == 0);
	*(volatile uint8_t*)&SCI9->RDR = 0x56;
	SCI9->SSR = SCI2_SSR_RDRF_Msk;
	assert(u.DevIntrf.RxData(&u.DevIntrf, data, 2) == 1 && data[0] == 0x56);
	assert((SCI9->SSR & SCI2_SSR_RDRF_Msk) == 0);
	cfg.bIntMode = true;
	exhaust(5); // TXI has no free line; RXI and ERI partial allocations must be released.
	int busy = countRoutes();
	assert(!UARTInit(&u, &cfg) && u.DevIntrf.EnCnt == 0 && SCI9->SCR == 0);
	assert(UARTGetInstance(6) == nullptr && countRoutes() == busy);
	restore();
	assert(UARTInit(&u, &cfg) && countRoutes() == 3);
	assert(UARTInit(&u, &cfg) && countRoutes() == 3);
	SCI9->SSR = SCI2_SSR_TDRE_Msk;
	data[0] = 0x12; data[1] = 0x34;
	assert(u.DevIntrf.TxData(&u.DevIntrf, data, 2) == 2 && SCI9->TDR == 0x12);
	assert(SCI9->SCR & SCI2_SCR_TIE_Msk);
	SCI9->SSR |= SCI2_SSR_TDRE_Msk;
	irq(5, 0x1C);
	assert(SCI9->TDR == 0x34 && (SCI9->SCR & SCI2_SCR_TIE_Msk) == 0);
	SCI9->SSR = SCI2_SSR_ORER_Msk | SCI2_SSR_FER_Msk | SCI2_SSR_PER_Msk;
	irq(7, 0x1C);
	assert(u.RxOvrErrCnt == 1 && u.FramErrCnt == 1 && u.ParErrCnt == 1);
	assert((SCI9->SSR & (SCI2_SSR_ORER_Msk | SCI2_SSR_FER_Msk | SCI2_SSR_PER_Msk)) == 0);
	uint8_t received[16];
	for (unsigned i = 0; i < 17; i++)
	{
		*(volatile uint8_t*)&SCI9->RDR = i;
		SCI9->SSR = SCI2_SSR_RDRF_Msk;
		irq(4, 0x1C);
	}
	assert(u.RxDropCnt == 1 && (SCI9->SSR & SCI2_SSR_RDRF_Msk) == 0);
	assert(u.DevIntrf.RxData(&u.DevIntrf, received, sizeof(received)) == 16);
	for (unsigned i = 0; i < 16; i++) assert(received[i] == i);
	// The handler clears the flag first: a byte received while it runs calls
	// it again
	for (int i = 4; i < 32; i += 8)
	{
		if ((routes[i] & ICU_IELSR0_IELS_Msk) == 0x1C)
		{
			s_UartRxIrq = i;
		}
	}
	assert(s_UartRxIrq >= 0);
	u.EvtCallback = uartRaise;
	*(volatile uint8_t*)&SCI9->RDR = 0x77;
	SCI9->SSR = SCI2_SSR_RDRF_Msk;
	routes[s_UartRxIrq] |= ICU_IELSR0_IR_Msk;
	handlers[s_UartRxIrq]();
	assert(s_UartRxCalls == 1 && (routes[s_UartRxIrq] & ICU_IELSR0_IR_Msk));
	*(volatile uint8_t*)&SCI9->RDR = 0x78;
	SCI9->SSR = SCI2_SSR_RDRF_Msk;
	handlers[s_UartRxIrq]();
	assert(s_UartRxCalls == 2 && (routes[s_UartRxIrq] & ICU_IELSR0_IR_Msk) == 0);
	u.EvtCallback = nullptr;
	assert(u.DevIntrf.RxData(&u.DevIntrf, received, sizeof(received)) == 2);
	assert(received[0] == 0x77 && received[1] == 0x78);
	cfg.bIntMode = false;
	assert(UARTInit(&u, &cfg) && countRoutes() == 0);
	cfg.bDMAMode = true;
	assert(!UARTInit(&u, &cfg));
	cfg.bDMAMode = false; cfg.DevNo = 0; cfg.bIntMode = true;
	assert(UARTInit(&fifo, &cfg));
	assert(fifo.Rate == (int)actualBaud((SCI2_Type*)SCI0, SystemPeriphClockGet(0)));
	uint8_t block[16]; memset(block, 0xA7, sizeof(block));
	*(volatile uint16_t*)&SCI0->FDR = 0;
	assert(fifo.DevIntrf.TxData(&fifo.DevIntrf, block, sizeof(block)) == 16);
	assert(SCI0->FTDRL == 0xA7 && CFifoUsed(fifo.hTxFifo) == 0);
	assert((SCI0->SCR & SCI0_SCR_TIE_Msk) == 0);
	*(volatile uint8_t*)&SCI0->FRDRL = 0x89;
	*(volatile uint16_t*)&SCI0->FDR = 16;
	SCI0->SSR_FIFO |= SCI0_SSR_FIFO_RDF_Msk | SCI0_SSR_FIFO_DR_Msk;
	irq(0, 0x10);
	assert(CFifoUsed(fifo.hRxFifo) == 16);
	*(volatile uint16_t*)&SCI0->FDR = 1;
	SCI0->SSR_FIFO |= SCI0_SSR_FIFO_DR_Msk;
	irq(0, 0x10);
	assert(fifo.RxDropCnt == 1 && (SCI0->SSR_FIFO & SCI0_SSR_FIFO_DR_Msk) == 0);
	assert(fifo.DevIntrf.RxData(&fifo.DevIntrf, block, sizeof(block)) == 16);
	for (uint8_t byte : block) assert(byte == 0x89);
	cfg.bIntMode = false;
	assert(UARTInit(&fifo, &cfg) && countRoutes() == 0);
}

// Rate requests out of the range take the closest end of it
static void busRates()
{
	const IOPinCfg_t i2cPins[] = {{1, 2, IOPINOP_FUNC7, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN},
		{1, 3, IOPINOP_FUNC7, IOPINDIR_BI, IOPINRES_NONE, IOPINTYPE_OPENDRAIN}};
	I2CCfg_t icfg = {};
	I2CDev_t i2c = {};
	icfg.DevNo = 0; icfg.Type = I2CTYPE_STANDARD; icfg.Mode = I2CMODE_MASTER;
	icfg.pIOPinMap = i2cPins; icfg.NbIOPins = 2; icfg.Rate = 1000000;
	assert(I2CInit(&i2c, &icfg));
	uint32_t pclkb = SystemPeriphClockGet(1);
	uint32_t fast = i2c.DevIntrf.SetRate(&i2c.DevIntrf, 400000);
	assert(fast > 100000 && fast <= 400000 && i2c.Cfg.Rate == fast);
	assert(i2c.DevIntrf.SetRate(&i2c.DevIntrf, 1000000) == fast);
	// Slowest: CKS 7, BRL = BRH = 31, 2 * (31 + 4) * 128 PCLKB clocks
	assert(i2c.DevIntrf.SetRate(&i2c.DevIntrf, 1) == pclkb / 8960);
	assert(i2c.DevIntrf.SetRate(&i2c.DevIntrf, 0) == 0 && i2c.Cfg.Rate == pclkb / 8960);

	const IOPinCfg_t spiPins[] = {{2, 0, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{2, 1, IOPINOP_FUNC6, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{2, 2, IOPINOP_FUNC6, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},
		{2, 3, IOPINOP_GPIO, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL}};
	SPICfg_t scfg = {};
	SPIDev_t spi = {};
	scfg.DevNo = 0; scfg.Mode = SPIMODE_MASTER; scfg.Phy = SPIPHY_NORMAL;
	scfg.pIOPinMap = spiPins; scfg.NbIOPins = 4; scfg.Rate = 1;
	scfg.DataSize = 8; scfg.ChipSel = SPICSEL_MAN; scfg.BitOrder = SPIDATABIT_MSB;
	scfg.ClkPol = SPICLKPOL_HIGH; scfg.DataPhase = SPIDATAPHASE_FIRST_CLK;
	assert(SPIInit(&spi, &scfg));
	uint32_t pclka = SystemPeriphClockGet(0);
	// Slowest: BRDV 3, SPBR 255, highest: PCLKA / 2
	assert(spi.Cfg.Rate == pclka / 4096);
	assert(spi.DevIntrf.SetRate(&spi.DevIntrf, UINT32_MAX) == pclka / 2);
	assert(spi.DevIntrf.SetRate(&spi.DevIntrf, 1) == pclka / 4096);
}

int main()
{
	assert(mmap((void*)0x40000000, 0xA0000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	assert(mmap((void*)FLASH_BASE, 0x1000, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, -1, 0) != MAP_FAILED);
	clocks(); icuAndGpio(); timers(); gptTimers(); uarts(); busRates();
	puts("RE01 clock, ICU, GPIO, timer, UART, I2C and SPI rate regression checks passed");
}
