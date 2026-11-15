// Timer core model: register operations are supplied by counter stand-ins.
#include <cassert>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "coredev/timer.h"

typedef int IRQn_Type;
#define TCC0_CC_NUM 4
#define TCC0_SIZE 24
#define RTC_IRQn 3
#define TC1_IRQn 18
#define TC2_IRQn 19
#define TCC0_IRQn 17
#define PM_APBCMASK_TC1_Msk 1
#define PM_APBCMASK_TC2_Msk 2
#define PM_APBCMASK_TCC0_Msk 4
#define GCLK_CLKCTRL_ID_RTC_Val 1
#define GCLK_CLKCTRL_ID_TC1_TC2_Val 2
#define GCLK_CLKCTRL_ID_TCC0_Val 3
#define EIC_EXTINT_NUM 16
#define SAMD_GCLK_MAIN 0
#define SAMD_GCLK_LF 1

static struct { uint32_t COUNT16; } s_Tc[2];
static uint32_t s_Tcc;
#define TC1_REGS (&s_Tc[0])
#define TC2_REGS (&s_Tc[1])
#define TCC0_REGS (&s_Tcc)
#include "timer_private.h"
#include "evsys_private.h"

struct CounterModel {
	uint32_t Count;
	bool Overflow;
	unsigned Matched;
};
static CounterModel s_Count[4];
static unsigned s_Events[4][4];
static unsigned s_OverflowEvents;
static unsigned s_Opens;
static unsigned s_Closes;
static bool s_CancelNext;
static bool s_ResetInHandler;
static uint32_t s_Pending;
static unsigned s_SysTickStops;
static uint32_t s_MainFreq = 48000000;
static uint32_t s_LfFreq = 32768;
static bool s_SetupFail;
static bool s_StopFail;
static bool s_StartFail;
static bool s_ResetFail;
static bool s_RtcWaitFail;
static int s_RtcRunSyncFail = -1;
bool SysTickTimerInit(TimerDev_t *pTimer, const TimerCfg_t *pCfg);
static uint32_t SystemPeriphClockGet(int);
static unsigned GetLowFreqOscType();
static unsigned SamdIrqPrio(int);
static void NVIC_EnableIRQ(int);
static void NVIC_DisableIRQ(int);
static void NVIC_SetPriority(int, unsigned);
static unsigned NVIC_GetPriority(int);
static void NVIC_SetPendingIRQ(int);
static void NVIC_ClearPendingIRQ(int);
#include "timer_driver.h"

static void StopSysTick(TimerDev_t *)
{
	s_SysTickStops++;
}

bool SysTickTimerInit(TimerDev_t *pTimer, const TimerCfg_t *)
{
	pTimer->DevNo = 0;
	pTimer->Disable = StopSysTick;
	pTimer->Freq = 48000000;

	return true;
}

static uint32_t SystemPeriphClockGet(int Id) { return Id == SAMD_GCLK_LF ? s_LfFreq : s_MainFreq; }
static unsigned GetLowFreqOscType() { return 0; }
static unsigned SamdIrqPrio(int Prio) { return (unsigned)Prio; }
static void NVIC_EnableIRQ(int) {}
static void NVIC_DisableIRQ(int) {}
static void NVIC_SetPriority(int, unsigned) {}
static unsigned NVIC_GetPriority(int) { return 1; }
static void NVIC_SetPendingIRQ(int Irq) { s_Pending |= 1U << Irq; }
static void NVIC_ClearPendingIRQ(int Irq) { s_Pending &= ~(1U << Irq); }

static bool ModelSetup(SamdTimerData_t *) { return !s_SetupFail; }
static bool ModelRun(SamdTimerData_t *d, bool Run)
{
	if (d->pTimer->DevNo == 0 && (int)Run == s_RtcRunSyncFail)
	{
		s_RtcWaitFail = true;
	}

	return Run ? !s_StartFail : !s_StopFail;
}
static bool ModelReset(SamdTimerData_t *d)
{
	if (s_ResetFail)
	{
		return false;
	}

	s_Count[d->pTimer->DevNo] = {};
	return true;
}
static bool ModelRead(SamdTimerData_t *d, uint32_t *count)
{
	*count = s_Count[d->pTimer->DevNo].Count;
	return true;
}
static bool ModelOverflow(SamdTimerData_t *d)
{
	bool flag = s_Count[d->pTimer->DevNo].Overflow;
	s_Count[d->pTimer->DevNo].Overflow = false;
	return flag;
}
static bool ModelArm(SamdTimerData_t *d, int n, bool)
{
	s_Count[d->pTimer->DevNo].Matched &= ~(1U << n);
	return true;
}
static bool ModelMatched(SamdTimerData_t *d, int n)
{
	bool flag = (s_Count[d->pTimer->DevNo].Matched & (1U << n)) != 0;
	s_Count[d->pTimer->DevNo].Matched &= ~(1U << n);
	return flag;
}
#define COUNTER_OPS(Name) \
	bool Name##Setup(SamdTimerData_t *d) { return ModelSetup(d); } \
	bool Name##Run(SamdTimerData_t *d, bool r) { return ModelRun(d, r); } \
	bool Name##Reset(SamdTimerData_t *d) { return ModelReset(d); } \
	bool Name##Read(SamdTimerData_t *d, uint32_t *c) { return ModelRead(d, c); } \
	bool Name##Overflow(SamdTimerData_t *d) { return ModelOverflow(d); } \
	bool Name##Arm(SamdTimerData_t *d, int n, bool e) { return ModelArm(d, n, e); } \
	bool Name##Matched(SamdTimerData_t *d, int n) { return ModelMatched(d, n); }
COUNTER_OPS(SamdRtc)
COUNTER_OPS(SamdTc)
COUNTER_OPS(SamdTcc)
bool SamdRtcWait(SamdTimerData_t *, bool) { return !s_RtcWaitFail; }
void SamdRtcResume(SamdTimerData_t *) {}
bool SamdRtcEvent(SamdTimerData_t *d)
{
	return s_Count[d->pTimer->DevNo].Matched != 0 || s_Count[d->pTimer->DevNo].Overflow;
}
uint32_t SamdRtcTickGen(SamdTimerData_t *d) { return d->bTick ? 4 : 0; }
uint32_t SamdRtcLag(SamdTimerData_t *) { return 0; }
bool SamdTcCapture(SamdTimerData_t *, bool) { return true; }
bool SamdTccCapture(SamdTimerData_t *, bool) { return true; }
bool SamdTcCaptured(SamdTimerData_t *, uint32_t *) { return false; }
bool SamdTccCaptured(SamdTimerData_t *, uint32_t *) { return false; }
int SamdEvsysOpen(int, int, int, int, SamdEvsysHandler_t, void *, int)
{
	s_Opens++;
	return 0;
}
void SamdEvsysClose(int, int) { s_Closes++; }
bool SamdEicEvent(int, IOPINSENSE) { return true; }

static void Event(TimerDev_t *timer, uint32_t event)
{
	if (event & TIMER_EVT_COUNTER_OVR)
	{
		s_OverflowEvents++;
	}
	for (int i = 0; i < 4; i++)
	{
		if (event & TIMER_EVT_TRIGGER(i))
		{
			s_Events[timer->DevNo][i]++;
			if (s_CancelNext && i == 0)
			{
				TimerDisableTrigger(timer, 1);
			}
			if (s_ResetInHandler)
			{
				TimerReset(timer);
			}
		}
	}
}
static void Advance(int DevNo, uint64_t Ticks)
{
	SamdTimerData_t *d = &g_SamdTimerData[DevNo];
	uint64_t next = s_Count[DevNo].Count + Ticks;
	uint64_t span = 1ULL << d->CntBits;

	s_Count[DevNo].Overflow = next >= span;
	s_Count[DevNo].Count = (uint32_t)(next & (span - 1));
	s_Count[DevNo].Matched = (1U << d->NbTrig) - 1;
	SamdTimerIrq(DevNo);
}
int main()
{
	TimerDev_t timer = {};
	TimerCfg_t cfg = { .DevNo = 0, .IntPrio = 1, .EvtHandler = Event };

	for (int dev = 0; dev < 4; dev++)
	{
		cfg.DevNo = dev;
		assert(TimerInit(&timer, &cfg));
		assert(g_SamdTimerData[dev].pOps == (dev == 0 ? &s_SamdRtcOps : dev < 3 ? &s_SamdTcOps : &s_SamdTccOps));
		assert(g_SamdTimerData[dev].pTickOps == nullptr);
		assert(TimerGetMaxTrigger(&timer) == (dev == 0 ? 1 : dev < 3 ? 2 : 4));
		assert(nsTimerEnableTrigger(&timer, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
		uint32_t ticks = g_SamdTimerData[dev].pTrig[0].Ticks;
		Advance(dev, ticks);
		assert(s_Events[dev][0] == 1);
		Advance(dev, ticks * 3);
		assert(s_Events[dev][0] == 2);
		assert(g_SamdTimerData[dev].pTrig[0].Deadline == (uint64_t)ticks * 5);
		TimerDisableTrigger(&timer, 0);
		assert(nsTimerEnableTrigger(&timer, 0, 1000000, TIMER_TRIG_TYPE_SINGLE, nullptr, nullptr));
		Advance(dev, ticks);
		assert(!g_SamdTimerData[dev].pTrig[0].bActive);
		assert(s_Events[dev][0] == 3);
		TimerReset(&timer);
		s_Count[dev].Count = (uint32_t)((1ULL << g_SamdTimerData[dev].CntBits) - 2);
		uint64_t before = TimerGetTickCount(&timer);
		Advance(dev, 4);
		assert(TimerGetTickCount(&timer) == before + 4);
		TimerDisable(&timer);
		assert(!g_SamdTimerData[dev].bRunning);
		assert(TimerEnable(&timer));
		// A changed source needs a new frequency selection before restart.
		assert(nsTimerEnableTrigger(&timer, 0, 1000000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
		TimerDisable(&timer);
		uint32_t &source = dev == 0 ? s_LfFreq : s_MainFreq;
		uint32_t original = source;
		uint32_t requested = timer.Freq;

		source /= 2;
		assert(!TimerEnable(&timer));
		assert(!g_SamdTimerData[dev].bRunning);
		uint32_t actual = timer.SetFrequency(&timer, requested);

		assert(actual != 0 && g_SamdTimerData[dev].BaseFreq == source);
		assert(g_SamdTimerData[dev].bRunning && TimerGetTickCount(&timer) == 0);
		assert(g_SamdTimerData[dev].pTrig[0].Ticks == actual);
		assert(g_SamdTimerData[dev].pTrig[0].Info.nsPeriod == 1000000000);
		TimerDisable(&timer);
		source = 0;
		assert(!TimerEnable(&timer));
		assert(timer.SetFrequency(&timer, requested) == 0);
		assert(!g_SamdTimerData[dev].bRunning);
		assert(g_SamdTimerData[dev].BaseFreq == original / 2);
		source = original;
		s_SetupFail = true;
		assert(timer.SetFrequency(&timer, requested) == 0);
		assert(g_SamdTimerData[dev].BaseFreq == 0);
		assert(!TimerEnable(&timer) && !g_SamdTimerData[dev].bRunning);
		s_SetupFail = false;
		assert(timer.SetFrequency(&timer, requested) != 0);
		// Each hardware failure must prevent Enable on an unchanged source.
		bool *failures[] = { &s_StopFail, &s_SetupFail, &s_ResetFail, &s_StartFail };

		for (bool *failure : failures)
		{
			*failure = true;
			assert(timer.SetFrequency(&timer, requested) == 0);
			assert(g_SamdTimerData[dev].BaseFreq == 0);
			*failure = false;
			assert(!TimerEnable(&timer));
			assert(timer.SetFrequency(&timer, requested) != 0);
			assert(g_SamdTimerData[dev].BaseFreq == source);
			assert(g_SamdTimerData[dev].bRunning);
		}
		if (dev == 0)
		{
			for (int run = 0; run < 2; run++)
			{
				TimerDisable(&timer);
				s_RtcRunSyncFail = run;
				if (run)
				{
					assert(!TimerEnable(&timer));
				}
				else
				{
					TimerDisable(&timer);
				}
				assert(!g_SamdTimerData[dev].bRunning);
				assert(g_SamdTimerData[dev].BaseFreq == 0);
				s_RtcRunSyncFail = -1;
				s_RtcWaitFail = false;
				assert(!TimerEnable(&timer));
				assert(timer.SetFrequency(&timer, requested) != 0);
			}
		}
	}
	assert(s_OverflowEvents == 4);
	// Capture routing is released before moving the same handle to RTC.
	assert(TimerEnableExtTrigger(&timer, 2, TIMER_EXTTRIG_SENSE_TOGGLE));
	assert(s_Opens == 1);
	cfg.DevNo = 0;
	cfg.bTickInt = true;
	assert(TimerInit(&timer, &cfg));
	assert(s_Closes == 1 && s_Opens == 2);
	assert(!TimerEnableExtTrigger(&timer, 2, TIMER_EXTTRIG_SENSE_TOGGLE));
	assert(TimerEnableExtTrigger(&timer, 2, TIMER_EXTTRIG_SENSE_DISABLE));
	cfg.bTickInt = false;
	assert(TimerInit(&timer, &cfg));
	assert(s_Closes == 2 && s_Opens == 2);
	// A callback changing another due trigger suppresses its old event.
	cfg.DevNo = 1;
	assert(TimerInit(&timer, &cfg));
	for (int i = 0; i < 2; i++)
	{
		assert(nsTimerEnableTrigger(&timer, i, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr));
	}
	s_CancelNext = true;
	Advance(1, g_SamdTimerData[1].pTrig[0].Ticks);
	assert(s_Events[1][1] == 0);
	s_CancelNext = false;
	s_ResetInHandler = true;
	Advance(1, g_SamdTimerData[1].pTrig[0].Ticks);
	assert(TimerGetTickCount(&timer) == 0);
	// SysTick follows the existing hardware device numbers.
	assert(TimerGetLowFreqDevCount() == 1);
	assert(TimerGetHighFreqDevCount() == 4);
	cfg.DevNo = SAMD_SYSTICK_TIMER_DEVNO;
	assert(cfg.DevNo == 4 && TimerInit(&timer, &cfg));
	assert(timer.DevNo == 4 && timer.Freq == 48000000);
	assert(g_SamdTimerData[1].pTimer == nullptr);
	assert(!g_SamdTimerData[1].bRunning);
	TimerDev_t other = {};
	assert(!TimerInit(&other, &cfg));
	assert(TimerInit(&timer, &cfg));
	assert(s_SysTickStops == 1);
	cfg.bTickInt = true;
	assert(!TimerInit(&timer, &cfg));
	assert(s_SysTickStops == 1);
	cfg.bTickInt = false;
	cfg.DevNo = 2;
	assert(TimerInit(&timer, &cfg));
	assert(s_SysTickStops == 2 && s_SamdSysTickOwner == nullptr);
	cfg.DevNo = 4;
	assert(TimerInit(&other, &cfg));
	cfg.DevNo = 5;
	assert(!TimerInit(&timer, &cfg));
	puts("SAMD timer family selection, rollover, triggers, routing and reinitialization: PASS");
}
