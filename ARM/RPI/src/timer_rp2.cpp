// Each hardware alarm is a separately owned timer on the shared 1 MHz counter.
#include "rp2.h"
#include "hardware/regs/timer.h"
#include "coredev/interrupt.h"
#include "coredev/timer.h"
#include "coredev/system_core_clock.h"
#include "timer_systick.h"
#ifdef RP2350
#define RP_TIMER_COUNT 8
#else
#define RP_TIMER_COUNT 4
#endif

struct RpAlarm {
	TimerDev_t *Owner;
	uint64_t Base;
	uint64_t Start;
	uint32_t Deadline;
	uint32_t Period;
	bool Running;
	TimerTrig_t Trigger;
};

static RpAlarm s_Alarm[RP_TIMER_COUNT];
static TimerDev_t *s_SysTick;

static uintptr_t RpTimerBase(unsigned dev)
{
#ifdef RP2350
	return dev < 4 ? TIMER0_BASE : TIMER1_BASE;
#else
	(void)dev;
	return TIMER_BASE;
#endif
}

static uint64_t RpTime(unsigned dev)
{
	uintptr_t base = RpTimerBase(dev);
	uint32_t hi, lo;

	do {
		hi = RP_REG(base, TIMER_TIMERAWH_OFFSET);
		lo = RP_REG(base, TIMER_TIMERAWL_OFFSET);
	} while (hi != RP_REG(base, TIMER_TIMERAWH_OFFSET));
	return ((uint64_t)hi << 32) | lo;
}

static void RpDisarm(unsigned dev)
{
	uintptr_t base = RpTimerBase(dev);
	uint32_t mask = 1U << (dev % 4);
	RP_CLR(base, TIMER_INTE_OFFSET) = mask;
	RP_CLR(base, TIMER_INTF_OFFSET) = mask;
	RP_REG(base, TIMER_ARMED_OFFSET) = mask;
	RP_REG(base, TIMER_INTR_OFFSET) = mask;
}

static void RpArm(unsigned dev)
{
	RpAlarm &a = s_Alarm[dev];
	uintptr_t base = RpTimerBase(dev);
	uint32_t mask = 1U << (dev % 4);

	RP_SET(base, TIMER_INTE_OFFSET) = mask;
	RP_REG(base, TIMER_ALARM0_OFFSET + 4 * (dev % 4)) = a.Deadline;
	// A short deadline can pass between reading the counter and arming it.
	if ((int32_t)(RP_REG(base, TIMER_TIMERAWL_OFFSET) - a.Deadline) >= 0)
	{
		RP_SET(base, TIMER_INTF_OFFSET) = mask;
	}
}

static uint64_t RpCount(TimerDev_t *t)
{
	uint32_t state = DisableInterrupt();
	RpAlarm &a = s_Alarm[t->DevNo];
	uint64_t ticks = a.Owner == t ? a.Base : 0;
	if (a.Owner == t && a.Running)
	{
		ticks += RpTime(t->DevNo) - a.Start;
	}
	EnableInterrupt(state);
	return ticks;
}

static bool RpEnable(TimerDev_t *t)
{
	RpAlarm &a = s_Alarm[t->DevNo];
	uint32_t state = DisableInterrupt();
	bool ok = a.Owner == t;
	if (ok && !a.Running)
	{
		ok = SystemLowFreqClockEnable();
		if (ok)
		{
			a.Start = RpTime(t->DevNo);
			a.Running = true;
			if (a.Period)
			{
				a.Deadline = (uint32_t)a.Start + a.Period;
				RpArm(t->DevNo);
			}
		}
	}
	EnableInterrupt(state);
	return ok;
}

static void RpDisable(TimerDev_t *t)
{
	RpAlarm &a = s_Alarm[t->DevNo];
	uint32_t state = DisableInterrupt();
	if (a.Owner == t && a.Running)
	{
		a.Base = RpCount(t);
		RpDisarm(t->DevNo);
		a.Running = false;
		SystemLowFreqClockDisable();
	}
	EnableInterrupt(state);
}

static bool RpPowerOff(TimerDev_t *t)
{
	RpAlarm &a = s_Alarm[t->DevNo];
	uint32_t state = DisableInterrupt();
	bool ok = a.Owner == t;
	if (ok)
	{
		RpDisable(t);
		a = {};
		t->PowerOff = nullptr;
	}
	EnableInterrupt(state);
	return ok;
}

static void RpReset(TimerDev_t *t)
{
	uint32_t state = DisableInterrupt();
	RpAlarm &a = s_Alarm[t->DevNo];
	if (a.Owner != t)
	{
		EnableInterrupt(state);
		return;
	}
	a.Base = 0;
	a.Start = RpTime(t->DevNo);
	EnableInterrupt(state);
}
static bool RpExtTrigger(TimerDev_t *, int, TIMER_EXTTRIG_SENSE) { return false; }
static void RpDisableExtTrigger(TimerDev_t *) {}
static int RpMax(TimerDev_t *) { return 1; }
static int RpAvail(TimerDev_t *t) { return s_Alarm[t->DevNo].Period ? -1 : 0; }
static uint32_t RpFreq(TimerDev_t *, uint32_t) { return 1000000U; }

static void RpDisableTrigger(TimerDev_t *t, int trigger)
{
	if (trigger == 0 && s_Alarm[t->DevNo].Owner == t)
	{
		uint32_t state = DisableInterrupt();
		RpDisarm(t->DevNo);
		s_Alarm[t->DevNo].Period = 0;
		EnableInterrupt(state);
	}
}

static uint64_t RpTrigger(TimerDev_t *t, int trigger, uint64_t ns, TIMER_TRIG_TYPE type,
	TimerTrigEvtHandler_t cb, void *ctx)
{
	RpAlarm &a = s_Alarm[t->DevNo];
	uint64_t ticks;
	uint32_t state;
	if (a.Owner != t || trigger || !ns || (type != TIMER_TRIG_TYPE_SINGLE && type != TIMER_TRIG_TYPE_CONTINUOUS))
	{
		return 0;
	}
	ticks = ns / 1000U + (ns % 1000U >= 500U);
	state = DisableInterrupt();
	RpDisarm(t->DevNo);
	a.Period = ticks < 1 ? 1 : ticks > INT32_MAX ? INT32_MAX : (uint32_t)ticks;
	ns = (uint64_t)a.Period * 1000U;
	a.Trigger = {type, ns, cb, ctx};
	if (a.Running)
	{
		a.Deadline = (uint32_t)RpTime(t->DevNo) + a.Period;
		RpArm(t->DevNo);
	}
	EnableInterrupt(state);
	return ns;
}

extern "C" void RpTimerIrq(void)
{
	unsigned dev = __get_IPSR() - 16U;
	RpAlarm &a = s_Alarm[dev];
	TimerTrig_t trig = a.Trigger;
	TimerDev_t *owner = a.Owner;
	int32_t late;

	if (!(RP_REG(RpTimerBase(dev), TIMER_INTS_OFFSET) & (1U << (dev % 4U))))
	{
		return;
	}
	RpDisarm(dev);
	if (!owner || !a.Period)
	{
		return;
	}
	if (trig.Type == TIMER_TRIG_TYPE_CONTINUOUS)
	{
		late = (int32_t)((uint32_t)RpTime(dev) - a.Deadline);
		a.Deadline += a.Period;
		if (late >= (int32_t)a.Period)
		{
			a.Deadline += ((uint32_t)late / a.Period) * a.Period;
		}
		RpArm(dev);
	}
	else
	{
		a.Period = 0;
	}
	if (trig.Handler)
	{
		trig.Handler(owner, 0, trig.pContext);
	}
	if (a.Owner == owner && owner->EvtHandler)
	{
		owner->EvtHandler(owner, TIMER_EVT_TRIGGER(0));
	}
}

static bool RpSysTickOff(TimerDev_t *t)
{
	if (s_SysTick != t || !SysTickTimerPowerOff(t))
	{
		return false;
	}
	s_SysTick = nullptr;
	return true;
}

int TimerGetLowFreqDevCount(void) { return RP_TIMER_COUNT; }
int TimerGetHighFreqDevCount(void) { return 1; }
int TimerGetHighFreqDevNo(void) { return RP_TIMER_COUNT; }

bool TimerInit(TimerDev_t *t, const TimerCfg_t *cfg)
{
	int dev;
	uint32_t state;
	bool ok;
	if (!t || !cfg || cfg->DevNo < 0 || cfg->DevNo > RP_TIMER_COUNT || cfg->bTickInt)
	{
		return false;
	}
	dev = cfg->DevNo;
	state = DisableInterrupt();
	if ((dev == RP_TIMER_COUNT && s_SysTick && s_SysTick != t) ||
		(dev < RP_TIMER_COUNT && s_Alarm[dev].Owner && s_Alarm[dev].Owner != t))
	{
		EnableInterrupt(state);
		return false;
	}
	for (int i = 0; i < RP_TIMER_COUNT; i++)
	{
		if (s_Alarm[i].Owner == t && s_Alarm[i].Running)
		{
			EnableInterrupt(state);
			return false;
		}
	}
	if (s_SysTick == t && SysTickTimerEnabled())
	{
		EnableInterrupt(state);
		return false;
	}
	for (int i = 0; i < RP_TIMER_COUNT; i++)
	{
		if (s_Alarm[i].Owner == t)
		{
			RpPowerOff(t);
		}
	}
	if (s_SysTick == t)
	{
		RpSysTickOff(t);
	}
	if (dev == RP_TIMER_COUNT)
	{
		ok = SysTickTimerInit(t, cfg);
		if (ok)
		{
			s_SysTick = t;
			t->DevNo = dev;
			t->nsSetup = 0;
			t->PowerOff = RpSysTickOff;
		}
	}
	else
	{
#ifdef RP2350
		RpReleaseReset(dev < 4 ? RESETS_RESET_TIMER0_BITS : RESETS_RESET_TIMER1_BITS);
#else
		RpReleaseReset(RESETS_RESET_TIMER_BITS);
#endif
		t->Rollover = 0;
		t->LastCount = 0;
		t->nsSetup = 0;
		t->GetTaskEndpoint = nullptr;
		t->ExecuteTask = nullptr;
		t->EnableExtTrigger = RpExtTrigger;
		t->DisableExtTrigger = RpDisableExtTrigger;
		t->DevNo = dev;
		t->Freq = 1000000U;
		t->nsPeriod = 1000U;
		t->EvtHandler = cfg->EvtHandler;
		t->Disable = RpDisable;
		t->Enable = RpEnable;
		t->PowerOff = RpPowerOff;
		t->Reset = RpReset;
		t->GetTickCount = RpCount;
		t->SetFrequency = RpFreq;
		t->GetMaxTrigger = RpMax;
		t->FindAvailTrigger = RpAvail;
		t->DisableTrigger = RpDisableTrigger;
		t->EnableTrigger = RpTrigger;
		s_Alarm[dev].Owner = t;
		RpDisarm(dev);
		RpEnableIrq(dev, cfg->IntPrio);
		ok = RpEnable(t);
	}
	EnableInterrupt(state);
	return ok;
}
