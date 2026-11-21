# Clock calibration from AppRun

The System Core calibration API performs one calibration when requested.
It does not schedule a periodic timer, own an RTC, or depend on AppRun.
A higher-level subsystem that already uses AppRun can use its Timer event
to queue calibration as deferred foreground work.

## Example: periodic LFRC calibration

This example owns a low-frequency Timer device. It demonstrates the
application-side scheduling mechanism, not an automatic System Core
timer allocation. The application must ensure that the selected timer
is free and that the configured LFRC is in use.

```cpp
#include "coredev/system_core_clock.h"
#include "coredev/timer.h"
#include "app_evt_handler.h"

static TimerDev_t s_CalTimer;
static volatile bool s_bCalQueued;
static volatile uint32_t s_CalSuccess;
static volatile uint32_t s_CalFailure;

static void ClockCalibrationWork(uint32_t Event, void *pCtx)
{
	bool success;

	(void)Event;
	(void)pCtx;
	success = SystemLowFreqClockCalibrate();
	if (success)
	{
		s_CalSuccess++;
	}
	else
	{
		s_CalFailure++;
	}
	s_bCalQueued = false;
}

static void ClockCalibrationTimer(TimerDev_t *pTimer, uint32_t Event)
{
	bool queued;

	(void)pTimer;
	if ((Event & TIMER_EVT_TRIGGER(0)) == 0U || s_bCalQueued)
	{
		return;
	}
	s_bCalQueued = true;
	queued = AppEvtHandlerQue(0U, nullptr, ClockCalibrationWork);
	if (!queued)
	{
		s_bCalQueued = false;
	}
}

static bool ClockCalibrationStart(int TimerNo)
{
	TimerCfg_t cfg;
	uint64_t period;

	cfg = {};
	cfg.DevNo = TimerNo;
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0U;
	cfg.IntPrio = 6;
	cfg.EvtHandler = ClockCalibrationTimer;
	if (!TimerInit(&s_CalTimer, &cfg))
	{
		return false;
	}
	period = nsTimerEnableTrigger(&s_CalTimer, 0, 4000000000ULL,
		TIMER_TRIG_TYPE_CONTINUOUS, nullptr, nullptr);
	if (period == 0U || !TimerEnable(&s_CalTimer))
	{
		TimerDisableTrigger(&s_CalTimer, 0);
		TimerDisable(&s_CalTimer);
		return false;
	}
	return true;
}
```

After selecting an available timer and starting it, the application's
main function can call `AppRun()`. The timer IRQ only queues
`ClockCalibrationWork`; AppRun executes that function when it dispatches
application events. The callback must **not** call
`SystemLowFreqClockCalibrate()` from interrupt context because the
calibration waits for hardware completion.

The example uses a four-second period, leaving margin beneath an
eight-second calibration interval. It coalesces pending requests so a
slow foreground loop does not accumulate redundant calibration work.
If `AppEvtHandlerQue()` refuses the event due to a full queue, the next
timer tick can retry. An application requiring a strict maximum
interval must account for queue latency, errors, and missed requests.

The Timer subsystem owns its hardware. System Core never accesses the
Timer object or the event queue. On nRF5340, the CAL hardware requests
HFXO temporarily; System Core holds an extra shared LFCLK reference
during the one-shot operation and releases it afterward.

This example is not automatically compiled into every ClockDemo target.
Other MCU implementations may return false for unsupported calibration.
`SystemHighFreqClockCalibrate()` is similarly available for an MCU that
implements high-frequency RC calibration.
