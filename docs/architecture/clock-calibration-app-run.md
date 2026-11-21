# Clock calibration from AppRun

`SystemHighFreqClockCalibrate()` and `SystemLowFreqClockCalibrate()`
perform one calibration in the System Core clock owner. Neither function
creates a timer, owns an RTC or enqueues an AppRun event.

The optional higher-level `AppClockCalibration` helper attaches to a
**caller-owned Timer trigger** and queues a one-shot calibration request
each time that trigger fires. AppRun performs the calibration in foreground
context. The helper never calls TimerInit, TimerEnable or TimerDisable.

## Application example

The application selects and owns its Timer device. A spare compare/trigger
channel must be available; use a low-frequency timer when calibration
must continue across System ON sleep.

```cpp
#include "app_clock_calibration.h"
#include "app_evt_handler.h"
#include "coredev/timer.h"

static TimerDev_t s_Timer;
static AppClockCalibration_t s_Calibration;

int main(void)
{
	TimerCfg_t cfg;
	bool started;

	cfg = {};
	cfg.DevNo = 0;				// Chosen by the application
	cfg.ClkSrc = TIMER_CLKSRC_DEFAULT;
	cfg.Freq = 0U;
	cfg.IntPrio = 6;
	cfg.EvtHandler = nullptr;
	if (!TimerInit(&s_Timer, &cfg))
	{
		return 1;
	}
	if (!TimerEnable(&s_Timer))
	{
		return 1;
	}
	started = AppClockCalibrationStart(&s_Calibration, &s_Timer,
		0, 4000U, false);		// Trigger 0, 4-second LFRC calibration
	if (!started)
	{
		TimerDisable(&s_Timer);
		return 1;
	}
	AppRun();
}
```

The application must configure its board and selected oscillator before
this example starts. The example assumes that the selected Timer device and
trigger are available. On nRF5340, the LFRC frequency must be selected
and the timer must be running, so that the clock owner already has an LFCLK
reference.

The ISR invokes the registered trigger callback, which queues an event
with `AppEvtHandlerQue`. AppRun executes that event and calls
`SystemLowFreqClockCalibrate` outside interrupt context. The helper
coalesces outstanding requests, counts completed calibrations, failures,
and queue refusals, and never reserves a second Timer device.

To stop periodic calibration:

```cpp
AppClockCalibrationStop(&s_Calibration);
TimerDisable(&s_Timer);
```

Stopping calibration disables only its trigger; the Timer is still owned
by the application. A queued event is ignored when dispatched after Stop.
A stopped helper cannot be restarted until that queued event has been
dispatched; keep the `AppClockCalibration_t` object alive until then.

For a 4-second interval, failures or queue congestion must be monitored.
A strict 8-second maximum requires verifying ISR delivery, AppRun latency,
and queue capacity on the target. A periodic Timer event alone does not
guarantee calibration completion within that interval.

The Core clock API is independent of the application event queue. The
AppClockCalibration helper is opt-in: application firmware must link
`src/app_clock_calibration.cpp` and `src/app_evt_handler.cpp` alongside
its normal Timer implementation. No MCU-specific code or compile switch
is required in the helper.

The nRF5340 CAL operation itself requests HFXO during calibration.
The System Clock implementation manages calibration serialization and holds
a temporary LFCLK reference. The helper does not request HFXO or change
the selected clock source.

On an MCU that does not implement the calibration API, a request is
counted as failed. High-frequency RC calibration uses the same helper
with the final argument set to `true`.
