/**-------------------------------------------------------------------------
@file	pei_hal_cpp_api.cpp
@brief	Compile-only check of C++ wrappers over the shared C PEI matrix
----------------------------------------------------------------------------*/
#include "coredev/pei.h"
#include "coredev/timer.h"
#include "coredev/iopincfg.h"

bool PeiHalCppApplication(Timer &timer)
{
	Pei pei;
	PeiEvent_t evt = 0U;
	int trigger;

	if (!IOPinEnableEvent(pei, evt, 0, 2, IOPINSENSE_LOW_TRANSITION))
	{
		return false;
	}
	if (!IOPinEnableTask(pei, evt, 0, 30, IOPIN_TASK_TOGGLE))
	{
		return false;
	}
	trigger = timer.EnableTimerTrigger(pei, evt, 500,
		TIMER_TRIG_TYPE_CONTINUOUS);
	if (trigger < 0)
	{
		return false;
	}
	if (!timer.ConnectTask(pei, evt, TIMER_TASK_START))
	{
		return false;
	}
	timer.ExecuteTask(TIMER_TASK_STOP);
	pei.Enable(evt);
	pei.Disable(evt);
	timer.DisableTimerTrigger(trigger);
	IOPinDisableTask(pei, evt, 0, 30);
	IOPinDisableEvent(pei, evt);
	return pei.ReleaseEvent(evt);
}

// Compile-only proof that RTC C++ event methods delegate to the C matrix.
bool PeiHalRtcCppApplication(Timer &timer, Pei &pei)
{
	PeiEvent_t event = 0U;

	if (!timer.EnableTimerEvent(pei, event, TIMER_EVT_COUNTER_OVR))
	{
		return false;
	}
	if (!timer.ConnectTask(pei, event, TIMER_TASK_TRIGOVRFLW))
	{
		timer.DisableTimerEvent(pei, event);
		return false;
	}
	if (!timer.DisableTimerEvent(pei, event))
	{
		return false;
	}
	return pei.ReleaseEvent(event);
}
