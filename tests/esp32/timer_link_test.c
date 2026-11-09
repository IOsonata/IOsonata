/**-------------------------------------------------------------------------
@file timer_link_test.c
@brief Cross-link the C timer interface; also compiled as C++ for its wrapper.
@license MIT License
Copyright (c) 2026 I-SYST inc. All rights reserved.
----------------------------------------------------------------------------*/
#include "coredev/timer.h"

static volatile unsigned s_Events;
static void Event(TimerDev_t *pTimer, uint32_t Events)
{
	(void)pTimer;
	(void)Events;
	s_Events = s_Events + 1;
}

int main(void)
{
	TimerCfg_t config = { 0 };
#ifdef __cplusplus
	static Timer timer;
#else
	static TimerDev_t timer;
#endif

	config.DevNo = TimerGetHighFreqDevNo();
	config.Freq = 1000000;
	config.EvtHandler = Event;
#ifdef __cplusplus
	if (!timer.Init(config))
	{
		return 1;
	}
	timer.EnableTimerTrigger(0, 100000000ULL, TIMER_TRIG_TYPE_CONTINUOUS);
#else
	if (!TimerInit(&timer, &config))
	{
		return 1;
	}
	nsTimerEnableTrigger(&timer, 0, 100000000ULL, TIMER_TRIG_TYPE_CONTINUOUS, 0, 0);
#endif
	while (1)
	{
	}
}
