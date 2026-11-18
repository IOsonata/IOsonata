#include "coredev/timer.h"

static TimerDev_t s_Timer;
#if TIMER_LINK_DEVNO < 0
static volatile int s_DevNo;
#endif

int main(void)
{
	TimerCfg_t cfg = {
#if TIMER_LINK_DEVNO < 0
		.DevNo = 0,
#else
		.DevNo = TIMER_LINK_DEVNO,
#endif
		.ClkSrc = TIMER_CLKSRC_DEFAULT,
		.IntPrio = 1
	};
#if TIMER_LINK_DEVNO < 0
	cfg.DevNo = s_DevNo;
#endif
	if (!TimerInit(&s_Timer, &cfg))
	{
		return 1;
	}
	nsTimerEnableTrigger(&s_Timer, 0, 1000000, TIMER_TRIG_TYPE_CONTINUOUS, 0, 0);
	return (int)TimerGetTickCount(&s_Timer);
}
