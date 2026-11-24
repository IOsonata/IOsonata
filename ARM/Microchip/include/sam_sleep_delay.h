/**-------------------------------------------------------------------------
@file	sam_sleep_delay.h
@brief	Seconds delay using the SAM D RTC or SAM4L AST.
@license MIT License
Copyright (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#ifndef SAM_SLEEP_DELAY_H
#define SAM_SLEEP_DELAY_H

#include "coredev/timer.h"
#include "coredev/interrupt.h"

static void SamDelayWake(TimerDev_t *pTimer, int Trigger, void *pContext)
{
	(void)pTimer;
	(void)Trigger;
	*(volatile bool *)pContext = true;
}

static bool SamSleepDelay(uint32_t Seconds, TimerDev_t * const *pOwner)
{
	static TimerDev_t timer;
	static volatile bool elapsed;
	TimerCfg_t cfg = {};
	uint32_t state;
	uint32_t chunk;
	uint32_t scr;
	bool ok;

	if (Seconds == 0U)
	{
		return true;
	}
	if (__get_IPSR() != 0U || __get_PRIMASK() != 0U)
	{
		return false;
	}

	cfg.DevNo = 0;
	cfg.Freq = 1024U;
	cfg.IntPrio = 1;
	state = DisableInterrupt();
	ok = *pOwner == NULL && TimerInit(&timer, &cfg);
	EnableInterrupt(state);
	if (!ok)
	{
		return false;
	}

	scr = SCB->SCR;
	SCB->SCR = scr & ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
	while (Seconds != 0U && ok)
	{
		chunk = Seconds > 86400U ? 86400U : Seconds;
		elapsed = false;
		ok = nsTimerEnableTrigger(&timer, 0, (uint64_t)chunk * 1000000000ULL,
			TIMER_TRIG_TYPE_SINGLE, SamDelayWake, (void *)&elapsed) != 0U;
		while (ok && !elapsed)
		{
			state = DisableInterrupt();
			if (!elapsed)
			{
				__DSB();
				__WFI();
			}
			EnableInterrupt(state);
		}
		Seconds -= chunk;
	}
	SCB->SCR = scr;
	__ISB();
	return TimerPowerOff(&timer) && ok;
}
#endif
