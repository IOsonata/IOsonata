/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * Modified for IOsonata - Zephyr k_timer replaced with IOsonata Timer
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/**
 * Thin wrapper that maps sdk-nrf-bm's bm_timer API onto IOsonata's
 * TimerDev_t trigger system (timer_lf_nrf54.cpp / GRTC).
 *
 * Each bm_timer instance claims one IOsonata trigger (= one GRTC CC channel).
 * The shared TimerDev_t is initialized once by bm_timer_sys_init().
 */

#include <stddef.h>
#include <string.h>
#include <errno.h>

#include "nrf.h"
#include "timer_nrfx.h"
#include "bm_timer.h"

/*--------------------------------------------------------------------------
 * Shared IOsonata timer device
 *--------------------------------------------------------------------------*/
static TimerDev_t s_TimerDev;
static bool s_Initialized = false;

/*--------------------------------------------------------------------------
 * IOsonata → bm_timer callback trampoline
 *
 * IOsonata:  void handler(TimerDev_t *pTimer, int TrigNo, void *pContext)
 * bm_timer:  void handler(void *context)
 *
 * pContext points to the bm_timer instance, from which we extract the
 * user handler and user context.
 *--------------------------------------------------------------------------*/
static void bm_timer_trig_handler(TimerDev_t * const pTimer, int TrigNo, void * const pContext)
{
	struct bm_timer *t = (struct bm_timer *)pContext;

	if (t && t->handler) {
		t->handler(t->context);
	}
}

/*--------------------------------------------------------------------------
 * Public API
 *--------------------------------------------------------------------------*/

int bm_timer_sys_init(void)
{
	if (s_Initialized) {
		return 0;
	}

	const TimerCfg_t cfg = {
		.DevNo    = BM_TIMER_IOSONATA_DEVNO,
		.ClkSrc   = TIMER_CLKSRC_DEFAULT,
		.Freq     = 0,       /* 0 = auto select max (16 MHz for GRTC) */
		.IntPrio  = CONFIG_BM_TIMER_IRQ_PRIO,
		.EvtHandler = NULL,
		.bTickInt = false,
	};

	if (!nRFxGrtcInit(&s_TimerDev, &cfg)) {
		return -EIO;
	}

	s_Initialized = true;

	return 0;
}

TimerDev_t *bm_timer_get_timerdev(void)
{
	return s_Initialized ? &s_TimerDev : NULL;
}

int bm_timer_init(struct bm_timer *timer, enum bm_timer_mode mode,
		  bm_timer_timeout_handler_t timeout_handler)
{
	if (!timer || !timeout_handler) {
		return -EFAULT;
	}

	memset(timer, 0, sizeof(*timer));
	timer->trigno  = -1;
	timer->mode    = mode;
	timer->handler = timeout_handler;
	timer->context = NULL;

	return 0;
}

int bm_timer_start(struct bm_timer *timer, uint32_t timeout_ticks, void *context)
{
	if (!timer) {
		return -EFAULT;
	}

	if (timeout_ticks < BM_TIMER_MIN_TIMEOUT_TICKS) {
		return -EINVAL;
	}

	if (!s_Initialized) {
		return -ENXIO;
	}

	/* If already running, stop first to release the trigger */
	if (timer->trigno >= 0) {
		s_TimerDev.DisableTrigger(&s_TimerDev, timer->trigno);
		timer->trigno = -1;
	}

	/* Find a free trigger slot */
	int trigno = s_TimerDev.FindAvailTrigger(&s_TimerDev);
	if (trigno < 0) {
		return -ENOMEM;
	}

	timer->context = context;
	timer->trigno  = trigno;

	/* Convert ticks to nanoseconds for IOsonata.
	 * IOsonata GRTC nsPeriod = 1000000000 / Freq.
	 * nsPeriod for 16 MHz = 62 ns per tick.
	 * ns = ticks * nsPeriod
	 */
	uint64_t nsPeriod = (uint64_t)timeout_ticks * s_TimerDev.nsPeriod;

	TIMER_TRIG_TYPE type = (timer->mode == BM_TIMER_MODE_REPEATED)
				? TIMER_TRIG_TYPE_CONTINUOUS
				: TIMER_TRIG_TYPE_SINGLE;

	uint64_t real_ns = s_TimerDev.EnableTrigger(&s_TimerDev, trigno, nsPeriod,
						    type, bm_timer_trig_handler,
						    (void *)timer);

	if (real_ns == 0) {
		timer->trigno = -1;
		return -EINVAL;
	}

	return 0;
}

int bm_timer_stop(struct bm_timer *timer)
{
	if (!timer) {
		return -EFAULT;
	}

	if (timer->trigno >= 0 && s_Initialized) {
		s_TimerDev.DisableTrigger(&s_TimerDev, timer->trigno);
		timer->trigno = -1;
	}

	return 0;
}
