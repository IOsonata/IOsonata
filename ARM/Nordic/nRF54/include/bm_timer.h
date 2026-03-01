/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 * Modified for IOsonata - Zephyr dependencies replaced with IOsonata Timer
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @defgroup bm_timer NCS Bare Metal Timer library
 * @{
 *
 * Thin wrapper around IOsonata's Timer (GRTC) for Nordic sdk-nrf-bm compatibility.
 *
 * Each bm_timer instance claims one trigger (CC channel) from a shared IOsonata
 * TimerDev_t.  By default uses GRTC device 0 (CC channels 0-1, 2 timers max)
 * or device 2 (CC 4-6, 3 timers max).
 *
 * IOsonata nRF54 GRTC runs at 16 MHz.  Tick conversion macros use this base.
 * Device 3 (CC 7-11) is reserved for MPSL/SoftDevice.
 */

#ifndef BM_TIMER_H__
#define BM_TIMER_H__

#include <stdint.h>
#include <stdbool.h>

#include "coredev/timer.h"

#ifdef __cplusplus
extern "C" {
#endif

/*--- Configuration ---*/

/**
 * @brief IOsonata timer device number to use (0-2, device 3 reserved for SD).
 *
 * DevNo 0: CC 0-1  (2 triggers)
 * DevNo 1: CC 2-3  (2 triggers)
 * DevNo 2: CC 4-6  (3 triggers)
 * DevNo 3: CC 7-11 (reserved for MPSL/SoftDevice)
 */
#ifndef BM_TIMER_IOSONATA_DEVNO
#define BM_TIMER_IOSONATA_DEVNO  0
#endif

/* Timer IRQ priority (must be > 4 when SoftDevice is used) */
#ifndef CONFIG_BM_TIMER_IRQ_PRIO
#define CONFIG_BM_TIMER_IRQ_PRIO  5
#endif

/*--- Tick conversion macros ---*/

/**
 * @brief IOsonata GRTC base frequency on nRF54L (16 MHz).
 */
#define BM_TIMER_TICK_FREQ  TIMER_NRFX_RTC_BASE_FREQ

/**
 * @brief Minimum timeout in microseconds.
 *
 * Calculated from a minimum of 5 ticks at GRTC frequency.
 */
#define BM_TIMER_MIN_TIMEOUT_US  (uint32_t)((5ULL * 1000000ULL) / BM_TIMER_TICK_FREQ)

/**
 * @brief Minimum value of the timeout_ticks parameter of @ref bm_timer_start.
 */
#define BM_TIMER_MIN_TIMEOUT_TICKS  5

/**
 * @brief Convert milliseconds to timer ticks (floor).
 */
#define BM_TIMER_MS_TO_TICKS(ms) \
	((uint32_t)(((uint64_t)(ms) * BM_TIMER_TICK_FREQ) / 1000ULL))

/**
 * @brief Convert microseconds to timer ticks (floor).
 */
#define BM_TIMER_US_TO_TICKS(us) \
	((uint32_t)(((uint64_t)(us) * BM_TIMER_TICK_FREQ) / 1000000ULL))

/**
 * @brief Convert timer ticks to microseconds (ceil).
 */
#define BM_TIMER_TICKS_TO_US_CEIL(ticks) \
	((uint32_t)(((uint64_t)(ticks) * 1000000ULL + BM_TIMER_TICK_FREQ - 1) / BM_TIMER_TICK_FREQ))

/**
 * @brief Convert timer ticks to nanoseconds.
 */
#define BM_TIMER_TICKS_TO_NS(ticks) \
	((uint64_t)(ticks) * (1000000000ULL / BM_TIMER_TICK_FREQ))

/**
 * @brief Timer modes.
 */
enum bm_timer_mode {
	/** The timer will expire only once. */
	BM_TIMER_MODE_SINGLE_SHOT,
	/** The timer will restart each time it expires. */
	BM_TIMER_MODE_REPEATED,
};

/**
 * @brief Application time-out handler type.
 *
 * @param context General purpose pointer. Set when calling @ref bm_timer_start.
 */
typedef void (*bm_timer_timeout_handler_t)(void *context);

/**
 * @brief Timer instance structure.
 */
struct bm_timer {
	int                     trigno;     /**< IOsonata trigger number (-1 = unassigned) */
	enum bm_timer_mode      mode;       /**< Timer mode */
	bm_timer_timeout_handler_t handler; /**< Timeout callback */
	void                    *context;   /**< User context pointer */
};

/**
 * @brief Initialize the timer subsystem.
 *
 * Configures the shared IOsonata GRTC timer device.
 * Call once at startup before any bm_timer_init().
 *
 * @retval 0 On success.
 * @retval -EIO If IOsonata timer init failed.
 */
int bm_timer_sys_init(void);

/**
 * @brief Get the underlying IOsonata TimerDev_t.
 *
 * @return Pointer to the shared TimerDev_t, or NULL if not initialized.
 */
TimerDev_t *bm_timer_get_timerdev(void);

/**
 * @brief Initialize a timer instance.
 *
 * @param timer           Pointer to timer instance.
 * @param mode            Timer mode.
 * @param timeout_handler Function to be executed when the timer expires.
 *
 * @retval 0       On success.
 * @retval -EFAULT If @p timer or @p timeout_handler is @c NULL.
 */
int bm_timer_init(struct bm_timer *timer, enum bm_timer_mode mode,
		  bm_timer_timeout_handler_t timeout_handler);

/**
 * @brief Start a timer.
 *
 * Claims an IOsonata trigger and arms it.  The timeout is specified in
 * GRTC ticks (16 MHz).  Use BM_TIMER_MS_TO_TICKS / BM_TIMER_US_TO_TICKS
 * for convenience.
 *
 * @param timer         Pointer to timer instance.
 * @param timeout_ticks Number of ticks to time-out event.
 * @param context       General purpose pointer passed to the handler.
 *
 * @retval 0       On success.
 * @retval -EFAULT If @p timer is @c NULL.
 * @retval -EINVAL If @p timeout_ticks is less than @ref BM_TIMER_MIN_TIMEOUT_TICKS.
 * @retval -ENOMEM If no trigger slots are available.
 */
int bm_timer_start(struct bm_timer *timer, uint32_t timeout_ticks, void *context);

/**
 * @brief Stop a timer.
 *
 * Releases the IOsonata trigger back to the pool.
 *
 * @param timer Pointer to timer instance.
 *
 * @retval 0       On success.
 * @retval -EFAULT If @p timer is @c NULL.
 */
int bm_timer_stop(struct bm_timer *timer);

#ifdef __cplusplus
}
#endif

#endif /* BM_TIMER_H__ */

/** @} */
