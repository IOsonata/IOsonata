/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/** @file
 *
 * @defgroup bm_scheduler NCS Bare Metal Event Scheduler library
 * @{
 *
 * @brief Deferred event execution backed by IOsonata AppEvtHandler.
 *
 * This is a compatibility wrapper around IOsonata's CFIFO-based
 * AppEvtHandler.  It provides the same API as the original Nordic
 * bm_scheduler but uses zero dynamic allocation.
 *
 * @note Variable-length data (data != NULL) is not supported.
 *       All sdk-nrf-bm call sites pass NULL/0.
 */

#ifndef BM_SCHEDULER_H__
#define BM_SCHEDULER_H__

#include <stdint.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Event handler prototype.
 */
typedef void (*bm_scheduler_fn_t)(void *evt, size_t len);

/**
 * @brief Initialize the event scheduler.
 *
 * Typically a no-op — AppEvtHandler is initialized by the BLE stack.
 * If using bm_scheduler standalone, call AppEvtHandlerInit() first.
 */
void bm_scheduler_init(void);

/**
 * @brief Schedule an event for execution in the main thread.
 *
 * This function can be called from an ISR to defer code execution
 * to the main thread.
 *
 * @param handler Event handler.
 * @param data Must be NULL (data copy not supported).
 * @param len Must be 0 (data copy not supported).
 *
 * @retval 0 On success.
 * @retval -EFAULT @p handler is NULL.
 * @retval -ENOTSUP @p data is not NULL or @p len is not 0.
 * @retval -ENOMEM Event queue is full.
 */
int bm_scheduler_defer(bm_scheduler_fn_t handler, void *data, size_t len);

/**
 * @brief Process deferred events.
 *
 * Dispatches all queued events in the main thread.
 *
 * @retval 0 On success.
 */
int bm_scheduler_process(void);

#ifdef __cplusplus
}
#endif

#endif /* BM_SCHEDULER_H__ */

/** @} */
