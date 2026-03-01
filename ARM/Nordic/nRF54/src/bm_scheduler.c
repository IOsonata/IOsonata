/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

/*
 * Reimplemented to use IOsonata AppEvtHandler (CFIFO-based).
 *
 * The original bm_scheduler used Zephyr k_heap to dynamically allocate
 * events with variable-length data copies.  This version uses the
 * IOsonata AppEvtHandler which queues fixed-size {handler, context}
 * tuples in a lock-free circular FIFO — zero dynamic allocation.
 *
 * Limitation: Variable-length data copy (data != NULL, len != 0) is
 * not supported.  All existing sdk-nrf-bm call sites pass NULL/0.
 */

#include <errno.h>
#include <bm/bm_scheduler.h>
#include "app_evt_handler.h"

/*
 * Trampoline: AppEvtHandler calls void(*)(uint32_t, void*),
 * bm_scheduler expects void(*)(void*, size_t).
 *
 * We store the bm_scheduler handler in pCtx and call it with (NULL, 0).
 * This matches the only usage pattern in sdk-nrf-bm.
 */
static void BmSchedTrampoline(uint32_t EvtId, void *pCtx)
{
	bm_scheduler_fn_t handler = (bm_scheduler_fn_t)pCtx;

	if (handler)
	{
		handler(NULL, 0);
	}
}

int bm_scheduler_defer(bm_scheduler_fn_t handler, void *data, size_t len)
{
	if (!handler)
	{
		return -EFAULT;
	}

	if (data != NULL || len != 0)
	{
		/* Variable-length data copy not supported.
		 * Use AppEvtHandlerQue directly for pointer-based context. */
		return -ENOTSUP;
	}

	if (!AppEvtHandlerQue(0, (void *)handler, BmSchedTrampoline))
	{
		return -ENOMEM;
	}

	return 0;
}

int bm_scheduler_process(void)
{
	AppEvtHandlerExec();

	return 0;
}

void bm_scheduler_init(void)
{
	/* AppEvtHandler is initialized by the BLE stack (BtAppInit, etc.)
	 * so this is typically a no-op.  If bm_scheduler is used standalone
	 * without the BLE stack, call AppEvtHandlerInit(NULL, 0) first. */
}
