/**-------------------------------------------------------------------------
@file nrf70_tasklet.cpp
@brief Nordic host tasklet dispatcher using IOsonata AppEvt.
@license MIT License
Copyright (c) 2026 I-SYST inc.
-------------------------------------------------------------------------*/
#include "net/nrf70_tasklet.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"

/* Single foreground owner; the IRQ producer uses the same critical section
 * as AppEvtHandlerQue. Do not destroy caller-owned tasklets with work queued. */
#define NRF70_TASKLET_MAX 16
static Nrf70Tasklet_t *s_Tasks[NRF70_TASKLET_MAX];

static void Nrf70TaskletDispatch(uint32_t Event, void *pContext)
{
	Nrf70Tasklet_t *pTask;
	Nrf70TaskletCb_t cb;
	unsigned long context;
	uint32_t state;

	(void)Event;
	pTask = (Nrf70Tasklet_t *)pContext;
	if (pTask == nullptr)
		return;

	state = DisableInterrupt();
	pTask->Queued = false;
	cb = pTask->Active && pTask->Pending ? pTask->Callback : nullptr;
	context = pTask->Context;
	pTask->Pending = false;
	EnableInterrupt(state);
	if (cb != nullptr)
		cb(context);
}

extern "C" void Nrf70TaskletInit(Nrf70Tasklet_t *pTask,
	Nrf70TaskletCb_t Callback, unsigned long Context)
{
	uint32_t state;
	int i;

	if (pTask == nullptr)
		return;
	state = DisableInterrupt();
	for (i = 0; i < NRF70_TASKLET_MAX; i++)
	{
		if (s_Tasks[i] == pTask)
			break;
	}
	if (i == NRF70_TASKLET_MAX)
	{
		for (i = 0; i < NRF70_TASKLET_MAX; i++)
		{
			if (s_Tasks[i] == nullptr)
			{
				s_Tasks[i] = pTask;
				break;
			}
		}
	}
	if (i < NRF70_TASKLET_MAX && !pTask->Queued)
	{
		pTask->Callback = Callback;
		pTask->Context = Context;
		pTask->Active = Callback != nullptr;
		pTask->Pending = false;
	}
	EnableInterrupt(state);
}

extern "C" bool Nrf70TaskletSchedule(Nrf70Tasklet_t *pTask)
{
	uint32_t state;
	bool queued;

	if (pTask == nullptr)
		return false;
	state = DisableInterrupt();
	if (!pTask->Active)
	{
		EnableInterrupt(state);
		return false;
	}
	pTask->Pending = true;
	queued = pTask->Queued;
	if (!queued)
	{
		pTask->Queued = true;
		queued = AppEvtHandlerQue(0, pTask, Nrf70TaskletDispatch);
		if (!queued)
			pTask->Queued = false;
	}
	EnableInterrupt(state);
	return queued;
}

extern "C" void Nrf70TaskletCancel(Nrf70Tasklet_t *pTask)
{
	uint32_t state;

	if (pTask == nullptr)
		return;
	state = DisableInterrupt();
	pTask->Pending = false;
	pTask->Active = false;
	EnableInterrupt(state);
}

extern "C" void Nrf70TaskletCheckStatus(void)
{
	int i;

	for (i = 0; i < NRF70_TASKLET_MAX; i++)
	{
		if (s_Tasks[i] != nullptr && s_Tasks[i]->Pending &&
			!s_Tasks[i]->Queued && s_Tasks[i]->Active)
			(void)Nrf70TaskletSchedule(s_Tasks[i]);
	}
}
