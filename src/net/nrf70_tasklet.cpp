/**-------------------------------------------------------------------------
@file nrf70_tasklet.cpp
@brief Nordic host tasklet dispatcher using IOsonata AppEvt.
@license MIT License
Copyright (c) 2026 I-SYST inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
-------------------------------------------------------------------------*/
#include "net/nrf70_tasklet.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"

/* Single foreground owner; the IRQ producer uses the same critical section
 * as AppEvtHandlerQue. Do not destroy caller-owned tasklets with work queued. */
#define NRF70_TASKLET_MAX 16
static Nrf70Tasklet_t *s_Tasks[NRF70_TASKLET_MAX];

static bool Nrf70TaskletRegistered(Nrf70Tasklet_t *pTask)
{
	int i;

	for (i = 0; i < NRF70_TASKLET_MAX; i++)
	{
		if (s_Tasks[i] == pTask)
			return true;
	}
	return false;
}

// Caller holds the interrupt mask while changing the registry.
static void Nrf70TaskletUnregister(Nrf70Tasklet_t *pTask)
{
	for (int i = 0; i < NRF70_TASKLET_MAX; i++)
	{
		if (s_Tasks[i] == pTask)
		{
			s_Tasks[i] = nullptr;
			break;
		}
	}
}

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
	if (!pTask->Active)
	{
		Nrf70TaskletUnregister(pTask);
	}
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
	// Only a registered tasklet has flags initialized by this dispatcher.
	if (i < NRF70_TASKLET_MAX && pTask->Queued)
	{
		EnableInterrupt(state);
		return;
	}
	pTask->Callback = Callback;
	pTask->Context = Context;
	pTask->Pending = false;
	pTask->Queued = false;
	pTask->Active = false;
	if (Callback == nullptr)
	{
		Nrf70TaskletUnregister(pTask);
	}
	else
	{
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
		pTask->Active = i < NRF70_TASKLET_MAX;
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
	if (!Nrf70TaskletRegistered(pTask) || !pTask->Active)
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
	if (Nrf70TaskletRegistered(pTask))
	{
		pTask->Pending = false;
		pTask->Active = false;
		// A queued callback still owns its context until dispatch drains it.
		if (!pTask->Queued)
		{
			Nrf70TaskletUnregister(pTask);
		}
	}
	EnableInterrupt(state);
}

extern "C" bool Nrf70TaskletPending(void)
{
	uint32_t state;
	bool pending;
	int i;

	state = DisableInterrupt();
	pending = false;
	for (i = 0; i < NRF70_TASKLET_MAX; i++)
	{
		if (s_Tasks[i] != nullptr &&
			(s_Tasks[i]->Queued ||
			(s_Tasks[i]->Active && s_Tasks[i]->Pending)))
		{
			pending = true;
			break;
		}
	}
	EnableInterrupt(state);
	return pending;
}

extern "C" void Nrf70TaskletCheckStatus(void)
{
	int i;

	for (i = 0; i < NRF70_TASKLET_MAX; i++)
	{
		uint32_t state = DisableInterrupt();
		if (s_Tasks[i] != nullptr && s_Tasks[i]->Pending &&
			!s_Tasks[i]->Queued && s_Tasks[i]->Active)
		{
			(void)Nrf70TaskletSchedule(s_Tasks[i]);
		}
		EnableInterrupt(state);
	}
}
