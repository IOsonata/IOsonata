/**-------------------------------------------------------------------------
@file nrf70_tasklet_test.cpp
@brief nRF70 tasklet registration, queue recovery and lifetime regression.
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
#include <assert.h>
#include <stdio.h>
#include "net/nrf70_tasklet.h"
#include "app_evt_handler.h"

static AppEvtHandler_t s_Handler;
static void *s_Context;
static bool s_Refuse;
static unsigned long s_Count;
static Nrf70Tasklet_t *s_Reschedule;

extern "C" bool AppEvtHandlerQue(uint32_t Event, void *pContext,
	AppEvtHandler_t Handler)
{
	(void)Event;
	if (s_Refuse || s_Handler != nullptr)
	{
		return false;
	}
	s_Handler = Handler;
	s_Context = pContext;
	return true;
}

static void Dispatch(void)
{
	assert(s_Handler != nullptr);
	AppEvtHandler_t handler = s_Handler;
	void *context = s_Context;
	s_Handler = nullptr;
	s_Context = nullptr;
	handler(0, context);
}

static void Callback(unsigned long Context)
{
	s_Count += Context;
	if (s_Reschedule != nullptr)
	{
		Nrf70Tasklet_t *task = s_Reschedule;
		s_Reschedule = nullptr;
		assert(Nrf70TaskletSchedule(task));
	}
}

int main(void)
{
	// Fresh caller storage must be initialized without trusting its flags.
	Nrf70Tasklet_t dirty = { nullptr, 0, true, true, true };
	Nrf70TaskletInit(&dirty, Callback, 1);
	assert(Nrf70TaskletSchedule(&dirty));
	assert(Nrf70TaskletSchedule(&dirty));
	Dispatch();
	assert(s_Count == 1 && !Nrf70TaskletPending());
	Nrf70TaskletCancel(&dirty);

	// Repeated lifecycles must release the bounded registry slots.
	Nrf70Tasklet_t tasks[32] = {};
	for (unsigned i = 0; i < 32; i++)
	{
		Nrf70TaskletInit(&tasks[i], Callback, 1);
		assert(Nrf70TaskletSchedule(&tasks[i]));
		Dispatch();
		Nrf70TaskletCancel(&tasks[i]);
	}
	assert(s_Count == 33);

	// Exhaustion is safe, and cancelling one entry permits retry.
	for (unsigned i = 0; i < 17; i++)
	{
		Nrf70TaskletInit(&tasks[i], Callback, 1);
	}
	assert(!Nrf70TaskletSchedule(&tasks[16]));
	Nrf70TaskletCancel(&tasks[0]);
	Nrf70TaskletInit(&tasks[16], Callback, 1);
	assert(Nrf70TaskletSchedule(&tasks[16]));
	Nrf70TaskletCancel(&tasks[16]);
	assert(Nrf70TaskletPending());
	Dispatch();
	assert(s_Count == 33);
	for (unsigned i = 1; i < 16; i++)
	{
		Nrf70TaskletCancel(&tasks[i]);
	}

	// A refused event is retried; callback self-scheduling is retained.
	Nrf70TaskletInit(&dirty, Callback, 2);
	s_Refuse = true;
	assert(!Nrf70TaskletSchedule(&dirty));
	assert(Nrf70TaskletPending());
	s_Refuse = false;
	Nrf70TaskletCheckStatus();
	s_Reschedule = &dirty;
	Dispatch();
	Dispatch();
	assert(s_Count == 37);

	// Reinitialization cannot replace a callback still referenced by a queue.
	assert(Nrf70TaskletSchedule(&dirty));
	Nrf70TaskletCancel(&dirty);
	Nrf70TaskletInit(&dirty, Callback, 100);
	assert(!Nrf70TaskletSchedule(&dirty));
	Dispatch();
	assert(s_Count == 37);
	Nrf70TaskletInit(&dirty, Callback, 3);
	assert(Nrf70TaskletSchedule(&dirty));
	Dispatch();
	Nrf70TaskletCancel(&dirty);
	assert(s_Count == 40);

	// No registry reference may survive caller storage after cancellation.
	{
		Nrf70Tasklet_t local;
		Nrf70TaskletInit(&local, Callback, 1);
		Nrf70TaskletCancel(&local);
	}
	assert(!Nrf70TaskletPending());
	Nrf70TaskletCheckStatus();
	{
		Nrf70Tasklet_t local;
		Nrf70TaskletInit(&local, Callback, 1);
		assert(Nrf70TaskletSchedule(&local));
		Nrf70TaskletCancel(&local);
		Dispatch();
	}
	assert(!Nrf70TaskletPending());
	Nrf70TaskletCheckStatus();
	puts("nRF70 tasklet lifecycle: PASS");
	return 0;
}
