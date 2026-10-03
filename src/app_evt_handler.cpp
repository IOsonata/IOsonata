/**-------------------------------------------------------------------------
@file	app_evt_handler.cpp

@brief	Event handler queuing

This is an implementation of event handler queuing to schedule event handler
in firmware application main loop.

Without an AppEvtHandlerInit call, the queue takes g_AppEvtHandlerQueMem, the
library default holding APPEVT_HANDLER_QUE_DEFAULT_SIZE events, the first time
an event is queued. An application that needs more defines its own
g_AppEvtHandlerQueMem and passes its size to AppEvtHandlerInit before any event
source is enabled.

@author	Hoang Nguyen Hoan
@date	Oct. 17, 2022

@license

MIT License

Copyright (c) 2022, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <memory.h>

#include "cfifo.h"
#include "app_evt_handler.h"
#include "coredev/interrupt.h"

#define APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE \
	CFIFO_TOTAL_MEMSIZE(APPEVT_HANDLER_QUE_DEFAULT_SIZE, sizeof(AppEvtHandlerQue_t))

// Default queue memory, APPEVT_HANDLER_QUE_DEFAULT_SIZE events. An application
// that defines its own g_AppEvtHandlerQueMem replaces it at link time, see
// app_evt_handler.h.
alignas(4) __attribute__((weak)) uint8_t g_AppEvtHandlerQueMem[
	APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE];

static hCFifo_t s_hAppEvtHandlerFifo;

// Without an AppEvtHandlerInit call, the queue takes the default memory the
// first time it is used. That can be from an interrupt, so the check and the
// init are one critical section.
static hCFifo_t AppEvtHandlerFifo(void)
{
	if (s_hAppEvtHandlerFifo == nullptr)
	{
		uint32_t state = DisableInterrupt();
		if (s_hAppEvtHandlerFifo == nullptr)
		{
			(void)AppEvtHandlerInit(nullptr, 0);
		}
		EnableInterrupt(state);
	}

	return s_hAppEvtHandlerFifo;
}

bool AppEvtHandlerInit(uint8_t *pFifoMem, size_t Size)
{
	if (pFifoMem == nullptr)
	{
		// This size is the library default even when the application
		// replaced the array: sizeof is fixed when the library is built.
		s_hAppEvtHandlerFifo = CFifoInit(g_AppEvtHandlerQueMem,
				sizeof(g_AppEvtHandlerQueMem),
				sizeof(AppEvtHandlerQue_t), true);
	}
	else if (Size < APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE)
	{
		return false;
	}
	else
	{
		s_hAppEvtHandlerFifo = CFifoInit(pFifoMem, Size,
				sizeof(AppEvtHandlerQue_t), true);
	}

	return s_hAppEvtHandlerFifo != nullptr;
}

bool AppEvtHandlerQue(uint32_t EvtId, void *pCtx, AppEvtHandler_t Handler)
{
	if (Handler == nullptr)
	{
		return false;
	}

	hCFifo_t hFifo = AppEvtHandlerFifo();

	// Interrupts and the application both queue here, and CFifoPut takes one
	// producer at a time. The entry is also filled before anything else can
	// run, so the application never takes a slot that is not filled yet.
	uint32_t state = DisableInterrupt();

	AppEvtHandlerQue_t *p = (AppEvtHandlerQue_t *)CFifoPut(hFifo);

	if (p != nullptr)
	{
		p->EvtId = EvtId;
		p->pCtx = pCtx;
		p->Handler = Handler;
	}

	EnableInterrupt(state);

	return p != nullptr;
}

// Keep the head owned until its event is copied. An IRQ producer can reuse
// the slot as soon as CFifoGet releases it, before the callback runs.
static bool AppEvtHandlerGet(AppEvtHandlerQue_t *pEvt)
{
	AppEvtHandlerQue_t *p =
		(AppEvtHandlerQue_t *)CFifoPeek(s_hAppEvtHandlerFifo);
	if (p != nullptr)
	{
		*pEvt = *p;
		CFifoGet(s_hAppEvtHandlerFifo);
	}

	return p != nullptr;
}

void AppEvtHandlerDispatch(void)
{
	if (s_hAppEvtHandlerFifo == nullptr)
	{
		return;
	}

	AppEvtHandlerQue_t evt;
	if (AppEvtHandlerGet(&evt) && evt.Handler != nullptr)
	{
		evt.Handler(evt.EvtId, evt.pCtx);
	}
}

bool AppEvtHandlerExec(void)
{
	if (s_hAppEvtHandlerFifo == nullptr)
	{
		return false;
	}

	int cnt = APPEVT_HANDLER_EXEC_MAX_COUNT;

	while (cnt-- > 0)
	{
		AppEvtHandlerQue_t evt;
		if (!AppEvtHandlerGet(&evt))
		{
			return false;
		}

		if (evt.Handler != nullptr)
		{
			evt.Handler(evt.EvtId, evt.pCtx);
		}
	}

	// Bounded drain: tell the caller whether events are still waiting.
	return CFifoPeek(s_hAppEvtHandlerFifo) != nullptr;
}
