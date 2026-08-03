/**-------------------------------------------------------------------------
@file	app_evt_handler.cpp

@brief	Event handler queuing

This is an implementation of event handler queuing to schedule event handler
in firmware application main loop.

AppEvtHandlerInit must be called first to initialize the queue before any other
function can be used

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

#define APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE \
	CFIFO_TOTAL_MEMSIZE(APPEVT_HANDLER_QUE_DEFAULT_SIZE, sizeof(AppEvtHandlerQue_t))

alignas(4) static uint8_t s_AppEvtHandlerFifoMem[
	APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE];

static hCFifo_t s_hAppEvtHandlerFifo;
static AppEvtHandlerIdle_t s_IdleHandler[APPEVT_HANDLER_IDLE_MAX_COUNT];
static uint8_t s_IdleHandlerCount;

bool AppEvtHandlerInit(uint8_t *pFifoMem, size_t Size)
{
	if (pFifoMem == nullptr)
	{
		s_hAppEvtHandlerFifo = CFifoInit(s_AppEvtHandlerFifoMem,
				APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE,
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
	if (s_hAppEvtHandlerFifo == nullptr || Handler == nullptr)
	{
		return false;
	}

	AppEvtHandlerQue_t *p =
		(AppEvtHandlerQue_t *)CFifoPut(s_hAppEvtHandlerFifo);

	if (p != nullptr)
	{
		p->EvtId = EvtId;
		p->pCtx = pCtx;
		p->Handler = Handler;
		return true;
	}

	return false;
}

bool AppEvtHandlerIdleRegister(AppEvtHandlerIdle_t Handler)
{
	if (Handler == nullptr)
	{
		return false;
	}

	for (uint8_t i = 0; i < s_IdleHandlerCount; i++)
	{
		if (s_IdleHandler[i] == Handler)
		{
			return true;
		}
	}

	if (s_IdleHandlerCount >= APPEVT_HANDLER_IDLE_MAX_COUNT)
	{
		return false;
	}

	s_IdleHandler[s_IdleHandlerCount++] = Handler;
	return true;
}

void AppEvtHandlerDispatch(void)
{
	if (s_hAppEvtHandlerFifo == nullptr)
	{
		return;
	}

	AppEvtHandlerQue_t *p =
		(AppEvtHandlerQue_t *)CFifoGet(s_hAppEvtHandlerFifo);

	if (p != nullptr && p->Handler != nullptr)
	{
		p->Handler(p->EvtId, p->pCtx);
	}
}

void AppEvtHandlerExec(void)
{
	if (s_hAppEvtHandlerFifo != nullptr)
	{
		int cnt = APPEVT_HANDLER_EXEC_MAX_COUNT;

		while (cnt-- > 0)
		{
			AppEvtHandlerQue_t *p =
				(AppEvtHandlerQue_t *)CFifoGet(s_hAppEvtHandlerFifo);

			if (p == nullptr)
			{
				break;
			}

			if (p->Handler != nullptr)
			{
				p->Handler(p->EvtId, p->pCtx);
			}
		}
	}

	// Queued handlers have released their FIFO slots. Deferred subsystems get
	// one chance to enqueue work that previously lost a race with a full queue.
	for (uint8_t i = 0; i < s_IdleHandlerCount; i++)
	{
		s_IdleHandler[i]();
	}
}
