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

----------------------------------------------------------------------------*/
#include <memory.h>

#include "cfifo.h"
#include "app_evt_handler.h"

#define APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE	CFIFO_TOTAL_MEMSIZE(APPEVT_HANDLER_QUE_DEFAULT_SIZE, sizeof(AppEvtHandlerQue_t))

alignas(4) static uint8_t s_AppEvtHandlerFifoMem[APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE];

alignas(4) static HCFIFO s_hAppEvtHandlerFifo;

/**
 * @brief	Initialize event handler que
 *
 * Initialize fifo to be used to queue the event handlers. Pass NULL in pFifoMem
 * parameter to use library default APP_EVT_HANDLER_QUE_DEFAULT_SIZE.
 *
 * Use the macro APPEVT_HANDLER_QUE_MEMSIZE(Count) to calculate require memory size
 * for user queue size. Usage example for custom queue size of 10 :
 *
 * #define APPEVT_HANDLER_TOTAL_MEM_SIZE	APPEVT_HANDLER_QUE_MEMSIZE(10)
 *
 * static uint8_t s_Fifomem[APPEVT_HANDLER_TOTAL_MEM_SIZE];
 *
 * ...
 * if (AppEvtHandlerInit(s_Fifomem, APPEVT_HANDLER_TOTAL_MEM_SIZE) == false)
 * {
 *     printf("Failed mem size too small\n");
 * }
 * ...
 *
 * @param	pFifoMem: Pointer to Fifo memory block.
 * 						NULL - use library default
 * @param	Size	: Total memory size in bytes
 *
 * @return	true - success
 */
bool AppEvtHandlerInit(uint8_t *pFifoMem, size_t Size)
{
	if (pFifoMem == nullptr)
	{
    	s_hAppEvtHandlerFifo = CFifoInit(s_AppEvtHandlerFifoMem, APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE, sizeof(AppEvtHandlerQue_t), true);
	}
	else if (Size < APPEVT_HANDLER_QUE_CFIFO_DEFAULT_MEMSIZE)
	{
		return false;
	}
	else
	{
    	s_hAppEvtHandlerFifo = CFifoInit(pFifoMem, Size, sizeof(AppEvtHandlerQue_t), true);
	}

	return true;
}

/**
 * @brief	Add new event to que for execution
 *
 * @param	EvtId	: Event Id number
 * @param	pCtx	: Context data to pass to handler
 * @param 	Handler	: Event handler function to call
 *
 * @return	true - success. false - que full
 */
bool AppEvtHandlerQue(uint32_t EvtId, void *pCtx, AppEvtHandler_t Handler)
{
	AppEvtHandlerQue_t *p = (AppEvtHandlerQue_t*)CFifoPut(s_hAppEvtHandlerFifo);

	if (p)
	{
		p->EvtId = EvtId;
		p->pCtx = pCtx;
		p->Handler = Handler;

		return true;
	}

	return false;
}

/**
 * @brief	Execute next event
 *
 * @param	None
 *
 * @return	None
 */
void AppEvtHandlerExec()
{
	while (1)
	{
		AppEvtHandlerQue_t *p = (AppEvtHandlerQue_t*)CFifoGet(s_hAppEvtHandlerFifo);

		if (p == nullptr)
			break;
		p->Handler(p->EvtId, p->pCtx);
	}
}
