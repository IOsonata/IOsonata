/**-------------------------------------------------------------------------
@file	evt_handler_que.cpp

@brief	Event handler queuing 


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
#include "evt_handler_que.h"

#define APPEVT_HANDLER_QUE_CFIFO_TOTAL_MEMSIZE	CFIFO_TOTAL_MEMSIZE(APP_EVT_HANDLER_QUE_MAX_SIZE, sizeof(AppEvtHandlerQue_t))

alignas(4) static uint8_t s_AppEvtHandlerFifoMem[APPEVT_HANDLER_QUE_CFIFO_TOTAL_MEMSIZE];

alignas(4) static HCFIFO s_hAppEvtHandlerFifo;

/**
 * @brief	Initialize event handler que
 *
 * @param	pCfg	: Config data
 *
 * @return	true - success
 */
bool EvtHandlerQueInit(AppEvtHandlerCfg_t const * pCfg)
{
	if (pCfg->pFifoMem == nullptr)
	{
    	s_hAppEvtHandlerFifo = CFifoInit(s_AppEvtHandlerFifoMem, APPEVT_HANDLER_QUE_CFIFO_TOTAL_MEMSIZE, sizeof(AppEvtHandlerQue_t), true);
	}
	else
	{
    	s_hAppEvtHandlerFifo = CFifoInit(pCfg->pFifoMem, pCfg->FifoMemSize, sizeof(AppEvtHandlerQue_t), true);
	}

	return true;
}

/**
 * @brief	Add new event to que for execution
 *
 * @param 	pEvtHandler : New event & hander to put in que
 *
 * @return	true - success. false - que full
 */
bool EvtHandlerQueAdd(AppEvtHandlerQue_t * pEvtHandler)
{
	AppEvtHandlerQue_t *p = (AppEvtHandlerQue_t*)CFifoPut(s_hAppEvtHandlerFifo);

	if (p)
	{
		memcpy(p, pEvtHandler, sizeof(AppEvtHandlerQue_t));

		return true;
	}

	return false;
}

AppEvtHandlerQue_t *EvtHandlerQueGet()
{
	return (AppEvtHandlerQue_t*)CFifoGet(s_hAppEvtHandlerFifo);
}

/**
 * @brief	Execute next event
 *
 * @param	None
 *
 * @return	None
 */
void EvtHandlerQueExec()
{
	AppEvtHandlerQue_t *p = (AppEvtHandlerQue_t*)CFifoGet(s_hAppEvtHandlerFifo);

	if (p)
	{
		p->Handler(p->EvtId, p->pCtx);
	}
}
