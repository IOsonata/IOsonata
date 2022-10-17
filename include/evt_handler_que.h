/**-------------------------------------------------------------------------
@file	evt_handler_que.h

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
#ifndef __EVT_HANDLER_QUE_H__
#define __EVT_HANDLER_QUE_H__

#ifndef APP_EVT_HANDLER_QUE_MAX_SIZE
#define APP_EVT_HANDLER_QUE_MAX_SIZE	4
#endif

#define APPEVT_HANDLER_QUE_FIFO_MEMSIZE(Count)		CFIFO_TOTAL_MEMSIZE(Count, sizeof(AppEvtHandlerQue_t))

typedef void (*AppEvtHandler_t)(uint32_t Evt, void *pCtx);

#pragma pack(push,4)

typedef struct __App_Event_Handler_Que {
	uint32_t EvtId;
	AppEvtHandler_t Handler;
	void *pCtx;
} AppEvtHandlerQue_t;


typedef struct __App_Event_Handler_Cfg {
	uint8_t *pFifoMem;
	int FifoMemSize;
} AppEvtHandlerCfg_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Initialize event handler que
 *
 * @param	pCfg	: Config data
 *
 * @return	true - success
 */
bool EvtHandlerQueInit(AppEvtHandlerCfg_t const * pCfg);

/**
 * @brief	Add new event to que for execution
 *
 * @param 	pEvtHandler : New event & hander to put in que
 *
 * @return	true - success. false - que full
 */
bool EvtHandlerQueAdd(AppEvtHandlerQue_t * pEvtHandler);

/**
 * @brief	Execute next event
 *
 * @param	None
 *
 * @return	None
 */
void EvtHandlerQueExec();

#ifdef __cplusplus
}
#endif

#endif // __EVT_HANDLER_QUE_H__
