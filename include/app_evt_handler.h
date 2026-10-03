/**-------------------------------------------------------------------------
@file	app_evt_handler.h

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
#ifndef __APP_EVT_HANDLER_H__
#define __APP_EVT_HANDLER_H__

#include <stdint.h>
#include <stddef.h>

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include "cfifo.h"

#ifndef APPEVT_HANDLER_QUE_DEFAULT_SIZE
#define APPEVT_HANDLER_QUE_DEFAULT_SIZE	4
#endif

#ifndef APPEVT_HANDLER_EXEC_MAX_COUNT
#define APPEVT_HANDLER_EXEC_MAX_COUNT	30	//!< Max events per AppEvtHandlerExec call
#endif

#define APPEVT_HANDLER_QUE_MEMSIZE(Count)		CFIFO_TOTAL_MEMSIZE(Count, sizeof(AppEvtHandlerQue_t))

typedef void (*AppEvtHandler_t)(uint32_t Evt, void *pCtx);

#pragma pack(push,4)

typedef struct __App_Event_Handler_Que {
	uint32_t EvtId;
	AppEvtHandler_t Handler;
	void *pCtx;
} AppEvtHandlerQue_t;

#pragma pack(pop)

/// Queue memory descriptor
typedef struct __App_Event_Handler_Que_Cfg {
	uint8_t *pMem;			//!< Queue memory, 4 bytes aligned
	size_t Size;			//!< Total pMem length in bytes
} AppEvtHandlerQueCfg_t;

#ifdef __cplusplus
extern "C" {
#endif

/// Memory of the queue. The library defines a weak default holding
/// APPEVT_HANDLER_QUE_DEFAULT_SIZE events. An application that needs more
/// defines its own
///
///   alignas(4) static uint8_t s_QueMem[APPEVT_HANDLER_QUE_MEMSIZE(MY_COUNT)];
///   const AppEvtHandlerQueCfg_t g_AppEvtHandlerQueCfg = { s_QueMem, sizeof(s_QueMem) };
///
/// in which case the library default memory is not linked. The queue takes
/// this memory the first time it is used, no init call is required.
extern const AppEvtHandlerQueCfg_t g_AppEvtHandlerQueCfg;

/**
 * @brief	Initialize the queue on the given memory.
 *
 * Optional. Without this call the queue uses g_AppEvtHandlerQueCfg. Events
 * already queued are dropped.
 *
 * @param	pFifoMem : Queue memory, NULL to use g_AppEvtHandlerQueCfg
 * @param	Size	 : Total pFifoMem length in bytes
 *
 * @return	true - success
 */
bool AppEvtHandlerInit(uint8_t *pFifoMem, size_t Size);
bool AppEvtHandlerQue(uint32_t EvtId, void *pCtx, AppEvtHandler_t Handler);
void AppEvtHandlerDispatch(void);
void AppEvtHandlerExec(void);

#ifdef __cplusplus
}
#endif

#endif // __APP_EVT_HANDLER_H__
