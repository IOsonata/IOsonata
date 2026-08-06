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

#ifndef APPEVT_HANDLER_QUE_DEFAULT_SIZE
#define APPEVT_HANDLER_QUE_DEFAULT_SIZE	4
#endif

#ifndef APPEVT_HANDLER_EXEC_MAX_COUNT
#define APPEVT_HANDLER_EXEC_MAX_COUNT	30	//!< Max events per AppEvtHandlerExec call
#endif

#ifndef APPEVT_HANDLER_IDLE_MAX_COUNT
#define APPEVT_HANDLER_IDLE_MAX_COUNT	4	//!< Max registered retry/idle pumps
#endif

#define APPEVT_HANDLER_QUE_MEMSIZE(Count)		CFIFO_TOTAL_MEMSIZE(Count, sizeof(AppEvtHandlerQue_t))

typedef void (*AppEvtHandler_t)(uint32_t Evt, void *pCtx);
typedef void (*AppEvtHandlerIdle_t)(void);

#pragma pack(push,4)

typedef struct __App_Event_Handler_Que {
	uint32_t EvtId;
	AppEvtHandler_t Handler;
	void *pCtx;
} AppEvtHandlerQue_t;

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

bool AppEvtHandlerInit(uint8_t *pFifoMem, size_t Size);
bool AppEvtHandlerQue(uint32_t EvtId, void *pCtx, AppEvtHandler_t Handler);
void AppEvtHandlerDispatch(void);
void AppEvtHandlerExec(void);

/**
 * @brief Register a main-loop retry/idle pump.
 *
 * The pump runs once after each AppEvtHandlerExec call, after queued handlers
 * have released their FIFO entries. Registering the same function twice is a
 * successful no-op. Registration is intended for initialization context.
 *
 * @return true when registered or already present; false on null or no slot.
 */
bool AppEvtHandlerIdleRegister(AppEvtHandlerIdle_t Handler);

#ifdef __cplusplus
}
#endif

#if defined(__cplusplus) && defined(STM32WBAxx_H) && \
	defined(HCI_VENDOR_SPECIFIC_DEBUG_EVT_CODE) && \
	defined(ACI_GATT_SERVER_CONFIRMATION_VSEVT_CODE)
#include "bluetooth/bt_wba_event_hook.h"
#endif

#endif // __APP_EVT_HANDLER_H__
