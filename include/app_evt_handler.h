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

#ifdef __cplusplus
extern "C" {
#endif

/// Queue memory. The library has a weak definition holding
/// APPEVT_HANDLER_QUE_DEFAULT_SIZE events. An application that needs more
/// defines its own, which replaces the library one at link time, and passes
/// its size to AppEvtHandlerInit before enabling any event source:
///
///   alignas(4) uint8_t g_AppEvtHandlerQueMem[APPEVT_HANDLER_QUE_MEMSIZE(MY_COUNT)];
///   AppEvtHandlerInit(g_AppEvtHandlerQueMem, sizeof(g_AppEvtHandlerQueMem));
///
/// The library cannot see the size of the application array. Without the
/// AppEvtHandlerInit call it uses only the default size of it.
extern uint8_t g_AppEvtHandlerQueMem[];

/**
 * @brief	Initialize the queue on the given memory.
 *
 * Optional. Without this call the queue takes g_AppEvtHandlerQueMem with the
 * library default size the first time it is used. Events already queued are
 * dropped.
 *
 * @param	pFifoMem : Queue memory, NULL for g_AppEvtHandlerQueMem with the
 * 					   library default size
 * @param	Size	 : Total pFifoMem length in bytes
 *
 * @return	true - success
 */
bool AppEvtHandlerInit(uint8_t *pFifoMem, size_t Size);
/**
 * @brief	Queue an event handler call, first in first out.
 *
 * Safe from interrupts and from the application at the same time: the entry
 * is reserved and filled with the interrupts masked. A refused event is
 * recorded, and AppEvtHandlerExec calls AppCheckStatus once the queue is
 * empty so that the subsystem can queue it again.
 *
 * @return	true - queued
 * 			false - queue full
 */
bool AppEvtHandlerQue(uint32_t EvtId, void *pCtx, AppEvtHandler_t Handler);
void AppEvtHandlerDispatch(void);

/// True while an event is queued. Does not dispatch any callback.
bool AppEvtHandlerPending(void);

/**
 * @brief	Run queued events, first in first out, at most
 * 			APPEVT_HANDLER_EXEC_MAX_COUNT of them.
 *
 * Called by the application only, from its main loop or AppRun. A subsystem
 * never calls it: the queue is shared by every subsystem of the application.
 * When the queue is empty and an event was refused since the last check, it
 * calls AppCheckStatus, so a main loop only needs to call this function.
 *
 * @return	true - events are still waiting
 */
bool AppEvtHandlerExec(void);

/**
 * @brief	Check subsystem status and retry work refused by a full queue.
 *
 * Called with an empty event queue: by AppEvtHandlerExec after an event was
 * refused, and by AppRun before it waits. The weak default checks linked USB,
 * Bluetooth and LTE subsystems. An application may override it to select its
 * checks or add others and report whether its system is idle. Checks may
 * queue work; AppRun also checks AppEvtHandlerPending before waiting. This
 * function does not drain events.
 *
 * An RTOS calls each subsystem's status check from its owning worker after
 * draining work and before blocking on that worker's empty queue.
 *
 * @return	true - system idle, may sleep if no event is queued
 * 			false - work remains, continue running
 */
bool AppCheckStatus(void);

/**
 * @brief	Bare metal main loop. Never returns.
 *
 * Runs the queued events and waits for the next interrupt when the queue is
 * empty and AppCheckStatus reports idle. Every subsystem (Bluetooth, USB,
 * ...) hands its deferred work to the queue through its <Sub>EvtQue, so this
 * loop serves all of them. Work that needs immediate service is done in the
 * interrupt, not queued. An application with its own loop calls
 * AppEvtHandlerExec, which also retries refused work. Before its own wait it
 * calls AppCheckStatus and waits only when that returns true and
 * AppEvtHandlerPending is false.
 */
void AppRun(void);

/**
 * @brief	Wait for the next interrupt, called by AppRun when the queue is
 * 			empty.
 *
 * The library has a weak default (WFE). A port that must sleep through its
 * own call, such as the nRF52 SoftDevice, overrides it. An application can
 * override it for its own low power policy. It must return on any interrupt.
 */
void AppWait(void);

#ifdef __cplusplus
}
#endif

#endif // __APP_EVT_HANDLER_H__
