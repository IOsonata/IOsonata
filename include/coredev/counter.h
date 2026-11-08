/**-------------------------------------------------------------------------
@file	counter.h

@brief	Generic event counter

An event counter counts the edges of an input pin, or hardware events routed
to it with PEI. It does not count time: the count is a number of events.

On most MCUs the counter is a mode of a hardware timer instance, clocked by
its input instead of its internal clock (nRF TIMER in counter mode, STM32 TIM
in external clock mode, LPC CTIMER counter mode, SAMD TC event count). The
Counter and the Timer are then two objects on the same instance: CounterInit
and TimerInit each put the instance in their mode, the last one called has it.
DevNo is the timer DevNo of the instance, see the timer documentation of the
MCU port. Other MCUs have a dedicated pulse counter (ESP32-C6 PCNT), DevNo is
then its unit number.

DevNo -1 lets the driver choose an instance not used by a timer or a counter,
on the MCUs where any pin and any instance can be used together (nRF). Where
an instance counts only its own input pins (STM32, LPC, SAM4, Renesas), DevNo
is the instance of the pin and InputNo selects which of its inputs the pin is,
see the MCU port.

The driver keeps a 64 bit count, from 0 at init. It folds the hardware counter
into it in interrupt context, so it does not overflow while the application
does not read it. The hardware counter is 16 or 32 bits depending on the MCU
and the instance.

The limit is the period of the hardware counter. Every Limit events the
hardware counter restarts from 0 and generates the limit event, in hardware:
the limit event can be routed with PEI to the task of another peripheral with
no CPU involved, a frequency divider. The event handler runs in interrupt
context at each limit event. Limit 0 is no limit, the hardware counter then
wraps at its largest value and no event is reported. A limit beyond the
hardware counter, or below the smallest period it supports, is replaced by the
closest supported one: CounterSetLimit returns the limit in effect. The count
is folded at each limit or wrap, the counter interrupt runs then even without
an event handler.

Up counting only. Down counting, level control and quadrature decoding are
not part of this interface, see qdec.h for the latter.

@author	Hoang Nguyen Hoan
@date	Nov. 8, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#ifndef __COUNTER_H__
#define __COUNTER_H__

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "coredev/iopincfg.h"
#include "coredev/pei.h"

/** @addtogroup Coredev
  * @{
  */

/// What the counter counts
typedef enum __Counter_Src {
	COUNTER_SRC_PIN,			//!< Edges of the input pin of the configuration
	COUNTER_SRC_EVENT			//!< Hardware events connected to COUNTER_TASK_COUNT with PEI
} COUNTER_SRC;

/// Hardware tasks of the counter, for PEI
typedef enum __Counter_Task {
	COUNTER_TASK_COUNT,			//!< Count one event
	COUNTER_TASK_START,			//!< Start counting
	COUNTER_TASK_STOP			//!< Stop counting, the count is kept
} COUNTER_TASK;

typedef struct __Counter_Device CounterDev_t;

/**
 * @brief	Limit event handler
 *
 * Called in interrupt context at each limit event.
 *
 * @param	pDev	: Counter device data
 * @param	Count	: Count at the limit event
 */
typedef void (*CounterEvtHandler_t)(CounterDev_t * const pDev, uint64_t Count);

/// Counter configuration data
typedef struct __Counter_Config {
	int DevNo;					//!< Instance, -1 for any free one that can count the source
	COUNTER_SRC Src;			//!< What is counted
	IOPinCfg_t Pin;				//!< Input pin, COUNTER_SRC_PIN only. PinOp is the pin function
								//!< of the input where the MCU needs one (STM32 AF)
	int InputNo;				//!< Input of the instance the pin is, MCU dependent, 0 where any
								//!< pin can be counted (nRF)
	IOPINSENSE Edge;			//!< Edges counted: IOPINSENSE_HIGH_TRANSITION (rising),
								//!< IOPINSENSE_LOW_TRANSITION (falling) or IOPINSENSE_TOGGLE (both)
	uint32_t Limit;				//!< Events between two limit events, 0 for none
	int IntPrio;				//!< Interrupt priority
	CounterEvtHandler_t EvtHandler;	//!< Limit event handler, NULL for none
} CounterCfg_t;

/// Counter device data
struct __Counter_Device {
	int DevNo;					//!< Instance in use
	uint32_t Limit;				//!< Limit in effect, 0 for none
	CounterEvtHandler_t EvtHandler;	//!< Limit event handler
	volatile uint64_t Count;	//!< Events folded by the driver, read the count with CounterGetCount
	void *pDevData;				//!< Implementation data
};

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Configure the counter, it does not start it
 *
 * The count is cleared. A device already initialized on another instance
 * releases it first. Initializing an instance used by another counter device
 * takes it from that device, whose calls then fail.
 *
 * @param	pDev : Counter device data to initialize
 * @param	pCfg : Configuration data
 *
 * @return	true - success
 */
bool CounterInit(CounterDev_t * const pDev, const CounterCfg_t * const pCfg);

/**
 * @brief	Stop the counter and release its instance
 *
 * The input pin route is removed. The instance can be used by another
 * driver again.
 *
 * @param	pDev : Counter device data
 */
void CounterDeinit(CounterDev_t * const pDev);

/**
 * @brief	Start counting
 *
 * @param	pDev : Counter device data
 *
 * @return	true - success
 */
bool CounterStart(CounterDev_t * const pDev);

/**
 * @brief	Stop counting
 *
 * The count is kept, a later CounterStart continues from it. Events are not
 * counted while stopped.
 *
 * @param	pDev : Counter device data
 */
void CounterStop(CounterDev_t * const pDev);

/**
 * @brief	Number of events since init or the last clear
 *
 * @param	pDev : Counter device data
 *
 * @return	Count
 */
uint64_t CounterGetCount(CounterDev_t * const pDev);

/**
 * @brief	Clear the count
 *
 * The next limit event is Limit events from now.
 *
 * @param	pDev : Counter device data
 */
void CounterClear(CounterDev_t * const pDev);

/**
 * @brief	Set the limit
 *
 * The count is kept. When the events counted since the last limit event are
 * already past the new limit, the hardware period restarts from 0 at once.
 *
 * @param	pDev	: Counter device data
 * @param	Limit	: Events between two limit events, 0 for none
 *
 * @return	Limit in effect, the closest supported one, 0 for none
 */
uint32_t CounterSetLimit(CounterDev_t * const pDev, uint32_t Limit);

/**
 * @brief	Hardware task endpoint, for PEI
 *
 * @param	pDev : Counter device data
 * @param	Task : Task
 *
 * @return	Task endpoint, 0 when the MCU port or the instance does not have it
 */
PeiEndpoint_t CounterGetTaskEndpoint(CounterDev_t * const pDev, COUNTER_TASK Task);

/**
 * @brief	Hardware limit event endpoint, for PEI
 *
 * @param	pDev : Counter device data
 *
 * @return	Event endpoint, 0 when the MCU port or the instance does not have it
 */
PeiEndpoint_t CounterGetEventEndpoint(CounterDev_t * const pDev);

/**
 * @brief	Route a PEI event to a counter task
 *
 * @param	pDev : Counter device data
 * @param	pPei : PEI matrix
 * @param	Evt  : Event, bound in the matrix
 * @param	Task : Counter task
 *
 * @return	true - success
 */
static inline bool CounterConnectTask(CounterDev_t * const pDev, PeiDev_t *pPei, PeiEvent_t Evt,
									  COUNTER_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pDev == NULL || pPei == NULL)
	{
		return false;
	}
	endpoint = CounterGetTaskEndpoint(pDev, Task);
	if (endpoint == 0U)
	{
		return false;
	}

	return PeiConnectTask(pPei, Evt, endpoint);
}

/**
 * @brief	Remove the route of a PEI event to a counter task
 *
 * @param	pDev : Counter device data
 * @param	pPei : PEI matrix
 * @param	Evt  : Event
 * @param	Task : Counter task
 *
 * @return	true - success
 */
static inline bool CounterDisconnectTask(CounterDev_t * const pDev, PeiDev_t *pPei, PeiEvent_t Evt,
										 COUNTER_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pDev == NULL || pPei == NULL)
	{
		return false;
	}
	endpoint = CounterGetTaskEndpoint(pDev, Task);
	if (endpoint == 0U)
	{
		return false;
	}

	return PeiDisconnectTask(pPei, Evt, endpoint);
}

/**
 * @brief	Bind the limit event to PEI
 *
 * The limit event handler, if any, still runs.
 *
 * @param	pDev : Counter device data
 * @param	pPei : PEI matrix
 * @param	pEvt : Event identity, 0 for a new one, the one bound on return
 *
 * @return	true - success
 */
static inline bool CounterEnablePeiEvent(CounterDev_t * const pDev, PeiDev_t *pPei, PeiEvent_t *pEvt)
{
	PeiEndpoint_t endpoint;

	if (pDev == NULL || pPei == NULL || pEvt == NULL)
	{
		return false;
	}
	endpoint = CounterGetEventEndpoint(pDev);
	if (endpoint == 0U)
	{
		return false;
	}

	return PeiBindEvent(pPei, pEvt, endpoint);
}

/**
 * @brief	Unbind the limit event from PEI
 *
 * @param	pDev : Counter device data
 * @param	pPei : PEI matrix
 * @param	Evt  : Event bound with CounterEnablePeiEvent
 *
 * @return	true - success
 */
static inline bool CounterDisablePeiEvent(CounterDev_t * const pDev, PeiDev_t *pPei, PeiEvent_t Evt)
{
	PeiEndpoint_t endpoint;

	if (pDev == NULL || pPei == NULL)
	{
		return false;
	}
	endpoint = CounterGetEventEndpoint(pDev);
	if (endpoint == 0U)
	{
		return false;
	}

	return PeiUnbindEvent(pPei, Evt, endpoint);
}

#ifdef __cplusplus
}

/// Event counter class
class Counter {
public:
	virtual ~Counter() {}

	/**
	 * @brief	Configure the counter, it does not start it
	 *
	 * @param	Cfg : Configuration data
	 *
	 * @return	true - success
	 */
	virtual bool Init(const CounterCfg_t &Cfg) { return CounterInit(&vDev, &Cfg); }

	/**
	 * @brief	Stop the counter and release its instance
	 */
	virtual void Deinit() { CounterDeinit(&vDev); }

	/**
	 * @brief	Start counting
	 *
	 * @return	true - success
	 */
	virtual bool Start() { return CounterStart(&vDev); }

	/**
	 * @brief	Stop counting, the count is kept
	 */
	virtual void Stop() { CounterStop(&vDev); }

	/**
	 * @brief	Number of events since init or the last clear
	 *
	 * @return	Count
	 */
	virtual uint64_t Count() { return CounterGetCount(&vDev); }

	/**
	 * @brief	Clear the count
	 */
	virtual void Clear() { CounterClear(&vDev); }

	/**
	 * @brief	Set the limit
	 *
	 * @param	Limit : Events between two limit events, 0 for none
	 *
	 * @return	Limit in effect
	 */
	virtual uint32_t Limit(uint32_t Limit) { return CounterSetLimit(&vDev, Limit); }

	/**
	 * @brief	Limit in effect
	 *
	 * @return	Limit, 0 for none
	 */
	virtual uint32_t Limit() { return vDev.Limit; }

	/**
	 * @brief	Route a PEI event to a counter task
	 *
	 * @param	pPei : PEI matrix
	 * @param	Evt  : Event, bound in the matrix
	 * @param	Task : Counter task
	 *
	 * @return	true - success
	 */
	bool ConnectTask(PeiDev_t *pPei, PeiEvent_t Evt, COUNTER_TASK Task) {
		return CounterConnectTask(&vDev, pPei, Evt, Task);
	}

	/**
	 * @brief	Remove the route of a PEI event to a counter task
	 *
	 * @param	pPei : PEI matrix
	 * @param	Evt  : Event
	 * @param	Task : Counter task
	 *
	 * @return	true - success
	 */
	bool DisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt, COUNTER_TASK Task) {
		return CounterDisconnectTask(&vDev, pPei, Evt, Task);
	}

	/**
	 * @brief	Bind the limit event to PEI
	 *
	 * @param	pPei : PEI matrix
	 * @param	Evt  : Event identity, 0 for a new one, the one bound on return
	 *
	 * @return	true - success
	 */
	bool EnablePeiEvent(PeiDev_t *pPei, PeiEvent_t &Evt) { return CounterEnablePeiEvent(&vDev, pPei, &Evt); }

	/**
	 * @brief	Unbind the limit event from PEI
	 *
	 * @param	pPei : PEI matrix
	 * @param	Evt  : Event bound with EnablePeiEvent
	 *
	 * @return	true - success
	 */
	bool DisablePeiEvent(PeiDev_t *pPei, PeiEvent_t Evt) { return CounterDisablePeiEvent(&vDev, pPei, Evt); }

	operator CounterDev_t * () { return &vDev; }

private:
	CounterDev_t vDev;
};

#endif

/** @} */

#endif
