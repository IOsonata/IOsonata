/**-------------------------------------------------------------------------
@file	timer.h

@brief	Generic timer class

This generic implementation combines all available core timer in to one
device list. Each timer is indexed by DevNo starting from 0..max. Indexing
low frequency timer to high frequency.

For example MCU having 2 low frequency timers and 4 high frequency timers
are indexed as follow:

DevNo : 0..1 are low frequency timers
DevNo : 2..5 are high frequency timers

There are 2 ways to determine the start DevNo for high frequency timer

int HFreqDevNoStart = TimerGetLowFreqDevCount();
int HFreqDevNoStart = TimerGetHighFreqDevNo();

@author	Hoang Nguyen Hoan
@date	Sep. 7, 2017

@license

MIT License

Copyright (c) 2017 I-SYST inc. All rights reserved.

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

#ifndef __TIMER_H__
#define __TIMER_H__

#include <stdint.h>
#include <string.h>
#ifndef __cplusplus
#include <stdbool.h>
#endif
#include "coredev/pei.h"

/// Clock source used for the timer
typedef enum __Timer_Clock_Src {
	TIMER_CLKSRC_DEFAULT,	//!< Device default clock source (auto select best src)
	TIMER_CLKSRC_LFRC,		//!< Low frequency internal RC oscillator
    TIMER_CLKSRC_HFRC,		//!< High frequency internal RC oscillator
    TIMER_CLKSRC_LFXTAL,	//!< Low frequency crystal
    TIMER_CLKSRC_HFXTAL,	//!< High frequency crystal
	TIMER_CLKSRC_EXT		//!< External clock signal
} TIMER_CLKSRC;

/// Timer interrupt enable type
typedef enum __Timer_Interrupt_Enable {
	TIMER_INTEN_NONE,		//!< No interrupt
	TIMER_INTEN_TICK,		//!< Enable tick count interrupt
	TIMER_INTEN_OVR			//!< Enable tick count overflow interrupt
} TIMER_INTEN;

/// Timer trigger type
typedef enum __Timer_Trigger_Type {
    TIMER_TRIG_TYPE_SINGLE,		//!< Single shot trigger
    TIMER_TRIG_TYPE_CONTINUOUS	//!< Continuous trigger
} TIMER_TRIG_TYPE;

// Hardware timer tasks. Timer owns the task endpoint; PEI connects events
// to that endpoint without changing timer configuration.
// TIMER_TASK_CLEAR from the CPU (ExecuteTask) is a Reset: the tick count
// restarts and the triggers restart their period. Once its endpoint is
// given out, a hardware event can clear the counter: the tick count then
// restarts from each clear, a counter lower than the last read being taken
// as a clear, not as a wrap.
typedef enum __Timer_Hardware_Task {
	TIMER_TASK_START,
	TIMER_TASK_STOP,
	TIMER_TASK_CLEAR,
	TIMER_TASK_TRIGOVRFLW
} TIMER_TASK;

typedef enum __Timer_ExtTrig_Sense {
	TIMER_EXTTRIG_SENSE_DISABLE,			//!< Disable external trig sense
	TIMER_EXTTRIG_SENSE_LOW_TRANSITION,		//!< Event on falling edge
	TIMER_EXTTRIG_SENSE_HIGH_TRANSITION,	//!< Event on raising edge
	TIMER_EXTTRIG_SENSE_TOGGLE				//!< Event on state change
} TIMER_EXTTRIG_SENSE;

#define TIMER_EVT_TICK                          (1<<0)	//!< Timer tick counter event
#define TIMER_EVT_COUNTER_OVR                   (1<<1)	//!< Timer overflow event
#define TIMER_EVT_EXTTRIG						(1<<2)	//!< External timer trigger event
#define TIMER_EVT_TRIGGER0                		(1<<3)	//!< Periodic timer event start at this bit

#define TIMER_EVT_TRIGGER(n)              		(1<<(n+3))	//!< Trigger event id

typedef struct __Timer_Device	TimerDev_t;
typedef TimerDev_t		TIMER;

/**
 * @brief	Timer event handler type.
 *
 * @param	Timer	: Pointer reference to Timer class generating the event
 * @param	Evt		: Event ID for which this callback is activated
 */
//typedef void (*TIMER_EVTCB)(Timer * const pTimer, uint32_t Evt);
typedef void (*TimerEvtHandler_t)(TimerDev_t * const pTimer, uint32_t Evt);

/**
 * @brief	Timer trigger handler type
 *
 * @param	Timer	: Pointer reference to Timer class generating the event
 * @param	TrigNo	: Trigger ID for which this callback is activated
 * @param   pContext: Pointer to user context (user private data, could be a class or structure)
 */
//typedef void (*TIMER_TRIGCB)(Timer * const pTimer, int TrigNo, void * const pContext);
typedef void (*TimerTrigEvtHandler_t)(TimerDev_t * const pTimer, int TrigNo, void * const pContext);

#pragma pack(push, 4)

typedef struct __Timer_Trigger_Info {
	TIMER_TRIG_TYPE Type;	//!< Trigger type
	uint64_t nsPeriod;		//!< Trigger period in nanosecond
	TimerTrigEvtHandler_t Handler;	//!< Trigger event callback
	void *pContext;         //!<< Pointer to user private data to be passed to callback
} TimerTrig_t;

typedef TimerTrig_t		TIMER_TRIGGER;

/// @brief	Timer configuration data.
///
/// NOTE : Interrupt priority should be as high as possible
/// Timing precision would be lost if other interrupt preempt
/// the timer interrupt.  Adjust IntPrio base on requirement
///
typedef struct __Timer_Config {
    int             DevNo;      //!< Device number. Usually is the timer number indexed at 0
    TIMER_CLKSRC    ClkSrc;     //!< Clock source.  Not all timer allows user select clock source
    uint32_t        Freq;       //!< Frequency in Hz, 0 - to auto select max timer frequency
    int             IntPrio;    //!< Interrupt priority. recommended to use highest
    							//!< priority if precision timing is required
    TimerEvtHandler_t EvtHandler; //!< Interrupt handler
    bool			bTickInt;	//!< Enable tick interrupt. Use with caution, could cause crashes due to excessive interrupts
} TimerCfg_t;

typedef TimerCfg_t	TIMER_CFG;

struct __Timer_Device {
    int DevNo;				//!< Timer device number.
	uint32_t Freq;			//!< Frequency in Hz
	uint64_t nsPeriod;		//!< Period in nsec
	uint64_t Rollover;     	//!< Rollover counter adjustment
	uint32_t LastCount;		//!< Last counter read value
	TimerEvtHandler_t EvtHandler;	//!< Pointer to user event handler callback
	void *pObj;				//!< Pointer reference to Timer class

	void (*Disable)(TimerDev_t * const pTimerDev);
	bool (*Enable)(TimerDev_t * const pTimerDev);
	void (*Reset)(TimerDev_t * const pTimerDev);

	/**
     * @brief   Get the current tick count.
     *
     * This function read the tick count with compensated overflow roll over
     *
     * @return  Total tick count since last reset
     */
	uint64_t (*GetTickCount)(TimerDev_t * const pTimerDev);

	/**
	 * @brief	Set timer main frequency.
	 *
	 * This function allows dynamically changing the timer frequency.  Timer
	 * will be reset and restarted with new frequency
	 *
	 * @param 	Freq : Frequency in Hz
	 *
	 * @return  Real frequency
	 */
	uint32_t (*SetFrequency)(TimerDev_t * const pTimerDev, uint32_t Freq);

	/**
	 * @brief	Get maximum available timer trigger event for the timer.
	 *
	 * @return	count
	 */
	int (*GetMaxTrigger)(TimerDev_t * const pTimerDev);

	/**
	 * @brief	Get first available timer trigger index.
	 *
	 * This function returns the first available timer trigger to be used to with
	 * EnableTimerTrigger
	 *
	 * @return	success : Timer trigger index
	 * 			fail : -1
	 */
	int (*FindAvailTrigger)(TimerDev_t * const pTimerDev);

	/**
	 * @brief   Disable timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to disable. Index value starting at 0
	 */
	void (*DisableTrigger)(TimerDev_t * const pTimer, int TrigNo);

    /**
	 * @brief	Enable a specific nanosecond timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   nsPeriod : Trigger period in nsec.
	 * @param   Type     : Trigger type single shot or continuous. A single
	 * 					   shot trigger is free again once it fired.
	 * @param	Handler	 : Optional Timer trigger user callback
	 * @param   pContext : Optional pointer to user private data to be passed
	 *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  real period in nsec based on clock calculation
	 */
	uint64_t (*EnableTrigger)(TimerDev_t * const pTimerDev, int TrigNo, uint64_t nsPeriod,
							  TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
							  void * const pContext);

	// Optional hardware task support. Resolving an endpoint does not
	// execute the task. ExecuteTask runs the same action from the CPU.
	uintptr_t (*GetTaskEndpoint)(TimerDev_t * const pTimerDev, TIMER_TASK Task);
	bool (*ExecuteTask)(TimerDev_t * const pTimerDev, TIMER_TASK Task);

	/**
	 * @brief   Disable timer trigger event.
	 *
     * @param   pTimerDev  : Trigger number to disable. Index value starting at 0
	 */
	void (*DisableExtTrigger)(TimerDev_t * const pTimerDev);

    /**
	 * @brief	Enable external timer trigger event.
	 *
	 * @param   TrigDevNo : External trigger device number to enable. Index value starting at 0
	 * @param	Sense : External level sense
	 *
	 * @return  true - Success
	 */
	bool (*EnableExtTrigger)(TimerDev_t * const pTimerDev, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense);
};

#pragma pack(pop)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief   Timer initialization.
 *
 * This is a required implementation specific to each architecture.
 *
 * @param	pTimer	: Pointer to Timer device private data (timer handle)
 * @param	pCfg	: Pointer timer configuration data.
 *
 * @return
 * 			- true 	: Success
 * 			- false : Otherwise
 */
bool TimerInit(TimerDev_t * const pTimer, const TimerCfg_t * const pCfg);

int TimerGetLowFreqDevCount(void);
int TimerGetHighFreqDevCount(void);
int TimerGetHighFreqDevNo(void);

/**
 * @brief   Turn off timer.
 *
 * This is used to disable timer for power saving. Call Enable() to
 * re-enable timer instead of full initialization sequence
 *
 * @param	pTimer	: Pointer to Timer device private data (timer handle)
 */
static inline void TimerDisable(TimerDev_t * const pTimer) { pTimer->Disable(pTimer); }

/**
 * @brief   Turn on timer.
 *
 * This is used to re-enable timer after it was disabled for power
 * saving.  It normally does not go through full initialization sequence
 *
 * @param	pTimer	: Pointer to Timer device private data (timer handle)
 *
 * @return	true : Success
 */
static inline bool TimerEnable(TimerDev_t * const pTimer)  { return pTimer->Enable(pTimer); }

/**
 * @brief   Reset timer.
 *
 * @param	pTimer	: Pointer to Timer device private data (timer handle)
 */
static inline void TimerReset(TimerDev_t * const pTimer) { pTimer->Reset(pTimer); }
static inline int TimerGetMaxTrigger(TimerDev_t * const pTimer) { return pTimer->GetMaxTrigger(pTimer); }
static inline uint64_t TimerGetTickCount(TimerDev_t * const pTimer) { return pTimer->GetTickCount(pTimer); }
static inline uint32_t TimerGetFrequency(TimerDev_t * const pTimer) { return pTimer->Freq; }
static inline uint32_t TimerSetFrequency(TimerDev_t * const pTimer, uint32_t Freq) {
	return pTimer->SetFrequency(pTimer, Freq);
}
/**
 * @brief   Disable timer trigger event.
 *
 * @param	pTimer	: Pointer to Timer device private data (timer handle)
 * @param   TrigNo : Trigger number to disable. Index value starting at 0
 */
static inline void TimerDisableTrigger(TimerDev_t * const pTimer, int TrigNo) { pTimer->DisableTrigger(pTimer, TrigNo); }

// TIMER and RTC PEI operations are implemented by the MCU driver.
int TimerEnablePeiTrigger(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvt, uint32_t msPeriod, TIMER_TRIG_TYPE Type);
bool TimerEnablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t *pEvt, uint32_t Event);
bool TimerDisablePeiEvent(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Evt);

/** @brief Route an event to a Timer hardware START or STOP task. */
static inline bool TimerConnectTask(TimerDev_t *pTimer, PeiDev_t *pPei,
	PeiEvent_t Evt, TIMER_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pTimer == NULL || pPei == NULL || pTimer->GetTaskEndpoint == NULL)
	{
		return false;
	}
	endpoint = pTimer->GetTaskEndpoint(pTimer, Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiConnectTask(pPei, Evt, endpoint);
}

/** @brief Execute a Timer START/STOP hardware task without clock release. */
static inline bool TimerExecuteTask(TimerDev_t *pTimer, TIMER_TASK Task)
{
	if (pTimer == NULL || pTimer->ExecuteTask == NULL)
	{
		return false;
	}
	return pTimer->ExecuteTask(pTimer, Task);
}
static inline uint64_t nsTimerEnableTrigger(TimerDev_t * const pTimer, int TrigNo, uint64_t nsPeriod,
										 	TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
											void * const pContext) {
	return pTimer->EnableTrigger(pTimer, TrigNo, nsPeriod, Type, Handler, pContext);
}
static inline uint64_t msTimerEnableTrigger(TimerDev_t * const pTimer,  int TrigNo, uint32_t msPeriod,
										 	TIMER_TRIG_TYPE Type, TimerTrigEvtHandler_t const Handler,
											void * const pContext) {
	return (uint32_t)((uint64_t )pTimer->EnableTrigger(pTimer, TrigNo, (uint64_t)((uint64_t)msPeriod * 1000000ULL),
													   Type, Handler, pContext) / 1000000ULL);
}

/**
 * @brief   Disable timer trigger event.
 *
 * @param   pTimerDev   : Pointer to Timer device private data (timer handle)
 */
static inline void TimerDisableExtTrigger(TimerDev_t * const pTimerDev) { pTimerDev->DisableExtTrigger(pTimerDev); }

/**
 * @brief	Enable external timer trigger event.
 *
 * @param	pTimerDev   : Pointer to Timer device private data (timer handle)
 * @param   TrigDevNo : External trigger device number to enable. Index value starting at 0
 *
 * @return  true - Success
 */
static inline bool TimerEnableExtTrigger(TimerDev_t * const pTimerDev, int TrigDevNo, TIMER_EXTTRIG_SENSE Sense) {
	return pTimerDev->EnableExtTrigger(pTimerDev, TrigDevNo, Sense);
}

/**
 * @brief	Convert a tick count to time.
 *
 * The conversion is done from the tick frequency, so it is exact for any
 * frequency. nsPeriod is not used for it: a tick period is rarely a whole
 * number of nanoseconds (62.5 ns at 16 MHz, 30517.58 ns at 32768 Hz) and the
 * rounding would add up with the count.
 *
 * @param   pTimerDev	: Pointer to Timer device private data (timer handle)
 * @param   Count		: Timer tick count value
 * @param	UnitPerSec	: Number of result units in one second, 1000 for msec,
 * 						  1000000 for usec, 1000000000 for nsec
 *
 * @return  Converted count, rounded down
 */
static inline uint64_t TimerTickToTime(TimerDev_t * const pTimerDev, uint64_t Count, uint32_t UnitPerSec) {
	uint64_t f = pTimerDev->Freq;

	if (f == 0)
	{
		// Timer not initialized
		return 0;
	}

	// Whole seconds and the rest apart, the product would overflow otherwise
	return (Count / f) * UnitPerSec + (Count % f) * UnitPerSec / f;
}

/**
 * @brief	Convert a time in nanosecond to tick count.
 *
 * Exact for any tick frequency, see TimerTickToTime.
 *
 * @param   pTimerDev	: Pointer to Timer device private data (timer handle)
 * @param   nsTime		: Time in nsec
 *
 * @return  Tick count, rounded to the nearest tick
 */
static inline uint64_t TimerNanosecondToTick(TimerDev_t * const pTimerDev, uint64_t nsTime) {
	uint64_t f = pTimerDev->Freq;

	// Whole seconds and the rest apart, the product would overflow otherwise
	return (nsTime / 1000000000ULL) * f + ((nsTime % 1000000000ULL) * f + 500000000ULL) / 1000000000ULL;
}

/**
 * @brief   Get current timer counter in millisecond.
 *
 * This function return the current timer in msec since last reset.
 *
 * @param   pTimerDev  : Pointer to Timer device private data (timer handle)
 * @return  Counter in millisecond
 */
static inline uint32_t TimerGetMilisecond(TimerDev_t * const pTimerDev) {
	return (uint32_t)TimerTickToTime(pTimerDev, pTimerDev->GetTickCount(pTimerDev), 1000UL);
}

/**
 * @brief   Convert tick count to millisecond.
 *
 * @param   pTimerDev : Pointer to Timer device private data (timer handle)
 * @param   Count : Timer tick count value
 *
 * @return  Converted count in millisecond
 */
static inline uint32_t TimerTickToMilisecond(TimerDev_t * const pTimerDev, uint64_t Count) {
	return (uint32_t)TimerTickToTime(pTimerDev, Count, 1000UL);
}

/**
 * @brief   Get current timer counter in microsecond.
 *
 * This function return the current timer in usec since last reset.
 *
 * @param   pTimerDev  : Pointer to Timer device private data (timer handle)
 *
 * @return  Counter in microsecond
 */
static inline uint32_t TimerGetMicrosecond(TimerDev_t * const pTimerDev) {
	return (uint32_t)TimerTickToTime(pTimerDev, pTimerDev->GetTickCount(pTimerDev), 1000000UL);
}

/**
 * @brief   Convert tick count to microsecond.
 *
 * @param   Count : Timer tick count value
 *
 * @param   pTimerDev  : Pointer to Timer device private data (timer handle)
 *
 * @return  Converted count in microsecond
 */
static inline uint32_t TimerTickToMicrosecond(TimerDev_t * const pTimerDev, uint64_t Count) {
	return (uint32_t)TimerTickToTime(pTimerDev, Count, 1000000UL);
}

/**
 * @brief   Get current timer counter in nanosecond.
 *
 * This function return the current timer in nsec since last reset.
 *
 * @param   pTimerDev  : Pointer to Timer device private data (timer handle)
 *
 * @return  Counter in nanosecond
 */
static inline uint64_t TimerGetNanosecond(TimerDev_t * const pTimerDev) {
	return TimerTickToTime(pTimerDev, pTimerDev->GetTickCount(pTimerDev), 1000000000UL);
}

/**
 * @brief   Convert tick count to nanosecond
 *
 * @param   pTimerDev  : Pointer to Timer device private data (timer handle)
 * @param   Count : Timer tick count value
 *
 * @return  Converted count in nanosecond
 */
static inline uint64_t TimerTickToNanosecond(TimerDev_t * const pTimerDev, uint64_t Count) {
	return TimerTickToTime(pTimerDev, Count, 1000000000UL);
}

#ifdef __cplusplus
}

/// Timer base class
class Timer {
public:

	Timer()
	{
		vTimer.pObj = this;
		vTimer.GetTaskEndpoint = NULL;
		vTimer.ExecuteTask = NULL;
	}
	virtual ~Timer() {}

	virtual operator TimerDev_t * () { return &vTimer; }

    /**
     * @brief   Timer initialization.
     *
     * This is specific to each architecture.
     *
     * @param	Cfg	: Timer configuration data.
     *
     * @return
     * 			- true 	: Success
     * 			- false : Otherwise
     */
    virtual bool Init(const TimerCfg_t &Cfg)
	{
		vTimer.GetTaskEndpoint = NULL;
		vTimer.ExecuteTask = NULL;
		return TimerInit(&vTimer, &Cfg);
	}

    /**
     * @brief   Turn on timer.
     *
     * This is used to re-enable timer after it was disabled for power
     * saving.  It normally does not go through full initialization sequence
     */
    virtual bool Enable() { return vTimer.Enable(&vTimer); }

    /**
     * @brief   Turn off timer.
     *
     * This is used to disable timer for power saving. Call Enable() to
     * re-enable timer instead of full initialization sequence
     */
    virtual void Disable() { vTimer.Disable(&vTimer); }

    /**
     * @brief   Reset timer.
     */
    virtual void Reset() { vTimer.Reset(&vTimer); }

    /**
     * @brief   Get the current tick count.
     *
     * This function read the tick count with compensated overflow roll over
     *
     * @return  Total tick count since last reset
     */
	virtual uint64_t TickCount() { return vTimer.GetTickCount(&vTimer); }

	/**
	 * @brief   Get current timer frequency
	 *
	 * @return  Timer frequency in Hz
	 */
	virtual uint32_t Frequency(void) { return vTimer.Freq; }

	/**
	 * @brief	Set timer main frequency.
	 *
	 * This function allows dynamically changing the timer frequency.  Timer
	 * will be reset and restarted with new frequency
	 *
	 * @param 	Freq : Frequency in Hz
	 *
	 * @return  Real frequency
	 */
	virtual uint32_t Frequency(uint32_t Freq) { return vTimer.SetFrequency(&vTimer, Freq); }

	/**
	 * @brief	Get maximum available timer trigger event for the timer.
	 *
	 * @return	count
	 */
	int MaxTimerTrigger() { return vTimer.GetMaxTrigger(&vTimer); }

	/**
	 * @brief	Enable a specific nanosecond timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   nsPeriod : Trigger period in nsec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 * @param   pContext : Optional pointer to user private data to be passed
	 *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  real period in nsec based on clock calculation
	 */
	virtual uint64_t EnableTimerTrigger(int TrigNo, uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
										TimerTrigEvtHandler_t const Handler = NULL, void * const pContext = NULL) {
		return vTimer.EnableTrigger(&vTimer, TrigNo, nsPeriod, Type, Handler, pContext);
	}

    /**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   msPeriod : Trigger period in msec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 * @param   pContext : Optional pointer to user private data to be passed
	 *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  real period in msec based on clock calculation
	 */
	virtual uint32_t EnableTimerTrigger(int TrigNo, uint32_t msPeriod, TIMER_TRIG_TYPE Type,
										TimerTrigEvtHandler_t const Handler = NULL, void * const pContext = NULL)
	{
		return (uint32_t)((uint64_t )EnableTimerTrigger(TrigNo, (uint64_t)((uint64_t)msPeriod * 1000000ULL), Type, Handler, pContext) / 1000000ULL);
	}

	/**
	 * @brief	Enable nanosecond timer trigger event.
	 *
	 * @param   nsPeriod : Trigger period in nsec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
     * @param   pContext : Optional pointer to user private data to be passed
     *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  Timer trigger ID on success
	 * 			-1 : Failed
	 */
	virtual int EnableTimerTrigger(uint64_t nsPeriod, TIMER_TRIG_TYPE Type,
								   TimerTrigEvtHandler_t const Handler = NULL, void * const pContext = NULL);

	/**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * @param   msPeriod : Trigger period in msec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
     * @param   pContext : Optional pointer to user private data to be passed
     *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  Timer trigger ID on success
	 * 			-1 : Failed
	 */
	int EnableTimerTrigger(uint32_t msPeriod, TIMER_TRIG_TYPE Type,
						   TimerTrigEvtHandler_t const Handler = NULL, void * const pContext = NULL);

	/**
	 * @brief Prepare a Timer compare event source for PEI.
	 *
	 * Timer owns CC configuration and continuous compare rescheduling.
	 * PEI owns hardware event-to-task routing, with no ISR forwarding.
	 * The elapsed counter is never stopped or cleared by a compare.
	 * TrigNo is returned for DisableTimerTrigger. Unsupported hardware
	 * configurations return -1.
	 *
	 * @return Allocated trigger number, or -1 on failure
	 */
	int EnableTimerTrigger(Pei &PeiDev, PeiEvent_t &Evt, uint32_t msPeriod,
						   TIMER_TRIG_TYPE Type)
	{
		return TimerEnablePeiTrigger(&vTimer, PeiDev, &Evt,
			msPeriod, Type);
	}


	/** @brief Bind an RTC TICK or COUNTER_OVR event without an ISR. */
	bool EnableTimerEvent(Pei &PeiDev, PeiEvent_t &Evt, uint32_t Event)
	{
		return TimerEnablePeiEvent(&vTimer, PeiDev, &Evt, Event);
	}

	/** @brief Unbind an RTC event producer, retaining its PEI identity. */
	bool DisableTimerEvent(Pei &PeiDev, PeiEvent_t Evt)
	{
		return TimerDisablePeiEvent(&vTimer, PeiDev, Evt);
	}

	/**
	 * @brief Connect a hardware event to the timer START or STOP task.
	 * The Timer driver exposes its hardware task endpoint; PEI owns the
	 * interconnect. No software interrupt or callback is involved.
	 */
	bool ConnectTask(Pei &PeiDev, PeiEvent_t Evt, TIMER_TASK Task)
	{
		return TimerConnectTask(&vTimer, PeiDev, Evt, Task);
	}

	/**
	 * @brief Execute a timer hardware task from software.
	 * In contrast to Disable(), TIMER_TASK_STOP halts the counter while
	 * keeping the timer device and high-frequency clock enabled.
	 */
	bool ExecuteTask(TIMER_TASK Task)
	{
		return TimerExecuteTask(&vTimer, Task);
	}

	/**
	 * @brief   Disable timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to disable. Index value starting at 0
	 */
    virtual void DisableTimerTrigger(int TrigNo) { vTimer.DisableTrigger(&vTimer, TrigNo); }

    /**
     * @brief   Disable timer trigger event.
     *
     */
    void DisableExtTrigger() { vTimer.DisableExtTrigger(&vTimer); }

    /**
     * @brief	Enable external timer trigger event.
     *
     * @param   TrigDevNo : External trigger device number to enable. Index value starting at 0
     *
     * @return  true - Success
     */
    bool EnableExtTrigger(int TrigDevNo, TIMER_EXTTRIG_SENSE Sense) {
    	return vTimer.EnableExtTrigger(&vTimer, TrigDevNo, Sense);
    }

    /**
     * @brief   Get current timer counter in millisecond.
     *
     * This function return the current timer in msec since last reset.
     *
     * @return  Counter in millisecond
     */
	virtual uint32_t mSecond() { return TimerGetMilisecond(&vTimer); }

	/**
	 * @brief   Convert tick count to millisecond.
	 *
	 * @param   Count : Timer tick count value
	 *
	 * @return  Converted count in millisecond
	 */
	virtual uint32_t mSecond(uint64_t Count) { return TimerTickToMilisecond(&vTimer, Count); }

	/**
     * @brief   Get current timer counter in microsecond.
     *
     * This function return the current timer in usec since last reset.
     *
     * @return  Counter in microsecond
     */
	virtual uint32_t uSecond() { return TimerGetMicrosecond(&vTimer); }

	/**
	 * @brief   Convert tick count to microsecond.
	 *
	 * @param   Count : Timer tick count value
	 *
	 * @return  Converted count in microsecond
	 */
	virtual uint32_t uSecond(uint64_t Count) { return TimerTickToMicrosecond(&vTimer, Count); }

	/**
     * @brief   Get current timer counter in nanosecond.
     *
     * This function return the current timer in nsec since last reset.
     *
     * @return  Counter in nanosecond
     */
	virtual uint64_t nSecond() { return TimerGetNanosecond(&vTimer); }

	/**
     * @brief   Convert tick count to nanosecond
     *
     * @param   Count : Timer tick count value
     *
     * @return  Converted count in nanosecond
     */
	virtual uint64_t nSecond(uint64_t Count) { return TimerTickToNanosecond(&vTimer, Count); }

	/**
	 * @brief	Get first available timer trigger index.
	 *
	 * This function returns the first available timer trigger to be used to with
	 * EnableTimerTrigger
	 *
	 * @return	success : Timer trigger index
	 * 			fail : -1
	 */
	int FindAvailTimerTrigger(void) { return vTimer.FindAvailTrigger(&vTimer); }

protected:

#if 0
	TIMER_EVTCB vEvtHandler;//!< Pointer to user event handler callback

    int      vDevNo;		//!< Timer device number
	uint32_t vFreq;			//!< Frequency in Hz
	uint64_t vnsPeriod;		//!< Period in nsec
	uint64_t vRollover;     //!< Rollover counter adjustment
	uint32_t vLastCount;	//!< Last counter read value
#endif
private:
	TimerDev_t vTimer;
};

#endif  //  __cplusplus

#endif // __TIMER_H__
