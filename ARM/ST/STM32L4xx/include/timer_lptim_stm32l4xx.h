/**-------------------------------------------------------------------------
@file	timer_lptim_stm32l4xx.h

@brief	Timer implementation of STM32L4xx LPTIM

This timer has 16bits counter.  Supported clock sources :
- 32KHz LSI (TIMER_CLKSRC_LFRC)
- 32768Hz LSE (TIMER_CLKSRC_LFXTAL(
- 16MHz HSI (TIMER_CLKSRC_HFRC or TIMER_CLKSRC_DEFAULT)


@author	Hoang Nguyen Hoan
@date	July 21, 2019

@license

Copyright (c) 2019, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __TIMER_STM32L4XX_H__
#define __TIMER_STM32L4XX_H__

#include <stdint.h>

#include "stm32l4xx.h"
#include "timer_stm32l4x.h"

#define STM32L4XX_LPTIMER_MAXFREQ			16000000

#define STM32L4XX_LPTIMER_MAXCNT			2
#define STM32L4XX_LPTIMER_TRIG_MAXCNT		1

class LPTimerSTM32L4xx : public Timer {
public:
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
    virtual bool Init(const TIMER_CFG &Cfg);

    /**
     * @brief   Turn on timer.
     *
     * This is used to re-enable timer after it was disabled for power
     * saving.  It normally does not go through full initialization sequence
     */
    virtual bool Enable();

    /**
     * @brief   Turn off timer.
     *
     * This is used to disable timer for power saving. Call Enable() to
     * re-enable timer instead of full initialization sequence
     */
    virtual void Disable();

    /**
     * @brief   Reset timer.
     */
    virtual void Reset();

    /**
     * @brief   Get the current tick count.
     *
     * This function read the tick count with compensated overflow roll over
     *
     * @return  Total tick count since last reset
     */
	virtual uint64_t TickCount() {
	    uint32_t cnt = vpReg->CNT;
	    while (cnt != vpReg->CNT) {
	    	cnt = vpReg->CNT;
	    }
		return (uint64_t)cnt + vRollover;
	}

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
	virtual uint32_t Frequency(uint32_t Freq);

	/**
	 * @brief   Get current timer frequency
	 *
	 * @return  Timer frequency in Hz
	 */
	virtual uint32_t Frequency(void) { return vFreq; }

	/**
	 * @brief	Get maximum available timer trigger event for the timer.
	 *
	 * @return	count
	 */
	virtual int MaxTimerTrigger();

    /**
	 * @brief	Enable millisecond timer trigger event.
	 *
	 * Note: this must be implemented in each class. Otherwise it won't be
	 * visible to compiler from base class.  This is due to limitation of
	 * polymorphism of C++
	 *
	 * @param   TrigNo : Trigger number to enable. Index value starting at 0
	 * @param   msPeriod : Trigger period in msec.
	 * @param   Type     : Trigger type single shot or continuous
	 * @param	Handler	 : Optional Timer trigger user callback
	 * @param   pContext : Optional pointer to user private data to be passed
	 *                     to the callback. This could be a class or structure pointer.
	 *
	 * @return  real period in nsec based on clock calculation
	 */
    virtual uint32_t EnableTimerTrigger(int TrigNo, uint32_t msPeriod, TIMER_TRIG_TYPE Type,
                                        TIMER_TRIGCB const Handler = NULL, void * const pContext = NULL) {
    	return (uint32_t)(EnableTimerTrigger(TrigNo, (uint64_t)msPeriod * 1000000ULL, Type, Handler, pContext) / 1000000ULL);
    }

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
	                                    TIMER_TRIGCB const Handler = NULL, void * const pContext = NULL);

	/**
	 * @brief   Disable timer trigger event.
	 *
	 * @param   TrigNo : Trigger number to disable. Index value starting at 0
	 */
    virtual void DisableTimerTrigger(int TrigNo);

	/**
	 * @brief	Get first available timer trigger index.
	 *
	 * This function returns the first available timer trigger to be used to with
	 * EnableTimerTrigger
	 *
	 * @return	success : Timer trigger index
	 * 			fail : -1
	 */
	virtual int FindAvailTimerTrigger(void);
    void IRQHandler();

private:
    LPTIM_TypeDef *vpReg;
    uint32_t vCC[STM32L4XX_LPTIMER_TRIG_MAXCNT];
    TIMER_TRIGGER vTrigger[STM32L4XX_LPTIMER_TRIG_MAXCNT];
    uint32_t vBaseFreq;
    TIMER vTimer;
};

#endif // __TIMER_STM32L4XX_H__

