/**-------------------------------------------------------------------------
@file	nrf_mpsl.cpp

@brief	MPSL configuration.

MPSL configuration & initialization.

@author	Nguyen Hoan Hoang
@date	Dec. 21, 2024

@license

MIT License

Copyright (c) 2024 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Softare.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include "nrf.h"

// MPSL rev 40edf29 added mpsl_memcpy() declared with the C99 'restrict'
// keyword inside the extern "C" block of mpsl.h. 'restrict' is not a C++
// keyword, so map it to the GCC/Clang __restrict extension across the
// mpsl.h include, then restore it.
#if defined(__cplusplus) && !defined(restrict)
#define restrict __restrict
#include "mpsl.h"
#undef restrict
#else
#include "mpsl.h"
#endif

#include "nrf_mpsl.h"
#include "mpsl_fem_init.h"
#include "nrfx_power.h"

#include "coredev/uart.h"
#include "coredev/system_core_clock.h"

#include <errno.h>

#include "mpsl_timeslot.h"			// the timeslot session and its signals

#include "idelay.h"
#include "storage/nvm_intrf.h"

/******** For DEBUG ************/
//#define DEBUG_ENABLE

#ifdef DEBUG_ENABLE
#include "syslog.h"
#define DEBUG_PRINTF(...)		SysLogPrintf(SysLogGet(), __VA_ARGS__)
#else
#define DEBUG_PRINTF(...)
#endif
/*******************************/

//---------------------------------------------------------------------------
// Memory arbiter. The memory controller is shared with the radio, so a write
// or an erase has to run inside a window MPSL says is free. The memory
// interface hands over one bounded step at a time and knows nothing about
// timeslots; this end knows nothing about memory.
//---------------------------------------------------------------------------

// Give up on one operation after this long. The longest legitimate one is an
// nRF52 page erase spread over several windows, well under a second; longer
// than this means the grant machinery is not delivering.
#ifndef MPSL_NVM_TIMEOUT_MS
#define MPSL_NVM_TIMEOUT_MS			2000
#endif

// Session memory, word aligned as MPSL requires. One session.
static uint8_t __attribute__((aligned(4))) s_NvmSessionMem[MPSL_TIMESLOT_CONTEXT_SIZE];
static mpsl_timeslot_session_id_t s_NvmSessionId;
static bool s_bNvmSessionOpen = false;

// The operation in flight and how it ended. Written in the timeslot callback,
// read by the caller waiting on it.
static NvmIntrfOp_t * volatile s_pNvmOp = nullptr;
static volatile bool s_bNvmDone = false;
static volatile int s_NvmResult = 0;

// Handed back to MPSL, which reads it after the callback returns.
static mpsl_timeslot_signal_return_param_t s_NvmRetParam;
static mpsl_timeslot_request_t s_NvmNextReq;

// What the callback saw. Recorded rather than printed: it runs at timeslot
// priority, where sending anything to a UART would overstay the window.
static volatile uint32_t s_NvmSigStart = 0;
static volatile uint32_t s_NvmSigBlocked = 0;
static volatile uint32_t s_NvmSigOther = 0;
static volatile uint32_t s_NvmSigAbnormal = 0;

// An earliest request one step long. Clamped to what MPSL accepts, and high
// priority so a bond write is not starved by low priority traffic.
static void NvmBuildReq(mpsl_timeslot_request_t *pReq, uint32_t LengthUs)
{
	if (LengthUs < MPSL_TIMESLOT_LENGTH_MIN_US)
	{
		LengthUs = MPSL_TIMESLOT_LENGTH_MIN_US;
	}
	if (LengthUs > MPSL_TIMESLOT_LENGTH_MAX_US)
	{
		LengthUs = MPSL_TIMESLOT_LENGTH_MAX_US;
	}

	pReq->request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST;
	pReq->params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE;
	pReq->params.earliest.priority = MPSL_TIMESLOT_PRIORITY_HIGH;
	pReq->params.earliest.length_us = LengthUs;
	pReq->params.earliest.timeout_us = MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US;
}

// Detach the operation and report it, if it asked to be reported. Runs where
// the operation finished, which for a granted window is timeslot priority, so
// it does nothing but hand the result over.
static void NvmOpFinish(int Res)
{
	NvmIntrfOp_t *op = s_pNvmOp;

	if (op == nullptr)
	{
		return;
	}

	s_pNvmOp = nullptr;

	if (op->Done != nullptr)
	{
		op->Done(op->pCtx, Res);
	}
}

// Runs at timeslot priority with the radio idle. Short, no blocking calls,
// nothing that touches RADIO or TIMER0.
static mpsl_timeslot_signal_return_param_t *NvmTimeslotCb(
		mpsl_timeslot_session_id_t SessionId, uint32_t Signal)
{
	(void)SessionId;

	switch (Signal)
	{
		case MPSL_TIMESLOT_SIGNAL_START:
		{
			s_NvmSigStart = s_NvmSigStart + 1;

			uint32_t remainUs = 0;

			if (s_pNvmOp != nullptr && s_pNvmOp->Step != nullptr)
			{
				remainUs = s_pNvmOp->Step(s_pNvmOp->pCtx);
			}

			if (remainUs == 0)
			{
				s_NvmResult = 0;
				s_bNvmDone = true;
				NvmOpFinish(0);
				s_NvmRetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
			}
			else
			{
				// More to do, an nRF52 page erase for one. Ask for another
				// window to carry on in.
				NvmBuildReq(&s_NvmNextReq, remainUs);
				s_NvmRetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
				s_NvmRetParam.params.request.p_next = &s_NvmNextReq;
			}
			break;
		}

		case MPSL_TIMESLOT_SIGNAL_BLOCKED:
		case MPSL_TIMESLOT_SIGNAL_CANCELLED:
			s_NvmSigBlocked = s_NvmSigBlocked + 1;
			// Could not be scheduled. Ask again; if it keeps failing the
			// caller gives up on its own bounded wait.
			NvmBuildReq(&s_NvmNextReq, s_pNvmOp != nullptr ?
						s_pNvmOp->StepBudgetUs : MPSL_TIMESLOT_LENGTH_MIN_US);
			s_NvmRetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
			s_NvmRetParam.params.request.p_next = &s_NvmNextReq;
			break;

		case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
		case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
			s_NvmRetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
			break;

		case MPSL_TIMESLOT_SIGNAL_OVERSTAYED:
		case MPSL_TIMESLOT_SIGNAL_INVALID_RETURN:
		default:
			s_NvmSigOther = s_NvmSigOther + 1;
			s_NvmSigAbnormal = Signal;
			s_NvmResult = -EIO;
			s_bNvmDone = true;
			NvmOpFinish(-EIO);
			s_NvmRetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
			break;
	}

	// Wake the waiter: this is a higher priority context, so its WFE needs
	// the event set explicitly.
	if (s_bNvmDone)
	{
		__SEV();
	}

	return &s_NvmRetParam;
}

// What the memory interface calls. One operation at a time, which the
// interface already serialises through its own busy latch.
static int NvmArbiterRun(NvmIntrfOp_t *pOp)
{
	if (pOp == nullptr || pOp->Step == nullptr)
	{
		return -EINVAL;
	}
	if (s_bNvmSessionOpen == false)
	{
		return -ENODEV;
	}

	s_bNvmDone = false;
	s_NvmResult = 0;
	s_pNvmOp = pOp;
	__DMB();

	NvmBuildReq(&s_NvmNextReq, pOp->StepBudgetUs);

	int32_t r = mpsl_timeslot_request(s_NvmSessionId, &s_NvmNextReq);
	if (r != 0)
	{
		s_pNvmOp = nullptr;

		return -EIO;
	}

	if (pOp->Done != nullptr)
	{
		// Under way. The signal handler runs it and reports it, so the
		// caller is not held while the memory works.
		return NVM_INTRF_OP_STARTED;
	}

	uint32_t elapsed = 0;
	while (s_bNvmDone == false)
	{
		if (elapsed >= MPSL_NVM_TIMEOUT_MS * 1000UL)
		{
			// Detach first: a late grant then finds no work, reports done
			// and ends its window without touching anything of ours.
			s_pNvmOp = nullptr;

			DEBUG_PRINTF("MPSL nvm: timeout, start %lu blocked %lu other %lu\r\n",
						 (unsigned long)s_NvmSigStart,
						 (unsigned long)s_NvmSigBlocked,
						 (unsigned long)s_NvmSigOther);

			return -ETIMEDOUT;
		}
		usDelay(100);
		elapsed += 100;
	}

	int result = s_NvmResult;

	if (result != 0)
	{
		DEBUG_PRINTF("MPSL nvm: failed %d, abnormal signal %lu\r\n",
					 result, (unsigned long)s_NvmSigAbnormal);
	}

	return result;
}

int MpslNvmArbiterStart(void)
{
	if (s_bNvmSessionOpen)
	{
		return 0;
	}

	int32_t r = mpsl_timeslot_session_count_set(s_NvmSessionMem, 1);
	if (r != 0)
	{
		DEBUG_PRINTF("MPSL nvm: session_count_set %ld\r\n", (long)r);

		return -EIO;
	}

	r = mpsl_timeslot_session_open(NvmTimeslotCb, &s_NvmSessionId);
	if (r != 0)
	{
		DEBUG_PRINTF("MPSL nvm: session_open %ld\r\n", (long)r);

		return -EIO;
	}

	s_bNvmSessionOpen = true;

	NvmIntrfSetArbiter(NvmArbiterRun);

	return 0;
}

void MpslNvmArbiterStop(void)
{
	if (s_bNvmSessionOpen == false)
	{
		return;
	}

	NvmIntrfSetArbiter(nullptr);

	// Whatever was in flight is not going to finish now. Fail it and say so
	// before the session goes, or the driver waits for a completion that
	// nothing is left to deliver.
	NvmOpFinish(-ECANCELED);

	(void)mpsl_timeslot_session_close(s_NvmSessionId);

	s_bNvmSessionOpen = false;
}

static void MpslAssert(const char * const file, const uint32_t line)
{
	DEBUG_PRINTF("MPSL Fault: %s, %d\n", file, line);
	while(1);
}

bool MpslInit(void)
{
	int32_t res = 0;

	mpsl_clock_lfclk_cfg_t lfclk = {MPSL_CLOCK_LF_SRC_RC, 0,};
	OscDesc_t const *lfosc = GetLowFreqOscDesc();

	// Set default clock based on system oscillator settings
	if (lfosc->Type == OSC_TYPE_RC)
	{
		lfclk.source = MPSL_CLOCK_LF_SRC_RC;
		lfclk.rc_ctiv = MPSL_RECOMMENDED_RC_CTIV;
		lfclk.rc_temp_ctiv = MPSL_RECOMMENDED_RC_TEMP_CTIV;
	}
	else
	{
		lfclk.accuracy_ppm = lfosc->Accuracy;
		lfclk.source = MPSL_CLOCK_LF_SRC_XTAL;
	}

	lfclk.skip_wait_lfclk_started = MPSL_DEFAULT_SKIP_WAIT_LFCLK_STARTED;

	mpsl_fem_init();

	DEBUG_PRINTF("mpsl_init\r\n");

	// Initialize Nordic multi-protocol support library (MPSL)
#ifdef NRF54L_SERIES
	//NVIC_DisableIRQ(GRTC_3_IRQn);
	//NVIC_SetPriority(GRTC_3_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	//NVIC_EnableIRQ(GRTC_3_IRQn);
	//NRF_GRTC->INTENSET3 = 0xF80;
	NRF_GRTC->MODE |= 2;
	NRF_GRTC->TASKS_START = 1;


	res = mpsl_init(&lfclk, SWI00_IRQn, MpslAssert);
	res = mpsl_clock_hfclk_latency_set(MPSL_CLOCK_HF_LATENCY_TYPICAL);
//	mpsl_pan_rfu();
#else
	// The low priority line must be a real NVIC interrupt: MPSL pends it
	// through the NVIC, and CMSIS silently ignores that for a system
	// exception, so with PendSV the low priority processing never ran. Every
	// signal delivered through it starved: BLOCKED, CANCELLED and the session
	// idle transition, which is why a timeslot session worked exactly once
	// and every request after the first failed.
	res = mpsl_init(&lfclk, SWI5_EGU5_IRQn, MpslAssert);
#endif
	DEBUG_PRINTF("mpsl_init res = %x\r\n", res);

	if (res < 0)
	{
		return false;
	}

#if 0
	static uint8_t timeslot_context = 0;
	res = mpsl_timeslot_session_count_set((void *) timeslot_context,
			MPSL_TIMESLOT_SESSION_COUNT);
#endif


#ifdef NRF54L_SERIES
	NVIC_SetPriority(SWI00_IRQn, MPSL_HIGH_IRQ_PRIORITY + 4);
	NVIC_EnableIRQ(SWI00_IRQn);
	NVIC_SetPriority(CLOCK_POWER_IRQn, MPSL_HIGH_IRQ_PRIORITY + 4);
	NVIC_EnableIRQ(CLOCK_POWER_IRQn);

	//NVIC_SetPriority(TIMER10_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	//NVIC_EnableIRQ(TIMER10_IRQn);
	NVIC_SetPriority(RADIO_0_IRQn, MPSL_HIGH_IRQ_PRIORITY);
	NVIC_EnableIRQ(RADIO_0_IRQn);
#else
	NVIC_SetPriority(SWI5_EGU5_IRQn, MPSL_HIGH_IRQ_PRIORITY + 15);
	NVIC_EnableIRQ(SWI5_EGU5_IRQn);
#endif


	return true;
}

extern "C" {

#ifdef NRF54L_SERIES
void SWI00_IRQHandler(void)
#else
void SWI5_EGU5_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("mpsl_low_priority_process\r\n");
	mpsl_low_priority_process();
}

#ifdef NRF54L_SERIES
void RADIO_0_IRQHandler(void)
#else
void RADIO_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RADIO_Handler\r\n");
	MPSL_IRQ_RADIO_Handler();
}

#ifdef NRF54L_SERIES
void CLOCK_POWER_IRQHandler(void)
#else
void POWER_CLOCK_IRQHandler()
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_CLOCK_Handler\r\n");
	MPSL_IRQ_CLOCK_Handler();
}

#ifdef NRF54L_SERIES
void GRTC_3_IRQHandler(void)
#else
void RTC0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("MPSL_IRQ_RTC0_Handler\r\n");
	MPSL_IRQ_RTC0_Handler();
}

#ifdef NRF54L15_XXAA
void TIMER10_IRQHandler(void)
#else
void TIMER0_IRQHandler(void)
#endif
{
	DEBUG_PRINTF("TIMER10_IRQHandler\r\n");
	MPSL_IRQ_TIMER0_Handler();
}

#if defined(NRF54L_SERIES)
/** @brief MPSL requesting CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it needs CONSTLAT to be on.
 * It only calls the function on nRF54L Series devices.
 */
void mpsl_constlat_request_callback(void)
{
	nrfx_power_constlat_mode_request();
}

/** @brief De-request CONSTLAT to be on.
 *
 * The application needs to implement this function.
 * MPSL will call the function when it no longer needs CONSTLAT to be on.
 * It only only calls the function on nRF54L Series devices.
 */
void mpsl_lowpower_request_callback(void)
{
	nrfx_power_constlat_mode_free();
}
#endif

}	// extern "C"
