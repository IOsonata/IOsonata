/**-------------------------------------------------------------------------
@file	bt_pds_sdc.cpp

@brief	MPSL timeslot flash arbitration core for SDC builds. See
		bt_pds_nvm_mpsl.h for the design rationale.

		Flow:
		  caller (BtPdsNvm_t.Write/Erase, thread context)
		    -> BtPdsMpslRun(op): open session, request EARLIEST timeslot, block
		    -> MPSL grants gap, calls s_TimeslotCb(SIGNAL_START) in IRQ context
		    -> callback runs op->Step; if done, END; if more needed, REQUEST next
		    -> on completion the callback sets s_Done; caller's WFE loop returns

		The session is opened once and kept open across operations to avoid
		repeated open/close churn. Only one operation runs at a time (the bond
		store is single-threaded and operations are serialized by BtPdsMpslRun).

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#include <errno.h>

#include "nrf.h"					// __WFE, __SEV
#include "mpsl_timeslot.h"

#include "bt_pds_sdc.h"

// Timeslot session memory. MPSL_TIMESLOT_CONTEXT_SIZE bytes per session, one
// session. Word aligned as MPSL requires.
static uint8_t __attribute__((aligned(4))) s_SessionMem[MPSL_TIMESLOT_CONTEXT_SIZE];
static mpsl_timeslot_session_id_t s_SessionId;
static bool s_SessionOpen;

// Current operation and its completion state. Set by BtPdsMpslRun, advanced by
// the timeslot callback. volatile: written in IRQ, read in the WFE wait loop.
static BtPdsMpslOp_t * volatile s_pOp;
static volatile bool s_Done;
static volatile int s_Result;

// Return-param storage the callback hands back to MPSL. Static because MPSL
// reads it after the callback returns.
static mpsl_timeslot_signal_return_param_t s_RetParam;
static mpsl_timeslot_request_t s_NextReq;

// Build an EARLIEST request sized for one Step budget. A radio-idle window of
// length_us is requested as soon as possible. Length is clamped to the MPSL
// min/max. Priority HIGH so bond writes are not starved by low-priority slots.
static void BuildEarliestReq(mpsl_timeslot_request_t *pReq, uint32_t lengthUs)
{
	if (lengthUs < MPSL_TIMESLOT_LENGTH_MIN_US)
	{
		lengthUs = MPSL_TIMESLOT_LENGTH_MIN_US;
	}
	if (lengthUs > MPSL_TIMESLOT_LENGTH_MAX_US)
	{
		lengthUs = MPSL_TIMESLOT_LENGTH_MAX_US;
	}

	pReq->request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST;
	pReq->params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE;
	pReq->params.earliest.priority = MPSL_TIMESLOT_PRIORITY_HIGH;
	pReq->params.earliest.length_us = lengthUs;
	pReq->params.earliest.timeout_us = MPSL_TIMESLOT_EARLIEST_TIMEOUT_MAX_US;
}

// Timeslot signal callback. Runs in high-priority interrupt context with the
// radio guaranteed idle. Must be short, must not touch RADIO/TIMER0, must not
// block. Drives the current operation's Step and decides the next action.
static mpsl_timeslot_signal_return_param_t *s_TimeslotCb(
		mpsl_timeslot_session_id_t sessionId, uint32_t signal)
{
	(void)sessionId;

	switch (signal)
	{
		case MPSL_TIMESLOT_SIGNAL_START:
		{
			// Radio is idle for the granted window. Advance the operation.
			uint32_t remainUs = 0;

			if (s_pOp != NULL && s_pOp->Step != NULL)
			{
				remainUs = s_pOp->Step(s_pOp->pCtx);
			}

			if (remainUs == 0)
			{
				// Operation complete. End the timeslot and signal the waiter.
				s_Result = 0;
				s_Done = true;
				s_RetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
			}
			else
			{
				// More work needed (e.g. a long NVMC erase split across slots).
				// Request another earliest timeslot to continue.
				BuildEarliestReq(&s_NextReq, remainUs);
				s_RetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
				s_RetParam.params.request.p_next = &s_NextReq;
			}
			break;
		}

		case MPSL_TIMESLOT_SIGNAL_BLOCKED:
		case MPSL_TIMESLOT_SIGNAL_CANCELLED:
			// The request could not be scheduled. Retry once with another
			// earliest request; persistent failure surfaces as a timeout in the
			// caller (it will give up after its bounded wait).
			BuildEarliestReq(&s_NextReq, s_pOp ? s_pOp->StepBudgetUs
											   : MPSL_TIMESLOT_LENGTH_MIN_US);
			s_RetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_REQUEST;
			s_RetParam.params.request.p_next = &s_NextReq;
			break;

		case MPSL_TIMESLOT_SIGNAL_SESSION_IDLE:
		case MPSL_TIMESLOT_SIGNAL_SESSION_CLOSED:
			s_RetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;
			break;

		case MPSL_TIMESLOT_SIGNAL_OVERSTAYED:
		case MPSL_TIMESLOT_SIGNAL_INVALID_RETURN:
		default:
			// Abnormal. End and report error so the caller does not hang.
			s_Result = -EIO;
			s_Done = true;
			s_RetParam.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
			break;
	}

	// Wake the waiter promptly when done. SEV sets the event so the caller's
	// WFE returns even though this is a higher-priority interrupt context.
	if (s_Done)
	{
		__SEV();
	}

	return &s_RetParam;
}

int BtPdsMpslInit(void)
{
	if (s_SessionOpen)
	{
		return 0;
	}

	int32_t r = mpsl_timeslot_session_count_set(s_SessionMem, 1);
	if (r != 0)
	{
		return -EIO;
	}

	r = mpsl_timeslot_session_open(s_TimeslotCb, &s_SessionId);
	if (r != 0)
	{
		return -EIO;
	}

	s_SessionOpen = true;
	return 0;
}

int BtPdsMpslRun(BtPdsMpslOp_t *pOp)
{
	if (pOp == NULL || pOp->Step == NULL)
	{
		return -EINVAL;
	}

	if (!s_SessionOpen)
	{
		int r = BtPdsMpslInit();
		if (r != 0)
		{
			return r;
		}
	}

	s_pOp = pOp;
	s_Done = false;
	s_Result = 0;

	// Kick off the first window. First request in a session must be EARLIEST.
	mpsl_timeslot_request_t req;
	BuildEarliestReq(&req, pOp->StepBudgetUs);

	int32_t r = mpsl_timeslot_request(s_SessionId, &req);
	if (r != 0)
	{
		s_pOp = NULL;
		return -EIO;
	}

	// Block until the timeslot callback completes the operation. The callback
	// runs in IRQ context and SEVs on completion; WFE wakes on that or any
	// interrupt, then we re-check the flag.
	while (!s_Done)
	{
		__WFE();
	}

	int result = s_Result;
	s_pOp = NULL;

	return result;
}
