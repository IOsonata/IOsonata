/**-------------------------------------------------------------------------
@file	bt_pds_nvm_mpsl.h

@brief	MPSL timeslot flash arbitration core for SDC (SoftDevice Controller)
		builds.

		On an SDC build there is no SoftDevice and no sd_flash_write to
		arbitrate flash against radio activity. The controller (nrfxlib
		softdevice_controller) owns the radio timeline through MPSL, so a flash
		write must run inside an MPSL timeslot, a gap MPSL grants where the
		radio is guaranteed idle. This module wraps the MPSL timeslot session
		so a target NVM backend (RRAMC on nRF54L, NVMC on nRF52) can perform its
		write or erase in a radio-safe window, behind a synchronous call.

		The target supplies a single callback that does the actual medium
		operation. The core requests a timeslot, invokes the callback at
		SIGNAL_START (radio idle), and blocks the caller until the operation
		signals done. This is the SDC analogue of the bm port's
		sd_flash_write + SoC-event-pump pattern, MPSL-arbitrated instead of
		SoftDevice-arbitrated, and free of nRF5_SDK.

		This header is target agnostic. RRAMC and NVMC backends include it and
		provide their own medium primitives plus the BtPdsNvm_t vtable.

@author	Hoang Nguyen Hoan
@date	Jun 09, 2026

@license MIT, (c) 2026 I-SYST.
----------------------------------------------------------------------------*/
#ifndef __BT_PDS_NVM_MPSL_H__
#define __BT_PDS_NVM_MPSL_H__

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// A medium operation to run inside a radio-safe timeslot. The core calls this
// at SIGNAL_START with the radio idle. It must complete (or make bounded
// progress on) the operation and return the number of microseconds it expects
// the work to take, so the core can size the timeslot. Returning 0 means the
// operation finished; a positive value means more time is needed and the core
// will extend or re-request. pCtx is the per-op context passed to BtPdsMpslRun.
//
// The operation runs in MPSL timeslot (high-priority interrupt) context. It
// must not call blocking APIs, must not touch the RADIO or TIMER0, and must be
// short and deterministic. Long erases (NVMC) report remaining time so the
// core can split work across timeslots.
typedef struct {
	// Perform one unit of the medium operation. Return 0 when fully done, or
	// the estimated microseconds still required (the core schedules more).
	uint32_t (*Step)(void *pCtx);
	// Worst-case microseconds for a single Step, used to size the timeslot.
	uint32_t StepBudgetUs;
	void *pCtx;
} BtPdsMpslOp_t;

// Run a medium operation under MPSL timeslot arbitration. Opens (or reuses) the
// timeslot session, requests a window, drives Op->Step at SIGNAL_START until it
// reports done, and returns synchronously to the caller. Returns 0 on success,
// negative errno on failure (timeslot blocked/cancelled, or Step error path).
//
// Synchronous from the caller's view: on return the medium reflects the result,
// matching the BtPdsNvm_t Write/Erase contract. Internally it blocks the caller
// (WFE) while the timeslot callback advances the operation, the SDC analogue of
// PumpSocEvents.
int BtPdsMpslRun(BtPdsMpslOp_t *pOp);

// One-time init: register the MPSL timeslot session memory. Called once before
// the first BtPdsMpslRun, typically from the target backend's init.
int BtPdsMpslInit(void);

#ifdef __cplusplus
}
#endif

#endif // __BT_PDS_NVM_MPSL_H__
