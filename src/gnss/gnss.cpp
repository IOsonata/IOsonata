/**-------------------------------------------------------------------------
@file	gnss.cpp

@brief	Generic GNSS receiver: status, events and work out of the interrupt.

The parts of a GNSS receiver that are the same for every driver, see
gnss.h. A driver reports updates and events from its interrupt with
UpdatePut and EvtPut. One work event per receiver, queued through
GnssEvtQue, takes them out of the interrupt: it updates the status and calls
the configuration handler.

@author	Hoang Nguyen Hoan
@date	Oct. 8, 2026

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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "app_evt_handler.h"
#include "coredev/interrupt.h"
#include "gnss/gnss.h"

/** @addtogroup GNSS
  * @{
  */

// GNSS_SYS bits this layer knows
#define GNSS_SYS_ALL				(GNSS_SYS_GPS | GNSS_SYS_GALILEO | GNSS_SYS_QZSS | \
									 GNSS_SYS_GLONASS | GNSS_SYS_BEIDOU)

// Events reported once each, a bit per GNSS_EVT
#define GNSS_EVT_BIT(Evt)			(1UL << (uint32_t)(Evt))

// Satellite counts kept in the status
#define GNSS_NBSAT_MAX				255

// State of the queued work, so that it is queued once and queued again
// when GnssEvtQue refused it
enum {
	GNSS_WORK_IDLE,
	GNSS_WORK_PENDING,
	GNSS_WORK_QUEUED,
};

// Receivers initialized, for GnssCheckStatus
static Gnss *s_pGnssList = nullptr;

static uint8_t GnssNbSat(int Nb)
{
	return Nb < 0 ? 0 : Nb > GNSS_NBSAT_MAX ? GNSS_NBSAT_MAX : (uint8_t)Nb;
}

Gnss::Gnss()
{
	memset(&vCfg, 0, sizeof(vCfg));
	memset(&vStatus, 0, sizeof(vStatus));
	memset(&vUpdFix, 0, sizeof(vUpdFix));
	vUpdNbFix = 0;
	vUpdTracked = 0;
	vUpdUsed = 0;
	vbUpd = false;
	vbUpdFix = false;
	vbBlockedIn = false;
	vEvtPend = 0;
	vWorkState = GNSS_WORK_IDLE;
	vpNext = nullptr;
}

Gnss::~Gnss()
{
	Gnss **pp = &s_pGnssList;

	while (*pp != nullptr && *pp != this)
	{
		pp = &(*pp)->vpNext;
	}
	if (*pp == this)
	{
		*pp = vpNext;
	}
}

void Gnss::Evt(GNSS_EVT Evt)
{
	if (vCfg.EvtHandler != nullptr)
	{
		vCfg.EvtHandler(this, Evt, &vStatus);
	}
}

// ---------------------------------------------------------------------------
// Work out of the interrupt
// ---------------------------------------------------------------------------

void Gnss::WorkEvt(uint32_t EvtId, void *pCtx)
{
	(void)EvtId;

	Gnss *dev = (Gnss *)pCtx;

	// What the interrupt reports from here on queues the next event
	dev->vWorkState = GNSS_WORK_IDLE;

	uint32_t state = DisableInterrupt();
	GnssFix_t fix = dev->vUpdFix;
	uint32_t evt = dev->vEvtPend;
	uint32_t nbfix = dev->vUpdNbFix;
	bool upd = dev->vbUpd;
	bool updfix = dev->vbUpdFix;
	bool blocked = dev->vbBlockedIn;
	uint8_t tracked = dev->vUpdTracked;
	uint8_t used = dev->vUpdUsed;

	dev->vEvtPend = 0;
	dev->vUpdNbFix = 0;
	dev->vbUpd = false;
	EnableInterrupt(state);

	GnssStatus_t &st = dev->vStatus;

	// A position waiting stays as the last known one, whatever follows
	if (nbfix > 0)
	{
		st.Fix = fix;
		st.NbFix += nbfix;
	}

	if (evt & GNSS_EVT_BIT(GNSS_EVT_FAULT))
	{
		// The receiver stopped: the state it reported no longer holds
		st.bFix = false;
		st.bBlocked = false;
		st.NbSatTracked = 0;
		st.NbSatUsed = 0;
		dev->Evt(GNSS_EVT_FAULT);

		return;
	}

	// Both waiting: the last one reported is the state now, sent last
	if ((evt & GNSS_EVT_BIT(GNSS_EVT_BLOCKED)) && (evt & GNSS_EVT_BIT(GNSS_EVT_UNBLOCKED)))
	{
		st.bBlocked = !blocked;
		dev->Evt(blocked ? GNSS_EVT_UNBLOCKED : GNSS_EVT_BLOCKED);
	}
	if (evt & (GNSS_EVT_BIT(GNSS_EVT_BLOCKED) | GNSS_EVT_BIT(GNSS_EVT_UNBLOCKED)))
	{
		st.bBlocked = blocked;
		dev->Evt(blocked ? GNSS_EVT_BLOCKED : GNSS_EVT_UNBLOCKED);
	}

	if (upd)
	{
		st.bFix = updfix;
		st.NbSatTracked = tracked;
		st.NbSatUsed = used;
		dev->Evt(updfix ? GNSS_EVT_FIX : GNSS_EVT_NOFIX);
	}

	if (evt & GNSS_EVT_BIT(GNSS_EVT_TIMEOUT))
	{
		dev->Evt(GNSS_EVT_TIMEOUT);
	}
}

void Gnss::WorkQue()
{
	uint32_t state = DisableInterrupt();

	if (vWorkState == GNSS_WORK_IDLE)
	{
		vWorkState = GNSS_WORK_PENDING;
	}
	if (vWorkState != GNSS_WORK_PENDING)
	{
		EnableInterrupt(state);

		return;
	}
	vWorkState = GNSS_WORK_QUEUED;
	EnableInterrupt(state);

	if (GnssEvtQue(0, this, WorkEvt) == false)
	{
		vWorkState = GNSS_WORK_PENDING;
	}
}

// Default for an application without an OS, see gnss.h. Interrupt context.
__attribute__((weak)) bool GnssEvtQue(uint32_t EvtId, void *pCtx, GnssEvtQueHandler_t Handler)
{
	return AppEvtHandlerQue(EvtId, pCtx, Handler);
}

void Gnss::CheckStatus()
{
	if (vWorkState == GNSS_WORK_PENDING)
	{
		WorkQue();
	}
}

void GnssCheckStatus(void)
{
	for (Gnss *p = s_pGnssList; p != nullptr; p = p->vpNext)
	{
		p->CheckStatus();
	}
}

void Gnss::UpdatePut(bool bFix, const GnssFix_t * const pFix, int NbTracked, int NbUsed)
{
	bool fix = bFix && pFix != nullptr;
	uint64_t t = vpTimer != nullptr ? vpTimer->nSecond() / 1000ULL : 0;
	uint32_t state = DisableInterrupt();

	if (fix)
	{
		vUpdFix = *pFix;
		vUpdFix.Timestamp = t;
		vUpdNbFix++;
	}
	vbUpdFix = fix;
	vUpdTracked = GnssNbSat(NbTracked);
	vUpdUsed = GnssNbSat(NbUsed);
	vbUpd = true;
	EnableInterrupt(state);

	WorkQue();
}

void Gnss::EvtPut(GNSS_EVT Evt)
{
	if (Evt != GNSS_EVT_TIMEOUT && Evt != GNSS_EVT_BLOCKED && Evt != GNSS_EVT_UNBLOCKED &&
		Evt != GNSS_EVT_FAULT)
	{
		return;
	}

	uint32_t state = DisableInterrupt();

	vEvtPend |= GNSS_EVT_BIT(Evt);
	if (Evt == GNSS_EVT_BLOCKED || Evt == GNSS_EVT_UNBLOCKED)
	{
		vbBlockedIn = Evt == GNSS_EVT_BLOCKED;
	}
	EnableInterrupt(state);

	WorkQue();
}

// ---------------------------------------------------------------------------
// Initialization
// ---------------------------------------------------------------------------

bool Gnss::CoreInit(const GnssCfg_t &Cfg, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	if (pIntrf == nullptr || (Cfg.SysMask & ~(uint32_t)GNSS_SYS_ALL) != 0 ||
		(unsigned)Cfg.Dyn > GNSS_DYN_AUTOMOTIVE || Cfg.NbCmd < 0 || (Cfg.NbCmd > 0 && Cfg.ppCmd == nullptr))
	{
		return false;
	}

	for (int i = 0; i < Cfg.NbCmd; i++)
	{
		if (Cfg.ppCmd[i] == nullptr)
		{
			return false;
		}
	}

	Gnss *p = s_pGnssList;

	while (p != nullptr && p != this)
	{
		p = p->vpNext;
	}
	if (p == nullptr)
	{
		vpNext = s_pGnssList;
		s_pGnssList = this;
	}

	Interface(pIntrf);
	vpTimer = pTimer;
	DeviceAddress(Cfg.DevAddr);

	uint32_t state = DisableInterrupt();

	vCfg = Cfg;
	memset(&vStatus, 0, sizeof(vStatus));
	vUpdNbFix = 0;
	vbUpd = false;
	vbUpdFix = false;
	vbBlockedIn = false;
	vEvtPend = 0;
	EnableInterrupt(state);

	return true;
}

/** @} End of group GNSS */
