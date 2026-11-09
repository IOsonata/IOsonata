/**-------------------------------------------------------------------------
@file	analog_comp_nrf5x.cpp

@brief	Analog comparator implementation on nRF52


@author	Hoang Nguyen Hoan
@date	Aug. 13, 2018

@license

Copyright (c) 2018, I-SYST inc., all rights reserved

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
#include <string.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_peripherals.h"

// The nRF5340 application core names its peripherals by security state, and
// the firmware runs secure
#if !defined(NRF_LPCOMP) && defined(NRF_LPCOMP_S)
#define NRF_LPCOMP	NRF_LPCOMP_S
#endif

#if defined(NRF5340_XXAA_APPLICATION)
#define LPCOMP_IRQn	COMP_LPCOMP_IRQn
#endif

// One comparator device. It is driven as LPCOMP where the chip has one and as
// COMP where it has COMP only (nRF52810, nRF52820). Where both exist they are
// the same peripheral, same base address and interrupt. The registers used by
// both have the same offsets, only these names differ.
#if !defined(LPCOMP_PRESENT) && defined(COMP_PRESENT)
#define ANALOG_COMP_NRF_COMP			1
#define NRF_ACOMP						NRF_COMP
#define ACOMP_ENABLE_Enabled			COMP_ENABLE_ENABLE_Enabled
#define ACOMP_IRQn						COMP_LPCOMP_IRQn
#define ACOMP_SHORTS_READY_SAMPLE_Msk	COMP_SHORTS_READY_SAMPLE_Msk
#define ACOMP_INTENSET_UP_Msk			COMP_INTENSET_UP_Msk
#define ACOMP_INTENSET_DOWN_Msk			COMP_INTENSET_DOWN_Msk
#else
#define NRF_ACOMP						NRF_LPCOMP
#define ACOMP_ENABLE_Enabled			LPCOMP_ENABLE_ENABLE_Enabled
#define ACOMP_IRQn						LPCOMP_IRQn
#define ACOMP_SHORTS_READY_SAMPLE_Msk	LPCOMP_SHORTS_READY_SAMPLE_Msk
#define ACOMP_INTENSET_UP_Msk			LPCOMP_INTENSET_UP_Msk
#define ACOMP_INTENSET_DOWN_Msk			LPCOMP_INTENSET_DOWN_Msk
#endif

#include "idelay.h"
#include "converters/analog_comp.h"

// References to the PEI matrix remain weak. A comparator-only firmware
// does not retain optional event/task routing sections at final link.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiUnbindEvent
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

alignas(4) static AnalogCompDev_t *s_pnRF5xLPCompDev = NULL;

bool nRF52LPCompEnable(AnalogCompDev_t * const pDev)
{
	NRF_ACOMP->ENABLE = ACOMP_ENABLE_Enabled;

	return true;
}

void nRF52LPCompDisable(AnalogCompDev_t * const pDev)
{
	NRF_ACOMP->ENABLE = 0;
}

bool nRF52LPCompStart(AnalogCompDev_t * const pDev)
{
	NRF_ACOMP->TASKS_START = 1;

	return true;
}

void nRF52LPCompStop(AnalogCompDev_t * const pDev)
{
	NRF_ACOMP->TASKS_STOP = 1;
}

void nRF52LPCompPowerOff(AnalogCompDev_t * const pDev)
{

}

extern "C" void COMP_LPCOMP_IRQHandler()
{
	ANALOG_COMP_EVT evt = ANALOG_COMP_EVT_UNKNOWN;

	if (NRF_ACOMP->EVENTS_UP)
	{
		evt = ANALOG_COMP_EVT_HIGHER;
		NRF_ACOMP->EVENTS_UP = 0;
	}

	if (NRF_ACOMP->EVENTS_DOWN)
	{
		evt = ANALOG_COMP_EVT_LOWER;
		NRF_ACOMP->EVENTS_DOWN = 0;
	}

	if (NRF_ACOMP->EVENTS_READY)
	{
		NRF_ACOMP->EVENTS_READY = 0;
	}

	if (NRF_ACOMP->EVENTS_CROSS)
	{
		NRF_ACOMP->EVENTS_CROSS = 0;
	}

	if (s_pnRF5xLPCompDev && s_pnRF5xLPCompDev->EvtHandler)
	{
		s_pnRF5xLPCompDev->EvtHandler(s_pnRF5xLPCompDev, evt);
	}

	NVIC_ClearPendingIRQ(ACOMP_IRQn);
}

bool AnalogCompInit(AnalogCompDev_t * const pDev, const AnalogCompCfg_t *pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 ||
		pCfg->AnalogIn < 0 || pCfg->AnalogIn >= 8 ||
		pCfg->RefVolt <= 0 ||
		(pCfg->Mode != ANALOG_COMP_MODE_SINGLE &&
		 pCfg->Mode != ANALOG_COMP_MODE_CONTINUOUS))
	{
		return false;
	}

	pDev->Mode = pCfg->Mode;
	pDev->RefSrc = pCfg->RefSrc;
	pDev->CompVolt = pCfg->CompVolt;
	pDev->AnalogIn = pCfg->AnalogIn;
	pDev->bHystersys = pCfg->bHystersys;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->Enable = nRF52LPCompEnable;
	pDev->Disable = nRF52LPCompDisable;
	pDev->Start = nRF52LPCompStart;
	pDev->PowerOff = nRF52LPCompPowerOff;

#ifdef ANALOG_COMP_NRF_COMP
	if (pCfg->RefVolt <= 0)
	{
		return false;
	}

	if (pCfg->RefSrc == 0)
	{
		// Internal reference: the band gap one when RefVolt is one of them,
		// VDD at RefVolt otherwise
		switch (pCfg->RefVolt)
		{
			case 1200:
				NRF_COMP->REFSEL = COMP_REFSEL_REFSEL_Int1V2;
				break;
			case 1800:
				NRF_COMP->REFSEL = COMP_REFSEL_REFSEL_Int1V8;
				break;
			case 2400:
				NRF_COMP->REFSEL = COMP_REFSEL_REFSEL_Int2V4;
				break;
			default:
				NRF_COMP->REFSEL = COMP_REFSEL_REFSEL_VDD;
				break;
		}
	}
	else
	{
		NRF_COMP->REFSEL = COMP_REFSEL_REFSEL_ARef;
		NRF_COMP->EXTREFSEL = (pCfg->RefSrc - 1) & COMP_EXTREFSEL_EXTREFSEL_Msk;
	}

	// Single ended. A threshold is (TH + 1) / 64 of the reference, take the
	// closest one. Hysteresis is done with the two thresholds, about 50 mV
	// apart like the LPCOMP hysteresis.
	int th = (pCfg->CompVolt * 64 + pCfg->RefVolt / 2) / pCfg->RefVolt - 1;

	if (th < 0)
	{
		th = 0;
	}
	else if (th > 63)
	{
		th = 63;
	}

	int thup = th;
	int thdown = th;

	if (pCfg->bHystersys == true)
	{
		// Window in 1/64 steps, at least one
		int hy = (50 * 64 + pCfg->RefVolt / 2) / pCfg->RefVolt;

		if (hy < 1)
		{
			hy = 1;
		}
		thdown = th - hy / 2;
		if (thdown < 0)
		{
			thdown = 0;
		}
		thup = thdown + hy;
		if (thup > 63)
		{
			thup = 63;
		}
	}

	NRF_COMP->TH = ((uint32_t)thup << COMP_TH_THUP_Pos) | ((uint32_t)thdown << COMP_TH_THDOWN_Pos);
	NRF_COMP->MODE = (COMP_MODE_SP_Low << COMP_MODE_SP_Pos) | (COMP_MODE_MAIN_SE << COMP_MODE_MAIN_Pos);
	NRF_COMP->PSEL = pCfg->AnalogIn;
#else
	if (pCfg->RefSrc == 0)
	{
#ifdef NRF52_SERIES
		// REFSEL 0 to 6 select VDD * 1/8 to 7/8 and 8 to 15 select VDD * 1/16,
		// 3/16 to 15/16. Take the closest sixteenth from 1/16 to 15/16.
		int d = (pCfg->CompVolt * 16 + pCfg->RefVolt / 2) / pCfg->RefVolt;

		if (d < 1)
		{
			d = 1;
		}
		else if (d > 15)
		{
			d = 15;
		}

		if (d & 1)
		{
			NRF_LPCOMP->REFSEL = (d >> 1) + 8;
		}
		else
		{
			NRF_LPCOMP->REFSEL = (d >> 1) - 1;
		}
#else
		int d = pCfg->CompVolt * 8 / pCfg->RefSrc;

		NRF_LPCOMP->REFSEL = d;
#endif
	}
	else
	{
		NRF_LPCOMP->REFSEL = 7;
		NRF_LPCOMP->EXTREFSEL = (pCfg->RefSrc - 1) & 1;
	}

	NRF_LPCOMP->PSEL = pCfg->AnalogIn;

#ifdef NRF52_SERIES
	if (pCfg->bHystersys == true)
	{
		NRF_LPCOMP->HYST = 1;
	}
	else
	{
		NRF_LPCOMP->HYST = 0;
	}
#endif

#endif

	if (pCfg->Mode == ANALOG_COMP_MODE_CONTINUOUS)
	{
		NRF_ACOMP->SHORTS = ACOMP_SHORTS_READY_SAMPLE_Msk;
	}
	else
	{
		NRF_ACOMP->SHORTS = 0;
	}

#ifndef ANALOG_COMP_NRF_COMP
	// Detect both Up & Down
	NRF_LPCOMP->ANADETECT = LPCOMP_ANADETECT_ANADETECT_Msk;
#endif

	nRF52LPCompEnable(pDev);

#ifndef ANALOG_COMP_NRF_COMP
	// Anomaly 76 workaround
	NRF_LPCOMP->EVENTS_READY = 0;
	NRF_LPCOMP->TASKS_START = 1;
	while (NRF_LPCOMP->EVENTS_READY == 0);

	usDelay(150);

	NRF_LPCOMP->TASKS_STOP = 1;
#endif
	NRF_ACOMP->EVENTS_READY = 0;
	NRF_ACOMP->EVENTS_CROSS = 0;
	NRF_ACOMP->EVENTS_DOWN = 0;
	NRF_ACOMP->EVENTS_UP = 0;

	// Only callback users need the comparator interrupt. PEI uses the
	// peripheral event directly, with no ISR forwarding.
	NRF_ACOMP->INTENCLR = 0xFFFFFFFFU;
	if (pCfg->EvtHandler != NULL)
	{
		NRF_ACOMP->INTENSET = ACOMP_INTENSET_UP_Msk |
			ACOMP_INTENSET_DOWN_Msk;
		NVIC_ClearPendingIRQ(ACOMP_IRQn);
		NVIC_SetPriority(ACOMP_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(ACOMP_IRQn);
	}
	else
	{
		NVIC_DisableIRQ(ACOMP_IRQn);
		NVIC_ClearPendingIRQ(ACOMP_IRQn);
	}
	s_pnRF5xLPCompDev = pDev;
	return true;
}

bool AnalogComp::Init(const AnalogCompCfg_t &Cfg)
{
	vDevData.pPrivate = this;

	return AnalogCompInit(&vDevData, &Cfg);
}

// Expose only configured comparator endpoints. The register maps of COMP
// and LPCOMP share event and task offsets, but the driver selects the
// instance using the selected nRF52 CMSIS header.
static PeiEndpoint_t AnalogCompPeiEventEndpoint(ANALOG_COMP_PEI_EVENT Event)
{
#if defined(NRF52_SERIES)
	switch (Event)
	{
		case ANALOG_COMP_PEI_EVENT_READY:
			return (PeiEndpoint_t)&NRF_ACOMP->EVENTS_READY;
		case ANALOG_COMP_PEI_EVENT_DOWN:
			return (PeiEndpoint_t)&NRF_ACOMP->EVENTS_DOWN;
		case ANALOG_COMP_PEI_EVENT_UP:
			return (PeiEndpoint_t)&NRF_ACOMP->EVENTS_UP;
		case ANALOG_COMP_PEI_EVENT_CROSS:
			return (PeiEndpoint_t)&NRF_ACOMP->EVENTS_CROSS;
		default:
			return 0U;
	}
#else
	(void)Event;
	return 0U;
#endif
}

static PeiEndpoint_t AnalogCompPeiTaskEndpoint(ANALOG_COMP_PEI_TASK Task)
{
#if defined(NRF52_SERIES)
	switch (Task)
	{
		case ANALOG_COMP_PEI_TASK_START:
			return (PeiEndpoint_t)&NRF_ACOMP->TASKS_START;
		case ANALOG_COMP_PEI_TASK_STOP:
			return (PeiEndpoint_t)&NRF_ACOMP->TASKS_STOP;
		case ANALOG_COMP_PEI_TASK_SAMPLE:
			return (PeiEndpoint_t)&NRF_ACOMP->TASKS_SAMPLE;
		default:
			return 0U;
	}
#else
	(void)Task;
	return 0U;
#endif
}

static bool AnalogCompPeiReady(AnalogCompDev_t *pDev)
{
#if defined(NRF52_SERIES)
	return pDev != NULL && s_pnRF5xLPCompDev == pDev &&
		pDev->Enable == nRF52LPCompEnable &&
		NRF_ACOMP->ENABLE == ACOMP_ENABLE_Enabled;
#else
	(void)pDev;
	return false;
#endif
}

bool AnalogCompPeiBindEvent(AnalogCompDev_t *pDev,
	PeiDev_t *pPei, PeiEvent_t *pEvt, ANALOG_COMP_PEI_EVENT Event)
{
	PeiEndpoint_t endpoint;

	if (pPei == NULL || pEvt == NULL || PeiBindEvent == NULL ||
		!AnalogCompPeiReady(pDev))
	{
		return false;
	}
	endpoint = AnalogCompPeiEventEndpoint(Event);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiBindEvent(pPei, pEvt, endpoint);
}

bool AnalogCompPeiUnbindEvent(AnalogCompDev_t *pDev,
	PeiDev_t *pPei, PeiEvent_t Evt)
{
	PeiEndpoint_t endpoint;
	int event;

	if (pDev == NULL || pPei == NULL || s_pnRF5xLPCompDev != pDev ||
		PeiGetEventEndpoint == NULL || PeiUnbindEvent == NULL)
	{
		return false;
	}
	endpoint = PeiGetEventEndpoint(pPei, Evt);
	if (endpoint == 0U)
	{
		return false;
	}
	for (event = ANALOG_COMP_PEI_EVENT_READY;
		event <= ANALOG_COMP_PEI_EVENT_CROSS; event++)
	{
		if (endpoint == AnalogCompPeiEventEndpoint(
			(ANALOG_COMP_PEI_EVENT)event))
		{
			return PeiUnbindEvent(pPei, Evt, endpoint);
		}
	}
	return false;
}

bool AnalogCompPeiConnectTask(AnalogCompDev_t *pDev,
	PeiDev_t *pPei, PeiEvent_t Evt, ANALOG_COMP_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pPei == NULL || PeiConnectTask == NULL ||
		!AnalogCompPeiReady(pDev))
	{
		return false;
	}
	// Continuous mode already schedules SAMPLE using READY->SAMPLE.
	// Do not mix external PPI SAMPLE tasks with that shortcut.
	if (Task == ANALOG_COMP_PEI_TASK_SAMPLE &&
		pDev->Mode == ANALOG_COMP_MODE_CONTINUOUS)
	{
		return false;
	}
	endpoint = AnalogCompPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiConnectTask(pPei, Evt, endpoint);
}

bool AnalogCompPeiDisconnectTask(AnalogCompDev_t *pDev,
	PeiDev_t *pPei, PeiEvent_t Evt, ANALOG_COMP_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	// Teardown must work after the comparator is stopped or disabled.
	if (pDev == NULL || pPei == NULL || s_pnRF5xLPCompDev != pDev ||
		PeiDisconnectTask == NULL)
	{
		return false;
	}
	endpoint = AnalogCompPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiDisconnectTask(pPei, Evt, endpoint);
}
