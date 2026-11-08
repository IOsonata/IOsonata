/**-------------------------------------------------------------------------
@file	qdec_nrfx.cpp

@brief	Quadrature decoder implementation on Nordic nRF52

The QDEC samples the A and B inputs every SAMPLEPER (128 us to 131 ms) and
adds each valid transition, +1 or -1, to ACC (-1024 to 1023). A sample where
both inputs changed is counted in ACCDBL (0 to 15) instead. After REPORTPER
samples (1, 10, 40 to 280) it generates REPORTRDY if ACC is not 0 and DBLRDY
if ACCDBL is not 0. ACCOF is generated when a sample is lost because ACC or
ACCDBL is full.

The interrupt handler reads and clears ACC and ACCDBL with READCLRACC and
adds them to the device position and error count. The application reads
them without clearing the hardware, adding ACC and ACCDBL as they are at that
time, so only the interrupt handler clears them. The report period is at
most 280 samples, ACC cannot overflow between two reports.

The nRF5 SDK Mesh bearer (bearer_event.c) uses the QDEC interrupt as its
event interrupt and also defines QDEC_IRQHandler. An application cannot use
both, the link fails with two definitions.

@author	Hoang Nguyen Hoan
@date	Nov. 7, 2026

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
#include <stddef.h>

#include "nrf.h"
#include "coredev/iopincfg.h"
#include "coredev/qdec.h"

// Optional PPI matrix: plain QDEC firmware links no PEI code.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiUnbindEvent
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

#ifndef NRF_QDEC
#error "qdec_nrfx: no single QDEC instance on this MCU"
#endif

// Shortest sampling period, SAMPLEPER n is QDEC_NRFX_SMPL_MIN << n
#define QDEC_NRFX_SMPL_MIN		128UL
#define QDEC_NRFX_SMPL_MAX_SEL	QDEC_SAMPLEPER_SAMPLEPER_131ms

// Longest LED lead time
#define QDEC_NRFX_LEDPRE_MAX	(QDEC_LEDPRE_LEDPRE_Msk >> QDEC_LEDPRE_LEDPRE_Pos)

// Pin select value of an unused pin
#define QDEC_NRFX_PSEL_NONE		0xFFFFFFFFUL

// Interrupts used by the driver
#define QDEC_NRFX_INT_MSK		(QDEC_INTENSET_REPORTRDY_Msk | QDEC_INTENSET_DBLRDY_Msk | \
								 QDEC_INTENSET_ACCOF_Msk)

// Samples per report for each REPORTPER value
typedef struct {
	uint32_t NbSmpl;
	uint32_t Sel;
} QdecnRFReportPer_t;

static const QdecnRFReportPer_t s_QdecnRFReportPer[] = {
#ifdef QDEC_REPORTPER_REPORTPER_1Smpl
	{ 1, QDEC_REPORTPER_REPORTPER_1Smpl },
#endif
	{ 10, QDEC_REPORTPER_REPORTPER_10Smpl },
	{ 40, QDEC_REPORTPER_REPORTPER_40Smpl },
	{ 80, QDEC_REPORTPER_REPORTPER_80Smpl },
	{ 120, QDEC_REPORTPER_REPORTPER_120Smpl },
	{ 160, QDEC_REPORTPER_REPORTPER_160Smpl },
	{ 200, QDEC_REPORTPER_REPORTPER_200Smpl },
	{ 240, QDEC_REPORTPER_REPORTPER_240Smpl },
	{ 280, QDEC_REPORTPER_REPORTPER_280Smpl },
};

static QdecDev_t *s_pQdecnRFDev = NULL;

// Position at the last movement event
static int32_t s_QdecnRFEvtPos = 0;

extern "C" void QDEC_IRQHandler(void);

static uint32_t QdecnRFAbsDiff(uint32_t a, uint32_t b)
{
	return a > b ? a - b : b - a;
}

// Read and clear ACC and ACCDBL into the device data. Called with the QDEC
// interrupt masked or from its handler.
static void QdecnRFFold(QdecDev_t * const pDev)
{
	NRF_QDEC->TASKS_READCLRACC = 1;

	pDev->Position += NRF_QDEC->ACCREAD;
	pDev->ErrCnt += NRF_QDEC->ACCDBLREAD;
}

bool QdecInit(QdecDev_t * const pDev, const QdecCfg_t * const pCfg)
{
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 || pCfg->pPins == NULL ||
		pCfg->NbPins < 2)
	{
		return false;
	}

	NVIC_DisableIRQ(QDEC_IRQn);
	NRF_QDEC->INTENCLR = 0xFFFFFFFFUL;

	if (NRF_QDEC->ENABLE)
	{
		NRF_QDEC->TASKS_STOP = 1;
		NRF_QDEC->ENABLE = 0;
	}

	// Closest sampling period
	uint32_t sel = 0;

	for (uint32_t i = 1; i <= QDEC_NRFX_SMPL_MAX_SEL; i++)
	{
		if (QdecnRFAbsDiff(QDEC_NRFX_SMPL_MIN << i, pCfg->usSamplePeriod) <
			QdecnRFAbsDiff(QDEC_NRFX_SMPL_MIN << sel, pCfg->usSamplePeriod))
		{
			sel = i;
		}
	}

	uint32_t smplper = QDEC_NRFX_SMPL_MIN << sel;

	NRF_QDEC->SAMPLEPER = sel;

	// Closest number of samples per report
	uint32_t nbsmpl = (pCfg->usReportPeriod + smplper / 2) / smplper;
	int rsel = 0;

	for (int i = 1; i < (int)(sizeof(s_QdecnRFReportPer) / sizeof(s_QdecnRFReportPer[0])); i++)
	{
		if (QdecnRFAbsDiff(s_QdecnRFReportPer[i].NbSmpl, nbsmpl) <
			QdecnRFAbsDiff(s_QdecnRFReportPer[rsel].NbSmpl, nbsmpl))
		{
			rsel = i;
		}
	}

	NRF_QDEC->REPORTPER = s_QdecnRFReportPer[rsel].Sel;

	IOPinCfg(pCfg->pPins, pCfg->NbPins);

	NRF_QDEC->PSEL.A = (pCfg->pPins[QDEC_PHA_PIN_IDX].PortNo << 5) | pCfg->pPins[QDEC_PHA_PIN_IDX].PinNo;
	NRF_QDEC->PSEL.B = (pCfg->pPins[QDEC_PHB_PIN_IDX].PortNo << 5) | pCfg->pPins[QDEC_PHB_PIN_IDX].PinNo;

	if (pCfg->NbPins > QDEC_LED_PIN_IDX && pCfg->pPins[QDEC_LED_PIN_IDX].PinNo >= 0)
	{
		// The LED is turned on before the sample, within the sampling period
		uint32_t ledpre = pCfg->usLedLead;

		if (ledpre > QDEC_NRFX_LEDPRE_MAX)
		{
			ledpre = QDEC_NRFX_LEDPRE_MAX;
		}
		if (ledpre >= smplper)
		{
			ledpre = smplper - 1;
		}

		NRF_QDEC->PSEL.LED = (pCfg->pPins[QDEC_LED_PIN_IDX].PortNo << 5) | pCfg->pPins[QDEC_LED_PIN_IDX].PinNo;
		NRF_QDEC->LEDPRE = ledpre;
		NRF_QDEC->LEDPOL = pCfg->bLedActHigh ? QDEC_LEDPOL_LEDPOL_ActiveHigh : QDEC_LEDPOL_LEDPOL_ActiveLow;
	}
	else
	{
		NRF_QDEC->PSEL.LED = QDEC_NRFX_PSEL_NONE;
		NRF_QDEC->LEDPRE = 0;
	}

	NRF_QDEC->DBFEN = pCfg->bDebounce ? QDEC_DBFEN_DBFEN_Enabled : QDEC_DBFEN_DBFEN_Disabled;
	NRF_QDEC->SHORTS = 0;

	pDev->DevNo = 0;
	pDev->usSamplePeriod = smplper;
	pDev->usReportPeriod = smplper * s_QdecnRFReportPer[rsel].NbSmpl;
	pDev->Position = 0;
	pDev->ErrCnt = 0;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->pDevData = (void *)NRF_QDEC;

	s_pQdecnRFDev = pDev;
	s_QdecnRFEvtPos = 0;

	NRF_QDEC->EVENTS_SAMPLERDY = 0;
	NRF_QDEC->EVENTS_REPORTRDY = 0;
	NRF_QDEC->EVENTS_ACCOF = 0;
	NRF_QDEC->EVENTS_DBLRDY = 0;
	NRF_QDEC->EVENTS_STOPPED = 0;

	// Always used, they keep ACC from overflowing
	NRF_QDEC->INTENSET = QDEC_NRFX_INT_MSK;
	NVIC_ClearPendingIRQ(QDEC_IRQn);
	NVIC_SetPriority(QDEC_IRQn, pCfg->IntPrio);
	NVIC_EnableIRQ(QDEC_IRQn);

	return true;
}

bool QdecStart(QdecDev_t * const pDev)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return false;
	}

	NRF_QDEC->ENABLE = QDEC_ENABLE_ENABLE_Enabled;
	NRF_QDEC->TASKS_START = 1;

	return true;
}

void QdecStop(QdecDev_t * const pDev)
{
	if (pDev == NULL || pDev->pDevData == NULL || NRF_QDEC->ENABLE == 0)
	{
		return;
	}

	NVIC_DisableIRQ(QDEC_IRQn);

	NRF_QDEC->EVENTS_STOPPED = 0;
	NRF_QDEC->TASKS_STOP = 1;

	// Wait for STOPPED so that no sample is added after the read below. The
	// wait is bounded in case the event does not come.
	for (uint32_t i = 0; i < 100000UL && NRF_QDEC->EVENTS_STOPPED == 0; i++);

	// Keep the counts of the last report period, the next event includes them
	QdecnRFFold(pDev);

	NRF_QDEC->EVENTS_STOPPED = 0;
	NRF_QDEC->EVENTS_REPORTRDY = 0;
	NRF_QDEC->EVENTS_ACCOF = 0;
	NRF_QDEC->EVENTS_DBLRDY = 0;
	NRF_QDEC->ENABLE = 0;

	NVIC_ClearPendingIRQ(QDEC_IRQn);
	NVIC_EnableIRQ(QDEC_IRQn);
}

int32_t QdecPosition(QdecDev_t * const pDev)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return 0;
	}

	NVIC_DisableIRQ(QDEC_IRQn);

	int32_t pos = pDev->Position;

	if (NRF_QDEC->ENABLE)
	{
		pos += NRF_QDEC->ACC;
	}

	NVIC_EnableIRQ(QDEC_IRQn);

	return pos;
}

void QdecSetPosition(QdecDev_t * const pDev, int32_t Pos)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return;
	}

	NVIC_DisableIRQ(QDEC_IRQn);

	// ACC is added at the next report, so it is taken out here
	pDev->Position = Pos - (NRF_QDEC->ENABLE ? NRF_QDEC->ACC : 0);
	s_QdecnRFEvtPos = Pos;

	NVIC_EnableIRQ(QDEC_IRQn);
}

uint32_t QdecErrCount(QdecDev_t * const pDev)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return 0;
	}

	NVIC_DisableIRQ(QDEC_IRQn);

	uint32_t cnt = pDev->ErrCnt;

	if (NRF_QDEC->ENABLE)
	{
		cnt += NRF_QDEC->ACCDBL;
	}

	NVIC_EnableIRQ(QDEC_IRQn);

	return cnt;
}

void QDEC_IRQHandler(void)
{
	bool report = NRF_QDEC->EVENTS_REPORTRDY || NRF_QDEC->EVENTS_DBLRDY;

	NRF_QDEC->EVENTS_REPORTRDY = 0;
	NRF_QDEC->EVENTS_DBLRDY = 0;
	NRF_QDEC->EVENTS_ACCOF = 0;

	QdecDev_t *dev = s_pQdecnRFDev;

	if (dev == NULL)
	{
		return;
	}

	QdecnRFFold(dev);

	if (report == true)
	{
		int32_t cnt = dev->Position - s_QdecnRFEvtPos;

		if (cnt != 0)
		{
			s_QdecnRFEvtPos = dev->Position;

			if (dev->EvtHandler != NULL)
			{
				dev->EvtHandler(dev, cnt);
			}
		}
	}
}


// The driver owns QDEC's accumulator and READCLRACC task. Hardware PEI
// may start/stop sampling and observe events, but never clears ACC.
static bool QdecPeiOwner(const QdecDev_t *pDev)
{
#if defined(NRF52_SERIES)
	return pDev != NULL && pDev == s_pQdecnRFDev &&
		pDev->pDevData == (void *)NRF_QDEC && pDev->DevNo == 0;
#else
	(void)pDev;
	return false;
#endif
}

static PeiEndpoint_t QdecPeiEventEndpoint(QDEC_PEI_EVENT Event)
{
#if defined(NRF52_SERIES)
	switch (Event)
	{
		case QDEC_PEI_EVENT_SAMPLERDY:
			return (PeiEndpoint_t)&NRF_QDEC->EVENTS_SAMPLERDY;
		case QDEC_PEI_EVENT_REPORTRDY:
			return (PeiEndpoint_t)&NRF_QDEC->EVENTS_REPORTRDY;
		case QDEC_PEI_EVENT_ACCOF:
			return (PeiEndpoint_t)&NRF_QDEC->EVENTS_ACCOF;
		case QDEC_PEI_EVENT_DBLRDY:
			return (PeiEndpoint_t)&NRF_QDEC->EVENTS_DBLRDY;
		case QDEC_PEI_EVENT_STOPPED:
			return (PeiEndpoint_t)&NRF_QDEC->EVENTS_STOPPED;
		default:
			return 0U;
	}
#else
	(void)Event;
	return 0U;
#endif
}

static PeiEndpoint_t QdecPeiTaskEndpoint(QDEC_PEI_TASK Task)
{
#if defined(NRF52_SERIES)
	switch (Task)
	{
		case QDEC_PEI_TASK_START:
			return (PeiEndpoint_t)&NRF_QDEC->TASKS_START;
		case QDEC_PEI_TASK_STOP:
			return (PeiEndpoint_t)&NRF_QDEC->TASKS_STOP;
		default:
			return 0U;
	}
#else
	(void)Task;
	return 0U;
#endif
}

extern "C" bool QdecPreparePeiStart(QdecDev_t *pDev)
{
#if defined(NRF52_SERIES)
	if (!QdecPeiOwner(pDev))
	{
		return false;
	}
	// Initialize the hardware for a subsequent PPI-driven START task;
	// keep the normal REPORTRDY/DBLRDY/ACCOF IRQs enabled so totals
	// continue to be folded by the existing QDEC driver.
	NRF_QDEC->ENABLE = QDEC_ENABLE_ENABLE_Enabled;
	return true;
#else
	(void)pDev;
	return false;
#endif
}

extern "C" bool QdecEnablePeiEvent(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t *pEvt, QDEC_PEI_EVENT Event)
{
	PeiEndpoint_t endpoint;

	if (!QdecPeiOwner(pDev) || pPei == NULL || pEvt == NULL ||
		PeiBindEvent == NULL)
	{
		return false;
	}
	endpoint = QdecPeiEventEndpoint(Event);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiBindEvent(pPei, pEvt, endpoint);
}

extern "C" bool QdecDisablePeiEvent(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt)
{
	PeiEndpoint_t endpoint;
	int event;

	if (!QdecPeiOwner(pDev) || pPei == NULL ||
		PeiGetEventEndpoint == NULL || PeiUnbindEvent == NULL)
	{
		return false;
	}
	endpoint = PeiGetEventEndpoint(pPei, Evt);
	for (event = QDEC_PEI_EVENT_SAMPLERDY;
		event <= QDEC_PEI_EVENT_STOPPED; event++)
	{
		if (endpoint != 0U &&
			endpoint == QdecPeiEventEndpoint((QDEC_PEI_EVENT)event))
		{
			return PeiUnbindEvent(pPei, Evt, endpoint);
		}
	}
	return false;
}

extern "C" bool QdecConnectPeiTask(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, QDEC_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (!QdecPeiOwner(pDev) || pPei == NULL ||
		PeiConnectTask == NULL)
	{
		return false;
	}
#if defined(NRF52_SERIES)
	if (NRF_QDEC->ENABLE != QDEC_ENABLE_ENABLE_Enabled)
	{
		return false;
	}
#endif
	endpoint = QdecPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiConnectTask(pPei, Evt, endpoint);
}

extern "C" bool QdecDisconnectPeiTask(QdecDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, QDEC_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (!QdecPeiOwner(pDev) || pPei == NULL ||
		PeiDisconnectTask == NULL)
	{
		return false;
	}
	endpoint = QdecPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiDisconnectTask(pPei, Evt, endpoint);
}
