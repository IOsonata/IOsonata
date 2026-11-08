/**-------------------------------------------------------------------------
@file	pdm_nrfx.h

@brief	Implementation of Pulse density modulation interface on nRFx series


@author	Hoang Nguyen Hoan
@date	May 19, 2019

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
#include "nrf.h"

#include "idelay.h"
#include "coredev/pdm.h"

// Optional interconnect functions must not extract PEI from a PDM-only
// application, even though the PDM driver itself is compiled from C++.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiUnbindEvent
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

// The MDK indexes the PDM instance on nRF53 but not on nRF91, so the secure
// alias and the interrupt number are spelled differently on the two families.
#if defined(NRF53_SERIES)
#define NRF_PDM			NRF_PDM0_S
#define PDM_IRQn		PDM0_IRQn
#elif defined(NRF91_SERIES)
#define NRF_PDM			NRF_PDM_S
#endif

// Only one DPM device on chip
alignas(4) static PdmDev_t *s_pnRFPdmDev = nullptr;

bool PdmEnable(PdmDev_t *pDev)
{
	NRF_PDM->ENABLE = 1;

	return true;
}

void PdmDisable(PdmDev_t *pDev)
{
	NRF_PDM->TASKS_STOP = 1;
	NRF_PDM->ENABLE = 0;
}

bool PdmStart(PdmDev_t *pDev)
{
	uint8_t *p = CFifoPut(pDev->hFifo);
	if (p)
	{
		NRF_PDM->SAMPLE.PTR = (uint32_t)p;
		NRF_PDM->SAMPLE.MAXCNT = pDev->NbSamples;


		NRF_PDM->TASKS_START = 1;

		return true;
	}

	return false;
}

void PdmStop(PdmDev_t *pDev)
{
	NRF_PDM->TASKS_STOP = 1;

}

uint16_t *PdmGetSamples(PdmDev_t *pDev)
{
	return (uint16_t*)CFifoGet(pDev->hFifo);
}

void PdmSetMode(PdmDev_t *pDev, PDM_OPMODE Mode)
{
	NRF_PDM->TASKS_STOP = 1;
	msDelay(1);
	//NRF_PDM->ENABLE = 0;

	uint32_t d = NRF_PDM->MODE & ~(PDM_MODE_OPERATION_Msk | PDM_MODE_EDGE_Msk);

	pDev->NbSamples = pDev->hFifo->BlkSize >> 1;

	switch (Mode)
	{
		case PDM_OPMODE_MONO_LEFT:
			d |= PDM_MODE_OPERATION_Msk;
			break;
		case PDM_OPMODE_MONO_RIGHT:
			d |= PDM_MODE_OPERATION_Msk | PDM_MODE_EDGE_Msk;
			break;
		case PDM_OPMODE_STEREO:
			break;
	}
	NRF_PDM->MODE = d;

	//NRF_PDM->ENABLE = 1;
}

uint32_t PdmSetClockFrequency(uint32_t Freq)
{
	uint32_t clkfreq = PDM_PDMCLKCTRL_FREQ_Default;

	if (Freq < 1032000)
	{
		Freq = 1000000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_1000K;
	}
	else if (Freq < 1067000)
	{
		Freq = 1032000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_Default;
	}
#ifdef PDM_PDMCLKCTRL_FREQ_1231K
	else if (Freq < 1231000)
	{
		Freq = 1067000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_1067K;
	}
	else if (Freq < 1280000)
	{
		Freq = 1231000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_1231K;
	}
	else if (Freq < 1333000)
	{
		Freq = 1280000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_1280K;
	}
	else
	{
		Freq = 1333000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_1333K;
	}
#else
	else
	{
		Freq = 1067000;
		clkfreq = PDM_PDMCLKCTRL_FREQ_1067K;
	}
#endif

	NRF_PDM->PDMCLKCTRL = clkfreq;

	return Freq;
}

void PdmPowerOff(PdmDev_t * const pDev)
{
	*(volatile uint32_t *)((uint32_t)NRF_PDM + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_PDM + 0xFFC) = 1;
	*(volatile uint32_t *)((uint32_t)NRF_PDM + 0xFFC) = 0;
}

bool PdmInit(PdmDev_t * const pDev, const PdmCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->pPins == nullptr || pCfg->NbPins < 2 || pCfg->pFifoMem == nullptr || pCfg->FifoMemSize == 0)
	{
		return false;
	}

	// Force power on in case it was powered off previously
	*(volatile uint32_t *)((uint32_t)NRF_PDM + 0xFFC);
	*(volatile uint32_t *)((uint32_t)NRF_PDM + 0xFFC) = 1;

	pDev->CfgData = *pCfg;
	s_pnRFPdmDev = pDev;

	pDev->hFifo = CFifoInit(pCfg->pFifoMem, pCfg->FifoMemSize, pCfg->FifoBlkSize, false);
	if (pDev->hFifo == nullptr || pCfg->FifoBlkSize < 2U ||
		(pCfg->FifoBlkSize & 1U) != 0U)
	{
		s_pnRFPdmDev = nullptr;
		return false;
	}

	IOPinCfg(pCfg->pPins, pCfg->NbPins);
	NRF_PDM->PSEL.CLK = (pCfg->pPins[PDM_CLKPIN_IDX].PortNo << 5) | pCfg->pPins[PDM_CLKPIN_IDX].PinNo;
	NRF_PDM->PSEL.DIN = (pCfg->pPins[PDM_DINPIN_IDX].PortNo << 5) | pCfg->pPins[PDM_DINPIN_IDX].PinNo;

	uint32_t mode = 0;

	if (pCfg->SmplMode == PDM_SMPLMODE_RISING)
	{
		mode |= PDM_MODE_EDGE_Msk;
	}

	pDev->NbSamples = pCfg->FifoBlkSize >> 1;
/*
	if (pCfg->OpMode == PDM_OPMODE_MONO)
	{
//		pDev->NbSamples = pCfg->FifoBlkSize / 2;
		mode |= PDM_MODE_OPERATION_Msk;
	}
//	else
//	{
//		pDev->NbSamples = pCfg->FifoBlkSize / 4;
//	}

	NRF_PDM->MODE = mode;
*/
	PdmSetMode(pDev, pCfg->OpMode);

	NRF_PDM->GAINL = pCfg->GainLeft & 0x7F;
	NRF_PDM->GAINR = pCfg->GainRight & 0x7F;

	pDev->CfgData.Freq = PdmSetClockFrequency(pCfg->Freq);

	NRF_PDM->EVENTS_STARTED = 0;
	NRF_PDM->EVENTS_END = 0;
	NRF_PDM->EVENTS_STOPPED = 0;
	NRF_PDM->SAMPLE.PTR = 0U;
	NRF_PDM->SAMPLE.MAXCNT = 0U;
	NRF_PDM->INTENCLR = 0xFFFFFFFFU;
	NVIC_DisableIRQ(PDM_IRQn);
	NVIC_ClearPendingIRQ(PDM_IRQn);

	if (pCfg->bIntEn)
	{
		NRF_PDM->INTEN = PDM_INTEN_END_Msk | PDM_INTEN_STARTED_Msk | PDM_INTEN_STOPPED_Msk;

		NVIC_ClearPendingIRQ(PDM_IRQn);
		NVIC_SetPriority(PDM_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(PDM_IRQn);
	}

	NRF_PDM->ENABLE = 1;

	return true;
}

void PdmIrqHandler(PdmDev_t *pDev)
{
	if (NRF_PDM->EVENTS_END)
	{
		// Data transfer completed
		if (pDev->CfgData.EvtHandler)
		{
			pDev->CfgData.EvtHandler(pDev, DEVINTRF_EVT_RX_DATA);
		}

		NRF_PDM->EVENTS_END = 0;
	}
	if (NRF_PDM->EVENTS_STARTED)
	{
		NRF_PDM->EVENTS_STARTED = 0;

		// SAMPLE.PTR and MAXCNT are double buffered. The PDM took the current
		// block when it generated STARTED and continues into the values
		// written now when that block is full. Writing them on END was too
		// late, the next block went into the same buffer again.
		uint8_t *p = CFifoPut(pDev->hFifo);
		if (p)
		{
			NRF_PDM->SAMPLE.PTR = (uint32_t)p;
			NRF_PDM->SAMPLE.MAXCNT = pDev->NbSamples;
		}
	}
	if (NRF_PDM->EVENTS_STOPPED)
	{
		NRF_PDM->EVENTS_STOPPED = 0;
		CFifoFlush(pDev->hFifo);
	}
}

extern "C" void PDM_IRQHandler()
{
	if (s_pnRFPdmDev)
	{
		PdmIrqHandler(s_pnRFPdmDev);
	}

	NVIC_ClearPendingIRQ(PDM_IRQn);
}


// Hardware PEI routing on nRF52. The normal driver owns the FIFO and
// refreshes the double-buffered SAMPLE.PTR in the STARTED interrupt.
// No PEI callback ever forwards events to tasks.
static PeiEndpoint_t PdmPeiEventEndpoint(PDM_PEI_EVENT Event)
{
#if defined(NRF52_SERIES)
	switch (Event)
	{
		case PDM_PEI_EVENT_STARTED:
			return (PeiEndpoint_t)&NRF_PDM->EVENTS_STARTED;
		case PDM_PEI_EVENT_END:
			return (PeiEndpoint_t)&NRF_PDM->EVENTS_END;
		case PDM_PEI_EVENT_STOPPED:
			return (PeiEndpoint_t)&NRF_PDM->EVENTS_STOPPED;
		default:
			return 0U;
	}
#else
	(void)Event;
	return 0U;
#endif
}

static PeiEndpoint_t PdmPeiTaskEndpoint(PDM_PEI_TASK Task)
{
#if defined(NRF52_SERIES)
	switch (Task)
	{
		case PDM_PEI_TASK_START:
			return (PeiEndpoint_t)&NRF_PDM->TASKS_START;
		case PDM_PEI_TASK_STOP:
			return (PeiEndpoint_t)&NRF_PDM->TASKS_STOP;
		default:
			return 0U;
	}
#else
	(void)Task;
	return 0U;
#endif
}

static bool PdmPeiReady(PdmDev_t *pDev)
{
#if defined(NRF52_SERIES)
	if (pDev == nullptr || pDev != s_pnRFPdmDev ||
		pDev->hFifo == nullptr || !pDev->CfgData.bIntEn ||
		pDev->NbSamples <= 0 || NRF_PDM->ENABLE == 0U ||
		NRF_PDM->SAMPLE.PTR == 0U ||
		NRF_PDM->SAMPLE.MAXCNT != (uint32_t)pDev->NbSamples)
	{
		return false;
	}
	return true;
#else
	(void)pDev;
	return false;
#endif
}

extern "C" bool PdmPreparePeiStart(PdmDev_t *pDev)
{
#if defined(NRF52_SERIES)
	uint8_t *p;

	if (pDev == nullptr || pDev != s_pnRFPdmDev ||
		pDev->hFifo == nullptr || !pDev->CfgData.bIntEn ||
		pDev->NbSamples <= 0 || NRF_PDM->ENABLE == 0U ||
		NRF_PDM->SAMPLE.PTR != 0U)
	{
		return false;
	}
	p = CFifoPut(pDev->hFifo);
	if (p == nullptr)
	{
		return false;
	}
	NRF_PDM->SAMPLE.PTR = (uint32_t)(uintptr_t)p;
	NRF_PDM->SAMPLE.MAXCNT = (uint32_t)pDev->NbSamples;
	NRF_PDM->EVENTS_STARTED = 0U;
	NRF_PDM->EVENTS_END = 0U;
	NRF_PDM->EVENTS_STOPPED = 0U;
	return true;
#else
	(void)pDev;
	return false;
#endif
}

extern "C" bool PdmEnablePeiEvent(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t *pEvt, PDM_PEI_EVENT Event)
{
	PeiEndpoint_t endpoint;

	if (pPei == nullptr || pEvt == nullptr || PeiBindEvent == nullptr ||
		!PdmPeiReady(pDev))
	{
		return false;
	}
	endpoint = PdmPeiEventEndpoint(Event);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiBindEvent(pPei, pEvt, endpoint);
}

extern "C" bool PdmDisablePeiEvent(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt)
{
	PeiEndpoint_t endpoint;
	int i;

	if (pPei == nullptr || pDev == nullptr ||
		pDev != s_pnRFPdmDev ||
		PeiGetEventEndpoint == nullptr || PeiUnbindEvent == nullptr)
	{
		return false;
	}
	endpoint = PeiGetEventEndpoint(pPei, Evt);
	for (i = PDM_PEI_EVENT_STARTED; i <= PDM_PEI_EVENT_STOPPED; i++)
	{
		if (endpoint != 0U &&
			endpoint == PdmPeiEventEndpoint((PDM_PEI_EVENT)i))
		{
			return PeiUnbindEvent(pPei, Evt, endpoint);
		}
	}
	return false;
}

extern "C" bool PdmConnectPeiTask(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, PDM_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pPei == nullptr || PeiConnectTask == nullptr ||
		!PdmPeiReady(pDev))
	{
		return false;
	}
	endpoint = PdmPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiConnectTask(pPei, Evt, endpoint);
}

extern "C" bool PdmDisconnectPeiTask(PdmDev_t *pDev, PeiDev_t *pPei,
	PeiEvent_t Evt, PDM_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pPei == nullptr || PeiDisconnectTask == nullptr ||
		pDev == nullptr || pDev != s_pnRFPdmDev)
	{
		return false;
	}
	endpoint = PdmPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiDisconnectTask(pPei, Evt, endpoint);
}
