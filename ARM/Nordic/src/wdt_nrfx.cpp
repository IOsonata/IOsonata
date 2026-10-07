/**-------------------------------------------------------------------------
@file	wdt_nrfx.cpp

@brief	Watchdog timer implementation on Nordic nRF52 and nRF91

The WDT counts down from CRV on the 32.768 kHz low frequency clock, which it
starts on its own (LFRC) when nothing else runs it. The timeout is
(CRV + 1) / 32768 s. Each enabled reload request register (RR[0] to RR[7])
is a channel: the counter reloads once every enabled one was written.

Once started, the WDT only stops with a reset, and CRV, RREN and CONFIG
cannot be written while it runs. The reset follows the TIMEOUT event by two
low frequency clock cycles.

@author	Hoang Nguyen Hoan
@date	Oct. 7, 2026

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
#include "coredev/wdt.h"

// The nRF91 names its peripherals by security state and the application runs
// secure
#if !defined(NRF_WDT) && defined(NRF_WDT_S)
#define NRF_WDT				NRF_WDT_S
#endif

#ifndef NRF_WDT
#error "wdt_nrfx: no single WDT instance on this MCU"
#endif

// Reload request registers
#define WDT_NRFX_CHAN_MAX	8

// Smallest counter reload value
#define WDT_NRFX_CRV_MIN	0xFUL

// Low frequency clock of the counter
#define WDT_NRFX_CLK		32768ULL

// Run status bit, named differently on nRF52 and nRF91
#ifdef WDT_RUNSTATUS_RUNSTATUSWDT_Msk
#define WDT_NRFX_RUNNING	WDT_RUNSTATUS_RUNSTATUSWDT_Msk
#else
#define WDT_NRFX_RUNNING	WDT_RUNSTATUS_RUNSTATUS_Msk
#endif

static WdtDev_t *s_pWdtnRFDev = NULL;

extern "C" void WDT_IRQHandler(void);

// Timeout of a CRV value in msec, rounded up
static uint32_t WdtnRFCrvToMs(uint32_t Crv)
{
	return (uint32_t)((((uint64_t)Crv + 1ULL) * 1000ULL + WDT_NRFX_CLK - 1ULL) / WDT_NRFX_CLK);
}

bool WdtInit(WdtDev_t * const pDev, const WdtCfg_t * const pCfg)
{
	// No window on this WDT
	if (pDev == NULL || pCfg == NULL || pCfg->DevNo != 0 || pCfg->msTimeout == 0 ||
		pCfg->msWindow != 0)
	{
		return false;
	}

	int nbchan = pCfg->NbChan > 0 ? pCfg->NbChan : 1;

	if (nbchan > WDT_NRFX_CHAN_MAX)
	{
		return false;
	}

	pDev->DevNo = 0;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->pDevData = (void *)NRF_WDT;

	NRF_WDT->INTENCLR = WDT_INTENSET_TIMEOUT_Msk;
	NVIC_DisableIRQ(WDT_IRQn);

	if (NRF_WDT->RUNSTATUS & WDT_NRFX_RUNNING)
	{
		// Started before: its configuration is locked until a reset, report it
		uint32_t rren = NRF_WDT->RREN & ((1UL << WDT_NRFX_CHAN_MAX) - 1);

		pDev->NbChan = rren != 0 ? 32 - __CLZ(rren) : 0;
		pDev->msTimeout = WdtnRFCrvToMs(NRF_WDT->CRV);
	}
	else
	{
		uint64_t crv = ((uint64_t)pCfg->msTimeout * WDT_NRFX_CLK) / 1000ULL;

		// Timeout is (CRV + 1) clock cycles
		crv = crv > 0 ? crv - 1 : 0;
		if (crv < WDT_NRFX_CRV_MIN)
		{
			crv = WDT_NRFX_CRV_MIN;
		}
		if (crv > 0xFFFFFFFFULL)
		{
			crv = 0xFFFFFFFFULL;
		}

		NRF_WDT->CRV = (uint32_t)crv;
		NRF_WDT->RREN = (1UL << nbchan) - 1;
		NRF_WDT->CONFIG = (pCfg->bRunSleep ? (WDT_CONFIG_SLEEP_Run << WDT_CONFIG_SLEEP_Pos) : 0) |
						  (pCfg->bRunHalt ? (WDT_CONFIG_HALT_Run << WDT_CONFIG_HALT_Pos) : 0);

		pDev->NbChan = nbchan;
		pDev->msTimeout = WdtnRFCrvToMs((uint32_t)crv);
	}

	s_pWdtnRFDev = pDev;

	NRF_WDT->EVENTS_TIMEOUT = 0;
	NVIC_ClearPendingIRQ(WDT_IRQn);

	if (pCfg->EvtHandler != NULL)
	{
		NVIC_SetPriority(WDT_IRQn, pCfg->IntPrio);
		NRF_WDT->INTENSET = WDT_INTENSET_TIMEOUT_Msk;
		NVIC_EnableIRQ(WDT_IRQn);
	}

	return true;
}

bool WdtStart(WdtDev_t * const pDev)
{
	if (pDev == NULL || pDev->pDevData == NULL)
	{
		return false;
	}

	if ((NRF_WDT->RUNSTATUS & WDT_NRFX_RUNNING) == 0)
	{
		NRF_WDT->TASKS_START = 1;
	}

	return true;
}

void WdtReload(WdtDev_t * const pDev, int Chan)
{
	if (pDev == NULL || Chan < 0 || Chan >= WDT_NRFX_CHAN_MAX)
	{
		return;
	}

	NRF_WDT->RR[Chan] = WDT_RR_RR_Reload;
}

bool WdtRunning(WdtDev_t * const pDev)
{
	(void)pDev;

	return (NRF_WDT->RUNSTATUS & WDT_NRFX_RUNNING) != 0;
}

void WDT_IRQHandler(void)
{
	if (NRF_WDT->EVENTS_TIMEOUT)
	{
		NRF_WDT->EVENTS_TIMEOUT = 0;

		if (s_pWdtnRFDev != NULL && s_pWdtnRFDev->EvtHandler != NULL)
		{
			s_pWdtnRFDev->EvtHandler(s_pWdtnRFDev);
		}
	}
}
