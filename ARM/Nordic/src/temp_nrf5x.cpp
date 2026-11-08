/**-------------------------------------------------------------------------
@file	temp_nrf5x.cpp

@brief	nRF5x on silicon temperature sensor implementation.

@author	Hoang Nguyen Hoan
@date	Apr. 5, 2019

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

// Peripherals carry a security suffix on nRF53 and nRF91. Only the nRF5340
// network core has a temperature sensor, and it is non secure.
#if !defined(NRF_TEMP) && defined(NRF_TEMP_S)
#define NRF_TEMP	NRF_TEMP_S
#elif !defined(NRF_TEMP) && defined(NRF_TEMP_NS)
#define NRF_TEMP	NRF_TEMP_NS
#endif

#include "temp_nrf5x.h"

static TempnRF5x *s_pTempnRF5x = NULL;
static bool s_bTempPeiReady = false;

// The TEMP handler is the only consumer of the measured data.
// Ordinary temperature firmware must not extract PEI/PPI symbols.
#if defined(__GNUC__)
#pragma weak PeiBindEvent
#pragma weak PeiGetEventEndpoint
#pragma weak PeiUnbindEvent
#pragma weak PeiConnectTask
#pragma weak PeiDisconnectTask
#endif

void TempnRF5x::IntHandler()
{
	if (UpdateData() == true)
	{
		if (vDataRdyHandler != NULL)
		{
			vDataRdyHandler(this, &vData);
		}
	}
}

bool TempnRF5x::Init(const TempSensorCfg_t &CfgData, DeviceIntrf * const pIntrf, Timer * const pTimer)
{
	vpTimer = pTimer;
	vpIntrf = NULL;
	vDataRdyHandler = CfgData.DataRdyCB;

	s_pTempnRF5x = this;
	s_bTempPeiReady = false;
	NRF_TEMP->INTENCLR = 1U;
	NRF_TEMP->EVENTS_DATARDY = 0U;

    NVIC_ClearPendingIRQ(TEMP_IRQn);
	NVIC_SetPriority(TEMP_IRQn, CfgData.IntPrio);
	NVIC_EnableIRQ(TEMP_IRQn);

	return true;
}

/**
 * @brief	Power on or wake up device
 *
 * @return	true - If success
 */
bool TempnRF5x::Enable()
{
    NRF_TEMP->INTENSET = 1U;
	s_bTempPeiReady = true;

	NVIC_ClearPendingIRQ(TEMP_IRQn);
	NVIC_EnableIRQ(TEMP_IRQn);

	return true;
}

/**
 * @brief	Put device in power down or power saving sleep mode
 */
void TempnRF5x::Disable()
{
    NRF_TEMP->INTENCLR = 1U;
	s_bTempPeiReady = false;
	NVIC_DisableIRQ(TEMP_IRQn);
}

/**
 * @brief	Start sampling data
 *
 * This is a require implementation by sensor implementer.\n
 * This function initiates sensor to do actual measurement.
 *
 * @return	true - success
 * 			false - in case of error or sensor busy measuring
 */
bool TempnRF5x::StartSampling()
{
	NRF_TEMP->TASKS_START = 1;

	return true;
}

bool TempnRF5x::UpdateData()
{
	if (NRF_TEMP->EVENTS_DATARDY)
	{
		if (vpTimer != NULL)
		{
			vData.Timestamp = vpTimer->uSecond();
		}
		else
		{
			vData.Timestamp = 0;
		}
		vData.Temperature = (NRF_TEMP->TEMP * 25);

		NRF_TEMP->EVENTS_DATARDY = 0;

		return true;
	}

	return false;
}

extern "C" void TEMP_IRQHandler(void)
{
	NRF_TEMP->TASKS_STOP = 1;

	if (s_pTempnRF5x != NULL)
	{
		s_pTempnRF5x->IntHandler();
	}
    NVIC_ClearPendingIRQ(TEMP_IRQn);
}


// TEMP uses a single one-shot conversion engine. Init/Enable and the
// DATARDY interrupt handler continue to own sensor data and STOP.
// PEI merely connects the hardware event and task registers.
static bool TempNrfPeiReady(void)
{
#if defined(NRF52_SERIES)
	return s_pTempnRF5x != NULL && s_bTempPeiReady;
#else
	return false;
#endif
}

static PeiEndpoint_t TempNrfPeiTaskEndpoint(TEMP_NRF_PEI_TASK Task)
{
#if defined(NRF52_SERIES)
	switch (Task)
	{
		case TEMP_NRF_PEI_TASK_START:
			return (PeiEndpoint_t)&NRF_TEMP->TASKS_START;
		case TEMP_NRF_PEI_TASK_STOP:
			return (PeiEndpoint_t)&NRF_TEMP->TASKS_STOP;
		default:
			return 0U;
	}
#else
	(void)Task;
	return 0U;
#endif
}

extern "C" bool TempNrfPeiEnableEvent(PeiDev_t *pPei,
	PeiEvent_t *pEvt, TEMP_NRF_PEI_EVENT Event)
{
#if defined(NRF52_SERIES)
	if (!TempNrfPeiReady() || pPei == NULL || pEvt == NULL ||
		PeiBindEvent == NULL || Event != TEMP_NRF_PEI_EVENT_DATARDY)
	{
		return false;
	}
	return PeiBindEvent(pPei, pEvt,
		(PeiEndpoint_t)&NRF_TEMP->EVENTS_DATARDY);
#else
	(void)pPei;
	(void)pEvt;
	(void)Event;
	return false;
#endif
}

extern "C" bool TempNrfPeiDisableEvent(PeiDev_t *pPei, PeiEvent_t Evt)
{
#if defined(NRF52_SERIES)
	PeiEndpoint_t endpoint;

	if (pPei == NULL || PeiGetEventEndpoint == NULL ||
		PeiUnbindEvent == NULL)
	{
		return false;
	}
	endpoint = (PeiEndpoint_t)&NRF_TEMP->EVENTS_DATARDY;
	if (PeiGetEventEndpoint(pPei, Evt) != endpoint)
	{
		return false;
	}
	return PeiUnbindEvent(pPei, Evt, endpoint);
#else
	(void)pPei;
	(void)Evt;
	return false;
#endif
}

extern "C" bool TempNrfPeiConnectTask(PeiDev_t *pPei,
	PeiEvent_t Evt, TEMP_NRF_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (!TempNrfPeiReady() || pPei == NULL || PeiConnectTask == NULL)
	{
		return false;
	}
	endpoint = TempNrfPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiConnectTask(pPei, Evt, endpoint);
}

extern "C" bool TempNrfPeiDisconnectTask(PeiDev_t *pPei,
	PeiEvent_t Evt, TEMP_NRF_PEI_TASK Task)
{
	PeiEndpoint_t endpoint;

	if (pPei == NULL || PeiDisconnectTask == NULL)
	{
		return false;
	}
	endpoint = TempNrfPeiTaskEndpoint(Task);
	if (endpoint == 0U)
	{
		return false;
	}
	return PeiDisconnectTask(pPei, Evt, endpoint);
}
