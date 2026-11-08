/**-------------------------------------------------------------------------
@file	temp_nrf_pei.h
@brief	nRF52 TEMP hardware PEI events and tasks (C/C++ compatible).

Only the existing TempnRF5x sensor owns and initializes TEMP. These
C routing functions require TempnRF5x::Init() and Enable() before use.
An external PPI START does not bypass the normal one-shot DATARDY IRQ
handler, which stops conversion and updates sensor data.
----------------------------------------------------------------------------*/
#ifndef __TEMP_NRF_PEI_H__
#define __TEMP_NRF_PEI_H__

#include "coredev/pei.h"

typedef enum {
	TEMP_NRF_PEI_EVENT_DATARDY
} TEMP_NRF_PEI_EVENT;

typedef enum {
	TEMP_NRF_PEI_TASK_START,
	TEMP_NRF_PEI_TASK_STOP
} TEMP_NRF_PEI_TASK;

#ifdef __cplusplus
extern "C" {
#endif

bool TempNrfPeiEnableEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	TEMP_NRF_PEI_EVENT Event);
bool TempNrfPeiDisableEvent(PeiDev_t *pPei, PeiEvent_t Evt);
bool TempNrfPeiConnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	TEMP_NRF_PEI_TASK Task);
bool TempNrfPeiDisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	TEMP_NRF_PEI_TASK Task);

#ifdef __cplusplus
}
#endif

#endif // __TEMP_NRF_PEI_H__
