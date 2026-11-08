/**-------------------------------------------------------------------------
@file	rng_nrf_pei.h
@brief	nRF52 RNG hardware event/task interconnect (C/C++ compatible).

The IOsonata RngPeriphIntrf owns RNG hardware. PEI preparation takes
its generic DeviceIntrf busy lock, excluding simultaneous CPU entropy
reads. RNG PEI must not coexist with an active S132/S140 SoftDevice
or a SoftDevice Controller using the RNG peripheral.
----------------------------------------------------------------------------*/
#ifndef __RNG_NRF_PEI_H__
#define __RNG_NRF_PEI_H__

#include "coredev/pei.h"

typedef enum {
	RNG_NRF_PEI_EVENT_VALRDY
} RNG_NRF_PEI_EVENT;

typedef enum {
	RNG_NRF_PEI_TASK_START,
	RNG_NRF_PEI_TASK_STOP
} RNG_NRF_PEI_TASK;

#ifdef __cplusplus
extern "C" {
#endif

bool RngNrfPeiPrepare(void);
bool RngNrfPeiRelease(void);
bool RngNrfPeiEnableEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	RNG_NRF_PEI_EVENT Event);
bool RngNrfPeiDisableEvent(PeiDev_t *pPei, PeiEvent_t Evt);
bool RngNrfPeiConnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	RNG_NRF_PEI_TASK Task);
bool RngNrfPeiDisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	RNG_NRF_PEI_TASK Task);

#ifdef __cplusplus
}
#endif

#endif // __RNG_NRF_PEI_H__
