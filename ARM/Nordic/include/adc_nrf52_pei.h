/**-------------------------------------------------------------------------
@file	adc_nrf52_pei.h
@brief	nRF52 SAADC PEI hardware event and task endpoints.

The ADC driver owns channel setup and the EasyDMA result buffer.
PEI owns only connections. Only a prepared, idle single-conversion
ADC without its automatic ISR sampling mode is accepted here.
This header is usable from C and C++; the existing ADC configuration
driver remains AdcnRF52 (C++).
----------------------------------------------------------------------------*/
#ifndef __ADC_NRF52_PEI_H__
#define __ADC_NRF52_PEI_H__

#include "coredev/pei.h"

typedef enum {
	ADC_NRF52_PEI_EVENT_STARTED,
	ADC_NRF52_PEI_EVENT_END,
	ADC_NRF52_PEI_EVENT_DONE,
	ADC_NRF52_PEI_EVENT_RESULTDONE,
	ADC_NRF52_PEI_EVENT_STOPPED
} ADC_NRF52_PEI_EVENT;

typedef enum {
	ADC_NRF52_PEI_TASK_START,
	ADC_NRF52_PEI_TASK_SAMPLE,
	ADC_NRF52_PEI_TASK_STOP
} ADC_NRF52_PEI_TASK;

#ifdef __cplusplus
extern "C" {
#endif

// Must be called after ADC initialization and OpenChannel, and before
// connecting any PEI tasks. Starts one EasyDMA acquisition buffer
// without sampling; hardware SAMPLE/START routing controls conversion.
bool AdcNrf52PreparePeiSampling(void);

// Bind/unbind supported SAADC hardware event registers. No IRQ forwarding.
bool AdcNrf52PeiBindEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	ADC_NRF52_PEI_EVENT Event);
bool AdcNrf52PeiUnbindEvent(PeiDev_t *pPei, PeiEvent_t Evt);

// Connect/disconnect a prepared SAADC hardware task from a PEI event.
// Disconnect every task before reconfiguring, stopping or disabling SAADC.
bool AdcNrf52PeiConnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	ADC_NRF52_PEI_TASK Task);
bool AdcNrf52PeiDisconnectTask(PeiDev_t *pPei, PeiEvent_t Evt,
	ADC_NRF52_PEI_TASK Task);

#ifdef __cplusplus
}
#endif

#endif // __ADC_NRF52_PEI_H__
