/**-------------------------------------------------------------------------
@file	pei_nrf52_power_clock.h
@brief	nRF52 POWER and CLOCK events through the PEI matrix.

These are passive, hardware PPI event sources. This interface does not
configure the power-fail monitor, request/stop a clock, change latency
modes, acknowledge interrupts or claim POWER_CLOCK IRQ ownership.
The application's existing power/clock/USB/SoftDevice owner continues
to configure and service those blocks.
----------------------------------------------------------------------------*/
#ifndef __PEI_NRF52_POWER_CLOCK_H__
#define __PEI_NRF52_POWER_CLOCK_H__

#include "coredev/pei.h"

typedef enum {
	PEI_NRF52_POWER_EVENT_POFWARN,
	PEI_NRF52_POWER_EVENT_SLEEPENTER,
	PEI_NRF52_POWER_EVENT_SLEEPEXIT,
	PEI_NRF52_POWER_EVENT_USBDETECTED,
	PEI_NRF52_POWER_EVENT_USBREMOVED,
	PEI_NRF52_POWER_EVENT_USBPWRRDY
} PEI_NRF52_POWER_EVENT;

typedef enum {
	PEI_NRF52_CLOCK_EVENT_HFCLKSTARTED,
	PEI_NRF52_CLOCK_EVENT_LFCLKSTARTED,
	PEI_NRF52_CLOCK_EVENT_DONE,
	PEI_NRF52_CLOCK_EVENT_CTTO,
	PEI_NRF52_CLOCK_EVENT_CTSTARTED,
	PEI_NRF52_CLOCK_EVENT_CTSTOPPED
} PEI_NRF52_CLOCK_EVENT;

#ifdef __cplusplus
extern "C" {
#endif

bool PeiNrf52BindPowerEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	PEI_NRF52_POWER_EVENT Event);
bool PeiNrf52UnbindPowerEvent(PeiDev_t *pPei, PeiEvent_t Evt);
bool PeiNrf52BindClockEvent(PeiDev_t *pPei, PeiEvent_t *pEvt,
	PEI_NRF52_CLOCK_EVENT Event);
bool PeiNrf52UnbindClockEvent(PeiDev_t *pPei, PeiEvent_t Evt);

#ifdef __cplusplus
}
#endif

#endif // __PEI_NRF52_POWER_CLOCK_H__
