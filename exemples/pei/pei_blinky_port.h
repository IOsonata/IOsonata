/**-------------------------------------------------------------------------
@file	pei_blinky_port.h

@brief	Target adapter for the shared PEI blinky example

Temporary adapter: peripheral setup will move to their IOsonata Timer and
GPIO drivers when those drivers expose hardware event/task endpoints.
This adapter does not program the PEI interconnect.

----------------------------------------------------------------------------*/
#ifndef __PEI_BLINKY_PORT_H__
#define __PEI_BLINKY_PORT_H__

#include <stdbool.h>
#include "coredev/pei.h"

#ifdef __cplusplus
extern "C" {
#endif

// Configure a timer compare event and a GPIO output task (not their link).
bool PeiBlinkyPrepare(PeiEndpoint_t *pEvent, PeiEndpoint_t *pTask);

// Start event generation only after PEI routing is enabled.
void PeiBlinkyStart(void);

// Target-specific idle; no software event forwarding.
void PeiBlinkyIdle(void);

#ifdef __cplusplus
}
#endif

#endif // __PEI_BLINKY_PORT_H__
