/**-------------------------------------------------------------------------
@file	system_clock_nrfx.h

@brief	Internal Nordic LFCLK requests for peripheral implementations.

Shared LFCLK hardware belongs to the system clock backend. This interface
is private to the Nordic MCU port, not part of the generic HAL.
----------------------------------------------------------------------------*/
#ifndef __SYSTEM_CLOCK_NRFX_H__
#define __SYSTEM_CLOCK_NRFX_H__

#include "coredev/timer.h"

bool nRFxLfclkAcquire(TIMER_CLKSRC Source);
void nRFxLfclkRelease(void);

#endif // __SYSTEM_CLOCK_NRFX_H__
