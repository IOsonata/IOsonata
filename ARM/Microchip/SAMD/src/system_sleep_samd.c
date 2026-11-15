/**-------------------------------------------------------------------------
@file	system_sleep_samd.c

@brief	Interrupt-resumable Standby for Microchip SAM D11 and D21.

Wake sources must be configured before this call. In particular, EIC
wake-up requires an enabled EIC_WAKEUP line and an asynchronous source
that remains functional in Standby. Ordinary EIC interrupts configured
from the main generic clock are not sufficient.

MIT License
Copyright (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "samdxxx.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

// EIC wake line ownership is managed in iopincfg_samd.c.
void SamdStandbyWakeArm(void);
void SamdStandbyWakeDone(void);

void SystemSleep(void)
{
	uint32_t previous;
	uint32_t state;

	// Keep maskable wake interrupts pending until WFI has completed. An
	// interrupt between rearming EIC and WFI could otherwise be consumed
	// before sleeping, leaving no interrupt to wake the CPU. The SAMD11
	// NMI handler leaves EIC_IRQn pending during this window.
	state = DisableInterrupt();
	SamdStandbyWakeArm();
	previous = SCB->SCR;
	SCB->SCR = previous | SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	__ISB();
	SCB->SCR = previous;
	SamdStandbyWakeDone();
	EnableInterrupt(state);
}
