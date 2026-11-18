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
#include <stddef.h>

#include "samdxxx.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

// EIC wake line ownership is managed in iopincfg_samd.c.
void SamdStandbyWakeArm(void);
void SamdStandbyWakeDone(void);
bool SamdUartPrepareSleep(void) __attribute__((weak));
void SamdUartResumeSleep(void) __attribute__((weak));
bool SamdI2cPrepareSleep(void) __attribute__((weak));
void SamdI2cResumeSleep(void) __attribute__((weak));
bool SamdSpiPrepareSleep(void) __attribute__((weak));
void SamdSpiResumeSleep(void) __attribute__((weak));
bool SamdUsbAllowStandby(void) __attribute__((weak));
bool SamdDacAllowStandby(void) __attribute__((weak));
bool SamdDmacAllowStandby(void) __attribute__((weak));
bool SamdTimerAllowStandby(void) __attribute__((weak));
bool SamdAdcPrepareSleep(void) __attribute__((weak));
void SamdAdcResumeSleep(void) __attribute__((weak));

void SystemSleep(void)
{
	uint32_t previous;
	uint32_t state;

	if (__get_IPSR() != 0U)
	{
		return;
	}

	// Keep maskable wake interrupts pending until WFI has completed. An
	// interrupt between rearming EIC and WFI could otherwise be consumed
	// before sleeping, leaving no interrupt to wake the CPU. The SAMD11
	// NMI handler leaves EIC_IRQn pending during this window.
	state = DisableInterrupt();
	bool ready = (SamdUsbAllowStandby == NULL || SamdUsbAllowStandby()) &&
		(SamdDacAllowStandby == NULL || SamdDacAllowStandby()) &&
		(SamdDmacAllowStandby == NULL || SamdDmacAllowStandby()) &&
		(SamdTimerAllowStandby == NULL || SamdTimerAllowStandby()) &&
		(SamdAdcPrepareSleep == NULL || SamdAdcPrepareSleep()) &&
		(SamdI2cPrepareSleep == NULL || SamdI2cPrepareSleep()) &&
		(SamdSpiPrepareSleep == NULL || SamdSpiPrepareSleep()) &&
		(SamdUartPrepareSleep == NULL || SamdUartPrepareSleep());
	if (!ready)
	{
		if (SamdUartResumeSleep != NULL)
		{
			SamdUartResumeSleep();
		}
		if (SamdSpiResumeSleep != NULL)
		{
			SamdSpiResumeSleep();
		}
		if (SamdI2cResumeSleep != NULL)
		{
			SamdI2cResumeSleep();
		}
		if (SamdAdcResumeSleep != NULL)
		{
			SamdAdcResumeSleep();
		}
		EnableInterrupt(state);
		return;
	}
	SamdStandbyWakeArm();
	previous = SCB->SCR;
	SCB->SCR = previous | SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	__WFI();
	__ISB();
	SCB->SCR = previous;
	SamdStandbyWakeDone();
	if (SamdUartResumeSleep != NULL)
	{
		SamdUartResumeSleep();
	}
	if (SamdSpiResumeSleep != NULL)
	{
		SamdSpiResumeSleep();
	}
	if (SamdI2cResumeSleep != NULL)
	{
		SamdI2cResumeSleep();
	}
	if (SamdAdcResumeSleep != NULL)
	{
		SamdAdcResumeSleep();
	}
	(void)SystemCoreClockGet();
	EnableInterrupt(state);
}
