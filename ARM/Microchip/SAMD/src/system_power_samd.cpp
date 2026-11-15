/**-------------------------------------------------------------------------
@file	system_power_samd.cpp

@brief	Core reset reason for Microchip SAM D11 and D21.

The PM reset-cause register is read-only and retained until the next reset.
Standby on these parts resumes execution after an interrupt; it must not
be used as SystemPowerOff, which requires wake by reset.

MIT License
Copyright (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdint.h>

#include "samdxxx.h"
#include "coredev/system_core_clock.h"

uint32_t SystemResetReason(void)
{
	uint32_t status;
	uint32_t reason;

	status = PM_REGS->PM_RCAUSE;
	reason = 0U;

	if (status & (PM_RCAUSE_POR_Msk |
		PM_RCAUSE_BOD12_Msk | PM_RCAUSE_BOD33_Msk))
	{
		reason |= SYSTEM_RESET_POWERON;
	}
	if (status & PM_RCAUSE_EXT_Msk)
	{
		reason |= SYSTEM_RESET_PIN;
	}
	if (status & PM_RCAUSE_WDT_Msk)
	{
		reason |= SYSTEM_RESET_WDT;
	}
	if (status & PM_RCAUSE_SYST_Msk)
	{
		reason |= SYSTEM_RESET_SOFT;
	}

	return reason;
}
