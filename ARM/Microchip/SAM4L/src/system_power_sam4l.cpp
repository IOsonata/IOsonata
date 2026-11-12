/**-------------------------------------------------------------------------
@file system_power_sam4l.cpp
@brief SAM4L Backup mode, reset reasons, wake pins and backup registers.

Backup mode loses all SRAM. Wakeup resets the core. EIC wake pins use the
backup pin assignments, not the alternative GPIO peripheral routes.
The caller stops DMA and peripheral transfers before entering Backup mode.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

----------------------------------------------------------------------------*/
#include <stddef.h>
#include "sam4lxxx.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"

// Datasheet 42023H, table 11-3. The device header defines no field names.
#define SAM4L_BACKUP_WAKE_EIC (1UL << 0)
#define SAM4L_BACKUP_WAKE_AST (1UL << 1)
#define SAM4L_BACKUP_WAKE_WDT (1UL << 2)
#define SAM4L_BACKUP_WAKE_BOD ((1UL << 3) | (1UL << 4))

static uint32_t s_Sam4lWakePins;

static const struct {
	int Port;
	int Pin;
	uint32_t Mask;
} s_Sam4lBackupPins[] = {
#if defined(SAM4LCXB) || defined(SAM4LSXB) || defined(SAM4LCXC) || defined(SAM4LSXC)
	{ IOPORTB, 1, 1UL << 0 },
#endif
	{ IOPORTA, 6, 1UL << 1 },
	{ IOPORTA, 4, 1UL << 2 },
	{ IOPORTA, 5, 1UL << 3 },
	{ IOPORTA, 7, 1UL << 4 },
#if defined(SAM4LCXC) || defined(SAM4LSXC)
	{ IOPORTC, 3, 1UL << 5 },
	{ IOPORTC, 4, 1UL << 6 },
	{ IOPORTC, 5, 1UL << 7 },
	{ IOPORTC, 6, 1UL << 8 },
#endif
};

static void Sam4lPowerClockEnable(uint32_t Mask)
{
	uint32_t value = SAM4L_PM->PM_PBDMASK | Mask;

	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xaa) |
			PM_UNLOCK_ADDR(offsetof(Pm, PM_PBDMASK));
	SAM4L_PM->PM_PBDMASK = value;
}

uint32_t SystemResetReason(void)
{
	uint32_t reset = SAM4L_PM->PM_RCAUSE;
	uint32_t reason = 0;
	uint32_t wake;
	uint32_t state;

	if (reset & (PM_RCAUSE_POR | PM_RCAUSE_POR33 | PM_RCAUSE_BOD | PM_RCAUSE_BOD33))
	{
		reason |= SYSTEM_RESET_POWERON;
	}
	if (reset & PM_RCAUSE_EXT)
	{
		reason |= SYSTEM_RESET_PIN;
	}
	if (reset & PM_RCAUSE_WDT)
	{
		reason |= SYSTEM_RESET_WDT;
	}
	// SYSRESETREQ is reported as OCDRST, including software reset requests.
	if (reset & PM_RCAUSE_OCDRST)
	{
		reason |= SYSTEM_RESET_SOFT;
	}
	if (reset & PM_RCAUSE_BKUP)
	{
		state = DisableInterrupt();
		Sam4lPowerClockEnable(PM_PBDMASK_BPM);
		wake = SAM4L_BPM->BPM_BKUPWCAUSE;
		EnableInterrupt(state);
		if (wake & SAM4L_BACKUP_WAKE_EIC)
		{
			reason |= SYSTEM_RESET_WAKE_PIN;
		}
		if (wake & SAM4L_BACKUP_WAKE_AST)
		{
			reason |= SYSTEM_RESET_WAKE_TIMER;
		}
		if (wake & SAM4L_BACKUP_WAKE_WDT)
		{
			reason |= SYSTEM_RESET_WDT;
		}
		if (wake & SAM4L_BACKUP_WAKE_BOD)
		{
			reason |= SYSTEM_RESET_POWERON;
		}
	}
	// Both cause registers retain their value until the next reset/wakeup.
	return reason;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	uint32_t bit = 0;
	uint32_t state;
	unsigned i;

	if (Sense != IOPINSENSE_DISABLE && Sense != IOPINSENSE_LOW_TRANSITION &&
			Sense != IOPINSENSE_HIGH_TRANSITION)
	{
		return false;
	}
	for (i = 0; i < sizeof(s_Sam4lBackupPins) / sizeof(s_Sam4lBackupPins[0]); i++)
	{
		if (s_Sam4lBackupPins[i].Port == PortNo && s_Sam4lBackupPins[i].Pin == PinNo)
		{
			bit = s_Sam4lBackupPins[i].Mask;
			break;
		}
	}
	if (bit == 0)
	{
		return false;
	}
	state = DisableInterrupt();
	Sam4lPowerClockEnable(PM_PBDMASK_BPM | PM_PBDMASK_EIC);
	// Do not take an EIC line already enabled by another driver.
	if ((SAM4L_EIC->EIC_CTRL & bit) && !(s_Sam4lWakePins & bit))
	{
		EnableInterrupt(state);
		return false;
	}
	SAM4L_EIC->EIC_DIS = bit;
	if (Sense == IOPINSENSE_DISABLE)
	{
		SAM4L_BPM->BPM_BKUPPMUX &= ~bit;
		s_Sam4lWakePins &= ~bit;
		// Keep the shared EIC wake enable for any other configured line.
		EnableInterrupt(state);
		return true;
	}
	SAM4L_EIC->EIC_MODE |= bit;
	SAM4L_EIC->EIC_ASYNC |= bit;
	SAM4L_EIC->EIC_FILTER &= ~bit;
	if (Sense == IOPINSENSE_HIGH_TRANSITION)
	{
		SAM4L_EIC->EIC_LEVEL |= bit;
	}
	else
	{
		SAM4L_EIC->EIC_LEVEL &= ~bit;
	}
	// EIC_WAKE is generated without enabling an NVIC interrupt.
	SAM4L_EIC->EIC_IDR = bit;
	SAM4L_EIC->EIC_ICR = bit;
	SAM4L_BPM->BPM_BKUPPMUX |= bit;
	SAM4L_BPM->BPM_BKUPWEN |= SAM4L_BACKUP_WAKE_EIC;
	SAM4L_EIC->EIC_EN = bit;
	s_Sam4lWakePins |= bit;
	EnableInterrupt(state);
	return true;
}

void SystemPowerOff(void)
{
	uint32_t value;

	(void)DisableInterrupt();
	Sam4lPowerClockEnable(PM_PBDMASK_BPM);
	value = (SAM4L_BPM->BPM_PMCON & ~BPM_PMCON_RET) | BPM_PMCON_BKUP;
	SAM4L_BPM->BPM_UNLOCK = BPM_UNLOCK_KEY(0xaa) |
			BPM_UNLOCK_ADDR(offsetof(Bpm, BPM_PMCON));
	SAM4L_BPM->BPM_PMCON = value;
	// Drain APB writes before stopping its clock (datasheet section 11.5.6).
	(void)SAM4L_BPM->BPM_PMCON;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	while (1)
	{
		__WFI();
	}
}

int SystemRetRegCount(void)
{
	return 4;
}

uint32_t SystemRetRegRead(int Idx)
{
	uint32_t value;
	uint32_t state;

	if (Idx < 0 || Idx >= SystemRetRegCount())
	{
		return 0;
	}
	state = DisableInterrupt();
	Sam4lPowerClockEnable(PM_PBDMASK_BSCIF);
	value = SAM4L_BSCIF->BSCIF_BR[Idx].BSCIF_BR;
	EnableInterrupt(state);
	return value;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	uint32_t state;
	bool result;

	if (Idx < 0 || Idx >= SystemRetRegCount())
	{
		return false;
	}
	state = DisableInterrupt();
	Sam4lPowerClockEnable(PM_PBDMASK_BSCIF);
	SAM4L_BSCIF->BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xaa) |
			BSCIF_UNLOCK_ADDR(offsetof(Bscif, BSCIF_BR) + Idx * sizeof(BscifBr));
	SAM4L_BSCIF->BSCIF_BR[Idx].BSCIF_BR = Val;
	result = SAM4L_BSCIF->BSCIF_BR[Idx].BSCIF_BR == Val;
	EnableInterrupt(state);
	return result;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	// Both SRAM banks lose power in Backup mode.
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
