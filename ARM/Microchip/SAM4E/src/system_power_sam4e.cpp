/**-------------------------------------------------------------------------
@file system_power_sam4e.cpp
@brief SAM4E reset reason, Backup mode, wake pins and retention registers.

Backup mode removes power from all SRAM. GPBR retains 20 words while VDDIO
is supplied. The caller configures wake pins as GPIO inputs before selecting
wake polarity. Pin assignments follow SAM4E datasheet tables 10-2 and 10-3.
Reset status is captured during SystemInit before another SUPC_SR read can
clear wake flags. RTC and RTT wake sources remain owned by their drivers.

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#include "sam4e.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"

static bool s_Sam4eResetRead;
static uint32_t s_Sam4eResetReason;

static const struct {
	int Port;
	int Pin;
} s_Sam4eWakePins[] = {
	{ IOPORTA, 0 }, { IOPORTA, 1 }, { IOPORTA, 2 }, { IOPORTA, 4 },
	{ IOPORTA, 5 }, { IOPORTA, 8 }, { IOPORTA, 9 }, { IOPORTA, 11 },
	{ IOPORTA, 14 }, { IOPORTA, 19 }, { IOPORTA, 20 }, { IOPORTA, 30 },
	{ IOPORTB, 2 }, { IOPORTB, 5 }, { IOPORTA, 15 }, { IOPORTA, 16 }
};

uint32_t SystemResetReason(void)
{
	uint32_t rst;
	uint32_t supc;
	uint32_t reason;

	if (s_Sam4eResetRead)
	{
		return s_Sam4eResetReason;
	}
	rst = SAM4E_RSTC->RSTC_SR & RSTC_SR_RSTTYP_Msk;
	supc = SAM4E_SUPC->SUPC_SR;
	reason = 0;
	switch (rst)
	{
		case RSTC_SR_RSTTYP_GENERAL_RST:
			reason = SYSTEM_RESET_POWERON;
			break;
		case RSTC_SR_RSTTYP_WDT_RST:
			reason = SYSTEM_RESET_WDT;
			break;
		case RSTC_SR_RSTTYP_SOFT_RST:
			reason = SYSTEM_RESET_SOFT;
			break;
		case RSTC_SR_RSTTYP_USER_RST:
			reason = SYSTEM_RESET_PIN;
			break;
		case RSTC_SR_RSTTYP_BACKUP_RST:
			if (supc & (SUPC_SR_WKUPS | SUPC_SR_LPDBCS0 | SUPC_SR_LPDBCS1))
			{
				reason |= SYSTEM_RESET_WAKE_PIN;
			}
			if ((SAM4E_SUPC->SUPC_WUMR & SUPC_WUMR_RTCEN) &&
				(SAM4E_RTC->RTC_SR & RTC_SR_ALARM))
			{
				reason |= SYSTEM_RESET_WAKE_TIMER;
			}
			if ((SAM4E_SUPC->SUPC_WUMR & SUPC_WUMR_RTTEN) &&
				(SAM4E_RTT->RTT_SR & RTT_SR_ALMS))
			{
				reason |= SYSTEM_RESET_WAKE_TIMER;
			}
			break;
		default:
			break;
	}
	if (supc & (SUPC_SR_SMWS | SUPC_SR_SMRSTS | SUPC_SR_BODRSTS))
	{
		reason |= SYSTEM_RESET_POWERON;
	}
	s_Sam4eResetReason = reason;
	s_Sam4eResetRead = true;
	return reason;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	uint32_t state;
	uint32_t reg;
	uint32_t bit;
	unsigned i;

	if (Sense != IOPINSENSE_DISABLE && Sense != IOPINSENSE_LOW_TRANSITION &&
		Sense != IOPINSENSE_HIGH_TRANSITION)
	{
		return false;
	}
	for (i = 0; i < sizeof(s_Sam4eWakePins) / sizeof(s_Sam4eWakePins[0]); i++)
	{
		if (PortNo == s_Sam4eWakePins[i].Port && PinNo == s_Sam4eWakePins[i].Pin)
		{
			bit = 1UL << i;
			state = DisableInterrupt();
			reg = SAM4E_SUPC->SUPC_WUIR & ~(bit | (bit << 16));
			if (Sense != IOPINSENSE_DISABLE)
			{
				reg |= bit;
			}
			if (Sense == IOPINSENSE_HIGH_TRANSITION)
			{
				reg |= bit << 16;
			}
			SAM4E_SUPC->SUPC_WUIR = reg;
			EnableInterrupt(state);
			return true;
		}
	}
	return false;
}

void SystemPowerOff(void)
{
	__DSB();
	SAM4E_SUPC->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_VROFF;
	__DSB();
	while (1)
	{
		__WFE();
	}
}

int SystemRetRegCount(void)
{
	return sizeof(SAM4E_GPBR->SYS_GPBR) / sizeof(SAM4E_GPBR->SYS_GPBR[0]);
}

uint32_t SystemRetRegRead(int Idx)
{
	if (Idx < 0 || Idx >= SystemRetRegCount())
	{
		return 0;
	}
	return SAM4E_GPBR->SYS_GPBR[Idx];
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if (Idx < 0 || Idx >= SystemRetRegCount())
	{
		return false;
	}
	SAM4E_GPBR->SYS_GPBR[Idx] = Val;
	return SAM4E_GPBR->SYS_GPBR[Idx] == Val;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	// SRAM has no supply in Backup mode.
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
