/**-------------------------------------------------------------------------
@file	system_power_lpc546xx.c

@brief	LPC546xx power and reset control

Implementation of the LPC546xx power and reset control for the NXP LPC546xx

@author	Hoang Nguyen Hoan
@date	Nov. 15, 2026

@licanse

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include "LPC546xx.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

// Deep power-down values from NXP libpower POWER_EnterDeepPowerDown.
// Reserved power bits are included as required by that entry sequence.
#define LPC546XX_POWER_OFF0	0xFEFFFFF7UL
#define LPC546XX_POWER_OFF1	0x300002AFUL
#define LPC546XX_RET_COUNT	7

static bool s_ResetRead;
static uint32_t s_ResetReason;

uint32_t SystemResetReason(void)
{
	uint32_t state = DisableInterrupt();

	if (!s_ResetRead)
	{
		uint32_t reset = SYSCON->SYSRSTSTAT & 0x1fU;
		uint32_t reason = 0;

		if (reset & (SYSCON_SYSRSTSTAT_POR_MASK | SYSCON_SYSRSTSTAT_BOD_MASK))
		{
			reason |= SYSTEM_RESET_POWERON;
		}
		if (reset & SYSCON_SYSRSTSTAT_EXTRST_MASK)
		{
			reason |= SYSTEM_RESET_PIN;
		}
		if (reset & SYSCON_SYSRSTSTAT_WDT_MASK)
		{
			reason |= SYSTEM_RESET_WDT;
		}
		if (reset & SYSCON_SYSRSTSTAT_SYSRST_MASK)
		{
			reason |= SYSTEM_RESET_SOFT;
		}
		SYSCON->SYSRSTSTAT = reset;
		s_ResetReason = reason;
		s_ResetRead = true;
	}
	EnableInterrupt(state);
	return s_ResetReason;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	// Deep power-down wakes through RESET or an application-configured RTC.
	(void)PortNo;
	(void)PinNo;
	(void)Sense;
	return false;
}

void SystemPowerOff(void)
{
	(void)DisableInterrupt();
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
	// RTC wake is selected in RTC CTRL, independently of NVIC delivery.
	for (unsigned i = 0; i < 2; i++)
	{
		NVIC->ICER[i] = UINT32_MAX;
		NVIC->ICPR[i] = UINT32_MAX;
	}
	SCB->SCR = (SCB->SCR & ~SCB_SCR_SLEEPONEXIT_Msk) | SCB_SCR_SLEEPDEEP_Msk;
	SYSCON->PDRUNCFG[1] = LPC546XX_POWER_OFF1;
	SYSCON->PDRUNCFG[0] = LPC546XX_POWER_OFF0;
	__DSB();
	while (1)
	{
		__WFI();
	}
}

void SystemSleep(void)
{
	uint32_t state = DisableInterrupt();
	uint32_t scr = SCB->SCR;

	SCB->SCR = scr & ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
	__DSB();
	__WFI();
	SCB->SCR = scr;
	__ISB();
	EnableInterrupt(state);
}

int SystemRetRegCount(void)
{
	// GPREG7 belongs to the RTC driver's time-valid marker.
	return LPC546XX_RET_COUNT;
}

uint32_t SystemRetRegRead(int Idx)
{
	if ((unsigned)Idx >= LPC546XX_RET_COUNT)
	{
		return 0;
	}
	uint32_t state = DisableInterrupt();
	uint32_t clock = SYSCON->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_RTC_MASK;

	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_RTC_MASK;
	(void)SYSCON->AHBCLKCTRL[0];
	uint32_t value = RTC->GPREG[Idx];

	if (clock == 0)
	{
		SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_RTC_MASK;
	}
	EnableInterrupt(state);
	return value;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if ((unsigned)Idx >= LPC546XX_RET_COUNT)
	{
		return false;
	}
	uint32_t state = DisableInterrupt();
	uint32_t clock = SYSCON->AHBCLKCTRL[0] & SYSCON_AHBCLKCTRL_RTC_MASK;

	SYSCON->AHBCLKCTRLSET[0] = SYSCON_AHBCLKCTRL_RTC_MASK;
	(void)SYSCON->AHBCLKCTRL[0];
	RTC->GPREG[Idx] = Val;
	bool written = RTC->GPREG[Idx] == Val;

	if (clock == 0)
	{
		SYSCON->AHBCLKCTRLCLR[0] = SYSCON_AHBCLKCTRL_RTC_MASK;
	}
	EnableInterrupt(state);
	return written;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
