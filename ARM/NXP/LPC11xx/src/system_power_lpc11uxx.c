/*--------------------------------------------------------------------------
File   : system_power_lpc11uxx.c

Author : Hoang Nguyen Hoan          Dec. 9, 2014

Desc   : LPC11Uxx reset reasons, power modes and retention registers

Copyright (c) 2014, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/

#include "LPC11Uxx.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"

#define LPC11_PCON_DPDFLAG	(1U << 11)

static bool s_ResetRead;
static uint32_t s_ResetReason;

uint32_t SystemResetReason(void)
{
	uint32_t state = DisableInterrupt();

	if (!s_ResetRead)
	{
		uint32_t reset = LPC_SYSCON->SYSRSTSTAT & 0x1fU;
		uint32_t power = LPC_PMU->PCON;
		uint32_t reason = 0;

		if (power & LPC11_PCON_DPDFLAG)
		{
			// Deep power-down wake generates POR internally, not a cold boot.
			reason = SYSTEM_RESET_WAKE_PIN;
		}
		else
		{
			if (reset & 9U)
			{
				reason |= SYSTEM_RESET_POWERON;
			}
			if (reset & 2U)
			{
				reason |= SYSTEM_RESET_PIN;
			}
			if (reset & 4U)
			{
				reason |= SYSTEM_RESET_WDT;
			}
			if (reset & 16U)
			{
				reason |= SYSTEM_RESET_SOFT;
			}
		}
		LPC_SYSCON->SYSRSTSTAT = reset;
		LPC_PMU->PCON = (power & 0xfU) | (power & LPC11_PCON_DPDFLAG);
		s_ResetReason = reason;
		s_ResetRead = true;
	}
	EnableInterrupt(state);
	return s_ResetReason;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	// UM10462: PIO0_16 is permanently the falling-edge Deep power-down wake.
	// It needs an external pull-up. Neither disabling it nor reversing it is supported.
	return PortNo == 0 && PinNo == 16 && Sense == IOPINSENSE_LOW_TRANSITION;
}

void SystemPowerOff(void)
{
	(void)DisableInterrupt();
	// Preserve a pending reset report before setting the Deep power-down flag.
	(void)SystemResetReason();
	LPC_SYSCON->PDRUNCFG &= ~(PDRUNCFG_IRC_PD | PDRUNCFG_IRCOUT_PD);
	// Ordinary interrupts cannot wake Deep power-down; discard pending work.
	NVIC->ICER[0] = UINT32_MAX;
	NVIC->ICPR[0] = UINT32_MAX;
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
	LPC_PMU->PCON = 3U;
	SCB->SCR = (SCB->SCR & ~SCB_SCR_SLEEPONEXIT_Msk) | SCB_SCR_SLEEPDEEP_Msk;
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
	uint32_t power = LPC_PMU->PCON & 0xfU;

	// Ordinary Sleep retains clocks and RAM; enabled NVIC interrupts wake it.
	LPC_PMU->PCON = power & 8U;
	SCB->SCR = scr & ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
	__DSB();
	__WFI();
	SCB->SCR = scr;
	LPC_PMU->PCON = power;
	__ISB();
	EnableInterrupt(state);
}

int SystemRetRegCount(void)
{
	return 4;
}

uint32_t SystemRetRegRead(int Idx)
{
	return (unsigned)Idx < 4U ? LPC_PMU->GPREG[Idx] : 0;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if ((unsigned)Idx >= 4U)
	{
		return false;
	}
	LPC_PMU->GPREG[Idx] = Val;
	return LPC_PMU->GPREG[Idx] == Val;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	// Deep power-down removes SRAM power.
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
