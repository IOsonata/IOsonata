/*--------------------------------------------------------------------------
File   : system_power_lpc17xx.c

Author : Hoang Nguyen Hoan          Dec. 9, 2014

Desc   : LPC17xx reset reasons, power modes and retention registers

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

#include "LPC17xx.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"

#define LPC17_PCON_DPDFLAG	(1U << 11)

static volatile uint32_t * const s_RetRegs[] = {
	&LPC_RTC->GPREG0, &LPC_RTC->GPREG1, &LPC_RTC->GPREG2,
	&LPC_RTC->GPREG3, &LPC_RTC->GPREG4
};

static bool s_ResetRead;
static uint32_t s_ResetReason;

uint32_t SystemResetReason(void)
{
	uint32_t state = DisableInterrupt();

	if (!s_ResetRead)
	{
		uint32_t clocks = LPC_SC->PCONP;

		LPC_SC->PCONP = clocks | (1U << 9);
		(void)LPC_SC->PCONP;
		uint32_t reset = LPC_SC->RSID & 0xfU;
		uint32_t power = LPC_SC->PCON;
		uint32_t reason = 0;

		if (power & LPC17_PCON_DPDFLAG)
		{
			// DPDFLAG survives reset in the VBAT domain.
			if (reset & 2U)
			{
				reason = SYSTEM_RESET_PIN | SYSTEM_RESET_WAKE_PIN;
			}
			else if (LPC_RTC->ILR & 3U)
			{
				reason = SYSTEM_RESET_WAKE_TIMER;
			}
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
		}
		LPC_SC->RSID = reset;
		LPC_SC->PCON = (power & 0x1fU) | (power & LPC17_PCON_DPDFLAG);
		LPC_SC->PCONP = clocks;
		s_ResetReason = reason;
		s_ResetRead = true;
	}
	EnableInterrupt(state);
	return s_ResetReason;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	// Deep Power-down wakes only through RESET or an enabled RTC interrupt.
	(void)PortNo;
	(void)PinNo;
	(void)Sense;
	return false;
}

void SystemPowerOff(void)
{
	(void)DisableInterrupt();
	// Preserve a pending reset report before setting the Deep power-down flag.
	(void)SystemResetReason();
	// Preserve the application RTC wake interrupt; discard other pending work.
	NVIC->ICER[0] = ~(1U << RTC_IRQn);
	NVIC->ICER[1] = UINT32_MAX;
	NVIC->ICPR[0] = ~(1U << RTC_IRQn);
	NVIC->ICPR[1] = UINT32_MAX;
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
	LPC_SC->PCON = (LPC_SC->PCON & 0x1cU) | 3U;
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
	uint32_t power = LPC_SC->PCON & 0x1fU;

	// Ordinary Sleep retains clocks and RAM; enabled NVIC interrupts wake it.
	LPC_SC->PCON = power & 0x1cU;
	SCB->SCR = scr & ~(SCB_SCR_SLEEPDEEP_Msk | SCB_SCR_SLEEPONEXIT_Msk);
	__DSB();
	__WFI();
	SCB->SCR = scr;
	LPC_SC->PCON = power;
	__ISB();
	EnableInterrupt(state);
}

int SystemRetRegCount(void)
{
	return 5;
}

uint32_t SystemRetRegRead(int Idx)
{
	if ((unsigned)Idx >= 5U)
	{
		return 0;
	}
	uint32_t state = DisableInterrupt();
	uint32_t clocks = LPC_SC->PCONP;

	LPC_SC->PCONP = clocks | (1U << 9);
	(void)LPC_SC->PCONP;
	uint32_t value = *s_RetRegs[Idx];

	LPC_SC->PCONP = clocks;
	EnableInterrupt(state);
	return value;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if ((unsigned)Idx >= 5U)
	{
		return false;
	}
	uint32_t state = DisableInterrupt();
	uint32_t clocks = LPC_SC->PCONP;

	LPC_SC->PCONP = clocks | (1U << 9);
	(void)LPC_SC->PCONP;
	*s_RetRegs[Idx] = Val;
	bool written = *s_RetRegs[Idx] == Val;

	LPC_SC->PCONP = clocks;
	EnableInterrupt(state);
	return written;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	// Deep power-down removes SRAM power.
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
