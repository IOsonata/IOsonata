/**-------------------------------------------------------------------------
@file	system_power_ra4m1.c

@brief	RA4M1 reset reporting and interrupt-resumable sleep.

@author	Hoang Nguyen Hoan
@date	Nov. 16, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

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
#include "ra4m1xxx.h"
#include "coredev/interrupt.h"

static uint32_t s_ResetReason;
static bool s_ResetRead;

uint32_t SystemResetReason(void)
{
	uint32_t state = DisableInterrupt();

	if (!s_ResetRead)
	{
		uint8_t reset0 = R_SYSTEM->RSTSR0 & 0x0fU;
		uint16_t reset1 = R_SYSTEM->RSTSR1 & 0x1f07U;
		uint32_t reason = 0;

		if (reset0)
		{
			reason |= SYSTEM_RESET_POWERON;
		}
		if (reset1 & 3U)
		{
			reason |= SYSTEM_RESET_WDT;
		}
		if (reset1 & 4U)
		{
			reason |= SYSTEM_RESET_SOFT;
		}
		// RES clears all reset flags. Memory/MPU faults have no generic bit.
		if (reset0 == 0 && reset1 == 0)
		{
			reason = SYSTEM_RESET_PIN;
		}
		R_SYSTEM->RSTSR0 = 0;
		R_SYSTEM->RSTSR1 = 0;
		s_ResetReason = reason;
		s_ResetRead = true;
	}
	EnableInterrupt(state);
	return s_ResetReason;
}

void SystemSleep(void)
{
	uint32_t state = DisableInterrupt();
	uint32_t scr = SCB->SCR;
	uint16_t protect = R_SYSTEM->PRCR & 0x0bU;
	uint16_t standby = R_SYSTEM->SBYCR;
	uint32_t enabled = NVIC->ISER[0];
	uint32_t wake = 0;
	uint32_t tick = SysTick->CTRL;

	// Standby currently supports external IRQ wake. Other wake policies use
	// ordinary Sleep. Do not stop an application's DMA or Snooze operation.
	bool deep = (R_ICU->WUPEN & 0xffffU) != 0 &&
		(R_ICU->WUPEN & 0xffff0000U) == 0 &&
		((R_SYSTEM->MSTPCRA & (1UL << 22)) != 0 ||
		 ((R_DMA->DMAST & 1U) == 0 && (R_DTC->DTCST & 1U) == 0)) &&
		R_SYSTEM->OSTDCR == 0 && (R_SYSTEM->SNZCR & 0x80U) == 0;

	if (deep)
	{
		for (unsigned i = 0; i < RA4M1_IRQ_COUNT; i++)
		{
			uint32_t event = R_ICU->IELSR[i] & 0xffU;

			if (event >= 1 && event <= 16 && (R_ICU->WUPEN & (1UL << (event - 1))))
			{
				wake |= 1UL << i;
			}
		}
		deep = (wake & enabled) != 0;
	}
	if (SCB->ICSR & (SCB_ICSR_PENDSTSET_Msk | SCB_ICSR_PENDSVSET_Msk))
	{
		EnableInterrupt(state);
		return;
	}
	R_SYSTEM->PRCR = 0xa500U | protect | 2U;
	R_SYSTEM->SBYCR = deep ? 0xc000U : 0x4000U;
	if ((R_SYSTEM->PRCR & 2U) == 0 || R_SYSTEM->SBYCR != (deep ? 0xc000U : 0x4000U))
	{
		R_SYSTEM->SBYCR = standby;
		R_SYSTEM->PRCR = 0xa500U | protect;
		EnableInterrupt(state);
		return;
	}
	SCB->SCR = scr & ~(SCB_SCR_SLEEPONEXIT_Msk | SCB_SCR_SLEEPDEEP_Msk);
	if (deep)
	{
		// TN-RA*-A0146A/E procedure B: preserve non-wake pending events.
		SysTick->CTRL = tick & ~SysTick_CTRL_TICKINT_Msk;
		NVIC->ICER[0] = enabled & ~wake;
	}
	__DSB();
	__WFI();
	R_SYSTEM->SBYCR = standby;
	R_SYSTEM->PRCR = 0xa500U | protect;
	SCB->SCR = scr;
	if (deep)
	{
		NVIC->ISER[0] = enabled;
		SysTick->CTRL = tick;
	}
	__ISB();
	EnableInterrupt(state);
}

int SystemRetRegCount(void)
{
	return 0;
}

uint32_t SystemRetRegRead(int Idx)
{
	(void)Idx;
	return 0;
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	(void)Idx;
	(void)Val;
	return false;
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	// RAM remains valid in SystemSleep; there is no reset-on-wake power off.
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
