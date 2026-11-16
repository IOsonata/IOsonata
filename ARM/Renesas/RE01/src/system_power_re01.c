/**-------------------------------------------------------------------------
@file	system_power_re01.c

@brief	RE01 reset reporting, sleep and Deep Software Standby.

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
#include "re01xxx.h"
#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"

static uint32_t s_ResetReason;
static bool s_ResetRead;

// IRQ event codes in each group of eight IELSR entries.
static const uint8_t s_WakeGroup[10] = {0, 1, 2, 3, 4, 5, 6, 7, 0, 2};
static const uint8_t s_WakeEvent[10] = {1, 1, 1, 1, 0x13, 0x12, 0x12, 0x13, 0x1e, 0x1f};

uint32_t SystemResetReason(void)
{
	uint32_t state = DisableInterrupt();
	if (!s_ResetRead)
	{
		uint8_t reset0 = SYSTEM->RSTSR0 & 0x87U;
		uint16_t reset1 = SYSTEM->RSTSR1 & 0x1c07U;
		uint32_t reason = 0;
		if (reset0 & 7U)
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
		if (reset0 & SYSTEM_RSTSR0_DPSRSTF_Msk)
		{
			if ((SYSTEM->DPSIFR0 & 0xfU) || (SYSTEM->DPSIFR1 & SYSTEM_DPSIFR1_DNMIF_Msk))
			{
				reason |= SYSTEM_RESET_WAKE_PIN;
			}
			if (SYSTEM->DPSIFR1 & SYSTEM_DPSIFR1_DCCCIF_Msk)
			{
				reason |= SYSTEM_RESET_WAKE_TIMER;
			}
		}
		if (reset0 == 0 && reset1 == 0)
		{
			reason = SYSTEM_RESET_PIN;
		}
		SYSTEM->RSTSR0 = 0;
		SYSTEM->RSTSR1 = 0;
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
	uint16_t protect = SYSTEM->PRCR & 0xfU;
	uint16_t standby = SYSTEM->SBYCR;
	uint8_t deepstandby = SYSTEM->DPSBYCR;
	uint8_t power = SYSTEM->PWSTCR;
	uint8_t ram = SYSTEM->RAMSDCR;
	uint32_t tick = SysTick->CTRL;
	uint32_t wake = 0;
	volatile uint32_t *routes = &ICU->IELSR0;

	// Boost cannot enter Software Standby. Keep an application's Snooze or
	// DTC configuration intact by using ordinary Sleep in those cases.
	bool deep = SYSTEM->PWSTF == SYSTEM_PWSTF_ALLPWONM_Msk &&
		(SYSTEM->SNZCR & SYSTEM_SNZCR_SNZE_Msk) == 0 &&
		(SYSTEM->MSTPCRA & SYSTEM_MSTPCRA_MSTPA22_Msk) != 0 &&
		(ICU->WUPEN & ~0x3ffUL) == 0;
	if (deep)
	{
		for (unsigned pin = 0; pin < 10; pin++)
		{
			if ((ICU->WUPEN & (1UL << pin)) == 0)
			{
				continue;
			}
			for (unsigned irq = 0; irq < 32; irq++)
			{
				unsigned group = irq & 7U;
				if ((group == s_WakeGroup[pin] || (pin < 4 && group == pin + 4)) &&
					(routes[irq] & ICU_IELSR0_IELS_Msk) == s_WakeEvent[pin])
				{
					wake |= 1UL << irq;
				}
			}
		}
		deep = (wake & NVIC->ISER[0]) != 0;
	}
	if (SCB->ICSR & (SCB_ICSR_PENDSTSET_Msk | SCB_ICSR_PENDSVSET_Msk))
	{
		EnableInterrupt(state);
		return;
	}
	SYSTEM->PRCR = 0xa502U | protect;
	SYSTEM->SBYCR = deep ? SYSTEM_SBYCR_SSBY_Msk : 0;
	if (deep)
	{
		// ALLPWON -> EXFPWON -> ALLPWON, with every SRAM bank retained.
		SYSTEM->DPSBYCR = 0;
		SYSTEM->RAMSDCR = 0;
		SYSTEM->PWSTCR = 0;
	}
	bool ready = (SYSTEM->PRCR & 2U) != 0 &&
		SYSTEM->SBYCR == (deep ? SYSTEM_SBYCR_SSBY_Msk : 0) &&
		(!deep || (SYSTEM->DPSBYCR == 0 && SYSTEM->RAMSDCR == 0 && SYSTEM->PWSTCR == 0));
	SYSTEM->PRCR = 0xa500U | protect;
	if (ready)
	{
		SCB->SCR = scr & ~(SCB_SCR_SLEEPONEXIT_Msk | SCB_SCR_SLEEPDEEP_Msk);
		if (deep)
		{
			SysTick->CTRL = tick & ~SysTick_CTRL_TICKINT_Msk;
		}
		__DSB();
		__WFI();
	}
	SYSTEM->PRCR = 0xa502U | protect;
	SYSTEM->SBYCR = standby;
	if (deep)
	{
		SYSTEM->DPSBYCR = deepstandby;
		SYSTEM->RAMSDCR = ram;
		SYSTEM->PWSTCR = power;
		SysTick->CTRL = tick;
	}
	SYSTEM->PRCR = 0xa500U | protect;
	SCB->SCR = scr;
	__ISB();
	EnableInterrupt(state);
}

void SystemPowerOff(void)
{
	(void)DisableInterrupt();
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk | SCB_ICSR_PENDSVCLR_Msk;
	NVIC->ICER[0] = UINT32_MAX;
	NVIC->ICPR[0] = UINT32_MAX;
	SCB->SCR &= ~(SCB_SCR_SLEEPONEXIT_Msk | SCB_SCR_SLEEPDEEP_Msk);

	// Debug prevents Boost transitions and can prevent Deep Standby entry.
	bool ready = (SYSTEM->SYOCDCR & (SYSTEM_SYOCDCR_DOCDF_Msk | SYSTEM_SYOCDCR_DBGEN_Msk)) == 0;
#if defined(CHIP_VERSION_WS1)
	// WS1 restrictions 51/52 require an application-configured CCC wake.
	ready = ready && (SYSTEM->DPSIER1 & SYSTEM_DPSIER1_DCCCIE_Msk) != 0;
#endif
	if (ready)
	{
		ready = Re01LeaveBoostMode();
	}
	uint16_t protection = SYSTEM->PRCR & 0xfU;
	SYSTEM->PRCR = 0xa502U | protection;
	if (ready)
	{
		SYSTEM->SNZCR_b.SNZE = 0;
		SYSTEM->SBYCR = SYSTEM_SBYCR_SSBY_Msk;
		SYSTEM->DPSBYCR = SYSTEM_DPSBYCR_DPSBY_Msk | SYSTEM_DPSBYCR_IOKEEP_Msk;
		SYSTEM->PWSTCR = 0;
		// Clear old wake flags once. A new edge after this point stays pending.
		SYSTEM->DPSIFR0 = 0;
		SYSTEM->DPSIFR1 = 0;
		ready = SYSTEM->SBYCR == SYSTEM_SBYCR_SSBY_Msk &&
			SYSTEM->DPSBYCR == (SYSTEM_DPSBYCR_DPSBY_Msk | SYSTEM_DPSBYCR_IOKEEP_Msk) &&
			SYSTEM->PWSTCR == 0;
	}
	if (!ready)
	{
		// A refused transition stays stopped until reset; it must not return
		// to an application that has already shut its devices down.
		SYSTEM->SBYCR = 0;
	}
	bool sleep = ready || SYSTEM->SBYCR == 0;
	SYSTEM->PRCR = 0xa500U | protection;
	__DSB();
	while (1)
	{
		if (sleep)
		{
			__WFI();
		}
		else
		{
			__NOP();
		}
	}
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
	// RAMSDCR retains banks in Software Standby only, not Deep Standby.
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
