/**-------------------------------------------------------------------------
@file	system_power_stm32wba.cpp

@brief	Core power functions for STM32WBA6 (TrustZone disabled).

Standby exits through reset. Applications must stop radio and autonomous
peripheral activity before calling SystemPowerOff. This implementation does
not manage Bluetooth stack ownership.

MIT License
Copyright (c) 2026 I-SYST inc.
----------------------------------------------------------------------------*/
#include <stdint.h>
#include <stdbool.h>

#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"

#if !defined(STM32WBA65xx)
#error "system_power_stm32wba.cpp is currently supported on STM32WBA65 only"
#endif

// WBA wakeup line routing. Each line has up to four selectable inputs.
// The index is the WUCR bit position. Selectors are two bits per line.
typedef struct {
	uint8_t Port;
	uint8_t Pin;
	uint8_t Line;
	uint8_t Select;
} StWbaWakePin_t;

static const StWbaWakePin_t s_WakePins[] = {
	{ IOPORTA, 0, 0, 0 },	// WKUP1
	{ IOPORTB, 2, 0, 1 },
	{ IOPORTA, 4, 1, 0 },	// WKUP2
	{ IOPORTC, 13, 1, 1 },
	{ IOPORTA, 1, 2, 1 },	// WKUP3
	{ IOPORTB, 6, 2, 2 },
	{ IOPORTA, 2, 3, 0 },	// WKUP4
	{ IOPORTB, 1, 3, 1 },
	{ IOPORTA, 3, 4, 1 },	// WKUP5
	{ IOPORTB, 7, 4, 2 },
	{ IOPORTA, 12, 5, 0 },	// WKUP6
	{ IOPORTA, 5, 5, 1 },
	{ IOPORTB, 14, 6, 0 },	// WKUP7
	{ IOPORTA, 6, 6, 1 },
	{ IOPORTA, 7, 7, 1 },	// WKUP8
	{ IOPORTB, 9, 7, 2 }
};

static uint32_t s_WakeConfigured;
static uint32_t s_WakeEnable;
static uint32_t s_WakeLow;
static uint32_t s_WakeSelect;
static bool s_bReasonRead;
static uint32_t s_ResetReason;

uint32_t SystemResetReason(void)
{
	uint32_t rst;
	uint32_t reason;
	uint32_t wake;
	bool standby;

	if (s_bReasonRead)
	{
		return s_ResetReason;
	}

	rst = RCC->CSR;
	wake = PWR->WUSR;
	standby = (PWR->SR & PWR_SR_SBF) != 0U;
	reason = 0;

#ifdef RCC_CSR_PORRSTF
	if (rst & RCC_CSR_PORRSTF)
	{
		reason |= SYSTEM_RESET_POWERON;
	}
#endif
#ifdef RCC_CSR_BORRSTF
	if (rst & RCC_CSR_BORRSTF)
	{
		reason |= SYSTEM_RESET_POWERON;
	}
#endif
#ifdef RCC_CSR_IWDGRSTF
	if (rst & RCC_CSR_IWDGRSTF)
	{
		reason |= SYSTEM_RESET_WDT;
	}
#endif
#ifdef RCC_CSR_WWDGRSTF
	if (rst & RCC_CSR_WWDGRSTF)
	{
		reason |= SYSTEM_RESET_WDT;
	}
#endif
#ifdef RCC_CSR_SFTRSTF
	if (rst & RCC_CSR_SFTRSTF)
	{
		reason |= SYSTEM_RESET_SOFT;
	}
#endif

	if (standby)
	{
		// Standby resets core logic; its power-on flags are not the cause.
		reason &= ~SYSTEM_RESET_POWERON;
		if (wake != 0U)
		{
			reason |= SYSTEM_RESET_WAKE_PIN;
		}
	}

	if (reason == 0U)
	{
#ifdef RCC_CSR_PINRSTF
		reason = (rst & RCC_CSR_PINRSTF) ? SYSTEM_RESET_PIN : SYSTEM_RESET_POWERON;
#else
		reason = SYSTEM_RESET_POWERON;
#endif
	}

	PWR->WUSCR = wake;
	PWR->SR |= PWR_SR_CSSF;
#ifdef RCC_CSR_RMVF
	RCC->CSR |= RCC_CSR_RMVF;
#endif

	s_ResetReason = reason;
	s_bReasonRead = true;

	return reason;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	uint32_t i;
	uint32_t bit;
	uint32_t shift;
	uint32_t selector;

	for (i = 0; i < sizeof(s_WakePins) / sizeof(s_WakePins[0]); i++)
	{
		if (s_WakePins[i].Port == PortNo && s_WakePins[i].Pin == PinNo)
		{
			break;
		}
	}

	if (i == sizeof(s_WakePins) / sizeof(s_WakePins[0]))
	{
		return false;
	}

	shift = (uint32_t)s_WakePins[i].Line * 2U;
	bit = 1UL << s_WakePins[i].Line;
	selector = (uint32_t)s_WakePins[i].Select << shift;

	switch (Sense)
	{
		case IOPINSENSE_DISABLE:
			s_WakeEnable &= ~bit;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			s_WakeEnable |= bit;
			s_WakeLow &= ~bit;
			s_WakeSelect = (s_WakeSelect & ~(3UL << shift)) | selector;
			break;
		case IOPINSENSE_LOW_TRANSITION:
			s_WakeEnable |= bit;
			s_WakeLow |= bit;
			s_WakeSelect = (s_WakeSelect & ~(3UL << shift)) | selector;
			break;
		default:
			return false;
	}

	s_WakeConfigured |= bit;
	return true;
}

void SystemPowerOff(void)
{
	uint32_t en;
	uint32_t selectMask;
	uint32_t i;

	en = s_WakeEnable & s_WakeConfigured;
	selectMask = 0;
	for (i = 0; i < 8U; i++)
	{
		if (s_WakeConfigured & (1UL << i))
		{
			selectMask |= 3UL << (2U * i);
		}
	}

	// Do not overwrite RTC, radio or other wake sources owned by drivers.
	PWR->WUCR1 &= ~s_WakeConfigured;
	PWR->WUCR2 = (PWR->WUCR2 & ~s_WakeConfigured) |
		(s_WakeLow & s_WakeConfigured);
	PWR->WUCR3 = (PWR->WUCR3 & ~selectMask) | (s_WakeSelect & selectMask);
	PWR->WUSCR = s_WakeConfigured;
	PWR->WUCR1 |= en;

	PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_2;
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	while (1)
	{
		__WFI();
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
	(void)Addr;
	(void)Size;
	(void)bRetain;
	return false;
}
