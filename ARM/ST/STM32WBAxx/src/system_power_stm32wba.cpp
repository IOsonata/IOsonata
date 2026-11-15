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
// STM32WBA65 I variant: SRAM1 448 KB, SRAM2 64 KB.
// Each bank's Standby retention is selected as a whole by this generic API.
// Applications must reserve test ranges in the linker script.
#define ST_WBA_SRAM1_BASE	0x20000000UL
#define ST_WBA_SRAM1_SIZE	0x70000UL
#define ST_WBA_SRAM2_BASE	0x20070000UL
#define ST_WBA_SRAM2_SIZE	0x10000UL

// On STM32WBA65 (WBA6), Standby retention uses one enable bit per SRAM
// bank. The page-granular R1RSB2..R1RSB567/R2RSB2 fields are not present.
#if !defined(PWR_CR1_R1RSB1) || !defined(PWR_CR1_R2RSB1)
#error "STM32WBA65 SRAM1/SRAM2 Standby retention bits missing"
#endif

#define ST_WBA_SRAM1_RET	PWR_CR1_R1RSB1
#define ST_WBA_SRAM2_RET	PWR_CR1_R2RSB1

static bool s_bRam1Configured;
static bool s_bRam2Configured;
static bool s_bRam1Retain;
static bool s_bRam2Retain;

static bool s_bReasonRead;
static uint32_t s_ResetReason;

uint32_t SystemResetReason(void)
{
	uint32_t rst;
	uint32_t reason;
	uint32_t wake;
	uint32_t rtcWake;
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
		// RTC/TAMP can be routed through WKUP6/7/8. They are not GPIO.
		rtcWake = 0U;
		if ((wake & (1UL << 5)) &&
			((PWR->WUCR3 >> 10U) & 3U) == 3U)
		{
			rtcWake |= 1UL << 5;
		}
		if ((wake & (1UL << 6)) &&
			((PWR->WUCR3 >> 12U) & 3U) == 3U)
		{
			rtcWake |= 1UL << 6;
		}
		if (rtcWake)
		{
			reason |= SYSTEM_RESET_WAKE_TIMER;
		}
		if (wake & ~((1UL << 5) | (1UL << 6) | (1UL << 7)))
		{
			reason |= SYSTEM_RESET_WAKE_PIN;
		}
		if ((wake & ((1UL << 5) | (1UL << 6))) != rtcWake)
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

// Preserve the GPIO pull setting of wakeup inputs through Standby.
// The GPIO PUPDR register itself is lost when the core supply is removed.
// WBA6 uses per-pin PWR IORETENR registers, not STM32L4 PUCR/PDCR.
static void StWbaWakePullRetain(void)
{
	uint32_t i;
	uint32_t bit;
	GPIO_TypeDef *gpio;
	volatile uint32_t *pRet;

	for (i = 0; i < sizeof(s_WakePins) / sizeof(s_WakePins[0]); i++)
	{
		bit = 1UL << s_WakePins[i].Line;
		if ((s_WakeEnable & bit) == 0U ||
			(s_WakeSelect & (3UL << (s_WakePins[i].Line * 2U))) !=
			((uint32_t)s_WakePins[i].Select << (s_WakePins[i].Line * 2U)))
		{
			continue;
		}

		switch (s_WakePins[i].Port)
		{
			case IOPORTA:
				gpio = GPIOA;
				pRet = &PWR->IORETENRA;
				break;
			case IOPORTB:
				gpio = GPIOB;
				pRet = &PWR->IORETENRB;
				break;
			case IOPORTC:
				gpio = GPIOC;
				pRet = &PWR->IORETENRC;
				break;
			default:
				continue;
		}

		// Retain only pull-up/down pins. Never modify other pin retention.
		if (((gpio->PUPDR >> (s_WakePins[i].Pin * 2U)) & 3U) != 0U)
		{
			*pRet |= 1UL << s_WakePins[i].Pin;
			(void)*pRet;
		}
	}
}

void SystemPowerOff(void)
{
	uint32_t en;
	uint32_t selectMask;
	uint32_t i;
	uint32_t ram;
	uint32_t ramMask;

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
	StWbaWakePullRetain();
	// Clear stale WKUP flags while the affected lines are disabled.
	// Enabling an active wake source can immediately set its wake flag;
	// clearing it afterward would discard that event.
	PWR->WUSCR = s_WakeConfigured;
	PWR->WUCR1 |= en;
	// Do not enter Standby if the wakeup selection was not programmed.
	if ((PWR->WUCR1 & s_WakeConfigured) != en ||
		(PWR->WUCR2 & s_WakeConfigured) !=
		(s_WakeLow & s_WakeConfigured) ||
		(PWR->WUCR3 & selectMask) != (s_WakeSelect & selectMask))
	{
		return;
	}

	ram = (s_bRam1Retain ? ST_WBA_SRAM1_RET : 0U) |
		(s_bRam2Retain ? ST_WBA_SRAM2_RET : 0U);
	ramMask = (s_bRam1Configured ? ST_WBA_SRAM1_RET : 0U) |
		(s_bRam2Configured ? ST_WBA_SRAM2_RET : 0U);
	PWR->CR1 = (PWR->CR1 & ~(PWR_CR1_LPMS | ramMask)) |
		PWR_CR1_LPMS_2 | (ram & ramMask);
	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();
	while (1)
	{
		__WFI();
	}
}

static void StWbaBackupAccess(void)
{
	RCC->AHB4ENR |= RCC_AHB4ENR_PWREN;
	(void)RCC->AHB4ENR;
#ifdef RCC_APB7ENR_RTCAPBEN
	RCC->APB7ENR |= RCC_APB7ENR_RTCAPBEN;
	(void)RCC->APB7ENR;
#endif
	PWR->DBPR |= PWR_DBPR_DBP;
	(void)PWR->DBPR;
}

int SystemRetRegCount(void)
{
	return IOSONATA_STM32_NB_BKP;
}

uint32_t SystemRetRegRead(int Idx)
{
	if (Idx < 0 || Idx >= IOSONATA_STM32_NB_BKP)
	{
		return 0;
	}

	StWbaBackupAccess();

#if defined(TAMP_BKP0R)
	return (&TAMP->BKP0R)[Idx];
#else
	return 0;
#endif
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if (Idx < 0 || Idx >= IOSONATA_STM32_NB_BKP)
	{
		return false;
	}

	StWbaBackupAccess();

#if defined(TAMP_BKP0R)
	(&TAMP->BKP0R)[Idx] = Val;
	return (&TAMP->BKP0R)[Idx] == Val;
#else
	(void)Val;
	return false;
#endif
}

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
	uintptr_t off;

	if (Size == 0U)
	{
		return false;
	}

	if (Addr >= ST_WBA_SRAM1_BASE)
	{
		off = Addr - ST_WBA_SRAM1_BASE;
		if (off < ST_WBA_SRAM1_SIZE &&
			Size <= ST_WBA_SRAM1_SIZE - off)
		{
			s_bRam1Configured = true;
			s_bRam1Retain = bRetain;
			return true;
		}
	}

	if (Addr >= ST_WBA_SRAM2_BASE)
	{
		off = Addr - ST_WBA_SRAM2_BASE;
		if (off < ST_WBA_SRAM2_SIZE &&
			Size <= ST_WBA_SRAM2_SIZE - off)
		{
			s_bRam2Configured = true;
			s_bRam2Retain = bRetain;
			return true;
		}
	}

	return false;
}
