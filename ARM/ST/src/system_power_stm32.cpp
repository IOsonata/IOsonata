/**-------------------------------------------------------------------------
@file	system_power_stm32.cpp

@brief	Power functions of system_core_clock.h on the STM32

One source for the F0, F4, L1, L4, L4+, G4 and H7 families, the differences
keyed on the CMSIS device header.

Reset reason: the reset flags of RCC_CSR (RCC_RSR on the H7) and the
Standby flag of PWR, read and cleared on the first call. An internal reset
is also output on NRST and sets PINRSTF, a reset pin reason is reported
only when no other flag is set. LPWRRSTF, a reset instead of a low power
mode (nRST_STDBY, nRST_SHDW option bytes), is reported as a software reset.
A Standby exit is a wake up: by a pin when a wake up pin flag is set, by a
timer when the internal wake up flag (L4, G4) or an RTC alarm or wakeup
timer flag is set. A Shutdown exit is a power on reset (BORRSTF), reported
as a power on.

Power off is Standby, Shutdown on the L4 and G4 when no RAM is kept. Only
the WKUP pins wake the MCU, SystemWakePin refuses the other pins:
- F030x8: WKUP1 PA0, WKUP2 PC13, rising level only;
- F401: WKUP PA0, rising level only;
- L152xE: WKUP1 PA0, WKUP2 PC13, WKUP3 PE6, rising level only;
- L4, L4+, G4: WKUP1 PA0, WKUP2 PC13, WKUP3 PE6, WKUP4 PA2, WKUP5 PC5;
- H7: WKUP1 PA0, WKUP2 PA2, WKUP3 PI8, WKUP4 PC13, WKUP5 PI11, WKUP6 PC1.
A pin missing on the package is accepted by the table but cannot wake the
MCU. On the F0, F4 and L1 the hardware forces the pin to input pull down.
On the L4 and G4 the pull resistor of the GPIO configuration is kept in
power off through PWR PUCRx/PDCRx and APC, on the H7 through WKUPEPR
WKUPPUPDx. A pin already at its wake up level wakes the MCU at once.

The retention registers are the backup registers of the RTC (TAMP on the
G4), the ones of RtcStm32 ReadBackup/WriteBackup: 32 bits each, none on the
F030x8, 20 on the F4, 32 on the others. They are kept over a system reset,
Standby and Shutdown, lost on a backup domain reset or a tamper event. A
write enables the backup domain write access (PWR DBP), left enabled as
RtcStm32 does.

RAM kept in power off:
- L4, L4+: SRAM2 in Standby (PWR CR3 RRS), at 0x10000000 and its alias
  after SRAM1 on the L43x, L49x, L4A6, L4R and L4S (none on the L47x);
- G4: SRAM2 at 0x20014000, CCM SRAM is not kept;
- H7: backup SRAM, 4 KB at 0x38800000 (PWR CR2 BREN);
- F0, F4, L1: none, SystemRamRetain refuses.
SRAM2 is kept whole, a range must be inside it.

@author	Hoang Nguyen Hoan
@date	Nov. 9, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "stm32.h"
#include "coredev/system_core_clock.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"

#if defined(IOSONATA_STM32_F0) || defined(IOSONATA_STM32_F4) || defined(IOSONATA_STM32_L1)
#define ST_PWR_V1						// PWR CR and CSR
#elif defined(IOSONATA_STM32_L4) || defined(IOSONATA_STM32_G4)
#define ST_PWR_V2						// PWR CR1 to CR4, SR1, SCR, PUCRx, PDCRx
#elif defined(IOSONATA_STM32_H7)
#define ST_PWR_H7						// PWR CPUCR, WKUPEPR, WKUPFR, WKUPCR
#else
#error "system_power_stm32.cpp: no power implementation for this family"
#endif

#if defined(RCC_RSR_RMVF)
#define ST_PWR_RSTREG				(RCC->RSR)
#define ST_PWR_RMVF					RCC_RSR_RMVF
#else
#define ST_PWR_RSTREG				(RCC->CSR)
#define ST_PWR_RMVF					RCC_CSR_RMVF
#endif

#define ST_PWR_NB_RETREG			IOSONATA_STM32_NB_BKP

// First wake up pin enable bit of PWR CSR
#if defined(PWR_CSR_EWUP1_Pos)
#define ST_PWR_EWUP_POS				PWR_CSR_EWUP1_Pos
#elif defined(PWR_CSR_EWUP_Pos)
#define ST_PWR_EWUP_POS				PWR_CSR_EWUP_Pos
#endif

#define ST_PWR_BRRDY_POLLS			100000U		// Backup regulator ready

#pragma pack(push, 4)

typedef struct {
	uint32_t Msk;		// Reset flag
	uint32_t Reason;	// SYSTEM_RESET_xxx
} StPwrResetMap_t;

typedef struct {
	uint8_t PortNo;
	uint8_t PinNo;
} StPwrWakePin_t;

#pragma pack(pop)

// Reset flags to reset reasons, PINRSTF apart
static const StPwrResetMap_t s_StPwrResetMap[] = {
#ifdef RCC_CSR_PORRSTF
	{ RCC_CSR_PORRSTF, SYSTEM_RESET_POWERON },
#endif
#ifdef RCC_CSR_BORRSTF
	{ RCC_CSR_BORRSTF, SYSTEM_RESET_POWERON },
#endif
#ifdef RCC_CSR_V18PWRRSTF
	{ RCC_CSR_V18PWRRSTF, SYSTEM_RESET_POWERON },
#endif
#ifdef RCC_CSR_IWDGRSTF
	{ RCC_CSR_IWDGRSTF, SYSTEM_RESET_WDT },
#endif
#ifdef RCC_CSR_WWDGRSTF
	{ RCC_CSR_WWDGRSTF, SYSTEM_RESET_WDT },
#endif
#ifdef RCC_CSR_SFTRSTF
	{ RCC_CSR_SFTRSTF, SYSTEM_RESET_SOFT },
#endif
#ifdef RCC_CSR_OBLRSTF
	{ RCC_CSR_OBLRSTF, SYSTEM_RESET_SOFT },
#endif
#ifdef RCC_CSR_LPWRRSTF
	{ RCC_CSR_LPWRRSTF, SYSTEM_RESET_SOFT },
#endif
#ifdef RCC_CSR_FWRSTF
	{ RCC_CSR_FWRSTF, SYSTEM_RESET_SOFT },
#endif
#ifdef RCC_RSR_PORRSTF
	{ RCC_RSR_PORRSTF, SYSTEM_RESET_POWERON },
	{ RCC_RSR_BORRSTF, SYSTEM_RESET_POWERON },
	{ RCC_RSR_IWDG1RSTF, SYSTEM_RESET_WDT },
	{ RCC_RSR_WWDG1RSTF, SYSTEM_RESET_WDT },
	{ RCC_RSR_SFTRSTF, SYSTEM_RESET_SOFT },
	{ RCC_RSR_LPWRRSTF, SYSTEM_RESET_SOFT },
#endif
};

#if defined(RCC_RSR_PINRSTF)
#define ST_PWR_PINRSTF				RCC_RSR_PINRSTF
#define ST_PWR_POWERRSTF			(RCC_RSR_PORRSTF | RCC_RSR_BORRSTF)
#else
#define ST_PWR_PINRSTF				RCC_CSR_PINRSTF
#if defined(RCC_CSR_PORRSTF) && defined(RCC_CSR_BORRSTF)
#define ST_PWR_POWERRSTF			(RCC_CSR_PORRSTF | RCC_CSR_BORRSTF)
#elif defined(RCC_CSR_PORRSTF) && defined(RCC_CSR_V18PWRRSTF)
#define ST_PWR_POWERRSTF			(RCC_CSR_PORRSTF | RCC_CSR_V18PWRRSTF)
#elif defined(RCC_CSR_PORRSTF)
#define ST_PWR_POWERRSTF			RCC_CSR_PORRSTF
#else
#define ST_PWR_POWERRSTF			RCC_CSR_BORRSTF
#endif
#endif

// WKUP pins, WKUP1 first
static const StPwrWakePin_t s_StPwrWakePins[] = {
#if defined(IOSONATA_STM32_F0)
	{ IOPORTA, 0 }, { IOPORTC, 13 },
#elif defined(IOSONATA_STM32_F4)
	{ IOPORTA, 0 },
#elif defined(IOSONATA_STM32_L1)
	{ IOPORTA, 0 }, { IOPORTC, 13 }, { IOPORTE, 6 },
#elif defined(ST_PWR_V2)
	{ IOPORTA, 0 }, { IOPORTC, 13 }, { IOPORTE, 6 }, { IOPORTA, 2 }, { IOPORTC, 5 },
#elif defined(ST_PWR_H7)
	{ IOPORTA, 0 }, { IOPORTA, 2 }, { IOPORTI, 8 }, { IOPORTC, 13 }, { IOPORTI, 11 }, { IOPORTC, 1 },
#endif
};

#define ST_PWR_NB_WAKEPIN			(sizeof(s_StPwrWakePins) / sizeof(s_StPwrWakePins[0]))
#define ST_PWR_WAKEPIN_MSK			((1UL << ST_PWR_NB_WAKEPIN) - 1U)

// RAM kept in power off
#if defined(ST_PWR_H7)
#define ST_PWR_RETRAM_BASE			D3_BKPSRAM_BASE
#define ST_PWR_RETRAM_SIZE			0x1000UL
#elif defined(ST_PWR_V2)
#define ST_PWR_RETRAM_BASE			SRAM2_BASE
#define ST_PWR_RETRAM_SIZE			SRAM2_SIZE
#if defined(STM32L412xx) || defined(STM32L422xx) || defined(STM32L431xx) || defined(STM32L432xx) || \
	defined(STM32L433xx) || defined(STM32L442xx) || defined(STM32L443xx) || defined(STM32L451xx) || \
	defined(STM32L452xx) || defined(STM32L462xx) || defined(STM32L496xx) || defined(STM32L4A6xx) || \
	defined(STM32L4R5xx) || defined(STM32L4R7xx) || defined(STM32L4R9xx) || defined(STM32L4S5xx) || \
	defined(STM32L4S7xx) || defined(STM32L4S9xx)
// SRAM2 also mapped right after SRAM1
#define ST_PWR_RETRAM_ALIAS			(SRAM1_BASE + SRAM1_SIZE_MAX)
#endif
#endif

static bool s_bStPwrResetRead = false;
static uint32_t s_StPwrResetReason = 0;
static uint32_t s_StPwrWakeEn = 0;			// Wake up pins, bit 0 WKUP1
static uint32_t s_StPwrWakeLow = 0;			// Wake up on low level
#if defined(ST_PWR_RETRAM_BASE)
static bool s_bStPwrRamRet = false;			// RAM kept in power off
#endif

// Clock of the PWR registers, where it has one
static void StPwrClock(void)
{
#if defined(RCC_APB1ENR1_PWREN)
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	(void)RCC->APB1ENR1;
#elif defined(RCC_APB1ENR_PWREN)
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;
#endif
}

// Bus clock of the RTC and TAMP registers, where it has one
static void StPwrRtcBusClock(void)
{
#if defined(RCC_APB1ENR1_RTCAPBEN)
	RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
	(void)RCC->APB1ENR1;
#elif defined(RCC_APB4ENR_RTCAPBEN)
	RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
	(void)RCC->APB4ENR;
#endif
}

// Write access to the backup domain: backup registers, backup SRAM
// regulator
static void StPwrBackupAccess(void)
{
	StPwrClock();
#if defined(PWR_CR1_DBP)
	PWR->CR1 |= PWR_CR1_DBP;
	(void)PWR->CR1;
#else
	PWR->CR |= PWR_CR_DBP;
	(void)PWR->CR;
#endif
}

#if defined(ST_PWR_V1) || defined(ST_PWR_H7)
// RTC alarm or wakeup timer flag, a wake up by a timer
static bool StPwrRtcWake(void)
{
	uint32_t msk = RTC_ISR_ALRAF;
#ifdef RTC_ISR_ALRBF
	msk |= RTC_ISR_ALRBF;
#endif
#ifdef RTC_ISR_WUTF
	msk |= RTC_ISR_WUTF;
#endif

	StPwrRtcBusClock();

	return (RTC->ISR & msk) != 0;
}
#endif

uint32_t SystemResetReason(void)
{
	if (s_bStPwrResetRead)
	{
		return s_StPwrResetReason;
	}

	StPwrClock();

	uint32_t rst = ST_PWR_RSTREG;
	uint32_t r = 0;
	bool sb = false;		// Standby exit
	bool pin = false;		// Wake up pin flag
	bool tmr = false;		// Wake up timer, internal source

#if defined(ST_PWR_V1)
	sb = (PWR->CSR & PWR_CSR_SBF) != 0;
	if (sb && (PWR->CSR & PWR_CSR_WUF))
	{
		// WUF for a pin and for the RTC events
		tmr = StPwrRtcWake();
		pin = !tmr;
	}
	PWR->CR |= PWR_CR_CSBF | PWR_CR_CWUF;
#elif defined(ST_PWR_V2)
	sb = (PWR->SR1 & PWR_SR1_SBF) != 0;
	pin = (PWR->SR1 & ST_PWR_WAKEPIN_MSK) != 0;
	tmr = (PWR->SR1 & PWR_SR1_WUFI) != 0;
	PWR->SCR = PWR_SCR_CSBF | PWR_SCR_CWUF;
#elif defined(ST_PWR_H7)
	sb = (PWR->CPUCR & PWR_CPUCR_SBF) != 0;
	pin = (PWR->WKUPFR & ST_PWR_WAKEPIN_MSK) != 0;
	tmr = sb && !pin && StPwrRtcWake();
	PWR->WKUPCR = ST_PWR_WAKEPIN_MSK;
	PWR->CPUCR |= PWR_CPUCR_CSSF;
#endif

	// Set then cleared, the H7 keeps the flags cleared while RMVF is set
	ST_PWR_RSTREG |= ST_PWR_RMVF;
	ST_PWR_RSTREG &= ~ST_PWR_RMVF;

	for (size_t i = 0; i < sizeof(s_StPwrResetMap) / sizeof(s_StPwrResetMap[0]); i++)
	{
		// A Standby exit is a power reset, its flags are not the reason
		if ((rst & s_StPwrResetMap[i].Msk) && !(sb && (s_StPwrResetMap[i].Msk & ST_PWR_POWERRSTF)))
		{
			r |= s_StPwrResetMap[i].Reason;
		}
	}

	if (sb)
	{
		if (pin)
		{
			r |= SYSTEM_RESET_WAKE_PIN;
		}
		if (tmr)
		{
			r |= SYSTEM_RESET_WAKE_TIMER;
		}
	}

	if (r == 0)
	{
		r = (rst & ST_PWR_PINRSTF) ? SYSTEM_RESET_PIN : SYSTEM_RESET_POWERON;
	}

	s_StPwrResetReason = r;
	s_bStPwrResetRead = true;

	return r;
}

bool SystemWakePin(int PortNo, int PinNo, int Sense)
{
	uint32_t idx = 0;

	while (idx < ST_PWR_NB_WAKEPIN &&
		   (s_StPwrWakePins[idx].PortNo != PortNo || s_StPwrWakePins[idx].PinNo != PinNo))
	{
		idx++;
	}
	if (idx >= ST_PWR_NB_WAKEPIN)
	{
		return false;
	}

	uint32_t bit = 1UL << idx;

	switch (Sense)
	{
		case IOPINSENSE_DISABLE:
			s_StPwrWakeEn &= ~bit;
			break;
		case IOPINSENSE_HIGH_TRANSITION:
			s_StPwrWakeEn |= bit;
			s_StPwrWakeLow &= ~bit;
			break;
		case IOPINSENSE_LOW_TRANSITION:
#if defined(ST_PWR_V1)
			// Rising level only
			return false;
#else
			s_StPwrWakeEn |= bit;
			s_StPwrWakeLow |= bit;
			break;
#endif
		default:
			return false;
	}

	return true;
}

#if defined(ST_PWR_V2) || defined(ST_PWR_H7)
// Pull resistor of a wake up pin in its GPIO configuration: 0 none, 1 pull
// up, 2 pull down
static uint32_t StPwrWakePull(uint32_t Idx)
{
	GPIO_TypeDef *reg = Stm32Gpio(s_StPwrWakePins[Idx].PortNo);

	if (reg == NULL)
	{
		return 0;
	}

	uint32_t pull = (reg->PUPDR >> (2U * s_StPwrWakePins[Idx].PinNo)) & 3U;

	return pull == 3U ? 0U : pull;
}
#endif

void SystemPowerOff(void)
{
	uint32_t en = s_StPwrWakeEn & ST_PWR_WAKEPIN_MSK;

	StPwrClock();

#if defined(ST_PWR_V1)
	// The pins enabled first, a pin already high sets WUF, cleared after
	PWR->CSR = (PWR->CSR & ~(ST_PWR_WAKEPIN_MSK << ST_PWR_EWUP_POS)) | (en << ST_PWR_EWUP_POS);
	PWR->CR |= PWR_CR_CWUF | PWR_CR_CSBF;
	PWR->CR |= PWR_CR_PDDS;
#elif defined(ST_PWR_V2)
	uint32_t low = s_StPwrWakeLow & en;
	bool apc = false;

	PWR->CR3 &= ~PWR_CR3_EWUP;
	PWR->CR4 = (PWR->CR4 & ~ST_PWR_WAKEPIN_MSK) | low;
	// Read back, a PWR_CR4 write can be pending at the low power mode entry
	(void)PWR->CR4;

	// Pull resistors of the wake up pins kept in power off
	for (uint32_t i = 0; i < ST_PWR_NB_WAKEPIN; i++)
	{
		if ((en & (1UL << i)) == 0)
		{
			continue;
		}

		volatile uint32_t *pucr = &PWR->PUCRA + 2U * s_StPwrWakePins[i].PortNo;
		volatile uint32_t *pdcr = pucr + 1;
		uint32_t pin = 1UL << s_StPwrWakePins[i].PinNo;
		uint32_t pull = StPwrWakePull(i);

		*pucr = pull == 1U ? *pucr | pin : *pucr & ~pin;
		*pdcr = pull == 2U ? *pdcr | pin : *pdcr & ~pin;
		apc = apc || pull != 0U;
	}
	if (apc)
	{
		PWR->CR3 |= PWR_CR3_APC;
	}

	PWR->CR3 |= en;
	PWR->SCR = PWR_SCR_CWUF | PWR_SCR_CSBF;

	// Standby keeps SRAM2, Shutdown keeps no RAM
	if (s_bStPwrRamRet)
	{
		PWR->CR3 |= PWR_CR3_RRS;
		PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_STANDBY;
	}
	else
	{
		PWR->CR3 &= ~PWR_CR3_RRS;
		PWR->CR1 = (PWR->CR1 & ~PWR_CR1_LPMS) | PWR_CR1_LPMS_SHUTDOWN;
	}
#elif defined(ST_PWR_H7)
	uint32_t epr = en | ((s_StPwrWakeLow & en) << PWR_WKUPEPR_WKUPP1_Pos);

	for (uint32_t i = 0; i < ST_PWR_NB_WAKEPIN; i++)
	{
		if (en & (1UL << i))
		{
			epr |= StPwrWakePull(i) << (PWR_WKUPEPR_WKUPPUPD1_Pos + 2U * i);
		}
	}
	PWR->WKUPEPR = epr;
	PWR->WKUPCR = ST_PWR_WAKEPIN_MSK;

	// Backup SRAM regulator, BREN behind DBP
	StPwrBackupAccess();
	if (s_bStPwrRamRet)
	{
		PWR->CR2 |= PWR_CR2_BREN;
		for (uint32_t i = 0; i < ST_PWR_BRRDY_POLLS && (PWR->CR2 & PWR_CR2_BRRDY) == 0; i++);
	}
	else
	{
		PWR->CR2 &= ~PWR_CR2_BREN;
	}

	PWR->CPUCR |= PWR_CPUCR_PDDS_D1 | PWR_CPUCR_PDDS_D2 | PWR_CPUCR_PDDS_D3 | PWR_CPUCR_CSSF;
#endif

	SCB->SCR |= SCB_SCR_SLEEPDEEP_Msk;
	__DSB();

	// Only stays here when a debugger keeps the MCU running
	while (1)
	{
		__WFI();
	}
}

int SystemRetRegCount(void)
{
	return ST_PWR_NB_RETREG;
}

uint32_t SystemRetRegRead(int Idx)
{
	if (Idx < 0 || Idx >= ST_PWR_NB_RETREG)
	{
		return 0;
	}

	StPwrRtcBusClock();

#if defined(TAMP_BKP0R)
	return (&TAMP->BKP0R)[Idx];
#elif defined(RTC_BKP0R)
	return (&RTC->BKP0R)[Idx];
#else
	return 0;
#endif
}

bool SystemRetRegWrite(int Idx, uint32_t Val)
{
	if (Idx < 0 || Idx >= ST_PWR_NB_RETREG)
	{
		return false;
	}

	StPwrRtcBusClock();
	StPwrBackupAccess();

#if defined(TAMP_BKP0R)
	(&TAMP->BKP0R)[Idx] = Val;
#elif defined(RTC_BKP0R)
	(&RTC->BKP0R)[Idx] = Val;
#else
	(void)Val;
#endif

	return true;
}

#if defined(ST_PWR_RETRAM_BASE)
// Range inside a memory area
static bool StPwrInRange(uintptr_t Addr, uint32_t Size, uintptr_t Base, uint32_t Len)
{
	return Addr >= Base && Addr - Base < Len && Size <= Len - (Addr - Base);
}
#endif

bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain)
{
#if defined(ST_PWR_RETRAM_BASE)
	bool ok = StPwrInRange(Addr, Size, ST_PWR_RETRAM_BASE, ST_PWR_RETRAM_SIZE);

#ifdef ST_PWR_RETRAM_ALIAS
	ok = ok || StPwrInRange(Addr, Size, ST_PWR_RETRAM_ALIAS, ST_PWR_RETRAM_SIZE);
#endif

	if (ok && Size > 0)
	{
		s_bStPwrRamRet = bRetain;
	}

	return ok;
#else
	(void)Addr;
	(void)Size;
	(void)bRetain;

	return false;
#endif
}
