/**-------------------------------------------------------------------------
@file	rtc_stm32.cpp

@brief	STM32 realtime clock (RTC)

Generic RTC API (miscdev/rtc.h) on the RTC of the STM32 families: F0, F4,
L1, L4, L4+, G4, H7, WBA. DevNo 0 only.

Clock: RTC_CLKSRC_DEFAULT starts the LSE, the external oscillator in bypass
when the board low frequency oscillator is a TCXO (g_McuOsc), and takes the
LSI when the LSE does not start within 4 s. RTC_CLKSRC_LFXTAL fails instead,
RTC_CLKSRC_LFRC takes the LSI. LSI typical frequency of the datasheets:
40 kHz F0, 37 kHz L1, 32 kHz other families, divided by 128 with the LSI
predivider (L412, L422, L4P5, L4Q5, WBA). The RTC clock selection changes
only with a backup domain reset, which clears the calendar and the backup
registers: Init does it only when the selected oscillator does not run. The
other settings of the backup domain control register are written back, and
the LSE started again when it ran.

An RTC found running (clock selected, running and enabled) keeps its clock
and its time: the prescalers are rewritten when they do not give 1 Hz from
that clock, a 12 hour format is converted to 24 hours, the day of the week
is set from the date (the calendar reset value has Monday for 2000-01-01),
and the WBA binary mode is cleared (the calendar then restarts). On the LSI
of the F0, F4, L1, L4, G4 and H7 the calendar stops from a system reset
until Init starts the LSI again: the LSI is not in the backup domain on
these families. A new RTC starts at 2000-01-01 00:00:00. bTimeValid is the
INITS flag after Init (calendar year other than 2000), and true after
RtcSetTime.

Prescalers: asynchronous division the largest of 1 to 128 that divides the
clock and leaves at least 1 kHz, synchronous division to 1 Hz. Sub-seconds
in steps of one synchronous count: 1/1024 s on the 32768 Hz LSE, 1/1000 s on
the LSI. The calendar is read from the counters (BYPSHAD): no shadow register
wait after Stop or Standby. SSR is read before and after TR and DR, the read
done again when a second started in between.

Calendar 2000 to 2099, 24 hours. Alarms 0 and 1 are alarm A and B, alarm A
only on the F0. Each field of RtcAlarm_t is matched or masked (-1); MDay and
WDay are the same field of the RTC, one of them only. Sub-seconds are not
matched.

Wakeup timer, not on the F030x8: RTCCLK divided by 2 to 16 with a 16 bit
counter up to 32 s at 32768 Hz, then the 1 Hz clock up to 65536 s, then up
to 131072 s. The smallest divider that holds the period gives the best
resolution.

Backup registers: 5 on the F0 with backup registers, none on the F030x8, 20
on the F4, 32 on the L1, L4, G4, H7 and WBA (TAMP on the G4, L412, L422 and
WBA).

Init leaves the backup domain write access enabled (PWR DBP) for the RTC
and backup register writes, and disables the alarms and the wakeup timer.
Events, with an EvtHandler: alarm and wakeup interrupts, through their EXTI
lines on the F0, F4, L1, L4, G4 and H7 (alarm line 17, 18 on the L4; wakeup
line 20, 22 on the F4, 19 on the H7), direct on the WBA.

@author	Hoang Nguyen Hoan
@date	Oct. 22, 2026

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
#include "miscdev/rtc.h"
#include "idelay.h"

// Initialization and status register: ISR with the flags, or ICSR with the
// flags in SR, cleared in SCR
#ifdef RTC_ICSR_INIT
#define ST_RTC_ICSR				ICSR
#define ST_RTC_INITF			RTC_ICSR_INITF
#define ST_RTC_INITS			RTC_ICSR_INITS
#define ST_RTC_WUTWF			RTC_ICSR_WUTWF
#ifdef RTC_ICSR_ALRAWF
#define ST_RTC_ALRAWF			RTC_ICSR_ALRAWF
#define ST_RTC_ALRBWF			RTC_ICSR_ALRBWF
#endif
#define ST_RTC_ALRAF			RTC_SR_ALRAF
#define ST_RTC_ALRBF			RTC_SR_ALRBF
#define ST_RTC_WUTF				RTC_SR_WUTF
static_assert(RTC_SCR_CALRAF == RTC_SR_ALRAF && RTC_SCR_CALRBF == RTC_SR_ALRBF && RTC_SCR_CWUTF == RTC_SR_WUTF,
			  "SCR clear bits at the SR flag positions");
#else
#define ST_RTC_ICSR				ISR
#define ST_RTC_INITF			RTC_ISR_INITF
#define ST_RTC_INITS			RTC_ISR_INITS
#define ST_RTC_ALRAWF			RTC_ISR_ALRAWF
#define ST_RTC_ALRAF			RTC_ISR_ALRAF
#ifdef RTC_CR_ALRBE
#define ST_RTC_ALRBWF			RTC_ISR_ALRBWF
#define ST_RTC_ALRBF			RTC_ISR_ALRBF
#endif
#ifdef RTC_CR_WUTE
#define ST_RTC_WUTWF			RTC_ISR_WUTWF
#define ST_RTC_WUTF				RTC_ISR_WUTF
#endif
#endif

#ifndef ST_RTC_ALRBF
#define ST_RTC_ALRBF			0U
#endif
#ifndef ST_RTC_WUTF
#define ST_RTC_WUTF				0U
#endif

// Backup domain control: LSE, RTC clock selection and backup domain reset
#if defined(RCC_BDCR1_LSEON)
// WBA: BDCR1, LSI1 in it, no RTC enable bit
#define ST_RTC_BDCR				BDCR1
#define ST_RTC_LSEON			RCC_BDCR1_LSEON
#define ST_RTC_LSERDY			RCC_BDCR1_LSERDY
#define ST_RTC_LSEBYP			RCC_BDCR1_LSEBYP
#define ST_RTC_RTCSEL			RCC_BDCR1_RTCSEL
#define ST_RTC_RTCSEL_Pos		RCC_BDCR1_RTCSEL_Pos
#define ST_RTC_BDRST			RCC_BDCR1_BDRST
#define ST_RTC_LSIREG			BDCR1
#define ST_RTC_LSION			RCC_BDCR1_LSI1ON
#define ST_RTC_LSIRDY			RCC_BDCR1_LSI1RDY
#elif defined(RCC_CSR_RTCEN)
// L1: LSE and RTC in CSR
#define ST_RTC_BDCR				CSR
#define ST_RTC_LSEON			RCC_CSR_LSEON
#define ST_RTC_LSERDY			RCC_CSR_LSERDY
#define ST_RTC_LSEBYP			RCC_CSR_LSEBYP
#define ST_RTC_RTCSEL			RCC_CSR_RTCSEL
#define ST_RTC_RTCSEL_Pos		RCC_CSR_RTCSEL_Pos
#define ST_RTC_BDRST			RCC_CSR_RTCRST
#define ST_RTC_RTCEN			RCC_CSR_RTCEN
#define ST_RTC_LSIREG			CSR
#define ST_RTC_LSION			RCC_CSR_LSION
#define ST_RTC_LSIRDY			RCC_CSR_LSIRDY
#else
#define ST_RTC_BDCR				BDCR
#define ST_RTC_LSEON			RCC_BDCR_LSEON
#define ST_RTC_LSERDY			RCC_BDCR_LSERDY
#define ST_RTC_LSEBYP			RCC_BDCR_LSEBYP
#define ST_RTC_RTCSEL			RCC_BDCR_RTCSEL
#define ST_RTC_RTCSEL_Pos		RCC_BDCR_RTCSEL_Pos
#define ST_RTC_BDRST			RCC_BDCR_BDRST
#define ST_RTC_RTCEN			RCC_BDCR_RTCEN
#define ST_RTC_LSIREG			CSR
#define ST_RTC_LSION			RCC_CSR_LSION
#define ST_RTC_LSIRDY			RCC_CSR_LSIRDY
#endif

// RTCSEL values
#define ST_RTC_SEL_LSE			1U
#define ST_RTC_SEL_LSI			2U

// Typical LSI frequency of the datasheets
#if defined(STM32F0)
#define ST_RTC_LSI_FREQ			40000U
#elif defined(STM32L1)
#define ST_RTC_LSI_FREQ			37000U
#elif defined(STM32F4) || defined(STM32L4) || defined(STM32G4) || defined(STM32H7) || defined(STM32WBA)
#define ST_RTC_LSI_FREQ			32000U
#else
#error "rtc_stm32: LSI frequency of this family not known"
#endif

#define ST_RTC_LSE_FREQ			32768U

// EXTI lines of the alarm and wakeup interrupts, none on the WBA
#if defined(STM32F0)
#define ST_RTC_EXTI_ALARM		17
#define ST_RTC_EXTI_WAKEUP		20
#elif defined(STM32F4)
#define ST_RTC_EXTI_ALARM		17
#define ST_RTC_EXTI_WAKEUP		22
#elif defined(STM32L1)
#define ST_RTC_EXTI_ALARM		17
#define ST_RTC_EXTI_WAKEUP		20
#elif defined(STM32L4)
#define ST_RTC_EXTI_ALARM		18
#define ST_RTC_EXTI_WAKEUP		20
#elif defined(STM32G4)
#define ST_RTC_EXTI_ALARM		17
#define ST_RTC_EXTI_WAKEUP		20
#elif defined(STM32H7)
#define ST_RTC_EXTI_ALARM		17
#define ST_RTC_EXTI_WAKEUP		19
#elif !defined(STM32WBA)
#error "rtc_stm32: RTC EXTI lines of this family not known"
#endif

// One interrupt for the alarms and the wakeup timer on the F0 and WBA, one
// each on the other families
#if defined(STM32F0) || defined(STM32WBA)
#define ST_RTC_ONE_IRQ
#endif

// Backup registers
#if defined(TAMP_BKP31R) || defined(RTC_BKP31R)
#define ST_RTC_NB_BKP			32
#elif defined(RTC_BKP19R)
#define ST_RTC_NB_BKP			20
#elif defined(RTC_BKP4R)
#define ST_RTC_NB_BKP			5
#else
#define ST_RTC_NB_BKP			0
#endif

#define ST_RTC_POLL_US			100U
#define ST_RTC_TIMEOUT_US		1000000U	// Init mode, write flags
#define ST_RTC_LSE_TIMEOUT_US	4000000U	// The LSE crystal can take 2 s to start
#define ST_RTC_LSI_TIMEOUT_US	100000U

// Calendar register fields, the tens above the units
#define ST_RTC_BCD(v)			((((uint32_t)(v) / 10U) << 4) | ((uint32_t)(v) % 10U))

#pragma pack(push, 4)

typedef struct {
	RtcDev_t *pDev;				//!< Device of the last Init
	uint32_t PredivS;			//!< Synchronous prescaler, sub-second counter top
} StRtc_t;

#pragma pack(pop)

static StRtc_t s_StRtc;

#ifdef ST_RTC_ONE_IRQ
extern "C" void RTC_IRQHandler(void);
#else
extern "C" void RTC_Alarm_IRQHandler(void);
extern "C" void RTC_WKUP_IRQHandler(void);
#endif

static StRtc_t *StRtcDev(RtcDev_t * const pDev)
{
	if (pDev == nullptr || s_StRtc.pDev != pDev)
	{
		return nullptr;
	}

	return &s_StRtc;
}

static bool StRtcWait(volatile uint32_t *pReg, uint32_t Mask, uint32_t usTimeout)
{
	for (uint32_t t = 0; (*pReg & Mask) == 0; t += ST_RTC_POLL_US)
	{
		if (t >= usTimeout)
		{
			return false;
		}
		usDelay(ST_RTC_POLL_US);
	}

	return true;
}

// Write access to the backup domain: RCC backup domain control, RTC, backup
// registers
static void StRtcBackupAccess(void)
{
#if defined(PWR_DBPR_DBP)
	RCC->AHB4ENR |= RCC_AHB4ENR_PWREN;
	(void)RCC->AHB4ENR;
	PWR->DBPR |= PWR_DBPR_DBP;
	(void)PWR->DBPR;
#elif defined(PWR_CR1_DBP)
#ifdef RCC_APB1ENR1_PWREN
	RCC->APB1ENR1 |= RCC_APB1ENR1_PWREN;
	(void)RCC->APB1ENR1;
#endif
	PWR->CR1 |= PWR_CR1_DBP;
	(void)PWR->CR1;
#else
	RCC->APB1ENR |= RCC_APB1ENR_PWREN;
	(void)RCC->APB1ENR;
	PWR->CR |= PWR_CR_DBP;
	(void)PWR->CR;
#endif
}

// Bus clock of the RTC registers, where it has one
static void StRtcBusClock(void)
{
#if defined(RCC_APB7ENR_RTCAPBEN)
	RCC->APB7ENR |= RCC_APB7ENR_RTCAPBEN;
	(void)RCC->APB7ENR;
#elif defined(RCC_APB1ENR1_RTCAPBEN)
	RCC->APB1ENR1 |= RCC_APB1ENR1_RTCAPBEN;
	(void)RCC->APB1ENR1;
#elif defined(RCC_APB4ENR_RTCAPBEN)
	RCC->APB4ENR |= RCC_APB4ENR_RTCAPBEN;
	(void)RCC->APB4ENR;
#endif
}

static uint32_t StRtcLsiFreq(void)
{
#if defined(RCC_BDCR1_LSI1PREDIV)
	return RCC->BDCR1 & RCC_BDCR1_LSI1PREDIV ? ST_RTC_LSI_FREQ / 128U : ST_RTC_LSI_FREQ;
#elif defined(RCC_CSR_LSIPREDIV)
	return RCC->CSR & RCC_CSR_LSIPREDIV ? ST_RTC_LSI_FREQ / 128U : ST_RTC_LSI_FREQ;
#else
	return ST_RTC_LSI_FREQ;
#endif
}

static uint32_t StRtcSel(void)
{
	return (RCC->ST_RTC_BDCR & ST_RTC_RTCSEL) >> ST_RTC_RTCSEL_Pos;
}

// LSE running, started if needed: crystal, or external oscillator (bypass)
// on a TCXO board
static bool StRtcLseStart(void)
{
	volatile uint32_t *bdcr = &RCC->ST_RTC_BDCR;

	if ((*bdcr & ST_RTC_LSEON) == 0)
	{
		// LSEBYP only changes with the LSE off
		if (GetLowFreqOscType() == OSC_TYPE_TCXO)
		{
			*bdcr |= ST_RTC_LSEBYP;
		}
		else
		{
			*bdcr &= ~ST_RTC_LSEBYP;
		}
	}
	*bdcr |= ST_RTC_LSEON;
	if (StRtcWait(bdcr, ST_RTC_LSERDY, ST_RTC_LSE_TIMEOUT_US))
	{
		return true;
	}
	*bdcr &= ~ST_RTC_LSEON;

	return false;
}

static bool StRtcLsiStart(void)
{
	volatile uint32_t *reg = &RCC->ST_RTC_LSIREG;

	if ((*reg & ST_RTC_LSIRDY) == 0)
	{
		*reg |= ST_RTC_LSION;
	}

	return StRtcWait(reg, ST_RTC_LSIRDY, ST_RTC_LSI_TIMEOUT_US);
}

// Backup domain reset, which frees the RTC clock selection. The other
// settings of the backup domain control register are written back and the
// oscillators that ran start again: LSE of the other peripherals, LSI and
// radio sleep timer clock of the WBA. The clock security system is not
// enabled again. False when the LSE ran and does not start again, it is
// then off.
static bool StRtcBackupReset(void)
{
	volatile uint32_t *bdcr = &RCC->ST_RTC_BDCR;
	uint32_t keep = *bdcr & ~(ST_RTC_RTCSEL | ST_RTC_BDRST);
	uint32_t on = ST_RTC_LSEON;
	uint32_t late = 0;

#ifdef ST_RTC_RTCEN
	keep &= ~ST_RTC_RTCEN;
#endif
#if defined(RCC_BDCR1_LSECSSON)
	keep &= ~RCC_BDCR1_LSECSSON;
#elif defined(RCC_BDCR_LSECSSON)
	keep &= ~RCC_BDCR_LSECSSON;
#elif defined(RCC_CSR_LSECSSON)
	keep &= ~RCC_CSR_LSECSSON;
#endif
#ifdef RCC_BDCR1_LSI1ON
	on |= RCC_BDCR1_LSI1ON;
#endif
#ifdef RCC_BDCR1_LSI2ON
	on |= RCC_BDCR1_LSI2ON;
#endif
#ifdef RCC_BDCR1_LSESYSEN
	// LSE of the other peripherals enabled once the LSE is ready
	late = RCC_BDCR1_LSESYSEN;
#endif

	*bdcr |= ST_RTC_BDRST;
	*bdcr &= ~ST_RTC_BDRST;

	// Oscillator settings with the oscillators off, then the oscillators
	*bdcr = keep & ~(on | late);
	*bdcr = keep & ~late;
	if ((keep & ST_RTC_LSEON) == 0 || StRtcWait(bdcr, ST_RTC_LSERDY, ST_RTC_LSE_TIMEOUT_US))
	{
		*bdcr = keep;

		return true;
	}
	*bdcr &= ~ST_RTC_LSEON;

	return false;
}

static void StRtcUnlock(void)
{
	RTC->WPR = 0xCAU;
	RTC->WPR = 0x53U;
}

static void StRtcLock(void)
{
	RTC->WPR = 0xFFU;
}

// Initialization mode: calendar stopped, TR, DR, PRER and FMT writable
static bool StRtcEnterInit(void)
{
	if (RTC->ST_RTC_ICSR & ST_RTC_INITF)
	{
		return true;
	}
#ifdef RTC_ICSR_INIT
	RTC->ICSR |= RTC_ICSR_INIT;
#else
	// INIT set, 1 leaves the flags of ISR unchanged
	RTC->ISR = 0xFFFFFFFFU;
#endif

	return StRtcWait(&RTC->ST_RTC_ICSR, ST_RTC_INITF, ST_RTC_TIMEOUT_US);
}

static void StRtcExitInit(void)
{
#ifdef RTC_ICSR_INIT
	RTC->ICSR &= ~RTC_ICSR_INIT;
#else
	RTC->ISR = ~RTC_ISR_INIT;
#endif
}

static uint32_t StRtcFlags(void)
{
#ifdef RTC_SCR_CALRAF
	return RTC->SR;
#else
	return RTC->ISR;
#endif
}

static void StRtcClearFlags(uint32_t Flags)
{
#ifdef RTC_SCR_CALRAF
	RTC->SCR = Flags;
#else
	// 0 clears a flag, 1 leaves it; INIT 0, not called in the initialization
	// mode
	RTC->ISR = ~(Flags | RTC_ISR_INIT);
#endif
}

#ifdef ST_RTC_EXTI_ALARM
static void StRtcExtiClear(int Line)
{
#if defined(EXTI_D1)
	EXTI_D1->PR1 = 1UL << Line;
#elif defined(EXTI_IMR1_IM0)
	EXTI->PR1 = 1UL << Line;
#else
	EXTI->PR = 1UL << Line;
#endif
}

// Rising edge interrupt of an RTC EXTI line
static void StRtcExtiEnable(int Line)
{
	uint32_t bit = 1UL << Line;

#if defined(EXTI_D1)
	EXTI->RTSR1 |= bit;
	EXTI_D1->PR1 = bit;
	EXTI_D1->IMR1 |= bit;
#elif defined(EXTI_IMR1_IM0)
	EXTI->RTSR1 |= bit;
	EXTI->PR1 = bit;
	EXTI->IMR1 |= bit;
#else
	EXTI->RTSR |= bit;
	EXTI->PR = bit;
	EXTI->IMR |= bit;
#endif
}
#endif

// Prescalers of a clock: asynchronous the largest division of 1 to 128 that
// divides the clock and leaves at least 1 kHz, synchronous to 1 Hz
static uint32_t StRtcPrer(uint32_t Freq)
{
	uint32_t a = 1;

	for (uint32_t d = 128; d > 1; d--)
	{
		if (Freq % d == 0 && Freq / d >= 1000U)
		{
			a = d;
			break;
		}
	}

	return ((a - 1U) << RTC_PRER_PREDIV_A_Pos) | (Freq / a - 1U);
}

static bool StRtcLeap(int Year)
{
	return (Year % 4 == 0 && Year % 100 != 0) || Year % 400 == 0;
}

// Days of a month, Mon 0 January to 11 December
static int StRtcMonthDays(int Year, int Mon)
{
	if (Mon == 1)
	{
		return StRtcLeap(Year) ? 29 : 28;
	}

	// 31 days in the even months to July and the odd months from August
	return 31 - ((Mon + (Mon >= 7 ? 1 : 0)) & 1);
}

// Day of the week, 0 Sunday, Mon 1 to 12
static int StRtcWeekDay(int Year, int Mon, int Day)
{
	static const uint8_t s_Offs[12] = { 0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4 };

	if (Mon < 3)
	{
		Year--;
	}

	return (Year + Year / 4 - Year / 100 + Year / 400 + s_Offs[Mon - 1] + Day) % 7;
}

static uint32_t StRtcBcdVal(uint32_t Reg, uint32_t Pos, uint32_t TensMask)
{
	return ((Reg >> (Pos + 4U)) & TensMask) * 10U + ((Reg >> Pos) & 0xFU);
}

// Day of the week field of a DR date, 1 Monday to 7 Sunday
static uint32_t StRtcWdu(uint32_t Dr)
{
	uint32_t mon = StRtcBcdVal(Dr, RTC_DR_MU_Pos, 1U);
	int wday = StRtcWeekDay(2000 + (int)StRtcBcdVal(Dr, RTC_DR_YU_Pos, 0xFU), mon >= 1U && mon <= 12U ? (int)mon : 1,
							(int)StRtcBcdVal(Dr, RTC_DR_DU_Pos, 3U));

	return wday == 0 ? 7U : (uint32_t)wday;
}

// Hour of TR in 24 hour format, from the 12 hour format when FMT is set
static uint32_t StRtcHour(uint32_t Tr)
{
	uint32_t hour = StRtcBcdVal(Tr, RTC_TR_HU_Pos, 3U);

	if (RTC->CR & RTC_CR_FMT)
	{
		hour %= 12U;
		if (Tr & RTC_TR_PM)
		{
			hour += 12U;
		}
	}

	return hour;
}

static void StRtcDisableEvents(void)
{
	uint32_t en = RTC_CR_ALRAE | RTC_CR_ALRAIE;

#ifdef RTC_CR_ALRBE
	en |= RTC_CR_ALRBE | RTC_CR_ALRBIE;
#endif
#ifdef RTC_CR_WUTE
	en |= RTC_CR_WUTE | RTC_CR_WUTIE;
#endif
	RTC->CR &= ~en;
	StRtcClearFlags(ST_RTC_ALRAF | ST_RTC_ALRBF | ST_RTC_WUTF);
}

bool RtcInit(RtcDev_t * const pDev, const RtcCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 ||
		(pCfg->ClkSrc != RTC_CLKSRC_DEFAULT && pCfg->ClkSrc != RTC_CLKSRC_LFXTAL &&
		 pCfg->ClkSrc != RTC_CLKSRC_LFRC))
	{
		return false;
	}

	StRtcBackupAccess();
	StRtcBusClock();

	// The selected clock running: the RTC keeps its time
	uint32_t sel = StRtcSel();
	int lse = -1;
	bool running = false;

	if (sel == ST_RTC_SEL_LSE)
	{
		lse = StRtcLseStart() ? 1 : 0;
		running = lse == 1;
	}
	else if (sel == ST_RTC_SEL_LSI)
	{
		running = StRtcLsiStart();
	}
#ifdef ST_RTC_RTCEN
	if ((RCC->ST_RTC_BDCR & ST_RTC_RTCEN) == 0)
	{
		// RTC clock disabled: the calendar did not count
		running = false;
	}
#endif

	if (running == false)
	{
		if (pCfg->ClkSrc != RTC_CLKSRC_LFRC && lse < 0)
		{
			lse = StRtcLseStart() ? 1 : 0;
		}
		if (lse != 1 && pCfg->ClkSrc == RTC_CLKSRC_LFXTAL)
		{
			return false;
		}

		uint32_t want = lse == 1 && pCfg->ClkSrc != RTC_CLKSRC_LFRC ? ST_RTC_SEL_LSE : ST_RTC_SEL_LSI;

		// RTCSEL changes only after a backup domain reset
		if (sel != 0 && sel != want && StRtcBackupReset() == false && want == ST_RTC_SEL_LSE)
		{
			return false;
		}
		if (want == ST_RTC_SEL_LSI && StRtcLsiStart() == false)
		{
			return false;
		}
		RCC->ST_RTC_BDCR = (RCC->ST_RTC_BDCR & ~ST_RTC_RTCSEL) | (want << ST_RTC_RTCSEL_Pos);
		sel = want;
	}
#ifdef ST_RTC_RTCEN
	RCC->ST_RTC_BDCR |= ST_RTC_RTCEN;
#endif

	uint32_t freq = ST_RTC_LSE_FREQ;

	if (sel == ST_RTC_SEL_LSE)
	{
		if (GetLowFreqOscType() != OSC_TYPE_RC && GetLowFreqOscFreq() != 0)
		{
			freq = GetLowFreqOscFreq();
		}
		pDev->ClkSrc = RTC_CLKSRC_LFXTAL;
	}
	else
	{
		freq = StRtcLsiFreq();
		pDev->ClkSrc = RTC_CLKSRC_LFRC;
	}

	uint32_t prer = StRtcPrer(freq);
	uint32_t cur = RTC->PRER & (RTC_PRER_PREDIV_A | RTC_PRER_PREDIV_S);
	uint32_t a = (cur >> RTC_PRER_PREDIV_A_Pos) + 1U;
	uint32_t s = (cur & RTC_PRER_PREDIV_S) + 1U;
	bool fresh = running == false;

#ifdef RTC_ICSR_BIN
	// Binary mode: the calendar restarts in BCD
	if (RTC->ICSR & RTC_ICSR_BIN)
	{
		fresh = true;
	}
#endif

	StRtcUnlock();

	// Calendar read from the counters
	RTC->CR |= RTC_CR_BYPSHAD;
	StRtcDisableEvents();

	// Day of the week from the date: the reset value of the calendar has
	// Monday for 2000-01-01
	uint32_t dr = RTC->DR;
	bool wdfix = ((dr & RTC_DR_WDU) >> RTC_DR_WDU_Pos) != StRtcWdu(dr);

	if (fresh || a * s != freq || (RTC->CR & RTC_CR_FMT) || wdfix)
	{
		if (StRtcEnterInit() == false)
		{
			StRtcLock();
			return false;
		}

		if (fresh)
		{
#ifdef RTC_ICSR_BIN
			RTC->ICSR &= ~RTC_ICSR_BIN;
#endif
			// 2000-01-01, a Saturday
			RTC->TR = 0;
			RTC->DR = (6UL << RTC_DR_WDU_Pos) | (1UL << RTC_DR_MU_Pos) | (1UL << RTC_DR_DU_Pos);
		}
		else
		{
			// Counters stopped: the date and time read are stable. The hour in
			// 24 hour format, written once FMT is cleared.
			uint32_t tr = RTC->TR;
			uint32_t hour = StRtcHour(tr);

			dr = RTC->DR;
			RTC->CR &= ~RTC_CR_FMT;
			RTC->TR = (tr & ~(RTC_TR_PM | RTC_TR_HT | RTC_TR_HU)) | (ST_RTC_BCD(hour) << RTC_TR_HU_Pos);
			RTC->DR = (dr & ~RTC_DR_WDU) | (StRtcWdu(dr) << RTC_DR_WDU_Pos);
		}
		RTC->CR &= ~RTC_CR_FMT;
		if (a * s != freq || fresh)
		{
			// Two separate writes, synchronous first
			RTC->PRER = prer & RTC_PRER_PREDIV_S;
			RTC->PRER = prer;
		}
		StRtcExitInit();
	}
	StRtcLock();

	s_StRtc.PredivS = RTC->PRER & RTC_PRER_PREDIV_S;
	s_StRtc.pDev = pDev;

	pDev->DevNo = 0;
	pDev->ClkFreq = freq;
#ifdef RTC_CR_ALRBE
	pDev->NbAlarm = 2;
#else
	pDev->NbAlarm = 1;
#endif
	pDev->NbBackup = ST_RTC_NB_BKP;
	pDev->msWakeup = 0;
	pDev->bTimeValid = (RTC->ST_RTC_ICSR & ST_RTC_INITS) != 0;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->pDevData = &s_StRtc;

	// Events
#ifdef ST_RTC_ONE_IRQ
	NVIC_DisableIRQ(RTC_IRQn);
	NVIC_ClearPendingIRQ(RTC_IRQn);
#else
	NVIC_DisableIRQ(RTC_Alarm_IRQn);
	NVIC_DisableIRQ(RTC_WKUP_IRQn);
	NVIC_ClearPendingIRQ(RTC_Alarm_IRQn);
	NVIC_ClearPendingIRQ(RTC_WKUP_IRQn);
#endif
	if (pCfg->EvtHandler != nullptr)
	{
#ifdef ST_RTC_EXTI_ALARM
		StRtcExtiEnable(ST_RTC_EXTI_ALARM);
#ifdef RTC_CR_WUTE
		StRtcExtiEnable(ST_RTC_EXTI_WAKEUP);
#endif
#endif
#ifdef ST_RTC_ONE_IRQ
		NVIC_SetPriority(RTC_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(RTC_IRQn);
#else
		NVIC_SetPriority(RTC_Alarm_IRQn, pCfg->IntPrio);
		NVIC_SetPriority(RTC_WKUP_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(RTC_Alarm_IRQn);
		NVIC_EnableIRQ(RTC_WKUP_IRQn);
#endif
	}

	return true;
}

bool RtcSetTime(RtcDev_t * const pDev, const struct tm * const pTime)
{
	if (StRtcDev(pDev) == nullptr || pTime == nullptr)
	{
		return false;
	}

	const struct tm &t = *pTime;

	if (t.tm_year < 100 || t.tm_year > 199 || t.tm_mon < 0 || t.tm_mon > 11 || t.tm_mday < 1 ||
		t.tm_mday > StRtcMonthDays(t.tm_year + 1900, t.tm_mon) || t.tm_hour < 0 || t.tm_hour > 23 ||
		t.tm_min < 0 || t.tm_min > 59 || t.tm_sec < 0 || t.tm_sec > 59)
	{
		return false;
	}

	int wday = StRtcWeekDay(t.tm_year + 1900, t.tm_mon + 1, t.tm_mday);
	uint32_t tr = (ST_RTC_BCD(t.tm_hour) << RTC_TR_HU_Pos) | (ST_RTC_BCD(t.tm_min) << RTC_TR_MNU_Pos) |
				  (ST_RTC_BCD(t.tm_sec) << RTC_TR_SU_Pos);
	uint32_t dr = (ST_RTC_BCD(t.tm_year - 100) << RTC_DR_YU_Pos) | ((uint32_t)(wday == 0 ? 7 : wday) << RTC_DR_WDU_Pos) |
				  (ST_RTC_BCD(t.tm_mon + 1) << RTC_DR_MU_Pos) | (ST_RTC_BCD(t.tm_mday) << RTC_DR_DU_Pos);

	StRtcUnlock();
	if (StRtcEnterInit() == false)
	{
		StRtcLock();
		return false;
	}
	RTC->TR = tr;
	RTC->DR = dr;
	StRtcExitInit();
	StRtcLock();
	pDev->bTimeValid = true;

	return true;
}

bool RtcGetTime(RtcDev_t * const pDev, struct tm * const pTime, uint32_t * const pUsec)
{
	StRtc_t *dev = StRtcDev(pDev);

	if (dev == nullptr || pTime == nullptr)
	{
		return false;
	}

	// SSR, TR, DR, then SSR again: no new second between the SSR reads (SSR
	// counts down, a second starts at PREDIV_S) gives TR and DR of the same
	// second. The read after a new second is coherent.
	uint32_t ssr, tr, dr;
	int n = 0;

	do
	{
		if (++n > 2)
		{
			return false;
		}
		ssr = RTC->SSR & RTC_SSR_SS;
		tr = RTC->TR;
		dr = RTC->DR;
	} while ((RTC->SSR & RTC_SSR_SS) > ssr);

	struct tm &t = *pTime;

	t.tm_sec = (int)StRtcBcdVal(tr, RTC_TR_SU_Pos, 7U);
	t.tm_min = (int)StRtcBcdVal(tr, RTC_TR_MNU_Pos, 7U);
	t.tm_hour = (int)StRtcHour(tr);
	t.tm_mday = (int)StRtcBcdVal(dr, RTC_DR_DU_Pos, 3U);
	t.tm_mon = (int)StRtcBcdVal(dr, RTC_DR_MU_Pos, 1U) - 1;
	t.tm_year = (int)StRtcBcdVal(dr, RTC_DR_YU_Pos, 0xFU) + 100;
	t.tm_wday = (int)(((dr & RTC_DR_WDU) >> RTC_DR_WDU_Pos) % 7U);
	t.tm_isdst = -1;

	int yday = t.tm_mday - 1;

	for (int m = 0; m < t.tm_mon && m < 12; m++)
	{
		yday += StRtcMonthDays(t.tm_year + 1900, m);
	}
	t.tm_yday = yday;

	if (pUsec != nullptr)
	{
		uint32_t s = dev->PredivS;

		if (ssr > s)
		{
			ssr = s;
		}
		*pUsec = (uint32_t)((uint64_t)(s - ssr) * 1000000U / (s + 1U));
	}

	return true;
}

bool RtcSetAlarm(RtcDev_t * const pDev, int Idx, const RtcAlarm_t * const pAlarm)
{
	if (StRtcDev(pDev) == nullptr || pAlarm == nullptr || Idx < 0 || Idx >= pDev->NbAlarm)
	{
		return false;
	}

	const RtcAlarm_t &a = *pAlarm;

	if (a.Sec < -1 || a.Sec > 59 || a.Min < -1 || a.Min > 59 || a.Hour < -1 || a.Hour > 23 ||
		a.MDay < -1 || a.MDay == 0 || a.MDay > 31 || a.WDay < -1 || a.WDay > 6 || (a.MDay >= 0 && a.WDay >= 0))
	{
		return false;
	}

	uint32_t val = a.Sec < 0 ? RTC_ALRMAR_MSK1 : ST_RTC_BCD(a.Sec) << RTC_ALRMAR_SU_Pos;

	val |= a.Min < 0 ? RTC_ALRMAR_MSK2 : ST_RTC_BCD(a.Min) << RTC_ALRMAR_MNU_Pos;
	val |= a.Hour < 0 ? RTC_ALRMAR_MSK3 : ST_RTC_BCD(a.Hour) << RTC_ALRMAR_HU_Pos;
	if (a.WDay >= 0)
	{
		val |= RTC_ALRMAR_WDSEL | ((uint32_t)(a.WDay == 0 ? 7 : a.WDay) << RTC_ALRMAR_DU_Pos);
	}
	else if (a.MDay > 0)
	{
		val |= ST_RTC_BCD(a.MDay) << RTC_ALRMAR_DU_Pos;
	}
	else
	{
		val |= RTC_ALRMAR_MSK4;
	}

	StRtcUnlock();
	if (Idx == 0)
	{
		RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
		StRtcClearFlags(ST_RTC_ALRAF);
#ifdef ST_RTC_ALRAWF
		if (StRtcWait(&RTC->ST_RTC_ICSR, ST_RTC_ALRAWF, ST_RTC_TIMEOUT_US) == false)
		{
			StRtcLock();
			return false;
		}
#endif
		RTC->ALRMAR = val;
		RTC->ALRMASSR = 0;
		RTC->CR |= RTC_CR_ALRAE | RTC_CR_ALRAIE;
	}
#ifdef RTC_CR_ALRBE
	else
	{
		RTC->CR &= ~(RTC_CR_ALRBE | RTC_CR_ALRBIE);
		StRtcClearFlags(ST_RTC_ALRBF);
#ifdef ST_RTC_ALRBWF
		if (StRtcWait(&RTC->ST_RTC_ICSR, ST_RTC_ALRBWF, ST_RTC_TIMEOUT_US) == false)
		{
			StRtcLock();
			return false;
		}
#endif
		RTC->ALRMBR = val;
		RTC->ALRMBSSR = 0;
		RTC->CR |= RTC_CR_ALRBE | RTC_CR_ALRBIE;
	}
#endif
	StRtcLock();

	return true;
}

void RtcDisableAlarm(RtcDev_t * const pDev, int Idx)
{
	if (StRtcDev(pDev) == nullptr || Idx < 0 || Idx >= pDev->NbAlarm)
	{
		return;
	}

	StRtcUnlock();
	if (Idx == 0)
	{
		RTC->CR &= ~(RTC_CR_ALRAE | RTC_CR_ALRAIE);
		StRtcClearFlags(ST_RTC_ALRAF);
	}
#ifdef RTC_CR_ALRBE
	else
	{
		RTC->CR &= ~(RTC_CR_ALRBE | RTC_CR_ALRBIE);
		StRtcClearFlags(ST_RTC_ALRBF);
	}
#endif
	StRtcLock();
}

bool RtcSetWakeup(RtcDev_t * const pDev, uint32_t msPeriod)
{
	if (StRtcDev(pDev) == nullptr)
	{
		return false;
	}

#ifdef RTC_CR_WUTE
	uint32_t sel = 0;
	uint32_t wut = 0;
	uint32_t ms = 0;

	if (msPeriod != 0)
	{
		uint64_t f = pDev->ClkFreq;
		bool ok = false;

		// RTCCLK / 2, 4, 8, 16: WUCKSEL 3, 2, 1, 0
		for (uint32_t div = 2, code = 3; div <= 16; div <<= 1, code--)
		{
			uint64_t ticks = ((uint64_t)msPeriod * f + div * 500U) / (div * 1000U);

			if (ticks == 0)
			{
				// Shorter than one count: the shortest period
				ticks = 1;
			}
			if (ticks <= 65536)
			{
				sel = code;
				wut = (uint32_t)ticks - 1U;
				ms = (uint32_t)((ticks * div * 1000U + f / 2U) / f);
				ok = true;
				break;
			}
		}
		if (ok == false)
		{
			// 1 Hz clock, WUCKSEL 4, then 6 with 65536 added
			uint32_t sec = (msPeriod + 500U) / 1000U;

			if (sec <= 65536U)
			{
				sel = 4;
				wut = sec - 1U;
			}
			else if (sec <= 131072U)
			{
				sel = 6;
				wut = sec - 65537U;
			}
			else
			{
				return false;
			}
			ms = sec * 1000U;
		}
	}

	StRtcUnlock();
	RTC->CR &= ~(RTC_CR_WUTE | RTC_CR_WUTIE);
	StRtcClearFlags(ST_RTC_WUTF);
	pDev->msWakeup = 0;
	if (msPeriod != 0)
	{
		// WUTR and WUCKSEL writable once WUTWF is set
		if (StRtcWait(&RTC->ST_RTC_ICSR, ST_RTC_WUTWF, ST_RTC_TIMEOUT_US) == false)
		{
			StRtcLock();
			return false;
		}
		RTC->WUTR = wut;
		RTC->CR = (RTC->CR & ~RTC_CR_WUCKSEL) | (sel << RTC_CR_WUCKSEL_Pos);
		RTC->CR |= RTC_CR_WUTE | RTC_CR_WUTIE;
		pDev->msWakeup = ms;
	}
	StRtcLock();

	return true;
#else
	// No wakeup timer on this RTC
	return msPeriod == 0;
#endif
}

bool RtcWriteBackup(RtcDev_t * const pDev, int Idx, uint32_t Data)
{
	if (StRtcDev(pDev) == nullptr || Idx < 0 || Idx >= ST_RTC_NB_BKP)
	{
		return false;
	}

#if defined(TAMP_BKP0R)
	(&TAMP->BKP0R)[Idx] = Data;
#elif defined(RTC_BKP0R)
	(&RTC->BKP0R)[Idx] = Data;
#else
	(void)Data;
#endif

	return true;
}

bool RtcReadBackup(RtcDev_t * const pDev, int Idx, uint32_t * const pData)
{
	if (StRtcDev(pDev) == nullptr || pData == nullptr || Idx < 0 || Idx >= ST_RTC_NB_BKP)
	{
		return false;
	}

#if defined(TAMP_BKP0R)
	*pData = (&TAMP->BKP0R)[Idx];
#elif defined(RTC_BKP0R)
	*pData = (&RTC->BKP0R)[Idx];
#endif

	return true;
}

// Events of the alarm and wakeup flags, the flags cleared. Each interrupt
// services all the flags, an event is reported once.
static void StRtcService(void)
{
	uint32_t flags = StRtcFlags() & (ST_RTC_ALRAF | ST_RTC_ALRBF | ST_RTC_WUTF);

	if (flags == 0)
	{
		return;
	}
	StRtcClearFlags(flags);

	uint32_t evt = 0;

	if (flags & ST_RTC_ALRAF)
	{
		evt |= RTC_EVT_ALARM0;
	}
#ifdef RTC_CR_ALRBE
	if (flags & ST_RTC_ALRBF)
	{
		evt |= RTC_EVT_ALARM1;
	}
#endif
#ifdef RTC_CR_WUTE
	if (flags & ST_RTC_WUTF)
	{
		evt |= RTC_EVT_WAKEUP;
	}
#endif

	RtcDev_t *dev = s_StRtc.pDev;

	if (dev != nullptr && dev->EvtHandler != nullptr)
	{
		dev->EvtHandler(dev, evt);
	}
}

#ifdef ST_RTC_ONE_IRQ
void RTC_IRQHandler(void)
{
#ifdef ST_RTC_EXTI_ALARM
	StRtcExtiClear(ST_RTC_EXTI_ALARM);
#ifdef RTC_CR_WUTE
	StRtcExtiClear(ST_RTC_EXTI_WAKEUP);
#endif
#endif
	StRtcService();
}
#else
void RTC_Alarm_IRQHandler(void)
{
	StRtcExtiClear(ST_RTC_EXTI_ALARM);
	StRtcService();
}

void RTC_WKUP_IRQHandler(void)
{
	StRtcExtiClear(ST_RTC_EXTI_WAKEUP);
	StRtcService();
}
#endif
