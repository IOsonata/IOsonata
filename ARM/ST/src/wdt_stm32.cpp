/**-------------------------------------------------------------------------
@file	wdt_stm32.cpp

@brief	STM32 independent watchdog (IWDG)

Generic watchdog API (coredev/wdt.h) on the IWDG of the STM32 families: F0,
F4, L1, L4, L4+, G4, H7 (IWDG1), WBA. DevNo 0 only.

The IWDG counts down a 12 bit counter at the LSI clock divided by 4 to 256,
4 to 1024 on the WBA. The timeout takes the smallest divider that holds
msTimeout, rounded up to the next counter tick, at the typical LSI frequency
of the datasheets: 40 kHz on the F0, 37 kHz on the L1, 32 kHz on the other
families, divided by 128 when the LSI predivider is set (L412, L422, L4P5,
L4Q5, WBA). The LSI is an RC oscillator, the timeout of a part follows its
LSI frequency (LSI characteristics of its datasheet). Init fails on a
timeout longer than 4096 ticks of the largest divider. WdtDev_t::msTimeout
is the timeout in effect, rounded up.

msWindow, on the parts with the window register (F0, L4, G4, H7, WBA): a
reload sooner than msWindow, rounded down to a counter tick, after the
previous reload resets the MCU. Init fails on a window on the F4 and L1, or
a window not shorter than the timeout.

NbChan 1 to 32 reload channels, in software: the IWDG is reloaded when each
channel has been reloaded since the previous IWDG reload.

bRunSleep must be true: the IWDG counts in Sleep mode. In Stop and Standby
it follows the IWDG_STOP and IWDG_STDBY option bytes where the part has
them. bRunHalt false stops the IWDG while a debugger halts the core (DBGMCU
freeze bit), true lets it count.

EvtHandler, WBA only: early wakeup interrupt one counter tick before the
reset, with IntPrio. Init fails with an EvtHandler on the other families,
their IWDG has no interrupt.

Init writes no IWDG register, except the early wakeup register on the WBA
when the IWDG runs. WdtStart starts the IWDG and writes its configuration;
the IWDG then runs until a reset. A system reset stops it, unless the IWDG_SW
option byte selects the hardware watchdog: the IWDG then runs from the reset
with its reset configuration (divider 4, reload 4095), and Init keeps that
configuration and reports it. WdtRunning is true after WdtStart, with the
hardware watchdog option, and on the WBA when the IWDG runs (SR ONF). On the
other families an IWDG started by code before the application (a boot
loader) is not seen: WdtStart writes the configuration of Init.

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

#include "stm32.h"
#include "coredev/wdt.h"
#include "coredev/interrupt.h"
#include "idelay.h"

#if defined(IWDG)
#define ST_IWDG					IWDG
#elif defined(IWDG1)
#define ST_IWDG					IWDG1
#else
#error "wdt_stm32: no IWDG on this MCU"
#endif

// Typical LSI frequency of the datasheets
#if defined(STM32F0)
#define ST_IWDG_LSI_FREQ		40000U
#elif defined(STM32L1)
#define ST_IWDG_LSI_FREQ		37000U
#elif defined(STM32F4) || defined(STM32L4) || defined(STM32G4) || defined(STM32H7) || defined(STM32WBA)
#define ST_IWDG_LSI_FREQ		32000U
#else
#error "wdt_stm32: LSI frequency of this family not known"
#endif

#ifndef LSI_STARTUP_TIME
#error "wdt_stm32: the device header gives no LSI_STARTUP_TIME"
#endif

#define ST_IWDG_KEY_RELOAD		0xAAAAU
#define ST_IWDG_KEY_ACCESS		0x5555U
#define ST_IWDG_KEY_START		0xCCCCU

#define ST_IWDG_TICK_MAX		4096U	// Counter ticks of the longest timeout, reload 4095
#define ST_IWDG_WINR_NONE		0xFFFU	// Window register value without window
#define ST_IWDG_CHAN_MAX		32

// Largest prescaler register value with its own divider: 6 (256) on the 3
// bit register, 8 (1024) on the 4 bit register of the WBA
#define ST_IWDG_PR_MAX			((IWDG_PR_PR >> IWDG_PR_PR_Pos) > 7U ? 8U : 6U)

// Status flags of the register updates
#if defined(IWDG_SR_EWU)
#define ST_IWDG_SR_UPDATE		(IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU | IWDG_SR_EWU)
#elif defined(IWDG_SR_WVU)
#define ST_IWDG_SR_UPDATE		(IWDG_SR_PVU | IWDG_SR_RVU | IWDG_SR_WVU)
#else
#define ST_IWDG_SR_UPDATE		(IWDG_SR_PVU | IWDG_SR_RVU)
#endif

// Status poll period while an update completes, usec
#define ST_IWDG_POLL_US			50U

#ifdef IWDG_EWCR_EWIE
// The early wakeup interrupt comes when the counter is EWIT - 1 or lower,
// EWIT above 1: 2 for the last tick before the reset
#define ST_IWDG_EWIT			2U
#endif

#pragma pack(push, 4)

typedef struct {
	WdtDev_t *pDev;					//!< Device of the last Init
	uint32_t Pr;					//!< Prescaler register value for WdtStart
	uint32_t Rlr;					//!< Reload register value for WdtStart
	uint32_t Winr;					//!< Window register value for WdtStart
	uint32_t Ewcr;					//!< Early wakeup register value
	uint32_t ChanMask;				//!< Reload channels
	uint32_t ReloadMask;			//!< Channels reloaded since the IWDG reload
	bool bStarted;					//!< Started by WdtStart
} StIwdg_t;

#pragma pack(pop)

static StIwdg_t s_StIwdg;

#ifdef IWDG_EWCR_EWIE
extern "C" void IWDG_IRQHandler(void);
#endif

// Counter clock of the IWDG before its divider: the LSI, divided by 128
// when its predivider is set
static uint32_t StIwdgLsiFreq(void)
{
#if defined(RCC_BDCR1_LSI1PREDIV)
	return RCC->BDCR1 & RCC_BDCR1_LSI1PREDIV ? ST_IWDG_LSI_FREQ / 128U : ST_IWDG_LSI_FREQ;
#elif defined(RCC_CSR_LSIPREDIV)
	return RCC->CSR & RCC_CSR_LSIPREDIV ? ST_IWDG_LSI_FREQ / 128U : ST_IWDG_LSI_FREQ;
#else
	return ST_IWDG_LSI_FREQ;
#endif
}

// Divider of a prescaler register value
static uint32_t StIwdgDiv(uint32_t Pr)
{
	return 4U << (Pr > ST_IWDG_PR_MAX ? ST_IWDG_PR_MAX : Pr);
}

// Timeout of a configuration, msec rounded up
static uint32_t StIwdgMs(uint32_t Pr, uint32_t Rlr, uint32_t Lsi)
{
	uint64_t n = (uint64_t)(Rlr + 1U) * StIwdgDiv(Pr) * 1000U;

	return (uint32_t)((n + Lsi - 1U) / Lsi);
}

// IWDG running: SR ONF where the IWDG has it, otherwise started by WdtStart
// or from the reset by the hardware watchdog option (IWDG_SW option bit 0)
static bool StIwdgRunning(void)
{
#if defined(IWDG_SR_ONF)
	return (ST_IWDG->SR & IWDG_SR_ONF) != 0;
#elif defined(FLASH_OPTR_IWDG_SW)
	return s_StIwdg.bStarted || (FLASH->OPTR & FLASH_OPTR_IWDG_SW) == 0;
#elif defined(FLASH_OBR_IWDG_SW)
	return s_StIwdg.bStarted || (FLASH->OBR & FLASH_OBR_IWDG_SW) == 0;
#elif defined(FLASH_OPTCR_WDG_SW)
	return s_StIwdg.bStarted || (FLASH->OPTCR & FLASH_OPTCR_WDG_SW) == 0;
#elif defined(FLASH_OPTSR_IWDG1_SW)
	return s_StIwdg.bStarted || (FLASH->OPTSR_CUR & FLASH_OPTSR_IWDG1_SW) == 0;
#else
#error "wdt_stm32: IWDG_SW option bit not known"
#endif
}

// Wait for the register updates, false after the longest update: the LSI
// start, then up to 6 periods of the counter clock at the largest divider
static bool StIwdgWaitUpdate(uint32_t Lsi)
{
	uint32_t us = LSI_STARTUP_TIME + (uint32_t)(6ULL * StIwdgDiv(ST_IWDG_PR_MAX) * 1000000ULL / Lsi);

	for (uint32_t t = 0; ST_IWDG->SR & ST_IWDG_SR_UPDATE; t += ST_IWDG_POLL_US)
	{
		if (t >= us)
		{
			return false;
		}
		usDelay(ST_IWDG_POLL_US);
	}

	return true;
}

// IWDG stopped while a debugger halts the core
static void StIwdgHaltFreeze(bool bFreeze)
{
	volatile uint32_t *reg;
	uint32_t bit;

#if defined(DBGMCU_APB1_FZ_DBG_IWDG_STOP)
#ifdef RCC_APB2ENR_DBGMCUEN
	// DBGMCU on APB2, its clock on for the write
	RCC->APB2ENR |= RCC_APB2ENR_DBGMCUEN;
#endif
	reg = &DBGMCU->APB1FZ;
	bit = DBGMCU_APB1_FZ_DBG_IWDG_STOP;
#elif defined(DBGMCU_APB1FZR1_DBG_IWDG_STOP)
	reg = &DBGMCU->APB1FZR1;
	bit = DBGMCU_APB1FZR1_DBG_IWDG_STOP;
#elif defined(DBGMCU_APB4FZ1_DBG_IWDG1)
	reg = &DBGMCU->APB4FZ1;
	bit = DBGMCU_APB4FZ1_DBG_IWDG1;
#elif defined(DBGMCU_APB1LFZR_DBG_IWDG_STOP)
	reg = &DBGMCU->APB1LFZR;
	bit = DBGMCU_APB1LFZR_DBG_IWDG_STOP;
#else
#error "wdt_stm32: IWDG debug freeze bit not known"
#endif

	if (bFreeze)
	{
		*reg |= bit;
	}
	else
	{
		*reg &= ~bit;
	}
}

bool WdtInit(WdtDev_t * const pDev, const WdtCfg_t * const pCfg)
{
	if (pDev == nullptr || pCfg == nullptr || pCfg->DevNo != 0 || pCfg->msTimeout == 0 ||
		pCfg->bRunSleep == false)
	{
		return false;
	}

#ifndef IWDG_EWCR_EWIE
	// No interrupt on this IWDG
	if (pCfg->EvtHandler != nullptr)
	{
		return false;
	}
#endif

	int nbchan = pCfg->NbChan > 0 ? pCfg->NbChan : 1;

	if (nbchan > ST_IWDG_CHAN_MAX)
	{
		return false;
	}

	// Smallest divider that holds the timeout, ticks rounded up
	uint32_t lsi = StIwdgLsiFreq();
	uint32_t pr = 0;
	uint64_t ticks = 0;

	for (; pr <= ST_IWDG_PR_MAX; pr++)
	{
		uint64_t d = (uint64_t)StIwdgDiv(pr) * 1000U;

		ticks = ((uint64_t)pCfg->msTimeout * lsi + d - 1U) / d;
		if (ticks <= ST_IWDG_TICK_MAX)
		{
			break;
		}
	}
	if (pr > ST_IWDG_PR_MAX)
	{
		return false;
	}

	uint32_t rlr = (uint32_t)ticks - 1U;
	uint32_t winr = ST_IWDG_WINR_NONE;

	if (pCfg->msWindow != 0)
	{
#ifdef IWDG_WINR_WIN
		if (pCfg->msWindow >= pCfg->msTimeout)
		{
			return false;
		}

		// Reload allowed when the counter is at the window value or lower:
		// window ticks after the reload, rounded down
		uint64_t win = (uint64_t)pCfg->msWindow * lsi / ((uint64_t)StIwdgDiv(pr) * 1000U);

		winr = rlr - (uint32_t)win;
#else
		return false;
#endif
	}

	bool running = StIwdgRunning();

#ifdef IWDG_EWCR_EWIE
	// The counter must start above the interrupt position
	uint32_t rlrnow = running ? (ST_IWDG->RLR & IWDG_RLR_RL) : rlr;

	if (pCfg->EvtHandler != nullptr && rlrnow < ST_IWDG_EWIT)
	{
		return false;
	}
	NVIC_DisableIRQ(IWDG_IRQn);
#endif

	uint32_t state = DisableInterrupt();

	s_StIwdg.pDev = pDev;
	s_StIwdg.ChanMask = nbchan == ST_IWDG_CHAN_MAX ? 0xFFFFFFFFU : (1UL << nbchan) - 1U;
	s_StIwdg.ReloadMask = 0;
	EnableInterrupt(state);

	pDev->DevNo = 0;
	pDev->NbChan = nbchan;
	pDev->EvtHandler = pCfg->EvtHandler;
	pDev->pDevData = &s_StIwdg;

	StIwdgHaltFreeze(pCfg->bRunHalt == false);

#ifdef IWDG_EWCR_EWIE
	// EWIC clears a pending interrupt
	s_StIwdg.Ewcr = pCfg->EvtHandler != nullptr ? (IWDG_EWCR_EWIE | IWDG_EWCR_EWIC | ST_IWDG_EWIT) : IWDG_EWCR_EWIC;
#endif

	if (running)
	{
		// Started before: its configuration is kept, report it
		pDev->msTimeout = StIwdgMs(ST_IWDG->PR & IWDG_PR_PR, ST_IWDG->RLR & IWDG_RLR_RL, lsi);

#ifdef IWDG_EWCR_EWIE
		// The interrupt of this Init
		if (StIwdgWaitUpdate(lsi) == false)
		{
			return false;
		}
		ST_IWDG->KR = ST_IWDG_KEY_ACCESS;
		ST_IWDG->EWCR = s_StIwdg.Ewcr;
#endif
	}
	else
	{
		s_StIwdg.Pr = pr;
		s_StIwdg.Rlr = rlr;
		s_StIwdg.Winr = winr;
		pDev->msTimeout = StIwdgMs(pr, rlr, lsi);
	}

#ifdef IWDG_EWCR_EWIE
	NVIC_ClearPendingIRQ(IWDG_IRQn);
	if (pCfg->EvtHandler != nullptr)
	{
		NVIC_SetPriority(IWDG_IRQn, pCfg->IntPrio);
		NVIC_EnableIRQ(IWDG_IRQn);
	}
#endif

	return true;
}

bool WdtStart(WdtDev_t * const pDev)
{
	if (pDev == nullptr || pDev->pDevData != &s_StIwdg)
	{
		return false;
	}

	// Started before: its configuration is kept
	if (StIwdgRunning())
	{
		return true;
	}

	uint32_t lsi = StIwdgLsiFreq();

	// LSI forced on, the counter runs from here
	ST_IWDG->KR = ST_IWDG_KEY_START;
	s_StIwdg.bStarted = true;

	// The registers are written once the updates in progress are done
	if (StIwdgWaitUpdate(lsi) == false)
	{
		return false;
	}

	ST_IWDG->KR = ST_IWDG_KEY_ACCESS;
	ST_IWDG->PR = s_StIwdg.Pr;
	ST_IWDG->RLR = s_StIwdg.Rlr;
#ifdef IWDG_EWCR_EWIE
	ST_IWDG->EWCR = s_StIwdg.Ewcr;
#endif

	// A reload takes the reload value once its update is done
	if (StIwdgWaitUpdate(lsi) == false)
	{
		return false;
	}

	uint32_t state = DisableInterrupt();

	s_StIwdg.ReloadMask = 0;

	// The counter takes the new reload value: from a window register write,
	// or a reload
#ifdef IWDG_WINR_WIN
	if ((ST_IWDG->WINR & IWDG_WINR_WIN) != s_StIwdg.Winr)
	{
		ST_IWDG->WINR = s_StIwdg.Winr;
	}
	else
	{
		ST_IWDG->KR = ST_IWDG_KEY_RELOAD;
	}
#else
	ST_IWDG->KR = ST_IWDG_KEY_RELOAD;
#endif
	EnableInterrupt(state);

	return true;
}

void WdtReload(WdtDev_t * const pDev, int Chan)
{
	if (pDev == nullptr || pDev->pDevData != &s_StIwdg || Chan < 0 || Chan >= pDev->NbChan)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t mask = s_StIwdg.ReloadMask | (1UL << Chan);

	if ((mask & s_StIwdg.ChanMask) == s_StIwdg.ChanMask)
	{
		ST_IWDG->KR = ST_IWDG_KEY_RELOAD;
		mask = 0;
	}
	s_StIwdg.ReloadMask = mask;
	EnableInterrupt(state);
}

bool WdtRunning(WdtDev_t * const pDev)
{
	(void)pDev;

	return StIwdgRunning();
}

#ifdef IWDG_EWCR_EWIE
void IWDG_IRQHandler(void)
{
	if (ST_IWDG->SR & IWDG_SR_EWIF)
	{
		ST_IWDG->EWCR |= IWDG_EWCR_EWIC;

		WdtDev_t *dev = s_StIwdg.pDev;

		if (dev != nullptr && dev->EvtHandler != nullptr)
		{
			dev->EvtHandler(dev);
		}
	}
}
#endif
