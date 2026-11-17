/**-------------------------------------------------------------------------
	@file  system_sam4l.c

	@brief CMSIS system initialization for the SAM4L MCU family.

	Oscillators are selected by the application's strong g_McuOsc definition.
	The weak default uses internal oscillators; it does not describe a board.

	SAM4L datasheet 42023H: sections 6.2, 10.6, 12.6, 13.6, 14.5,
	20.5; tables 42-26, 42-27, 42-31, 42-32, 42-33; errata 45.1.

	@author Hoang Nguyen Hoan
	@date   June. 30, 2021
	@author Thinh Tran
	@date   Mar. 18, 2022

 @license

 MIT License

 Copyright (c) 2021 I-SYST inc. All rights reserved.

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
#include <stdbool.h>
#include <stdint.h>

#include "sam4lxxx.h"
#include "system_sam4l.h"
#include "coredev/system_core_clock.h"

#define SYSTEM_CORE_CLOCK               48000000UL
#define SAM4L_USDELAY_LOOP_CYCLES       13UL
#define SAM4L_NSDELAY_LOOP_CYCLES       3UL
#define RCSYS_FREQ                      115000UL
#define USB_FREQ                        48000000UL
#define DFLL_REF_FREQ                   62500UL
#define DFLL_CORE_MHZ                   80U
#define FLASH_ERRORS                    (FLASHCALW_FSR_LOCKE | FLASHCALW_FSR_PROGE)

// Poll budgets, not time in microseconds. Oscillator startup is performed
// while the CPU is on RCSYS, before selecting a high-frequency main clock.
#define STARTUP_POLL_LIMIT              1000000UL

// MCU oscillator settings, overridable by the application build. These are
// stabilization masks, not measurements of an attached crystal's startup.
#ifndef SAM4L_OSC0_STARTUP
#define SAM4L_OSC0_STARTUP               5U   // 8192 RCSYS cycles, about 71 ms
#endif
#ifndef SAM4L_OSC32_STARTUP
#define SAM4L_OSC32_STARTUP              5U   // 131072 RCSYS cycles, about 1.1 s
#endif
#ifndef SAM4L_OSC32_SELCURR
#define SAM4L_OSC32_SELCURR              10U  // Datasheet recommended 300 nA
#endif
#if SAM4L_OSC0_STARTUP > 15 || SAM4L_OSC32_STARTUP > 7 || SAM4L_OSC32_SELCURR > 15
#error Invalid SAM4L oscillator startup/current setting
#endif

// SCIF generic-clock source encodings, table 13-8.
#define RCSYS_GEN_CLK_SRC               0U
#define OSC32K_GEN_CLK_SRC              1U
#define DFLL0_GEN_CLK_SRC               2U
#define OSC0_GEN_CLK_SRC                3U
#define RC80M_GEN_CLK_SRC               4U
#define RCFAST_GEN_CLK_SRC              5U
#define RC1M_GEN_CLK_SRC                6U
#define RC32K_GEN_CLK_SRC               13U
#define OSC1K_GEN_CLK_SRC               15U
#define PLL0_GEN_CLK_SRC                16U

// OscDesc_t: { type, frequency Hz, accuracy ppm, load capacitance x10 pF }.
// A zero accuracy here means unspecified, not a zero-error RC oscillator.
// Applications needing external clocks or USB override this weak object.
__WEAK McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, 12000000UL, 0, 0 },
	{ OSC_TYPE_RC,    32768UL, 0, 0 },
	false
};

// Delay-loop iterations per microsecond, with 16 fractional bits.
extern uint32_t SystemMicroSecLoopCnt;

uint32_t SystemCoreClock = RCSYS_FREQ;
uint32_t SystemnsDelayFactor = 65536UL;
static uint32_t s_MainClock = RCSYS_FREQ;
static uint32_t s_PllFreq;
static uint32_t s_DfllFreq = DFLL_CORE_MHZ * 1000000UL;

// Debugger-visible failure reason. Startup never continues into the
// application with a missing requested clock or an unqualified Flash mode.
enum {
	SAM4L_STARTUP_OK = 0,
	SAM4L_STARTUP_CONFIG = 1,
	SAM4L_STARTUP_WDT = 2,
	SAM4L_STARTUP_FLASH = 3,
	SAM4L_STARTUP_RAMFUNC = 4,
	SAM4L_STARTUP_POWER = 5,
	SAM4L_STARTUP_OSC0 = 6,
	SAM4L_STARTUP_PLL = 7,
	SAM4L_STARTUP_GCLK = 8,
	SAM4L_STARTUP_DFLL = 9,
	SAM4L_STARTUP_RCFAST = 10,
	SAM4L_STARTUP_RC80M = 11,
	SAM4L_STARTUP_LFCLK = 12,
	SAM4L_STARTUP_CLOCK_SWITCH = 13,
	SAM4L_STARTUP_CACHE = 14,
	SAM4L_STARTUP_FREQM = 15
};
static volatile uint32_t s_StartupError;
// Last failed poll, and the power/Flash status samples retained before any
// clear-on-read bits are lost. These are diagnostics, not control state.
static volatile uintptr_t s_StartupReg;
static volatile uint32_t s_StartupValue;
static volatile uint32_t s_PowerControl;
static volatile uint32_t s_PowerStatus;
static volatile uint32_t s_FlashStatus;

// A clock-only application must not pull the timer implementation into its link.
extern bool Sam4lTimerClockBusy(uint32_t ClockMask) __attribute__((weak));

__attribute__((noreturn, noinline))
static void Sam4lStartupFailed(uint32_t Error)
{
	s_StartupError = Error;
	__disable_irq();
	for (;;) {
		__NOP();
	}
}

static bool Sam4lClockFailed(uint32_t Error)
{
	s_StartupError = Error;
	return false;
}

static bool Sam4lWaitBits(volatile const uint32_t *pReg,
						 uint32_t Mask, uint32_t Value)
{
	uint32_t timeout = STARTUP_POLL_LIMIT;
	uint32_t value;
	do {
		value = *pReg;
		if ((value & Mask) == Value) {
			return true;
		}
	} while (--timeout != 0U);
	s_StartupReg = (uintptr_t)pReg;
	s_StartupValue = value;
	return false;
}

// All values, including any read-modify-write, are evaluated BEFORE unlock.
// SystemInit holds PRIMASK throughout the protected register transactions.
static void Sam4lWritePm(volatile uint32_t *pReg, uint32_t Value)
{
	SAM4L_PM->PM_UNLOCK = PM_UNLOCK_KEY(0xAAU) |
		PM_UNLOCK_ADDR((uint32_t)pReg - (uint32_t)SAM4L_PM);
	*pReg = Value;
}

static void Sam4lWriteScif(volatile uint32_t *pReg, uint32_t Value)
{
	SAM4L_SCIF->SCIF_UNLOCK = SCIF_UNLOCK_KEY(0xAAU) |
		SCIF_UNLOCK_ADDR((uint32_t)pReg - (uint32_t)SAM4L_SCIF);
	*pReg = Value;
}

static void Sam4lWriteBscif(volatile uint32_t *pReg, uint32_t Value)
{
	SAM4L_BSCIF->BSCIF_UNLOCK = BSCIF_UNLOCK_KEY(0xAAU) |
		BSCIF_UNLOCK_ADDR((uint32_t)pReg - (uint32_t)SAM4L_BSCIF);
	*pReg = Value;
}

static void Sam4lWriteBpm(uint32_t Value)
{
	SAM4L_BPM->BPM_UNLOCK = BPM_UNLOCK_KEY(0xAAU) |
		BPM_UNLOCK_ADDR((uint32_t)&SAM4L_BPM->BPM_PMCON - (uint32_t)SAM4L_BPM);
	SAM4L_BPM->BPM_PMCON = Value;
}

// CPU and HSB share CPUSEL. Each PB bus has its own divider. There is no
// independent HSBSEL register on SAM4L. DIV=1 means 2^(SEL+1), not 2^SEL.
static inline uint32_t Sam4lApplyClkDiv(uint32_t Freq, uint32_t Sel)
{
	return (Sel & PM_CPUSEL_CPUDIV) != 0U ?
		Freq >> ((Sel & PM_CPUSEL_CPUSEL_Msk) + 1U) : Freq;
}

static uint32_t Sam4lDivider(uint32_t Freq, uint32_t Limit)
{
	uint32_t shift = 0;
	if (Freq == 0U || Limit == 0U) {
		return UINT32_MAX;
	}
	while (Freq > Limit && shift < 8U) {
		Freq = (Freq >> 1U) + (Freq & 1U); // Rounded up, without overflow.
		++shift;
	}
	if (Freq > Limit) {
		return UINT32_MAX;
	}
	return shift == 0U ? 0U : PM_CPUSEL_CPUDIV | PM_CPUSEL_CPUSEL(shift - 1U);
}

static bool Sam4lWriteDivider(volatile uint32_t *pReg, uint32_t Sel)
{
	if (!Sam4lWaitBits(&SAM4L_PM->PM_SR, PM_SR_CKRDY, PM_SR_CKRDY)) {
		return Sam4lClockFailed(SAM4L_STARTUP_CLOCK_SWITCH);
	}
	Sam4lWritePm(pReg, Sel);
	if (!Sam4lWaitBits(&SAM4L_PM->PM_SR, PM_SR_CKRDY, PM_SR_CKRDY) ||
		*pReg != Sel) {
		return Sam4lClockFailed(SAM4L_STARTUP_CLOCK_SWITCH);
	}
	return true;
}

// No peripheral/AHB transfers may be active. When slowing the
// CPU, slow the PB clocks first; when speeding it up, change CPUSEL first.
// The final PBB and CPU/HSB frequencies are equal, as required by AHB modules.
static bool Sam4lSetSyncClkDividers(uint32_t Sel)
{
	bool slowing = Sam4lApplyClkDiv(256U, Sel) <
		Sam4lApplyClkDiv(256U, SAM4L_PM->PM_CPUSEL);

	if (!slowing && !Sam4lWriteDivider(&SAM4L_PM->PM_CPUSEL, Sel))
	{
		return false;
	}
	if (!Sam4lWriteDivider(&SAM4L_PM->PM_PBASEL, Sel) ||
		!Sam4lWriteDivider(&SAM4L_PM->PM_PBCSEL, Sel) ||
		!Sam4lWriteDivider(&SAM4L_PM->PM_PBDSEL, Sel) ||
		!Sam4lWriteDivider(&SAM4L_PM->PM_PBBSEL, Sel))
	{
		return false;
	}
	return !slowing || Sam4lWriteDivider(&SAM4L_PM->PM_CPUSEL, Sel);
}

static bool Sam4lSelectMain(uint32_t Source)
{
	Sam4lWritePm(&SAM4L_PM->PM_MCCTRL, Source);
	__DSB();
	__ISB();
	if (SAM4L_PM->PM_MCCTRL != Source) {
		return Sam4lClockFailed(SAM4L_STARTUP_CLOCK_SWITCH);
	}
	return true;
}

// FRDY/HSMODE reflect the latest sample. Command error bits are accumulated
// because LOCKE and PROGE clear when FSR is read.
static uint32_t Sam4lFlashWait(uint32_t Timeout)
{
	uint32_t errors = 0;
	while (Timeout-- != 0U) {
		uint32_t status = SAM4L_HFLASHC->FLASHCALW_FSR;
		errors |= status & FLASH_ERRORS;
		if ((status & FLASHCALW_FSR_FRDY) != 0U) {
			s_FlashStatus = status | errors;
			return status | errors;
		}
	}
	s_FlashStatus = errors; // FRDY is deliberately clear on timeout.
	return errors;
}

bool FlashWaitReady(uint32_t Timeout)
{
	return (Sam4lFlashWait(Timeout) & FLASHCALW_FSR_FRDY) != 0U;
}

static bool Sam4lFlashCommand(uint32_t Cmd, int PgNo)
{
	// Drain an earlier command and clear its old errors. Only the new
	// command's status determines the result below.
	if (!FlashWaitReady(STARTUP_POLL_LIMIT)) {
		return false;
	}
	uint32_t page = PgNo < 0 ?
		SAM4L_HFLASHC->FLASHCALW_FCMD & FLASHCALW_FCMD_PAGEN_Msk :
		FLASHCALW_FCMD_PAGEN((uint32_t)PgNo);
	SAM4L_HFLASHC->FLASHCALW_FCMD = FLASHCALW_FCMD_KEY_KEY |
		page | (Cmd & FLASHCALW_FCMD_CMD_Msk);
	uint32_t status = Sam4lFlashWait(STARTUP_POLL_LIMIT);
	if ((status & (FLASHCALW_FSR_FRDY | FLASH_ERRORS)) != FLASHCALW_FSR_FRDY) {
		return false;
	}
	if (Cmd == FLASHCALW_FCMD_CMD_HSEN) {
		return (status & FLASHCALW_FSR_HSMODE) != 0U;
	}
	if (Cmd == FLASHCALW_FCMD_CMD_HSDIS) {
		return (status & FLASHCALW_FSR_HSMODE) == 0U;
	}
	return true;
}

// Preserve the legacy void API's best-effort behavior. Startup uses the
// checked helper, rather than making arbitrary application commands fatal.
// s_FlashStatus retains the last completion/error sample for the debugger.
void FlashSendCommand(uint32_t Cmd, int PgNo)
{
	(void)Sam4lFlashCommand(Cmd, PgNo);
}

static bool Sam4lFlashReadMode(uint32_t Cmd)
{
	uint32_t status = Sam4lFlashWait(STARTUP_POLL_LIMIT);
	uint32_t mode = Cmd == FLASHCALW_FCMD_CMD_HSEN ? FLASHCALW_FSR_HSMODE : 0U;
	if ((status & FLASHCALW_FSR_FRDY) == 0U ||
		((status & FLASHCALW_FSR_HSMODE) != mode && !Sam4lFlashCommand(Cmd, -1))) {
		return Sam4lClockFailed(SAM4L_STARTUP_FLASH);
	}
	return true;
}

static bool Sam4lFlashSettings(uint32_t Freq, uint32_t Pmcon,
							  uint32_t *pCmd, uint32_t *pFws)
{
	uint32_t ps = Pmcon & BPM_PMCON_PS_Msk;
	uint32_t zeroMax;
	uint32_t oneMax;
	if (ps == BPM_PMCON_PS(2)) {
		*pCmd = FLASHCALW_FCMD_CMD_HSEN;
		zeroMax = 24000000UL;
		oneMax = 48000000UL;
	} else if (ps == BPM_PMCON_PS(0) || ps == BPM_PMCON_PS(1)) {
		*pCmd = FLASHCALW_FCMD_CMD_HSDIS;
		if ((Pmcon & BPM_PMCON_FASTWKUP) != 0U) {
			zeroMax = 0;
			oneMax = 12000000UL;
		} else if (ps == BPM_PMCON_PS(0)) {
			zeroMax = 18000000UL;
			oneMax = 36000000UL;
		} else {
			zeroMax = 8000000UL;
			oneMax = 12000000UL;
		}
	} else {
		return false;
	}
	if (Freq == 0U || Freq > oneMax) {
		return false;
	}
	*pFws = Freq <= zeroMax ? 0U : FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	return true;
}

// Freq is an HSB frequency bound, not an oscillator frequency. Set timing
// BEFORE increasing the clock. SystemCoreClockUpdate is deliberately read-only.
static bool Sam4lFlashWaitState(uint32_t Freq)
{
	uint32_t cmd;
	uint32_t fws;
	uint32_t pmcon = SAM4L_BPM->BPM_PMCON;
	if ((pmcon & BPM_PMCON_PSCREQ) != 0U ||
		(SAM4L_BPM->BPM_SR & BPM_SR_PSOK) == 0U ||
		!Sam4lFlashSettings(Freq, pmcon, &cmd, &fws)) {
		return Sam4lClockFailed(SAM4L_STARTUP_FLASH);
	}
	uint32_t fcr = SAM4L_HFLASHC->FLASHCALW_FCR &
		~(FLASHCALW_FCR_FWS | FLASHCALW_FCR_WS1OPT);
	SAM4L_HFLASHC->FLASHCALW_FCR = fcr | FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	if (!Sam4lFlashReadMode(cmd))
	{
		return false;
	}
	SAM4L_HFLASHC->FLASHCALW_FCR = fcr | fws;
	if ((SAM4L_HFLASHC->FLASHCALW_FCR &
		 (FLASHCALW_FCR_FWS | FLASHCALW_FCR_WS1OPT)) != fws) {
		return Sam4lClockFailed(SAM4L_STARTUP_FLASH);
	}
	__DSB();
	__ISB();
	return true;
}

void SetFlashWaitState(uint32_t Freq)
{
	if (!Sam4lFlashWaitState(Freq))
	{
		Sam4lStartupFailed(s_StartupError);
	}
}

// gcc_arm_flash.ld places .fastrun in initialized SRAM. ResetEntry copies
// it before SystemInit. PSCM selects the no-halt mode used by Microchip's
// SAM4L BPM driver; the 42023H PDF describes only the halting alternative.
//
// The caller is on RCSYS, has set one Flash wait state/read mode, and has
// masked interrupts. No call or Flash data access is allowed from request
// through completion. A timeout stays in SRAM, never returning to Flash.
// There is no WFI, PM wake interrupt, NVIC reconfiguration or timer here.
#if defined(__GNUC__) && !defined(__clang__)
__attribute__((section(".fastrun"), noinline, noclone, long_call,
			   no_instrument_function, no_stack_protector))
#else
__attribute__((section(".fastrun"), noinline, no_instrument_function,
			   no_stack_protector))
#endif
static void Sam4lPowerScale(uint32_t Pmcon)
{
	uint32_t timeout = STARTUP_POLL_LIMIT;
	uint32_t target = Pmcon & BPM_PMCON_PS_Msk;
	uint32_t control;
	uint32_t status;
	SAM4L_BPM->BPM_UNLOCK = BPM_UNLOCK_KEY(0xAAU) |
		BPM_UNLOCK_ADDR((uint32_t)&SAM4L_BPM->BPM_PMCON - (uint32_t)SAM4L_BPM);
	SAM4L_BPM->BPM_PMCON = Pmcon;
	do {
		control = SAM4L_BPM->BPM_PMCON; // Also drains the request write.
		status = SAM4L_BPM->BPM_SR;
		if ((control & (BPM_PMCON_PS_Msk | BPM_PMCON_PSCREQ)) == target &&
			(status & BPM_SR_PSOK) != 0U) {
			__DSB();
			__ISB();
			return;
		}
	} while (--timeout != 0U);
	s_PowerControl = control;
	s_PowerStatus = status;
	s_StartupReg = (uintptr_t)&SAM4L_BPM->BPM_PMCON;
	s_StartupValue = control;
	s_StartupError = SAM4L_STARTUP_POWER;
	for (;;) {
		__NOP();
	}
}

// Called only on RCSYS. Returns the allowed synchronous-clock bound.
// Calibration-version-zero silicon retains the PS0 fallback instead of
// being rejected just because it cannot support PS2 (erratum 45.1.1).
static uint32_t Sam4lPreparePower(void)
{
	if (!Sam4lWaitBits(&SAM4L_BPM->BPM_PMCON, BPM_PMCON_PSCREQ, 0U) ||
		!Sam4lWaitBits(&SAM4L_BPM->BPM_SR, BPM_SR_PSOK, BPM_SR_PSOK)) {
		Sam4lStartupFailed(SAM4L_STARTUP_POWER);
	}
	bool ps2 = (*(volatile const uint32_t *)0x0080020CUL & 0xF0U) != 0U;
	uint32_t target = BPM_PMCON_PS(ps2 ? 2U : 0U);
	uint32_t limit = ps2 ? SYSTEM_CORE_CLOCK : 36000000UL;
	uint32_t pmcon = SAM4L_BPM->BPM_PMCON;
	SAM4L_HFLASHC->FLASHCALW_FCR |= FLASHCALW_FCR_FWS_1 | FLASHCALW_FCR_WS1OPT;
	if (!Sam4lFlashReadMode(ps2 ? FLASHCALW_FCMD_CMD_HSEN : FLASHCALW_FCMD_CMD_HSDIS))
	{
		Sam4lStartupFailed(SAM4L_STARTUP_FLASH);
	}
	if ((pmcon & BPM_PMCON_PS_Msk) != target) {
		// Both SAM4L SRAM sizes use the 0x20000000 region. Catch an absent
		// .fastrun placement before requesting a transition. Final ELF
		// inspection must also check the complete function and its literals.
		uintptr_t run = (uintptr_t)Sam4lPowerScale;
		if (run < 0x20000000UL || run >= 0x20010000UL) {
			Sam4lStartupFailed(SAM4L_STARTUP_RAMFUNC);
		}
		uint32_t request = (pmcon & ~(BPM_PMCON_PS_Msk | BPM_PMCON_FASTWKUP |
									 BPM_PMCON_PSCREQ)) |
			target | BPM_PMCON_PSCM | BPM_PMCON_PSCREQ;
		__DSB();
		Sam4lPowerScale(request);
	}
	// Restore the incoming change-mode/sleep fields, but keep the completed
	// target and normal analog wakeup. Never restore an old PSCREQ.
	pmcon = (pmcon & ~(BPM_PMCON_PS_Msk | BPM_PMCON_FASTWKUP |
					  BPM_PMCON_PSCREQ)) | target;
	Sam4lWriteBpm(pmcon);
	if (!Sam4lWaitBits(&SAM4L_BPM->BPM_PMCON,
					  BPM_PMCON_PS_Msk | BPM_PMCON_PSCREQ | BPM_PMCON_FASTWKUP,
					  target)) {
		Sam4lStartupFailed(SAM4L_STARTUP_POWER);
	}
	SetFlashWaitState(limit);
	return limit;
}

static void Sam4lDisableWatchdog(void)
{
	uint32_t ctrl = SAM4L_WDT->WDT_CTRL;
	if ((ctrl & WDT_CTRL_EN) != 0U) {
		ctrl &= ~(WDT_CTRL_EN | WDT_CTRL_KEY_Msk);
		SAM4L_WDT->WDT_CTRL = ctrl | WDT_CTRL_KEY(0x55U);
		SAM4L_WDT->WDT_CTRL = ctrl | WDT_CTRL_KEY(0xAAU);
		if (!Sam4lWaitBits(&SAM4L_WDT->WDT_CTRL, WDT_CTRL_EN, 0U)) {
			Sam4lStartupFailed(SAM4L_STARTUP_WDT);
		}
	}
}

static bool Sam4lGenericClock(unsigned Idx, uint32_t Config)
{
	volatile uint32_t *reg = &SAM4L_SCIF->SCIF_GCCTRL[Idx].SCIF_GCCTRL;
	// GCCTRL is not unlock-protected. Change only CEN until the old
	// source has supplied the falling edge that completes disabling.
	*reg = *reg & ~SCIF_GCCTRL_CEN;
	if (!Sam4lWaitBits(reg, SCIF_GCCTRL_CEN, 0U)) {
		return false;
	}
	*reg = Config;
	uint32_t mask = SCIF_GCCTRL_CEN | SCIF_GCCTRL_DIVEN |
		SCIF_GCCTRL_OSCSEL_Msk | SCIF_GCCTRL_DIV_Msk;
	return Sam4lWaitBits(reg, mask, Config & mask);
}

// Measure the DFLL against CLK32K before selecting it as the main clock.
// Open-loop RCFAST tolerance is multiplied by the DFLL; its nominal rate
// cannot be used to calculate a UART baud divisor. Accuracy follows CLK32K.
static bool Sam4lMeasureDfll(uint32_t *pRate)
{
	uint32_t mask = SAM4L_PM->PM_PBCMASK;
	uint32_t mode;
	uint32_t reference = g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC ?
		32768UL : g_McuOsc.LowPwrOsc.Freq;
	uint32_t ready = (SAM4L_BPM->BPM_PMCON & BPM_PMCON_CK32S) ?
		BSCIF_PCLKSR_RC32KRDY : BSCIF_PCLKSR_OSC32RDY;
	bool result;

	if (reference == 0U || !(SAM4L_BSCIF->BSCIF_PCLKSR & ready))
	{
		return Sam4lClockFailed(SAM4L_STARTUP_FREQM);
	}
	Sam4lWritePm(&SAM4L_PM->PM_PBCMASK, mask | PM_PBCMASK_FREQM);
	mode = SAM4L_FREQM->FREQM_MODE;
	if ((mode & FREQM_MODE_REFCEN) || SAM4L_FREQM->FREQM_STATUS ||
		SAM4L_FREQM->FREQM_IMR)
	{
		Sam4lWritePm(&SAM4L_PM->PM_PBCMASK, mask);
		return Sam4lClockFailed(SAM4L_STARTUP_FREQM);
	}
	SAM4L_FREQM->FREQM_MODE = FREQM_MODE_REFSEL(1U) |
		FREQM_MODE_CLKSEL(9U) | FREQM_MODE_REFNUM(128U);
	SAM4L_FREQM->FREQM_MODE |= FREQM_MODE_REFCEN;
	result = Sam4lWaitBits(&SAM4L_FREQM->FREQM_STATUS, FREQM_STATUS_RCLKBUSY, 0U);
	if (result)
	{
		SAM4L_FREQM->FREQM_ICR = FREQM_ICR_DONE;
		SAM4L_FREQM->FREQM_CTRL = FREQM_CTRL_START;
		result = Sam4lWaitBits(&SAM4L_FREQM->FREQM_ISR, FREQM_ISR_DONE, FREQM_ISR_DONE);
	}
	if (result)
	{
		*pRate = (uint64_t)(SAM4L_FREQM->FREQM_VALUE & FREQM_VALUE_VALUE_Msk) *
			reference / 128U;
		result = *pRate != 0U;
	}
	// A missing reference cannot be switched while a measurement is busy.
	// Leave it enabled on timeout so it can finish if the reference recovers.
	if (SAM4L_FREQM->FREQM_STATUS == 0U)
	{
		SAM4L_FREQM->FREQM_MODE &= ~FREQM_MODE_REFCEN;
		if (Sam4lWaitBits(&SAM4L_FREQM->FREQM_STATUS, FREQM_STATUS_RCLKBUSY, 0U))
		{
			SAM4L_FREQM->FREQM_ICR = FREQM_ICR_DONE | FREQM_ICR_RCLKRDY;
			SAM4L_FREQM->FREQM_MODE = mode;
			Sam4lWritePm(&SAM4L_PM->PM_PBCMASK, mask);
		}
		else
		{
			result = false;
		}
	}
	else
	{
		result = false;
	}
	if (!result)
	{
		return Sam4lClockFailed(SAM4L_STARTUP_FREQM);
	}
	return true;
}

static bool Sam4lOsc0Gain(uint32_t Freq, uint32_t *pGain)
{
	if (Freq < 600000UL || Freq > 16000000UL) {
		// 42023H lists GAIN=4 for >16 MHz but defines a two-bit GAIN
		// field. Reject that ambiguous crystal encoding instead of
		// truncating it to zero or writing the AGC test bit.
		return false;
	}
	*pGain = Freq <= 2000000UL ? 0U : Freq <= 4000000UL ? 1U :
		Freq <= 8000000UL ? 2U : 3U;
	return true;
}

static bool Sam4lEnableOsc0(void)
{
	uint32_t gain = 0;
	uint32_t config = 0;
	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL) {
		if (!Sam4lOsc0Gain(g_McuOsc.CoreOsc.Freq, &gain)) {
			return Sam4lClockFailed(SAM4L_STARTUP_CONFIG);
		}
		config = SCIF_OSCCTRL0_MODE | SCIF_OSCCTRL0_GAIN(gain) |
			SCIF_OSCCTRL0_STARTUP(SAM4L_OSC0_STARTUP);
	} else if (g_McuOsc.CoreOsc.Type != OSC_TYPE_TCXO ||
			   g_McuOsc.CoreOsc.Freq == 0U ||
			   g_McuOsc.CoreOsc.Freq > 50000000UL) {
		return Sam4lClockFailed(SAM4L_STARTUP_CONFIG);
	}
	Sam4lWriteScif(&SAM4L_SCIF->SCIF_OSCCTRL0, config | SCIF_OSCCTRL0_OSCEN);
	if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_PCLKSR,
					  SCIF_PCLKSR_OSC0RDY, SCIF_PCLKSR_OSC0RDY)) {
		return Sam4lClockFailed(SAM4L_STARTUP_OSC0);
	}
	return true;
}

// Keep the existing preferred 192/96 MHz plan. Both are legal VCO rates.
// 48 MHz is not a legal undivided VCO setting. Exact cross-products avoid
// false matches caused by truncated division. Input checks are conservative:
// OSC0 and the divided PLL reference must both be within 4..16 MHz.
static bool Sam4lPllConfig(uint32_t Freq, uint32_t *pConfig, uint32_t *pRate)
{
	static const uint32_t rates[] = { 192000000UL, 96000000UL };
	if (Freq < 4000000UL || Freq > 16000000UL) {
		return false;
	}
	for (unsigned r = 0; r < sizeof(rates) / sizeof(rates[0]); ++r) {
		for (uint32_t div = 1; div <= 15U; ++div) {
			if (Freq < 4000000UL * div || Freq > 16000000UL * div) {
				continue;
			}
			for (uint32_t mul = 15; mul >= 2U; --mul) {
				if (Freq * (mul + 1U) == rates[r] * div) {
					*pRate = rates[r];
					*pConfig = SCIF_PLL_PLLDIV(div) | SCIF_PLL_PLLMUL(mul) |
						SCIF_PLL_PLLOPT(rates[r] > 160000000UL ? 1U : 0U);
					return true;
				}
			}
		}
		// PLLDIV=0 is the documented input doubler, not divide-by-zero.
		// Try it only after ordinary divisors, retaining the 12 MHz plan.
		if (Freq <= 8000000UL) {
			for (uint32_t mul = 15; mul >= 2U; --mul) {
				if (Freq * 2U * (mul + 1U) == rates[r]) {
					*pRate = rates[r];
					*pConfig = SCIF_PLL_PLLMUL(mul) |
						SCIF_PLL_PLLOPT(rates[r] > 160000000UL ? 1U : 0U);
					return true;
				}
			}
		}
	}
	return false;
}

static bool Sam4lSetPll(void)
{
	uint32_t config;
	uint32_t rate;
	s_PllFreq = 0;
	if (!Sam4lPllConfig(g_McuOsc.CoreOsc.Freq, &config, &rate)) {
		return Sam4lClockFailed(SAM4L_STARTUP_PLL);
	}
	volatile uint32_t *reg = &SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL;
	// Called on RCSYS with GCLK7 stopped.
	Sam4lWriteScif(reg, *reg & ~SCIF_PLL_PLLEN);
	if (!Sam4lWaitBits(reg, SCIF_PLL_PLLEN, 0U)) {
		return Sam4lClockFailed(SAM4L_STARTUP_PLL);
	}
	Sam4lWriteScif(reg, config);
	// Erratum 45.1.2: PLLCOUNT must remain zero, including on wake-up.
	Sam4lWriteScif(reg, config | SCIF_PLL_PLLEN);
	if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_PCLKSR,
					  SCIF_PCLKSR_PLL0LOCK, SCIF_PCLKSR_PLL0LOCK) ||
		*reg != (config | SCIF_PLL_PLLEN)) {
		return Sam4lClockFailed(SAM4L_STARTUP_PLL);
	}
	s_PllFreq = rate;
	return true;
}

void SystemSetPLL(void)
{
	if (!Sam4lSetPll())
	{
		Sam4lStartupFailed(s_StartupError);
	}
}

static bool Sam4lDfllWrite(volatile uint32_t *pReg, uint32_t Value)
{
	if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_PCLKSR,
					  SCIF_PCLKSR_DFLL0RDY, SCIF_PCLKSR_DFLL0RDY)) {
		return Sam4lClockFailed(SAM4L_STARTUP_DFLL);
	}
	Sam4lWriteScif(pReg, Value);
	return true;
}

// Output in nominal MHz, using GCLK0 at nominal 62.5 kHz. The old 1 MHz
// reference was outside table 42-27's 8..150 kHz operating range.
static bool Sam4lConfigDfll(uint16_t MHz)
{
	if (MHz < 20U || MHz > 150U) {
		return Sam4lClockFailed(SAM4L_STARTUP_DFLL);
	}
	SAM4L_SCIF->SCIF_DFLL0SYNC = SCIF_DFLL0SYNC_SYNC;
	if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_PCLKSR,
					  SCIF_PCLKSR_DFLL0RDY, SCIF_PCLKSR_DFLL0RDY)) {
		return Sam4lClockFailed(SAM4L_STARTUP_DFLL);
	}
	uint32_t conf = SAM4L_SCIF->SCIF_DFLL0CONF;
	if ((conf & SCIF_DFLL0CONF_EN) == 0U) {
		// Enable alone before configuring the closed loop.
		conf |= SCIF_DFLL0CONF_EN;
		if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0CONF, conf))
		{
			return false;
		}
	}
	uint32_t range = MHz < 30U ? 3U : MHz < 55U ? 2U : MHz < 110U ? 1U : 0U;
	conf &= ~(SCIF_DFLL0CONF_MODE | SCIF_DFLL0CONF_RANGE_Msk |
			  SCIF_DFLL0CONF_STABLE | SCIF_DFLL0CONF_LLAW |
			  SCIF_DFLL0CONF_CCDIS | SCIF_DFLL0CONF_QLDIS);
	conf |= SCIF_DFLL0CONF_RANGE(range);
	if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0CONF, conf))
	{
		return false;
	}
	if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0MUL,
				  SCIF_DFLL0MUL_MUL((uint32_t)MHz * (1000000UL / DFLL_REF_FREQ))))
	{
		return false;
	}
	if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0STEP,
				  SCIF_DFLL0STEP_CSTEP(4U) | SCIF_DFLL0STEP_FSTEP(4U)))
	{
		return false;
	}
	if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0VAL,
				  SCIF_DFLL0VAL_COARSE(16U) | SCIF_DFLL0VAL_FINE(128U)))
	{
		return false;
	}
	if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0SSG, 0U))
	{
		return false;
	}
	if (!Sam4lDfllWrite(&SAM4L_SCIF->SCIF_DFLL0CONF, conf | SCIF_DFLL0CONF_MODE))
	{
		return false;
	}
	uint32_t ready = SCIF_PCLKSR_DFLL0RDY | SCIF_PCLKSR_DFLL0LOCKC |
		SCIF_PCLKSR_DFLL0LOCKF;
	if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_PCLKSR, ready, ready)) {
		return Sam4lClockFailed(SAM4L_STARTUP_DFLL);
	}
	s_DfllFreq = (uint32_t)MHz * 1000000UL;
	return true;
}

void ConfigDFLL0Freq(uint16_t MHz)
{
	if (!Sam4lConfigDfll(MHz))
	{
		Sam4lStartupFailed(s_StartupError);
	}
}

static uint32_t Sam4lRcFastFreq(void)
{
	uint32_t range = (SAM4L_SCIF->SCIF_RCFASTCFG & SCIF_RCFASTCFG_FRANGE_Msk) >>
		SCIF_RCFASTCFG_FRANGE_Pos;
	return range <= 2U ? (range + 1U) * 4000000UL : 0U;
}

static bool Sam4lEnableRcFast(uint32_t Freq)
{
	uint32_t config = SAM4L_SCIF->SCIF_RCFASTCFG;
	if ((config & SCIF_RCFASTCFG_EN) != 0U) {
		Sam4lWriteScif(&SAM4L_SCIF->SCIF_RCFASTCFG, config & ~SCIF_RCFASTCFG_EN);
		if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_RCFASTCFG, SCIF_RCFASTCFG_EN, 0U)) {
			return Sam4lClockFailed(SAM4L_STARTUP_RCFAST);
		}
	}
	// Preserve factory CALIB/FCD. FRANGE and CALIB may change only while
	// disabled. Do not OR a new range into an old range field.
	config &= ~(SCIF_RCFASTCFG_EN | SCIF_RCFASTCFG_TUNEEN | SCIF_RCFASTCFG_FRANGE_Msk);
	config |= SCIF_RCFASTCFG_FRANGE(Freq / 4000000UL - 1U);
	Sam4lWriteScif(&SAM4L_SCIF->SCIF_RCFASTCFG, config);
	Sam4lWriteScif(&SAM4L_SCIF->SCIF_RCFASTCFG, config | SCIF_RCFASTCFG_EN);
	if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_RCFASTCFG, SCIF_RCFASTCFG_EN,
					  SCIF_RCFASTCFG_EN)) {
		return Sam4lClockFailed(SAM4L_STARTUP_RCFAST);
	}
	return true;
}

static bool Sam4lLowClockInit(const OscDesc_t *pOsc)
{
	uint32_t pmcon = SAM4L_BPM->BPM_PMCON;
	if (pOsc->Type == OSC_TYPE_RC) {
		uint32_t control = SAM4L_BSCIF->BSCIF_RC32KCR;
		control &= ~(BSCIF_RC32KCR_MODE | BSCIF_RC32KCR_REF);
		control |= BSCIF_RC32KCR_TCEN | BSCIF_RC32KCR_EN32K | BSCIF_RC32KCR_EN;
		Sam4lWriteBscif(&SAM4L_BSCIF->BSCIF_RC32KCR, control);
		if (!Sam4lWaitBits(&SAM4L_BSCIF->BSCIF_PCLKSR,
						  BSCIF_PCLKSR_RC32KRDY, BSCIF_PCLKSR_RC32KRDY)) {
			return false;
		}
		pmcon |= BPM_PMCON_CK32S;
	} else {
		uint32_t mode = pOsc->Type == OSC_TYPE_XTAL ?
			BSCIF_OSCCTRL32_MODE(1U) : 0U;
		uint32_t control = SAM4L_BSCIF->BSCIF_OSCCTRL32;
		// OSC32 survives non-POR resets. Reuse a running matching mode.
		// Enabling its 32 kHz output does not change OSC32EN.
		if ((control & BSCIF_OSCCTRL32_OSC32EN) != 0U &&
			(control & BSCIF_OSCCTRL32_MODE_Msk) == mode) {
			Sam4lWriteBscif(&SAM4L_BSCIF->BSCIF_OSCCTRL32,
						   control | BSCIF_OSCCTRL32_EN32K);
		} else {
			if ((control & BSCIF_OSCCTRL32_OSC32EN) != 0U) {
				control &= ~BSCIF_OSCCTRL32_OSC32EN;
				Sam4lWriteBscif(&SAM4L_BSCIF->BSCIF_OSCCTRL32, control);
				if (!Sam4lWaitBits(&SAM4L_BSCIF->BSCIF_OSCCTRL32,
								  BSCIF_OSCCTRL32_OSC32EN, 0U)) {
					return false;
				}
			}
			control &= ~(BSCIF_OSCCTRL32_MODE_Msk | BSCIF_OSCCTRL32_STARTUP_Msk |
						 BSCIF_OSCCTRL32_SELCURR_Msk | BSCIF_OSCCTRL32_OSC32EN);
			control |= mode | BSCIF_OSCCTRL32_EN32K |
				(pOsc->Type == OSC_TYPE_XTAL ?
				 BSCIF_OSCCTRL32_STARTUP(SAM4L_OSC32_STARTUP) |
				 BSCIF_OSCCTRL32_SELCURR(SAM4L_OSC32_SELCURR) : 0U);
			Sam4lWriteBscif(&SAM4L_BSCIF->BSCIF_OSCCTRL32, control);
			// Separate enable: every other bit remains unchanged.
			Sam4lWriteBscif(&SAM4L_BSCIF->BSCIF_OSCCTRL32,
						   control | BSCIF_OSCCTRL32_OSC32EN);
		}
		if (!Sam4lWaitBits(&SAM4L_BSCIF->BSCIF_PCLKSR,
						  BSCIF_PCLKSR_OSC32RDY, BSCIF_PCLKSR_OSC32RDY)) {
			return false;
		}
		pmcon &= ~BPM_PMCON_CK32S;
	}
	Sam4lWriteBpm(pmcon);
	if ((SAM4L_BPM->BPM_PMCON & BPM_PMCON_CK32S) != (pmcon & BPM_PMCON_CK32S)) {
		return false;
	}
	return true;
}

static bool Sam4lLowClockValid(OSC_TYPE Type, uint32_t Freq)
{
	if (Type == OSC_TYPE_RC) {
		return Freq == 32000UL || Freq == 32768UL; // RC32K nominal aliases.
	}
	if (Type == OSC_TYPE_XTAL) {
		return Freq == 32768UL;
	}
	return Type == OSC_TYPE_TCXO && Freq != 0U && Freq <= 6000000UL;
}

static bool Sam4lConfigurationValid(const McuOsc_t *pOsc)
{
	OSC_TYPE core = pOsc->CoreOsc.Type;
	OSC_TYPE low = pOsc->LowPwrOsc.Type;
	uint32_t freq = pOsc->CoreOsc.Freq;
	uint32_t lf = pOsc->LowPwrOsc.Freq;
	uint32_t unused;
	if (core == OSC_TYPE_RC) {
		if (pOsc->bUSBClk || (freq != 1000000UL && freq != 4000000UL &&
			freq != 8000000UL && freq != 12000000UL && freq != 80000000UL)) {
			return false;
		}
	} else if (core == OSC_TYPE_XTAL) {
		if (!Sam4lOsc0Gain(freq, &unused)) {
			return false;
		}
	} else if (core != OSC_TYPE_TCXO || freq == 0U || freq > 50000000UL) {
		return false;
	}
	if (pOsc->bUSBClk) {
		uint32_t config, rate;
		if (!Sam4lPllConfig(freq, &config, &rate)) {
			return false;
		}
	}
	return Sam4lLowClockValid(low, lf);
}

static bool Sam4lClockInit(const McuOsc_t *pOsc, bool Startup)
{
	s_StartupError = SAM4L_STARTUP_OK;
	s_StartupReg = 0U;
	s_StartupValue = 0U;
	s_PowerControl = 0U;
	s_PowerStatus = 0U;
	if (!Sam4lConfigurationValid(pOsc)) {
		return Sam4lClockFailed(SAM4L_STARTUP_CONFIG);
	}
	// SystemInit is entered after reset, before peripheral initialization.
	// Return to the always-available clock before touching clock sources.
	if (!Sam4lSelectMain(PM_MCCTRL_MCSEL(0U)))
	{
		return false;
	}
	g_McuOsc.CoreOsc = pOsc->CoreOsc;
	s_MainClock = RCSYS_FREQ;
	if (!Sam4lSetSyncClkDividers(0U))
	{
		return false;
	}
	SystemCoreClock = RCSYS_FREQ;
	SystemMicroSecLoopCnt = ((uint64_t)SystemCoreClock << 16) / (SAM4L_USDELAY_LOOP_CYCLES * 1000000UL);
	SystemnsDelayFactor = ((uint64_t)SystemCoreClock << 16) / (SAM4L_NSDELAY_LOOP_CYCLES * 1000000UL);

	if (Startup)
	{
		uint32_t mask = SAM4L_PM->PM_PBBMASK | PM_PBBMASK_HCACHE;
		Sam4lWritePm(&SAM4L_PM->PM_PBBMASK, mask);
		if ((SAM4L_PM->PM_PBBMASK & PM_PBBMASK_HCACHE) == 0U) {
			return Sam4lClockFailed(SAM4L_STARTUP_CACHE);
		}
		if ((SAM4L_HCACHE->HCACHE_SR & HCACHE_SR_CSTS_EN) == 0U) {
			SAM4L_HCACHE->HCACHE_CTRL = HCACHE_CTRL_CEN_YES;
			if (!Sam4lWaitBits(&SAM4L_HCACHE->HCACHE_SR,
							  HCACHE_SR_CSTS_EN, HCACHE_SR_CSTS_EN)) {
				return Sam4lClockFailed(SAM4L_STARTUP_CACHE);
			}
		}
	}
	// Stop owned generated clocks before changing a PLL/reference. All
	// clients must be stopped on explicit re-entry, just as on reset.
	if (!Sam4lGenericClock(7U, 0U) || !Sam4lGenericClock(0U, 0U)) {
		return Sam4lClockFailed(SAM4L_STARTUP_GCLK);
	}
	uint32_t coreLimit;
	if (Startup)
	{
		coreLimit = Sam4lPreparePower();
		if (!Sam4lLowClockInit(&g_McuOsc.LowPwrOsc))
		{
			return Sam4lClockFailed(SAM4L_STARTUP_LFCLK);
		}
	}
	else
	{
		// Runtime keeps the established voltage scale and low-frequency clock.
		uint32_t pmcon = SAM4L_BPM->BPM_PMCON;
		uint32_t ps = pmcon & BPM_PMCON_PS_Msk;
		if ((pmcon & BPM_PMCON_PSCREQ) != 0U ||
			(SAM4L_BPM->BPM_SR & BPM_SR_PSOK) == 0U || ps == BPM_PMCON_PS(3))
		{
			return Sam4lClockFailed(SAM4L_STARTUP_POWER);
		}
		coreLimit = ps == BPM_PMCON_PS(2) ? 48000000UL :
			ps == BPM_PMCON_PS(0) && !(pmcon & BPM_PMCON_FASTWKUP) ?
			36000000UL : 12000000UL;
	}
	s_PllFreq = 0U;

	uint32_t source;
	uint32_t mainRate;
	uint32_t mainMax;
	if (g_McuOsc.CoreOsc.Type != OSC_TYPE_RC) {
		if (!Sam4lEnableOsc0())
		{
			return false;
		}
		uint32_t config;
		uint32_t rate;
		if (Sam4lPllConfig(g_McuOsc.CoreOsc.Freq, &config, &rate)) {
			if (!Sam4lSetPll())
			{
				return false;
			}
			mainRate = s_PllFreq;
			source = PM_MCCTRL_MCSEL_PLL0;
		} else {
			// A valid external oscillator need not have an exact PLL ratio.
			// It can supply the CPU directly, but not this PLL-based USB plan.
			if (g_McuOsc.bUSBClk) {
				return Sam4lClockFailed(SAM4L_STARTUP_PLL);
			}
			mainRate = g_McuOsc.CoreOsc.Freq;
			source = PM_MCCTRL_MCSEL(1U);
		}
		mainMax = mainRate;
	} else if (g_McuOsc.CoreOsc.Freq == 80000000UL) {
		// No need to disable an already-enabled, factory-calibrated RC80M.
		Sam4lWriteScif(&SAM4L_SCIF->SCIF_RC80MCR,
					  SAM4L_SCIF->SCIF_RC80MCR | SCIF_RC80MCR_EN);
		if (!Sam4lWaitBits(&SAM4L_SCIF->SCIF_RC80MCR,
						  SCIF_RC80MCR_EN, SCIF_RC80MCR_EN)) {
			return Sam4lClockFailed(SAM4L_STARTUP_RC80M);
		}
		mainRate = 80000000UL;
		mainMax = 100000000UL; // Table 42-32: /2 could reach 50 MHz.
		source = PM_MCCTRL_MCSEL_RC80M;
	} else {
		uint32_t refSource;
		uint32_t refMax;
		uint32_t freq = g_McuOsc.CoreOsc.Freq;
		if (freq == 1000000UL) {
			Sam4lWriteBscif(&SAM4L_BSCIF->BSCIF_RC1MCR,
						   SAM4L_BSCIF->BSCIF_RC1MCR | BSCIF_RC1MCR_CLKOE);
			if (!Sam4lWaitBits(&SAM4L_BSCIF->BSCIF_RC1MCR,
							  BSCIF_RC1MCR_CLKOE, BSCIF_RC1MCR_CLKOE)) {
				return Sam4lClockFailed(SAM4L_STARTUP_DFLL);
			}
			refSource = RC1M_GEN_CLK_SRC;
			refMax = 1120000UL;
		} else {
			if (!Sam4lEnableRcFast(freq))
			{
				return false;
			}
			refSource = RCFAST_GEN_CLK_SRC;
			refMax = freq == 4000000UL ? 4600000UL :
				freq == 8000000UL ? 8500000UL : 12300000UL;
		}
		uint32_t refDiv = freq / DFLL_REF_FREQ;
		if (!Sam4lGenericClock(0U, SCIF_GCCTRL_OSCSEL(refSource) |
							  SCIF_GCCTRL_DIVEN | SCIF_GCCTRL_DIV(refDiv / 2U - 1U) | SCIF_GCCTRL_CEN)) {
			return Sam4lClockFailed(SAM4L_STARTUP_GCLK);
		}
		// 80 MHz nominal allows the RC reference tolerance and up to 1%
		// DFLL tracking error without taking CPU/HSB over 48 MHz at /2.
		// These RC bounds are characterized values, not precision clocks.
		if (!Sam4lConfigDfll(DFLL_CORE_MHZ))
		{
			return false;
		}
		if (!Sam4lMeasureDfll(&s_DfllFreq))
		{
			return false;
		}
		mainRate = s_DfllFreq;
		// Accepted RC nominal rates are integral MHz. These products fit
		// uint32_t; round upwards before adding the 1% tracking allowance.
		uint32_t refMHz = freq / 1000000UL;
		mainMax = (refMax * DFLL_CORE_MHZ + refMHz - 1U) / refMHz;
		mainMax += (mainMax + 99U) / 100U;
		source = PM_MCCTRL_MCSEL_DFLL0;
	}

	// Keep the characterized bound, and also cover an outlying measurement.
	if (mainRate > mainMax)
	{
		mainMax = mainRate;
	}
	uint32_t sel = Sam4lDivider(mainMax, coreLimit);
	if (sel == UINT32_MAX)
	{
		return Sam4lClockFailed(SAM4L_STARTUP_CLOCK_SWITCH);
	}
	if (!Sam4lFlashWaitState(Sam4lApplyClkDiv(mainMax - 1U, sel) + 1U) ||
		!Sam4lSetSyncClkDividers(sel) || !Sam4lSelectMain(source))
	{
		return false;
	}
	s_MainClock = mainRate;
	SystemCoreClock = Sam4lApplyClkDiv(mainRate, sel);
	SystemMicroSecLoopCnt = ((uint64_t)SystemCoreClock << 16) / (SAM4L_USDELAY_LOOP_CYCLES * 1000000UL);
	SystemnsDelayFactor = ((uint64_t)SystemCoreClock << 16) / (SAM4L_NSDELAY_LOOP_CYCLES * 1000000UL);

	if (g_McuOsc.bUSBClk) {
		// Only the external OSC0/PLL path reaches here. The application must
		// provide an oscillator meeting USB accuracy. GCLK7 is independent of CPUSEL: 192 MHz /4 or 96 MHz /2.
		uint32_t div = s_PllFreq / USB_FREQ;
		if (s_PllFreq == 0U || s_PllFreq % USB_FREQ != 0U ||
			(div != 1U && ((div & 1U) != 0U || div > 512U))) {
			return Sam4lClockFailed(SAM4L_STARTUP_GCLK);
		}
		uint32_t config = SCIF_GCCTRL_OSCSEL(PLL0_GEN_CLK_SRC);
		if (div != 1U) {
			config |= SCIF_GCCTRL_DIVEN | SCIF_GCCTRL_DIV(div / 2U - 1U);
		}
		if (!Sam4lGenericClock(7U, config | SCIF_GCCTRL_CEN)) {
			return Sam4lClockFailed(SAM4L_STARTUP_GCLK);
		}
		Sam4lWritePm(&SAM4L_PM->PM_HSBMASK,
					 SAM4L_PM->PM_HSBMASK | PM_HSBMASK_USBC);
		Sam4lWritePm(&SAM4L_PM->PM_PBBMASK,
					 SAM4L_PM->PM_PBBMASK | PM_PBBMASK_USBC);
		if ((SAM4L_PM->PM_HSBMASK & PM_HSBMASK_USBC) == 0U ||
			(SAM4L_PM->PM_PBBMASK & PM_PBBMASK_USBC) == 0U) {
			return Sam4lClockFailed(SAM4L_STARTUP_GCLK);
		}
	}
	return true;
}

// The watchdog stays disabled at startup. Applications enable it explicitly.
void SystemInit(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();
	Sam4lDisableWatchdog();
	if (!Sam4lClockInit(&g_McuOsc, true))
	{
		Sam4lStartupFailed(s_StartupError);
	}
	__set_PRIMASK(primask);
}

static uint32_t Sam4lMainClockGet(void)
{
	switch (SAM4L_PM->PM_MCCTRL & PM_MCCTRL_MCSEL_Msk) {
	case PM_MCCTRL_MCSEL(0U): return RCSYS_FREQ;
	case PM_MCCTRL_MCSEL(1U): return g_McuOsc.CoreOsc.Freq;
	case PM_MCCTRL_MCSEL_PLL0: {
		uint32_t pll = SAM4L_SCIF->SCIF_PLL[0].SCIF_PLL;
		uint32_t div = (pll & SCIF_PLL_PLLDIV_Msk) >> SCIF_PLL_PLLDIV_Pos;
		uint32_t mul = ((pll & SCIF_PLL_PLLMUL_Msk) >> SCIF_PLL_PLLMUL_Pos) + 1U;
		// OSC0 is at most 50 MHz, MUL at most 16: even the input
		// doubler fits in uint32_t; no 64-bit division helper is needed.
		uint32_t freq = g_McuOsc.CoreOsc.Freq * mul;
		// SystemInit configures OSC0 as PLL reference. Do not invent a rate
		// for an application-selected GCLK9 reference.
		if ((pll & SCIF_PLL_PLLOSC_Msk) != 0U) {
			return 0U;
		}
		freq = div != 0U ? freq / div : freq * 2U;
		if ((pll & SCIF_PLL_PLLOPT(2U)) != 0U) {
			freq /= 2U;
		}
		return (uint32_t)freq;
	}
	case PM_MCCTRL_MCSEL_DFLL0: return s_DfllFreq;
	case PM_MCCTRL_MCSEL_RC80M: return 80000000UL;
	case PM_MCCTRL_MCSEL(5U): return Sam4lRcFastFreq();
	case PM_MCCTRL_MCSEL(6U): return 1000000UL;
	default: return 0U;
	}
}

void SystemCoreClockUpdate(void)
{
	// In particular, detect hardware clock-failure fallback to RCSYS.
	// Reporting a clock must not change Flash timing underneath the CPU.
	s_MainClock = Sam4lMainClockGet();
	SystemCoreClock = Sam4lApplyClkDiv(s_MainClock, SAM4L_PM->PM_CPUSEL);
	SystemMicroSecLoopCnt = ((uint64_t)SystemCoreClock << 16) / (SAM4L_USDELAY_LOOP_CYCLES * 1000000UL);
	SystemnsDelayFactor = ((uint64_t)SystemCoreClock << 16) / (SAM4L_NSDELAY_LOOP_CYCLES * 1000000UL);
}

uint32_t SystemCoreClockGet(void)
{
	SystemCoreClockUpdate();
	return SystemCoreClock;
}

uint32_t SystemPeriphClockGet(int Idx)
{
	uint32_t mainClock = Sam4lMainClockGet();
	switch (Idx) {
	case 0: return Sam4lApplyClkDiv(mainClock, SAM4L_PM->PM_PBASEL);
	case 1: return Sam4lApplyClkDiv(mainClock, SAM4L_PM->PM_PBBSEL);
	case 2: return Sam4lApplyClkDiv(mainClock, SAM4L_PM->PM_PBCSEL);
	case 3: return Sam4lApplyClkDiv(mainClock, SAM4L_PM->PM_PBDSEL);
	default: return 0U;
	}
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	// Independent runtime bus retiming is not implemented. A no-op succeeds;
	// an unsupported request must not be reported as a successful change.
	uint32_t current = SystemPeriphClockGet(Idx);
	return current != 0U && current == Freq ? current : 0U;
}

// Check the existing transport clock gates without reading interrupt status.
// A disabled transport releases its PBA clock through its driver.
static bool Sam4lCoreClockBusy(void)
{
	uint32_t pba = SAM4L_PM->PM_PBAMASK;
	uint32_t tc = pba & (PM_PBAMASK_TC0 | PM_PBAMASK_TC1);

	if ((SAM4L_PM->PM_PBCMASK & PM_PBCMASK_FREQM) &&
		(SAM4L_FREQM->FREQM_STATUS || SAM4L_FREQM->FREQM_IMR ||
		(SAM4L_FREQM->FREQM_MODE & FREQM_MODE_REFCEN)))
	{
		return true;
	}

	// The remaining PBA users include transports, analog, audio and LCD.
	// Refuse their enabled clock gates; do not consume peripheral status.
	if (pba & (PM_PBAMASK_MASK & ~(PM_PBAMASK_TC0 | PM_PBAMASK_TC1)))
	{
		return true;
	}
	if (tc && (!Sam4lTimerClockBusy || Sam4lTimerClockBusy(tc)))
	{
		return true;
	}
	if ((SAM4L_PM->PM_PBDMASK & PM_PBDMASK_AST) &&
		((SAM4L_AST->AST_SR & (AST_SR_BUSY | AST_SR_CLKBUSY)) ||
		(SAM4L_AST->AST_CR & AST_CR_EN)))
	{
		// Keep AST synchronization timing unchanged while it is running.
		return true;
	}
	if ((SAM4L_PM->PM_PBBMASK & PM_PBBMASK_USBC) &&
		(SAM4L_USBC->USBC_USBCON & USBC_USBCON_USBE))
	{
		// Suspended/frozen USB still owns its clock and resume state.
		return true;
	}
	if (((SAM4L_PM->PM_HSBMASK & PM_HSBMASK_PDCA) &&
		!(SAM4L_PM->PM_PBBMASK & PM_PBBMASK_PDCA)) ||
		((SAM4L_PM->PM_HSBMASK & PM_HSBMASK_USBC) &&
		!(SAM4L_PM->PM_PBBMASK & PM_PBBMASK_USBC)))
	{
		// Cannot inspect an active bus master with its register clock gated.
		return true;
	}
	if (SAM4L_PM->PM_PBBMASK & PM_PBBMASK_PDCA)
	{
		for (unsigned i = 0; i < 16U; i++)
		{
			if (SAM4L_PDCA->PDCA_CHANNEL[i].PDCA_SR & PDCA_SR_TEN)
			{
				return true;
			}
		}
	}
	for (unsigned i = 1; i < 12U; i++)
	{
		uint32_t clock = SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL;
		// GCLK0 is the system DFLL reference. GCLK7 is owned here only
		// when the application requested the system USB clock setup.
		if ((clock & SCIF_GCCTRL_CEN) && !(i == 7U && g_McuOsc.bUSBClk))
		{
			return true;
		}
	}
	return false;
}

// Explicit full clock-tree reinitialization, preserving the existing API.
// The caller MUST stop all clock consumers first (USB, DMA, timers and other
// peripherals) and retime/reinitialize them afterwards. IRQ masking alone
// does not quiesce those engines. Do not call from an interrupt handler.
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t Freq)
{
	McuOsc_t previous = g_McuOsc;
	McuOsc_t requested = previous;
	requested.CoreOsc.Type = ClkSrc;
	requested.CoreOsc.Freq = Freq;
	if (__get_IPSR() != 0U || !Sam4lConfigurationValid(&requested)) {
		return false;
	}
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (Sam4lCoreClockBusy())
	{
		__set_PRIMASK(primask);
		return false;
	}
	bool result = Sam4lClockInit(&requested, false);
	if (!result)
	{
		// Keep the previous descriptor unless the requested source was selected.
		// RCSYS needs neither oscillator descriptor nor PLL lock to run.
		if (Sam4lSelectMain(PM_MCCTRL_MCSEL(0U)))
		{
			g_McuOsc.CoreOsc = previous.CoreOsc;
		}
	}
	SystemCoreClockUpdate();
	__set_PRIMASK(primask);
	return result;
}

static bool Sam4lLowClockBusy(void)
{
	uint32_t required = PM_PBDMASK_BPM | PM_PBDMASK_BSCIF | PM_PBDMASK_AST |
		PM_PBDMASK_WDT | PM_PBDMASK_PICOUART;

	// Gating register access does not stop a backup-domain clock user.
	// Refuse when its state cannot be inspected without changing clock gates.
	if ((SAM4L_PM->PM_PBDMASK & required) != required ||
		(SAM4L_PM->PM_PBCMASK & (PM_PBCMASK_FREQM | PM_PBCMASK_SCIF)) !=
		(PM_PBCMASK_FREQM | PM_PBCMASK_SCIF))
	{
		return true;
	}
	if ((SAM4L_AST->AST_SR & (AST_SR_BUSY | AST_SR_CLKBUSY)) ||
		(SAM4L_AST->AST_CR & AST_CR_EN) ||
		(SAM4L_PICOUART->PICOUART_SR & PICOUART_SR_EN) ||
		(SAM4L_FREQM->FREQM_STATUS & (FREQM_STATUS_BUSY | FREQM_STATUS_RCLKBUSY)) ||
		(SAM4L_PM->PM_PBAMASK & (PM_PBAMASK_LCDCA | PM_PBAMASK_CATB)))
	{
		return true;
	}
	uint32_t wdt = SAM4L_WDT->WDT_CTRL;
	if ((wdt & (WDT_CTRL_EN | WDT_CTRL_CEN)) &&
		(wdt & (WDT_CTRL_CSSEL | WDT_CTRL_CSSEL1)))
	{
		return true;
	}
	// Leave both brownout monitors untouched. The vendor headers do not
	// expose BOD18SAMPLING, so conservatively refuse either sampled monitor.
	uint32_t bod33 = BSCIF_BOD33CTRL_EN | BSCIF_BOD33CTRL_MODE;
	uint32_t bod18 = BSCIF_BOD18CTRL_EN | BSCIF_BOD18CTRL_MODE;
	if ((SAM4L_BSCIF->BSCIF_BOD33CTRL & bod33) == bod33 ||
		(SAM4L_BSCIF->BSCIF_BOD18CTRL & bod18) == bod18)
	{
		return true;
	}
	for (unsigned i = 0; i < 12U; i++)
	{
		uint32_t clock = SAM4L_SCIF->SCIF_GCCTRL[i].SCIF_GCCTRL;
		uint32_t source = (clock & SCIF_GCCTRL_OSCSEL_Msk) >> SCIF_GCCTRL_OSCSEL_Pos;
		if ((clock & SCIF_GCCTRL_CEN) &&
			(source == OSC32K_GEN_CLK_SRC || source == RC32K_GEN_CLK_SRC || source == OSC1K_GEN_CLK_SRC))
		{
			return true;
		}
	}
	return false;
}

// LF consumers (including AST and a WDT using this source) must be stopped.
// Unlike the old descriptor-only setter, this configures the oscillator and
// CK32S before reporting success. It does not reset the core/PLL/USB clocks.
bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	McuOsc_t requested = g_McuOsc;
	requested.LowPwrOsc.Type = ClkSrc;
	requested.LowPwrOsc.Freq = OscFreq;
	if (__get_IPSR() != 0U || !Sam4lLowClockValid(ClkSrc, OscFreq)) {
		return false;
	}
	uint32_t primask = __get_PRIMASK();
	__disable_irq();
	if (Sam4lLowClockBusy())
	{
		__set_PRIMASK(primask);
		return false;
	}
	bool result = Sam4lLowClockInit(&requested.LowPwrOsc);
	if (result)
	{
		g_McuOsc.LowPwrOsc = requested.LowPwrOsc;
	}
	__set_PRIMASK(primask);
	return result;
}
