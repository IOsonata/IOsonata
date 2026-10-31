/**-------------------------------------------------------------------------
@file	system_samd.c

@brief	SAM D11/D21 system clock initialization

SystemInit sets the main clock to 48 MHz, from the oscillators of g_McuOsc:

	CoreOsc crystal or oscillator on XIN : FDPLL96M, XOSC reference
	LowPwrOsc crystal or oscillator      : DFLL48M closed loop on XOSC32K
	bUSBClk without crystal              : DFLL48M USB clock recovery
	internal oscillators only            : FDPLL96M, OSC8M / 8 reference

The FDPLL96M runs at up to 96 MHz, the main clock is half of it.

Generic clock generators, see clock_samd.h:

	0 : main clock, CPU, buses and the SERCOM, TC and EIC core clocks
	1 : 32.768 kHz, XOSC32K or the calibrated OSC32K, RTC and DFLL48M reference
	2 : OSC8M / 8, FDPLL96M reference without crystal

One flash wait state, the datasheet value at 48 MHz for VDD 2.7 to 3.63 V.

The 32.768 kHz clock is set right after the main clock is on OSC8M, it
does not depend on the steps after it. A main clock step that fails, an
oscillator that does not start for example, leaves the main clock on OSC8M
at 8 MHz. s_SamdStartupError tells the first stage that failed and
s_SamdStartupLine the source line, SystemCoreClock the main clock.

@author	Hoang Nguyen Hoan
@date	Oct. 28, 2026

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
#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"

#include "coredev/system_core_clock.h"
#include "coredev/interrupt.h"
#include "clock_samd.h"

#define SAMD_OSC8M_FREQ			8000000UL
#define SAMD_OSC32K_FREQ		32768UL
#define SAMD_DFLL_USB_FREQ		48000000UL
#define SAMD_DFLL_USB_MUL		48000UL		// 48 MHz from the 1 kHz USB start of frame
#define SAMD_MAIN_FREQ_MAX		48000000UL
#define SAMD_DPLL_FREQ_MIN		48000000UL	// FDPLL96M output range
#define SAMD_DPLL_FREQ_MAX		96000000UL
#define SAMD_DPLL_REF_MIN		32000UL		// FDPLL96M reference input range
#define SAMD_DPLL_REF_MAX		2000000UL
#define SAMD_DPLL_MAIN_DIV		2U			// Main clock is FDPLL96M / 2
#define SAMD_DPLL_OSC8M_DIV		8U			// OSC8M / 8 = 1 MHz FDPLL96M reference
#define SAMD_DFLL_REF_MIN		732UL		// DFLL48M closed loop reference range
#define SAMD_DFLL_REF_MAX		33000UL
#define SAMD_DFLL_FINE_MID		512U
#define SAMD_DFLL_COARSE_DEFAULT	31U		// Middle value when the factory calibration is blank
#define SAMD_FLASH_WAIT_STATE	1U			// Up to 48 MHz, VDD 2.7 to 3.63 V
#define SAMD_NSDELAY_LOOP_CLK	3UL			// Cycles of one nsDelay loop

// Oscillator startup times, SYSCTRL STARTUP fields
#define SAMD_XOSC_STARTUP		10U			// 1024 OSCULP32K cycles, about 31 ms
#define SAMD_XOSC32K_STARTUP	6U			// 32771 cycles, about 1 s
#define SAMD_OSC32K_STARTUP		6U			// 66 cycles, about 2 ms

// Poll budgets, not time. Oscillators are started with the CPU on OSC8M at
// 8 MHz, the longest one, XOSC32K, takes about 1 s.
#define SAMD_STARTUP_POLL_LIMIT	10000000UL
#define SAMD_SYNC_POLL_LIMIT	100000UL
#define SAMD_CLKEN_POLL_LIMIT	10000UL		// About 9 ms at 48 MHz, CLKEN follows in 3 generator periods
#define SAMD_DPLL_LOCK_POLL_LIMIT	100000UL	// About 100 ms at 8 MHz, the FDPLL96M locks in 2 ms
#define SAMD_DFLL_LOCK_POLL_LIMIT	1000000UL	// About 1 s at 8 MHz
#define SAMD_OSC32K_POLL_LIMIT	200000UL	// About 100 ms, the start takes 2 ms

// The OSC32K calibration field has a different name in the D11 header
#ifdef FUSES_OTP4_WORD_1_OSC32K_CAL_Msk
#define SAMD_OSC32K_CAL_Msk		FUSES_OTP4_WORD_1_OSC32K_CAL_Msk
#define SAMD_OSC32K_CAL_Pos		FUSES_OTP4_WORD_1_OSC32K_CAL_Pos
#else
#define SAMD_OSC32K_CAL_Msk		FUSES_OTP4_WORD_1_OSC32K_Msk
#define SAMD_OSC32K_CAL_Pos		FUSES_OTP4_WORD_1_OSC32K_Pos
#endif

// Calibration word of the software calibration row with the OSC32K and
// DFLL48M COARSE values
#define SAMD_OTP4_WORD_1		(*(volatile const uint32_t *)(OTP4_ADDR + 4UL))

// Startup failure stage, debugger visible
enum {
	SAMD_STARTUP_OK = 0,
	SAMD_STARTUP_CONFIG,
	SAMD_STARTUP_GCLK,
	SAMD_STARTUP_OSC8M,
	SAMD_STARTUP_XOSC,
	SAMD_STARTUP_LFCLK,
	SAMD_STARTUP_DFLL,
	SAMD_STARTUP_DPLL,
	SAMD_STARTUP_CLOCK_SWITCH,
};

// OscDesc_t: { type, frequency Hz, accuracy ppm, load capacitance x10 pF }.
// Applications with crystals or USB override this weak object.
__attribute__((weak)) McuOsc_t g_McuOsc = {
	{ OSC_TYPE_RC, SAMD_OSC8M_FREQ, 0UL, 0UL },
	{ OSC_TYPE_RC, SAMD_OSC32K_FREQ, 0UL, 0UL },
	false
};

// CPU clock after reset is OSC8M divided by 8
uint32_t SystemCoreClock = SAMD_OSC8M_FREQ / 8UL;
uint32_t SystemnsDelayFactor = SAMD_NSDELAY_LOOP_CLK * 1000000000UL / (SAMD_OSC8M_FREQ / 8UL);

// The frequencies of the DFLL48M and the FDPLL96M come from their registers:
// the C startup clears .bss after SystemInit, a variable set there is lost.
// The startup stage and line are kept in .data for the same reason.
__attribute__((section(".data"))) static volatile uint32_t s_SamdStartupError;

// Source line of the failure. The compiler can merge the calls: the
// debugger then shows one line for all of them, this one is exact.
__attribute__((section(".data"))) static volatile uint32_t s_SamdStartupLine;

// All the failures in order, stage << 16 | source line: the first one is
// also in s_SamdStartupError and s_SamdStartupLine, the next ones are the
// steps run after it
#define SAMD_STARTUP_LOG_LEN	4U
__attribute__((section(".data"))) static volatile uint32_t s_SamdStartupLog[SAMD_STARTUP_LOG_LEN];

// Clock registers at the end of the startup:
// [0] CLKCTRL DFLL48M channel, [1] CLKCTRL FDPLL96M channel,
// [2] [3] [4] GENCTRL generators 0, 1, 2, [5] GENDIV generator 2,
// [6] GCLK STATUS, [7] PCLKSR, [8] OSC8M, [9] OSC32K, [10] DFLLCTRL,
// [11] DPLLSTATUS << 8 | DPLLCTRLA
#define SAMD_STARTUP_REG_LEN	12U
__attribute__((section(".data"))) static volatile uint32_t s_SamdStartupReg[SAMD_STARTUP_REG_LEN];

// Failure of a startup step, the caller returns false
static bool SamdStartupFailed(uint32_t Error, uint32_t Line)
{
	if (s_SamdStartupError == SAMD_STARTUP_OK)
	{
		s_SamdStartupError = Error;
		s_SamdStartupLine = Line;
	}

	for (uint32_t i = 0; i < SAMD_STARTUP_LOG_LEN; i++)
	{
		if (s_SamdStartupLog[i] == 0U)
		{
			s_SamdStartupLog[i] = (Error << 16) | Line;
			break;
		}
	}

	return false;
}

static bool SamdGclkSync(void)
{
	uint32_t timeout = SAMD_SYNC_POLL_LIMIT;

	while ((GCLK_REGS->GCLK_STATUS & GCLK_STATUS_SYNCBUSY_Msk) != 0U)
	{
		if (--timeout == 0U)
		{
			return false;
		}
	}

	return true;
}

// Wait until all bits of Mask are set in PCLKSR
static bool SamdPclkWait(uint32_t Mask, uint32_t Limit)
{
	while ((SYSCTRL_REGS->SYSCTRL_PCLKSR & Mask) != Mask)
	{
		if (--Limit == 0U)
		{
			return false;
		}
	}

	return true;
}

// GENCTRL and GENDIV are read for the generator whose ID was written in their
// first byte. Interrupts disabled by the caller.
static uint32_t SamdGclkGenRead(volatile uint32_t *pReg, uint32_t Gen)
{
	if (!SamdGclkSync())
	{
		return 0U;
	}

	*(volatile uint8_t *)pReg = (uint8_t)Gen;

	if (!SamdGclkSync())
	{
		return 0U;
	}

	return *pReg;
}

// Generator Gen from source Src divided by Div, 0 and 1 undivided. GENDIV is
// written first, GENCTRL then switches the generator.
static bool SamdGclkGenSet(uint32_t Gen, uint32_t Src, uint32_t Div)
{
	if (!SamdGclkSync())
	{
		return false;
	}

	GCLK_REGS->GCLK_GENDIV = GCLK_GENDIV_ID(Gen) | GCLK_GENDIV_DIV(Div);

	if (!SamdGclkSync())
	{
		return false;
	}

	GCLK_REGS->GCLK_GENCTRL = GCLK_GENCTRL_ID(Gen) | GCLK_GENCTRL_SRC(Src) |
							  GCLK_GENCTRL_GENEN_Msk | GCLK_GENCTRL_IDC_Msk;

	return SamdGclkSync();
}

// CLKCTRL is read for the channel whose ID was written in its first byte.
// The GCLK synchronization is waited for around the selection, as the main
// branch does.
static uint16_t SamdGclkChannelRead(uint8_t Channel)
{
	(void)SamdGclkSync();
	*(volatile uint8_t *)&GCLK_REGS->GCLK_CLKCTRL = Channel;
	(void)SamdGclkSync();

	return GCLK_REGS->GCLK_CLKCTRL;
}

// Channel stop as ASF system_gclk_chan_disable does: the channel is moved to
// generator 0, which always runs, CLKEN is cleared and read back, then the
// previous generator is written back with CLKEN cleared. CLKEN of a channel
// on a generator that does not run does not read back cleared. The CLKCTRL
// write selects the channel for the reads.
static bool SamdGclkChannelStop(uint16_t Ctrl)
{
	uint16_t id = Ctrl & GCLK_CLKCTRL_ID_Msk;
	bool res = false;

	GCLK_REGS->GCLK_CLKCTRL = id | GCLK_CLKCTRL_GEN(SAMD_GCLK_MAIN) | GCLK_CLKCTRL_CLKEN_Msk;
	GCLK_REGS->GCLK_CLKCTRL = id | GCLK_CLKCTRL_GEN(SAMD_GCLK_MAIN);

	for (uint32_t i = 0; i < SAMD_CLKEN_POLL_LIMIT && !res; i++)
	{
		res = (GCLK_REGS->GCLK_CLKCTRL & GCLK_CLKCTRL_CLKEN_Msk) == 0U;
	}

	GCLK_REGS->GCLK_CLKCTRL = Ctrl & (GCLK_CLKCTRL_ID_Msk | GCLK_CLKCTRL_GEN_Msk);

	return res;
}

bool SamdGclkChannelEnable(uint8_t Channel, uint8_t Generator)
{
	uint32_t want = GCLK_CLKCTRL_ID(Channel) | GCLK_CLKCTRL_GEN(Generator) | GCLK_CLKCTRL_CLKEN_Msk;
	uint32_t state;
	uint16_t ctrl;
	bool res = true;

	if (Channel > (GCLK_CLKCTRL_ID_Msk >> GCLK_CLKCTRL_ID_Pos) || Generator >= GCLK_GEN_NUM)
	{
		return false;
	}

	state = DisableInterrupt();

	ctrl = SamdGclkChannelRead(Channel);

	if ((ctrl & (GCLK_CLKCTRL_GEN_Msk | GCLK_CLKCTRL_CLKEN_Msk)) !=
		(want & (GCLK_CLKCTRL_GEN_Msk | GCLK_CLKCTRL_CLKEN_Msk)))
	{
		if ((ctrl & GCLK_CLKCTRL_WRTLOCK_Msk) != 0U)
		{
			// Locked on another setting until the next reset
			res = false;
		}
		else
		{
			// The generator of a running channel changes only once stopped
			if ((ctrl & GCLK_CLKCTRL_CLKEN_Msk) != 0U)
			{
				res = SamdGclkChannelStop(ctrl);
			}

			// CLKEN is not read back, as in ASF system_gclk_chan_enable and
			// the Harmony clock setup: on the SAM D11 Xplained Pro the
			// FDPLL96M channel on generator 2 did not read back enabled. The
			// peripheral shows whether its clock runs, the FDPLL96M and the
			// DFLL48M by their lock.
			if (res && SamdGclkSync())
			{
				GCLK_REGS->GCLK_CLKCTRL = (uint16_t)want;
				res = SamdGclkSync();
			}
			else
			{
				res = false;
			}
		}
	}

	EnableInterrupt(state);

	return res;
}

bool SamdGclkChannelDisable(uint8_t Channel)
{
	uint32_t state;
	uint16_t ctrl;
	bool res = true;

	if (Channel > (GCLK_CLKCTRL_ID_Msk >> GCLK_CLKCTRL_ID_Pos))
	{
		return false;
	}

	state = DisableInterrupt();

	ctrl = SamdGclkChannelRead(Channel);

	if ((ctrl & GCLK_CLKCTRL_CLKEN_Msk) != 0U)
	{
		res = (ctrl & GCLK_CLKCTRL_WRTLOCK_Msk) == 0U && SamdGclkChannelStop(ctrl);
	}

	EnableInterrupt(state);

	return res;
}

static uint32_t SamdGclkGenFreq(uint32_t Gen);
static uint32_t SamdSourceFreq(uint32_t Src);

// Generator of a GCLK channel
static uint32_t SamdGclkChannelGen(uint8_t Channel)
{
	uint32_t state = DisableInterrupt();
	uint16_t ctrl = SamdGclkChannelRead(Channel);

	EnableInterrupt(state);

	return ((uint32_t)ctrl & GCLK_CLKCTRL_GEN_Msk) >> GCLK_CLKCTRL_GEN_Pos;
}

// DFLL48M: MUL x the reference in closed loop, 48 MHz nominal in open loop
static uint32_t SamdDfllFreq(void)
{
	uint32_t ctrl = SYSCTRL_REGS->SYSCTRL_DFLLCTRL;
	uint32_t mul = (SYSCTRL_REGS->SYSCTRL_DFLLMUL & SYSCTRL_DFLLMUL_MUL_Msk) >> SYSCTRL_DFLLMUL_MUL_Pos;

	if ((ctrl & SYSCTRL_DFLLCTRL_ENABLE_Msk) == 0U)
	{
		return 0U;
	}
	if ((ctrl & SYSCTRL_DFLLCTRL_MODE_Msk) == 0U)
	{
		return SAMD_MAIN_FREQ_MAX;
	}
	if ((ctrl & SYSCTRL_DFLLCTRL_USBCRM_Msk) != 0U)
	{
		return mul * (SAMD_DFLL_USB_FREQ / SAMD_DFLL_USB_MUL);
	}

	return mul * SamdGclkGenFreq(SamdGclkChannelGen(GCLK_CLKCTRL_ID_DFLL48_Val));
}

// FDPLL96M: reference x (LDR + 1 + LDRFRAC / 16), 0 until it is locked
static uint32_t SamdDpllFreq(void)
{
	uint32_t ready = SYSCTRL_DPLLSTATUS_LOCK_Msk | SYSCTRL_DPLLSTATUS_CLKRDY_Msk;
	uint32_t ctrlb = SYSCTRL_REGS->SYSCTRL_DPLLCTRLB;
	uint32_t ratio = SYSCTRL_REGS->SYSCTRL_DPLLRATIO;
	uint32_t ref;

	if ((SYSCTRL_REGS->SYSCTRL_DPLLSTATUS & ready) != ready)
	{
		return 0U;
	}

	switch ((ctrlb & SYSCTRL_DPLLCTRLB_REFCLK_Msk) >> SYSCTRL_DPLLCTRLB_REFCLK_Pos)
	{
		case SYSCTRL_DPLLCTRLB_REFCLK_REF0_Val:
			ref = SamdSourceFreq(GCLK_GENCTRL_SRC_XOSC32K_Val);
			break;
		case SYSCTRL_DPLLCTRLB_REFCLK_REF1_Val:
			ref = SamdSourceFreq(GCLK_GENCTRL_SRC_XOSC_Val) /
				  (2U * (((ctrlb & SYSCTRL_DPLLCTRLB_DIV_Msk) >> SYSCTRL_DPLLCTRLB_DIV_Pos) + 1U));
			break;
		case SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val:
			ref = SamdGclkGenFreq(SamdGclkChannelGen(GCLK_CLKCTRL_ID_FDPLL_Val));
			break;
		default:
			return 0U;
	}

	uint32_t mul16 = ((((ratio & SYSCTRL_DPLLRATIO_LDR_Msk) >> SYSCTRL_DPLLRATIO_LDR_Pos) + 1U) << 4) +
					 ((ratio & SYSCTRL_DPLLRATIO_LDRFRAC_Msk) >> SYSCTRL_DPLLRATIO_LDRFRAC_Pos);

	return (uint32_t)(((uint64_t)ref * mul16) >> 4);
}

// Frequency of a GCLK generator source
static uint32_t SamdSourceFreq(uint32_t Src)
{
	switch (Src)
	{
		case GCLK_GENCTRL_SRC_XOSC_Val:
			return (SYSCTRL_REGS->SYSCTRL_XOSC & SYSCTRL_XOSC_ENABLE_Msk) != 0U ? g_McuOsc.CoreOsc.Freq : 0U;
		case GCLK_GENCTRL_SRC_GCLKGEN1_Val:
			return SamdGclkGenFreq(1U);
		case GCLK_GENCTRL_SRC_OSCULP32K_Val:
			return SAMD_OSC32K_FREQ;
		case GCLK_GENCTRL_SRC_OSC32K_Val:
			return (SYSCTRL_REGS->SYSCTRL_OSC32K & SYSCTRL_OSC32K_ENABLE_Msk) != 0U ? SAMD_OSC32K_FREQ : 0U;
		case GCLK_GENCTRL_SRC_XOSC32K_Val:
			return (SYSCTRL_REGS->SYSCTRL_XOSC32K & SYSCTRL_XOSC32K_ENABLE_Msk) != 0U ? g_McuOsc.LowPwrOsc.Freq : 0U;
		case GCLK_GENCTRL_SRC_OSC8M_Val:
			return SAMD_OSC8M_FREQ >> ((SYSCTRL_REGS->SYSCTRL_OSC8M & SYSCTRL_OSC8M_PRESC_Msk) >> SYSCTRL_OSC8M_PRESC_Pos);
		case GCLK_GENCTRL_SRC_DFLL48M_Val:
			return SamdDfllFreq();
		case GCLK_GENCTRL_SRC_DPLL96M_Val:
			return SamdDpllFreq();
	}

	// GCLKIN pin, frequency unknown
	return 0U;
}

static uint32_t SamdGclkGenFreq(uint32_t Gen)
{
	uint32_t state;
	uint32_t ctrl;
	uint32_t div;
	uint32_t src;
	uint32_t freq;

	if (Gen >= GCLK_GEN_NUM)
	{
		return 0U;
	}

	state = DisableInterrupt();
	ctrl = SamdGclkGenRead(&GCLK_REGS->GCLK_GENCTRL, Gen);
	div = (SamdGclkGenRead(&GCLK_REGS->GCLK_GENDIV, Gen) & GCLK_GENDIV_DIV_Msk) >> GCLK_GENDIV_DIV_Pos;
	EnableInterrupt(state);

	src = (ctrl & GCLK_GENCTRL_SRC_Msk) >> GCLK_GENCTRL_SRC_Pos;

	// Generator 1 cannot take generator 1 as its source
	if ((ctrl & GCLK_GENCTRL_GENEN_Msk) == 0U || (Gen == 1U && src == GCLK_GENCTRL_SRC_GCLKGEN1_Val))
	{
		return 0U;
	}

	freq = SamdSourceFreq(src);

	if ((ctrl & GCLK_GENCTRL_DIVSEL_Msk) != 0U)
	{
		freq = div < 31U ? freq >> (div + 1U) : 0U;
	}
	else if (div > 1U)
	{
		freq /= div;
	}

	return freq;
}

static bool SamdLowFreqValid(OSC_TYPE Type, uint32_t Freq)
{
	if (Type == OSC_TYPE_RC)
	{
		// OSC32K, nominal 32.768 kHz
		return Freq == 32000UL || Freq == SAMD_OSC32K_FREQ;
	}

	if (Type == OSC_TYPE_XTAL)
	{
		return Freq == SAMD_OSC32K_FREQ;
	}

	// Clock on XIN32, used as DFLL48M reference
	return Type == OSC_TYPE_TCXO && Freq >= SAMD_DFLL_REF_MIN && Freq <= SAMD_DFLL_REF_MAX;
}

static bool SamdConfigValid(const McuOsc_t *pOsc)
{
	if (pOsc->CoreOsc.Type == OSC_TYPE_RC)
	{
		// OSC8M
		if (pOsc->CoreOsc.Freq != SAMD_OSC8M_FREQ)
		{
			return false;
		}
	}
	else if (pOsc->CoreOsc.Type == OSC_TYPE_XTAL)
	{
		// XOSC crystal range
		if (pOsc->CoreOsc.Freq < 400000UL || pOsc->CoreOsc.Freq > 32000000UL)
		{
			return false;
		}
	}
	else if (pOsc->CoreOsc.Type != OSC_TYPE_TCXO || pOsc->CoreOsc.Freq < 2UL * SAMD_DPLL_REF_MIN ||
			 pOsc->CoreOsc.Freq > 32000000UL)
	{
		return false;
	}

	return SamdLowFreqValid(pOsc->LowPwrOsc.Type, pOsc->LowPwrOsc.Freq);
}

// OSC8M undivided, 8 MHz, always on.
// OSC8M is the CPU clock after reset and is enabled then: the code runs on
// it, a prescaler change does not restart it. OSC8MRDY stayed low on
// hardware in that case, it is waited for only when this enables OSC8M (a
// start without reset, from a firmware that stopped it).
static bool SamdOsc8mStart(void)
{
	uint32_t osc = SYSCTRL_REGS->SYSCTRL_OSC8M;
	bool enabled = (osc & SYSCTRL_OSC8M_ENABLE_Msk) != 0U;

	osc &= ~(SYSCTRL_OSC8M_PRESC_Msk | SYSCTRL_OSC8M_ONDEMAND_Msk);
	osc |= SYSCTRL_OSC8M_PRESC(SYSCTRL_OSC8M_PRESC_0_Val) | SYSCTRL_OSC8M_ENABLE_Msk;
	SYSCTRL_REGS->SYSCTRL_OSC8M = osc;

	if (!enabled && !SamdPclkWait(SYSCTRL_PCLKSR_OSC8MRDY_Msk, SAMD_STARTUP_POLL_LIMIT))
	{
		return SamdStartupFailed(SAMD_STARTUP_OSC8M, __LINE__);
	}

	return true;
}

// Low frequency oscillator and generator 1. The oscillator not selected is
// stopped once generator 1 has switched.
static bool SamdLowFreqInit(void)
{
	uint32_t src;

	if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_RC)
	{
		uint32_t calib = (SAMD_OTP4_WORD_1 & SAMD_OSC32K_CAL_Msk) >> SAMD_OSC32K_CAL_Pos;
		uint32_t ctrl = SYSCTRL_OSC32K_CALIB(calib) | SYSCTRL_OSC32K_STARTUP(SAMD_OSC32K_STARTUP) |
						SYSCTRL_OSC32K_EN32K_Msk;

		// Configured first, enabled by a separate write
		if ((SYSCTRL_REGS->SYSCTRL_OSC32K & ~SYSCTRL_OSC32K_ONDEMAND_Msk) != (ctrl | SYSCTRL_OSC32K_ENABLE_Msk))
		{
			SYSCTRL_REGS->SYSCTRL_OSC32K = ctrl;
			SYSCTRL_REGS->SYSCTRL_OSC32K = ctrl | SYSCTRL_OSC32K_ENABLE_Msk;
		}

		if (!SamdPclkWait(SYSCTRL_PCLKSR_OSC32KRDY_Msk, SAMD_OSC32K_POLL_LIMIT))
		{
			return SamdStartupFailed(SAMD_STARTUP_LFCLK, __LINE__);
		}
		src = GCLK_GENCTRL_SRC_OSC32K_Val;
	}
	else
	{
		uint16_t ctrl = SYSCTRL_XOSC32K_STARTUP(SAMD_XOSC32K_STARTUP) | SYSCTRL_XOSC32K_EN32K_Msk;

		if (g_McuOsc.LowPwrOsc.Type == OSC_TYPE_XTAL)
		{
			ctrl |= SYSCTRL_XOSC32K_XTALEN_Msk;
		}

		// Configured first, enabled by a separate write
		if ((SYSCTRL_REGS->SYSCTRL_XOSC32K & ~SYSCTRL_XOSC32K_ONDEMAND_Msk) != (ctrl | SYSCTRL_XOSC32K_ENABLE_Msk))
		{
			SYSCTRL_REGS->SYSCTRL_XOSC32K = 0U;
			SYSCTRL_REGS->SYSCTRL_XOSC32K = ctrl;
			SYSCTRL_REGS->SYSCTRL_XOSC32K = ctrl | SYSCTRL_XOSC32K_ENABLE_Msk;
		}

		if (!SamdPclkWait(SYSCTRL_PCLKSR_XOSC32KRDY_Msk, SAMD_STARTUP_POLL_LIMIT))
		{
			return SamdStartupFailed(SAMD_STARTUP_LFCLK, __LINE__);
		}
		src = GCLK_GENCTRL_SRC_XOSC32K_Val;
	}

	if (!SamdGclkGenSet(SAMD_GCLK_LF, src, 0U))
	{
		return SamdStartupFailed(SAMD_STARTUP_GCLK, __LINE__);
	}

	if (src == GCLK_GENCTRL_SRC_OSC32K_Val)
	{
		SYSCTRL_REGS->SYSCTRL_XOSC32K = 0U;
	}
	else
	{
		SYSCTRL_REGS->SYSCTRL_OSC32K &= ~SYSCTRL_OSC32K_ENABLE_Msk;
	}

	return true;
}

// XOSC, crystal or clock on XIN
static bool SamdXoscStart(void)
{
	uint32_t freq = g_McuOsc.CoreOsc.Freq;
	uint16_t ctrl = SYSCTRL_XOSC_STARTUP(SAMD_XOSC_STARTUP);

	if (g_McuOsc.CoreOsc.Type == OSC_TYPE_XTAL)
	{
		// Gain for 2, 4, 8, 16 and 30 MHz crystals
		uint32_t gain = freq <= 2000000UL ? 0U : freq <= 4000000UL ? 1U :
						freq <= 8000000UL ? 2U : freq <= 16000000UL ? 3U : 4U;

		ctrl |= SYSCTRL_XOSC_XTALEN_Msk | SYSCTRL_XOSC_GAIN(gain);
	}

	SYSCTRL_REGS->SYSCTRL_XOSC = 0U;
	SYSCTRL_REGS->SYSCTRL_XOSC = ctrl;
	SYSCTRL_REGS->SYSCTRL_XOSC = ctrl | SYSCTRL_XOSC_ENABLE_Msk;

	if (!SamdPclkWait(SYSCTRL_PCLKSR_XOSCRDY_Msk, SAMD_STARTUP_POLL_LIMIT))
	{
		return SamdStartupFailed(SAMD_STARTUP_XOSC, __LINE__);
	}

	return true;
}

// FDPLL96M off. Its configuration changes only while disabled.
static bool SamdDpllStop(void)
{
	uint32_t timeout = SAMD_SYNC_POLL_LIMIT;

	SYSCTRL_REGS->SYSCTRL_DPLLCTRLA = 0U;

	while ((SYSCTRL_REGS->SYSCTRL_DPLLSTATUS & SYSCTRL_DPLLSTATUS_ENABLE_Msk) != 0U)
	{
		if (--timeout == 0U)
		{
			return SamdStartupFailed(SAMD_STARTUP_DPLL, __LINE__);
		}
	}

	return true;
}

// FDPLL96M at up to 96 MHz from a reference of RefFreq after the XOSC divider:
// fCK = RefFreq x (LDR + 1 + LDRFRAC / 16)
static bool SamdDpllStart(uint32_t RefClk, uint32_t RefFreq, uint32_t Div)
{
	uint32_t ratio;
	uint32_t freq;

	if (RefFreq < SAMD_DPLL_REF_MIN || RefFreq > SAMD_DPLL_REF_MAX)
	{
		return SamdStartupFailed(SAMD_STARTUP_DPLL, __LINE__);
	}

	// (LDR + 1) x 16 + LDRFRAC, rounded down to stay within 96 MHz
	ratio = (uint32_t)(((uint64_t)SAMD_DPLL_FREQ_MAX * 16U) / RefFreq);
	if (ratio > 4096U * 16U)
	{
		ratio = 4096U * 16U;
	}
	freq = (uint32_t)(((uint64_t)RefFreq * ratio) / 16U);

	if (freq < SAMD_DPLL_FREQ_MIN)
	{
		return SamdStartupFailed(SAMD_STARTUP_DPLL, __LINE__);
	}

	if (!SamdDpllStop())
	{
		return false;
	}

	SYSCTRL_REGS->SYSCTRL_DPLLCTRLB = SYSCTRL_DPLLCTRLB_REFCLK(RefClk) | SYSCTRL_DPLLCTRLB_DIV(Div);
	SYSCTRL_REGS->SYSCTRL_DPLLRATIO = SYSCTRL_DPLLRATIO_LDR(ratio / 16U - 1U) |
									  SYSCTRL_DPLLRATIO_LDRFRAC(ratio & 15U);
	SYSCTRL_REGS->SYSCTRL_DPLLCTRLA = SYSCTRL_DPLLCTRLA_ENABLE_Msk;

	// The output stays off until the loop is locked. The lock also shows that
	// the reference runs, the GCLK channel CLKEN is not read back.
	uint32_t timeout = SAMD_DPLL_LOCK_POLL_LIMIT;
	uint32_t ready = SYSCTRL_DPLLSTATUS_LOCK_Msk | SYSCTRL_DPLLSTATUS_CLKRDY_Msk;

	while ((SYSCTRL_REGS->SYSCTRL_DPLLSTATUS & ready) != ready)
	{
		if (--timeout == 0U)
		{
			SYSCTRL_REGS->SYSCTRL_DPLLCTRLA = 0U;

			return SamdStartupFailed(SAMD_STARTUP_DPLL, __LINE__);
		}
	}

	return true;
}

// DFLL48M registers are written only while DFLLRDY is set
static bool SamdDfllWrite(volatile uint32_t *pReg, uint32_t Value)
{
	if (!SamdPclkWait(SYSCTRL_PCLKSR_DFLLRDY_Msk, SAMD_SYNC_POLL_LIMIT))
	{
		return SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
	}

	*pReg = Value;

	return true;
}

// DFLL48M off, with ONDEMAND cleared: the other DFLL registers do not take
// a write while the DFLL is on demand (erratum, reset value of ONDEMAND is 1)
static bool SamdDfllStop(void)
{
	SYSCTRL_REGS->SYSCTRL_DFLLCTRL = 0U;

	if (!SamdPclkWait(SYSCTRL_PCLKSR_DFLLRDY_Msk, SAMD_SYNC_POLL_LIMIT))
	{
		return SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
	}

	return true;
}

// DFLL48M closed loop, Mul x reference of the DFLL48M channel. USB clock
// recovery takes the 1 kHz start of frame as reference.
static bool SamdDfllStart(uint32_t Mul, bool bUsb)
{
	uint32_t coarse = (SAMD_OTP4_WORD_1 & FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Msk) >>
					  FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Pos;
	uint32_t ctrl;

	if (!SamdDfllStop())
	{
		return false;
	}

	if (coarse == (FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Msk >> FUSES_OTP4_WORD_1_DFLL48M_COARSE_CAL_Pos))
	{
		coarse = SAMD_DFLL_COARSE_DEFAULT;
	}

	// Start from the factory calibration, the loop corrects from there
	if (!SamdDfllWrite(&SYSCTRL_REGS->SYSCTRL_DFLLVAL,
					   SYSCTRL_DFLLVAL_COARSE(coarse) | SYSCTRL_DFLLVAL_FINE(SAMD_DFLL_FINE_MID)))
	{
		return false;
	}

	if (bUsb)
	{
		if (!SamdDfllWrite(&SYSCTRL_REGS->SYSCTRL_DFLLMUL, SYSCTRL_DFLLMUL_CSTEP(7U) |
						   SYSCTRL_DFLLMUL_FSTEP(10U) | SYSCTRL_DFLLMUL_MUL(Mul)))
		{
			return false;
		}
		ctrl = SYSCTRL_DFLLCTRL_MODE_Msk | SYSCTRL_DFLLCTRL_USBCRM_Msk | SYSCTRL_DFLLCTRL_CCDIS_Msk |
			   SYSCTRL_DFLLCTRL_BPLCKC_Msk;
	}
	else
	{
		// Half of the largest steps
		if (!SamdDfllWrite(&SYSCTRL_REGS->SYSCTRL_DFLLMUL, SYSCTRL_DFLLMUL_CSTEP(31U) |
						   SYSCTRL_DFLLMUL_FSTEP(511U) | SYSCTRL_DFLLMUL_MUL(Mul)))
		{
			return false;
		}
		ctrl = SYSCTRL_DFLLCTRL_MODE_Msk | SYSCTRL_DFLLCTRL_WAITLOCK_Msk | SYSCTRL_DFLLCTRL_QLDIS_Msk;
	}

	// Mode first, then enabled
	if (!SamdPclkWait(SYSCTRL_PCLKSR_DFLLRDY_Msk, SAMD_SYNC_POLL_LIMIT))
	{
		return SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
	}
	SYSCTRL_REGS->SYSCTRL_DFLLCTRL = (uint16_t)ctrl;
	if (!SamdPclkWait(SYSCTRL_PCLKSR_DFLLRDY_Msk, SAMD_SYNC_POLL_LIMIT))
	{
		return SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
	}
	SYSCTRL_REGS->SYSCTRL_DFLLCTRL = (uint16_t)(ctrl | SYSCTRL_DFLLCTRL_ENABLE_Msk);

	if (bUsb)
	{
		// Open loop until the USB start of frame comes
		if (!SamdPclkWait(SYSCTRL_PCLKSR_DFLLRDY_Msk, SAMD_SYNC_POLL_LIMIT))
		{
			return SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
		}
	}
	else if (!SamdPclkWait(SYSCTRL_PCLKSR_DFLLLCKC_Msk | SYSCTRL_PCLKSR_DFLLLCKF_Msk |
						   SYSCTRL_PCLKSR_DFLLRDY_Msk, SAMD_DFLL_LOCK_POLL_LIMIT))
	{
		// No lock, the reference does not run: the DFLL48M is stopped
		(void)SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
		(void)SamdDfllStop();

		return false;
	}

	return true;
}

void SystemCoreClockUpdate(void)
{
	uint32_t freq = SamdGclkGenFreq(SAMD_GCLK_MAIN);

	SystemCoreClock = freq >> ((PM_REGS->PM_CPUSEL & PM_CPUSEL_CPUDIV_Msk) >> PM_CPUSEL_CPUDIV_Pos);

	if (SystemCoreClock != 0U)
	{
		SystemnsDelayFactor = SAMD_NSDELAY_LOOP_CLK * 1000000000UL / SystemCoreClock;
	}
}

// Clock registers at the end of the startup, see s_SamdStartupReg
static void SamdStartupRegs(void)
{
	s_SamdStartupReg[0] = SamdGclkChannelRead(GCLK_CLKCTRL_ID_DFLL48_Val);
	s_SamdStartupReg[1] = SamdGclkChannelRead(GCLK_CLKCTRL_ID_FDPLL_Val);
	for (uint32_t i = 0; i < 3U; i++)
	{
		s_SamdStartupReg[2U + i] = SamdGclkGenRead(&GCLK_REGS->GCLK_GENCTRL, i);
	}
	s_SamdStartupReg[5] = SamdGclkGenRead(&GCLK_REGS->GCLK_GENDIV, SAMD_GCLK_DPLLREF);
	s_SamdStartupReg[6] = GCLK_REGS->GCLK_STATUS;
	s_SamdStartupReg[7] = SYSCTRL_REGS->SYSCTRL_PCLKSR;
	s_SamdStartupReg[8] = SYSCTRL_REGS->SYSCTRL_OSC8M;
	s_SamdStartupReg[9] = SYSCTRL_REGS->SYSCTRL_OSC32K;
	s_SamdStartupReg[10] = SYSCTRL_REGS->SYSCTRL_DFLLCTRL;
	s_SamdStartupReg[11] = ((uint32_t)SYSCTRL_REGS->SYSCTRL_DPLLSTATUS << 8) | SYSCTRL_REGS->SYSCTRL_DPLLCTRLA;
}

// FDPLL96M from OSC8M / 8 on generator 2, 1 MHz reference x 96
static bool SamdDpllFromOsc8m(void)
{
	if (!SamdGclkGenSet(SAMD_GCLK_DPLLREF, GCLK_GENCTRL_SRC_OSC8M_Val, SAMD_DPLL_OSC8M_DIV))
	{
		return SamdStartupFailed(SAMD_STARTUP_GCLK, __LINE__);
	}

	if (!SamdGclkChannelEnable(GCLK_CLKCTRL_ID_FDPLL_Val, SAMD_GCLK_DPLLREF))
	{
		return SamdStartupFailed(SAMD_STARTUP_GCLK, __LINE__);
	}

	return SamdDpllStart(SYSCTRL_DPLLCTRLB_REFCLK_GCLK_Val, SAMD_OSC8M_FREQ / SAMD_DPLL_OSC8M_DIV, 0U);
}

// DFLL48M closed loop on the 32.768 kHz clock of generator 1, rounded down to
// at most 48 MHz: 1464 x 32768 Hz = 47.972 MHz
static bool SamdDfllFromLf(void)
{
	uint32_t ref = SamdGclkGenFreq(SAMD_GCLK_LF);

	if (ref < SAMD_DFLL_REF_MIN || ref > SAMD_DFLL_REF_MAX)
	{
		return SamdStartupFailed(SAMD_STARTUP_DFLL, __LINE__);
	}

	if (!SamdGclkChannelEnable(GCLK_CLKCTRL_ID_DFLL48_Val, SAMD_GCLK_LF))
	{
		return SamdStartupFailed(SAMD_STARTUP_GCLK, __LINE__);
	}

	return SamdDfllStart(SAMD_MAIN_FREQ_MAX / ref, false);
}

// Main clock on OSC8M undivided, generator 0, the main branch setting that
// runs on both boards. Interrupts disabled.
static bool SamdMainOsc8m(void)
{
	return SamdOsc8mStart() &&
		   (SamdGclkGenSet(SAMD_GCLK_MAIN, GCLK_GENCTRL_SRC_OSC8M_Val, 0U) ||
			SamdStartupFailed(SAMD_STARTUP_GCLK, __LINE__));
}

// 48 MHz main clock from g_McuOsc, the main clock on OSC8M first. Stops at
// the first failure. Interrupts disabled.
static bool SamdClockInit(void)
{
	uint32_t src;
	uint32_t div;

	if (!SamdConfigValid(&g_McuOsc))
	{
		return SamdStartupFailed(SAMD_STARTUP_CONFIG, __LINE__);
	}

	// Wait state for the final clock, manual page write. The CPU and the
	// buses are not divided.
	NVMCTRL_REGS->NVMCTRL_CTRLB = (NVMCTRL_REGS->NVMCTRL_CTRLB & ~NVMCTRL_CTRLB_RWS_Msk) |
								  NVMCTRL_CTRLB_RWS(SAMD_FLASH_WAIT_STATE) | NVMCTRL_CTRLB_MANW_Msk;
	PM_REGS->PM_CPUSEL = 0U;
	PM_REGS->PM_APBASEL = 0U;
	PM_REGS->PM_APBBSEL = 0U;
	PM_REGS->PM_APBCSEL = 0U;

	// Main clock on OSC8M before any other source changes. The oscillator
	// starts take less time at 8 MHz.
	if (!SamdMainOsc8m())
	{
		return false;
	}

	// The 32.768 kHz clock on generator 1, set whatever the main clock steps
	// after it do. Only the DFLL48M from a crystal or a 32 kHz clock needs it.
	bool lf = SamdLowFreqInit();

	// The FDPLL96M and the DFLL48M are off after reset, they are stopped only
	// when a firmware started before without a reset left them running
	if ((SYSCTRL_REGS->SYSCTRL_DPLLCTRLA & SYSCTRL_DPLLCTRLA_ENABLE_Msk) != 0U && !SamdDpllStop())
	{
		return false;
	}
	if ((SYSCTRL_REGS->SYSCTRL_DFLLCTRL & SYSCTRL_DFLLCTRL_ENABLE_Msk) != 0U && !SamdDfllStop())
	{
		return false;
	}

	if (g_McuOsc.CoreOsc.Type != OSC_TYPE_RC)
	{
		// Reference XOSC / (2 x (DIV + 1)), at most 2 MHz
		uint32_t xdiv = (g_McuOsc.CoreOsc.Freq + 2UL * SAMD_DPLL_REF_MAX - 1UL) / (2UL * SAMD_DPLL_REF_MAX);

		if (!SamdXoscStart() ||
			!SamdDpllStart(SYSCTRL_DPLLCTRLB_REFCLK_REF1_Val, g_McuOsc.CoreOsc.Freq / (2UL * xdiv), xdiv - 1UL))
		{
			return false;
		}
		src = GCLK_GENCTRL_SRC_DPLL96M_Val;
		div = SAMD_DPLL_MAIN_DIV;
	}
	else
	{
		SYSCTRL_REGS->SYSCTRL_XOSC = 0U;

		if (g_McuOsc.LowPwrOsc.Type != OSC_TYPE_RC)
		{
			// DFLL48M locked on the 32 kHz crystal or clock of generator 1
			if (!lf || !SamdDfllFromLf())
			{
				return false;
			}
			src = GCLK_GENCTRL_SRC_DFLL48M_Val;
			div = 0U;
		}
		else if (g_McuOsc.bUSBClk)
		{
			if (!SamdDfllStart(SAMD_DFLL_USB_MUL, true))
			{
				return false;
			}
			src = GCLK_GENCTRL_SRC_DFLL48M_Val;
			div = 0U;
		}
		else if (SamdDpllFromOsc8m())
		{
			src = GCLK_GENCTRL_SRC_DPLL96M_Val;
			div = SAMD_DPLL_MAIN_DIV;
		}
		else if (lf && SamdDfllFromLf())
		{
			// The FDPLL96M did not start: the DFLL48M locked on the OSC32K
			// clock of generator 1 is the main clock, the failed FDPLL96M
			// step stays in s_SamdStartupError.
			src = GCLK_GENCTRL_SRC_DFLL48M_Val;
			div = 0U;
		}
		else
		{
			return false;
		}
	}

	if (!SamdGclkGenSet(SAMD_GCLK_MAIN, src, div))
	{
		return SamdStartupFailed(SAMD_STARTUP_CLOCK_SWITCH, __LINE__);
	}

	SystemCoreClockUpdate();

	if (SystemCoreClock == 0U || SystemCoreClock > SAMD_MAIN_FREQ_MAX)
	{
		return SamdStartupFailed(SAMD_STARTUP_CLOCK_SWITCH, __LINE__);
	}

	return true;
}

// A failed step leaves the main clock on OSC8M at 8 MHz, as the main branch
// does: s_SamdStartupError and s_SamdStartupLine tell the first step that
// failed, also when the DFLL48M took over from the FDPLL96M, s_SamdStartupLog
// all the failed steps and s_SamdStartupReg the clock registers after them.
void SystemInit(void)
{
	uint32_t primask = __get_PRIMASK();

	__disable_irq();

	s_SamdStartupError = SAMD_STARTUP_OK;
	s_SamdStartupLine = 0U;
	for (uint32_t i = 0; i < SAMD_STARTUP_LOG_LEN; i++)
	{
		s_SamdStartupLog[i] = 0U;
	}

	if (!SamdClockInit())
	{
		(void)SamdMainOsc8m();
	}

	SamdStartupRegs();

	SystemCoreClockUpdate();

	__set_PRIMASK(primask);
}

uint32_t SystemCoreClockGet(void)
{
	SystemCoreClockUpdate();

	return SystemCoreClock;
}

/**
 * @brief	Frequency of a generic clock generator.
 *
 * @param	Idx : Generator number, see clock_samd.h
 *
 * @return	Frequency in Hz, 0 for a generator not running or unknown source
 */
uint32_t SystemPeriphClockGet(int Idx)
{
	if (Idx < 0)
	{
		return 0U;
	}

	return SamdGclkGenFreq((uint32_t)Idx);
}

uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq)
{
	// The generators are set by SystemInit. Their frequency is not changed
	// at run time, a request for another frequency fails.
	uint32_t current = SystemPeriphClockGet(Idx);

	return current != 0U && current == Freq ? current : 0U;
}

// Full clock tree setup again. The caller stops the clock consumers first
// (USB, DMA, timers, serial interfaces) and sets them up again after.
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	McuOsc_t requested = g_McuOsc;
	uint32_t primask;

	requested.CoreOsc.Type = ClkSrc;
	requested.CoreOsc.Freq = OscFreq;

	if (__get_IPSR() != 0U || !SamdConfigValid(&requested))
	{
		return false;
	}

	if (ClkSrc == g_McuOsc.CoreOsc.Type && OscFreq == g_McuOsc.CoreOsc.Freq)
	{
		return true;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	g_McuOsc.CoreOsc = requested.CoreOsc;
	SystemInit();
	__set_PRIMASK(primask);

	return true;
}

// The low frequency oscillator and generator 1 only. The consumers of
// generator 1 (RTC, DFLL48M reference) are stopped by the caller first.
bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t primask;

	if (__get_IPSR() != 0U || !SamdLowFreqValid(ClkSrc, OscFreq))
	{
		return false;
	}

	if (ClkSrc == g_McuOsc.LowPwrOsc.Type && OscFreq == g_McuOsc.LowPwrOsc.Freq)
	{
		return true;
	}

	primask = __get_PRIMASK();
	__disable_irq();
	g_McuOsc.LowPwrOsc.Type = ClkSrc;
	g_McuOsc.LowPwrOsc.Freq = OscFreq;
	SamdLowFreqInit();
	__set_PRIMASK(primask);

	return true;
}

void SystemOscInit(void)
{
	// No programmable crystal load capacitance. SystemInit starts the
	// oscillators.
}
