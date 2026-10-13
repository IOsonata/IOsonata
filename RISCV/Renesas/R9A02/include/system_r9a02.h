/**-------------------------------------------------------------------------
@file	system_r9a02.h

@brief	R9A02G021 system clock and module stop control

Clock sources, R9A02G021 User's Manual R01UH1036EJ0110, chapter 8:

	HOCO	24, 32 or 48 MHz internal RC, core clock source
	MOCO	8 MHz internal RC, core clock source
	EXTAL	1 to 20 MHz external clock input on P300, core clock source.
			P300 is also the cJTAG TCKC pin.
	LOCO	32768 Hz internal RC, machine timer clock (MTCLK)
	SOSC	32768 Hz crystal on XT1/XT2

There is no main crystal oscillator and no PLL. ICLK and PCLKB are the core
clock source divided by 1 to 64.

g_McuOsc.CoreOsc selects the core clock:
	OSC_TYPE_RC,   24000000, 32000000 or 48000000	HOCO
	OSC_TYPE_RC,   8000000							MOCO
	OSC_TYPE_TCXO, 1000000 to 20000000				EXTAL
OSC_TYPE_XTAL is not supported for the core clock.

g_McuOsc.LowPwrOsc selects the clock of RTC, TML32, UARTA and REMC
(OSMCR.WUTMMCK0):
	OSC_TYPE_RC,   32768	LOCO
	OSC_TYPE_XTAL, 32768	SOSC

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef __SYSTEM_R9A02_H__
#define __SYSTEM_R9A02_H__

#include <stdint.h>
#include <stdbool.h>

#include "coredev/system_core_clock.h"

// Sub-clock crystal start time. The manual gives 0.5 s typical (37.3).
// Overridable in the library build.
#ifndef R9A02_SOSC_STARTUP_US
#define R9A02_SOSC_STARTUP_US		1000000UL
#endif

// Poll count limit for oscillator and mode transition flags
#ifndef R9A02_STARTUP_TIMEOUT
#define R9A02_STARTUP_TIMEOUT		1000000UL
#endif

typedef enum __R9a02_Startup_Error {
	R9A02_STARTUP_OK = 0,
	R9A02_STARTUP_BAD_OSC,			//!< Core oscillator type or frequency not supported
	R9A02_STARTUP_BAD_LF_OSC,		//!< Low frequency oscillator type or frequency not supported
	R9A02_STARTUP_PROTECT,			//!< PRCR did not unlock
	R9A02_STARTUP_POWER_MODE,		//!< Operating power mode transition did not complete
	R9A02_STARTUP_HOCO,				//!< HOCO did not stop or stabilize
	R9A02_STARTUP_CLOCK,			//!< Clock source or divider write not taken
	R9A02_STARTUP_FLASH_WAIT,		//!< MEMWAIT or FLDWAITR write not taken
	R9A02_STARTUP_LF_CLOCK,			//!< LOCO, SOSC or OSMCR write not taken
} R9a02StartupError_t;

/// Module stop control registers, manual 10.2.2 to 10.2.5
typedef enum __R9a02_Mstp_Reg {
	R9A02_MSTPCR_A,
	R9A02_MSTPCR_B,
	R9A02_MSTPCR_C,
	R9A02_MSTPCR_D
} R9A02_MSTPCR;

#ifdef __cplusplus
extern "C" {
#endif

extern uint32_t SystemCoreClock;

/// Inspect this value if startup stops before main
extern volatile uint32_t g_R9a02StartupError;

void SystemInit(void);
void SystemCoreClockUpdate(void);

/// @brief	Release modules from the module-stop state.
///
/// @param	Reg		: Module stop control register
/// @param	Mask	: R9A02_MSTPCRx_xxx bits of the modules to start
void R9a02ModuleStart(R9A02_MSTPCR Reg, uint32_t Mask);

/// @brief	Put modules in the module-stop state.
///
/// @param	Reg		: Module stop control register
/// @param	Mask	: R9A02_MSTPCRx_xxx bits of the modules to stop
void R9a02ModuleStop(R9A02_MSTPCR Reg, uint32_t Mask);

/// @brief	Check whether a module is running.
///
/// @param	Reg		: Module stop control register
/// @param	Mask	: R9A02_MSTPCRx_xxx bits of the modules
///
/// @return	true if all modules in Mask are out of the module-stop state
bool R9a02ModuleRunning(R9A02_MSTPCR Reg, uint32_t Mask);

/// @brief	HOCO frequency, for a peripheral with its own clock (UARTA).
///
/// @return	24, 32 or 48 MHz, 0 when the HOCO is stopped or not stable
uint32_t R9a02HocoClockGet(void);

/// @brief	EXTAL clock frequency set by SystemCoreClockSelect.
///
/// @return	Frequency in Hz, 0 when the EXTAL input is off
uint32_t R9a02ExtalClockGet(void);

/// @brief	Enable or disable an ICU event as a Software Standby wake-up source.
///
/// The event must also be routed to the CPU (R9a02RegisterIntHandler).
///
/// @param	EvtId	: R9A02_EVT_xxx: PORT_IRQ0 to 7, IWDT, KEY, LVD1, LVD2, RTC,
/// 				  TML32, IIC0, IIC1, UARTA receive, COMP_DET0, COMP_DET1
/// @param	bEnable	: true to enable
///
/// @return	false if the event cannot end Software Standby
bool R9a02WakeupSet(uint8_t EvtId, bool bEnable);

/// @brief	Sleep or Software Standby until an interrupt.
///
/// Sleep stops the CPU only. Software Standby also stops the HOCO, MOCO,
/// EXTAL, flash and most peripherals, SOSC, LOCO, RTC and the wake-up
/// sources of R9a02WakeupSet keep running. The clock settings are the same
/// after the wake-up. The machine timer cannot end Software Standby.
///
/// @param	bStandby	: false for Sleep, true for Software Standby
///
/// @return	false when the mode could not be entered (flash P/E, operating
/// 			mode change in progress)
bool R9a02LowPowerEnter(bool bStandby);

/// @brief	Busy wait on the CPU cycle counter (mcycle).
///
/// Uses the ICLK frequency read from the clock registers, so it is valid
/// before and after a clock change, from any context.
///
/// @param	Us	: Microseconds to wait
void R9a02DelayUs(uint32_t Us);

#ifdef __cplusplus
}
#endif

#endif
