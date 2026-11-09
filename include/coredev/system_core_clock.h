/**-------------------------------------------------------------------------
@file	system_core_clock.h

@brief	Contains generic system core definitions: clocks, power settings and
		power functions of the MCU

These functions can be implemented per target.

The power functions are those of the MCU itself, not of an external power
management chip (see pwrmgnt/pwrmgnt.h for those):
- reset reason;
- power off, the lowest power mode, left only through a reset;
- retention registers, kept through power off and software resets;
- RAM retention in power off.

Power off stops the MCU until a wake up source resets it. The wake up
sources are set with their own drivers before power off, for example
IOPinSetSense for a pin or the analog comparator. The reset reason of the
next start tells which one woke the MCU.

On a multi-core MCU each core has its own library. Its functions act for the
core that calls them: its clock, its reset reason, its retention registers
and RAM. Power off is done only by the core that controls it.

A power function an MCU does not have returns false or 0.

@author	Hoang Nguyen Hoan
@date	Aug. 30, 2017

@license

Copyright (c) 2017, I-SYST inc., all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/

#ifndef __SYSTEM_CORE_CLOCK_H__
#define __SYSTEM_CORE_CLOCK_H__

#ifndef __cplusplus
#include <stdbool.h>
#endif

#include <stdint.h>

/// Reset reasons, ORed, returned by SystemResetReason
#define SYSTEM_RESET_POWERON		(1UL << 0)	//!< Power on or brown out
#define SYSTEM_RESET_PIN			(1UL << 1)	//!< Reset pin
#define SYSTEM_RESET_WDT			(1UL << 2)	//!< Watchdog
#define SYSTEM_RESET_SOFT			(1UL << 3)	//!< Software reset request
#define SYSTEM_RESET_LOCKUP			(1UL << 4)	//!< CPU lockup
#define SYSTEM_RESET_DEBUG			(1UL << 5)	//!< Debug interface
#define SYSTEM_RESET_WAKE_PIN		(1UL << 8)	//!< Wake up from power off by a pin
#define SYSTEM_RESET_WAKE_COMP		(1UL << 9)	//!< Wake up from power off by the analog comparator
#define SYSTEM_RESET_WAKE_NFC		(1UL << 10)	//!< Wake up from power off by an NFC field
#define SYSTEM_RESET_WAKE_USB		(1UL << 11)	//!< Wake up from power off by USB power
#define SYSTEM_RESET_WAKE_TIMER		(1UL << 12)	//!< Wake up from power off by a timer or the RTC
#define SYSTEM_RESET_WAKE_MSK		(0x1FUL << 8)	//!< All wake up from power off reasons

/// DC/DC regulator stages, ORed in McuOsc_t DcDc
#define SYSTEM_DCDC_MAIN			(1U << 0)	//!< Stage supplying the core
#define SYSTEM_DCDC_HIGHVOLT		(1U << 1)	//!< High voltage input stage, on MCU with one

#pragma pack(push, 4)

///
/// Enum defining clock oscillator types
///
/// Many integrated circuits allow the section of different type of oscillator to use as clock
/// source. This enum defines commonly used types.
typedef enum __Osc_Type {
	OSC_TYPE_RC,	//!< internal RC
	OSC_TYPE_XTAL,	//!< external crystal
	OSC_TYPE_TCXO,	//!< external oscillator
} OSC_TYPE;

///
/// Oscillator descriptor
typedef struct __Oscillator_Desc {
	OSC_TYPE Type;		//!< Oscillator type RC, crystal, oscillator
	uint32_t Freq;		//!< Frequency in Hz
	uint32_t Accuracy;	//!< Accuracy in PPM
	uint32_t LoadCap;	//!< Crystal's load capacitance in 10 x pF. SystemInit could not handle FPU
	 	 	 	 	 	//!< therefore cap value is x10 to include the first decimal number
} OscDesc_t;

///
/// This structure defines the the MCU oscillators and the regulator setting
///
/// Most MCU has 2 oscillator.
/// The core oscillator often referred to as high frequency or main or core
/// The low frequency is often a 32768Hz oscillator for realtime clock or low power clock
///
/// DcDc selects the regulator stages running in DC/DC mode, the others use the
/// linear regulator. The DC/DC regulator needs its inductor on the board. A stage
/// enabled on a board without one stops the MCU from running properly. 0 keeps
/// the linear regulators, the reset setting. SystemInit applies it.
///
/// Modern compiler such as the GNU GCC allow declaration of weak type global variable
/// that can be overloaded.  In the library system startup code contains a weak global
/// variable '__WEAK MCU_OSC g_McuOsc' that define a default oscillator.  Firmware can
/// overload this variable in the main firmware with the oscillator selection for the
/// target board.
///
/// A generic default value is set in system_xxx.c for the target MCU family
typedef struct __Mcu_Osc {
	OscDesc_t CoreOsc;		//!< Core high frequency oscillator descriptor
	OscDesc_t LowPwrOsc;	//!< Low power, low frequency oscillator descriptor
	bool bUSBClk;			//!< Enable USB clock support
	uint8_t DcDc;			//!< DC/DC regulator stages, SYSTEM_DCDC_xxx ORed. 0 - linear regulators
} McuOsc_t;

//typedef McuOsc_t	MCU_OSC;

#pragma pack(pop)

extern McuOsc_t g_McuOsc;

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Get system low frequency oscillator type
 *
 * @return	Return oscillator type either internal RC or external crystal/osc
 */
static inline OSC_TYPE GetLowFreqOscType(void) { return g_McuOsc.LowPwrOsc.Type; }

/**
 * @brief	Get system low frequency oscillator frequency in Hz
 *
 * @return	Return oscillator frequency in Hz
 */
static inline uint32_t GetLowFreqOscFreq(void) { return g_McuOsc.LowPwrOsc.Freq; }

/**
 * @brief	Get system low frequency oscillator accuracy in PPM
 *
 * @return	Return oscillator accuracy in PPM
 */
static inline uint32_t GetLowFreqOscAcc(void) { return g_McuOsc.LowPwrOsc.Accuracy; }

/**
 * @brief	Get systtem low frequency crystal load capacitance
 *
 * @return	Capacitance value in pF
 */
static inline uint32_t GetLowFreqLoadCap(void) { return g_McuOsc.LowPwrOsc.LoadCap; }

/**
 * @brief	Get system low frequency oscillator descriptor
 *
 * @return	Return pointer to oscillator descriptor
 */
static inline const OscDesc_t *GetLowFreqOscDesc(void) { return &g_McuOsc.LowPwrOsc; }

/**
 * @brief	Get system core high frequency oscillator type
 *
 * @return	Return oscillator type either internal RC or external crystal/osc
 */
static inline OSC_TYPE GetHighFreqOscType(void) { return g_McuOsc.CoreOsc.Type; }

/**
 * @brief	Get system core high frequency oscillator frequency in Hz
 *
 * @return	Return oscillator frequency in Hz
 */
static inline uint32_t GetHighFreqOscFreq(void) { return g_McuOsc.CoreOsc.Freq; }

/**
 * @brief	Get system core high frequency oscillator accuracy in PPM
 *
 * @return	Return oscillator accuracy in PPM
 */
static inline uint32_t GetHighFreqOscAcc(void) { return g_McuOsc.CoreOsc.Accuracy; }

/**
 * @brief	Get systtem low frequency crystal load capacitance
 *
 * @return	Capacitance value in pF
 */
static inline uint32_t GetHighFreqLoadCap(void) { return g_McuOsc.CoreOsc.LoadCap; }

/**
 * @brief	Get system core high frequency oscillator descriptor
 *
 * @return	Return pointer to oscillator descriptor
 */
static inline const OscDesc_t *GetHighFreqOscDesc(void) { return &g_McuOsc.CoreOsc; }

/**
 * @brief	Select core clock oscillator type
 *
 * On an MCU whose core always runs from its internal clock and uses the
 * crystal only for accuracy, starting it on demand for the radio, USB or a
 * timer, a call with OSC_TYPE_XTAL or OSC_TYPE_TCXO is one request for the
 * crystal and a call with OSC_TYPE_RC releases one. The requests of the
 * drivers and of the application are counted: the crystal starts on the
 * first request and stops when the last one is released. Each request is
 * matched by one release. A request returns once the crystal runs, false if
 * it did not start.
 *
 * @param	ClkSrc : Clock source selection
 *						OSC_TYPE_RC - Internal RC
 *						OSC_TYPE_XTAL - External crystal
 *						OSC_TYPE_CTXO -	External oscillator
 * @param	OscFreq : Oscillator frequency
 *
 * @return	true - success
 *
 */
bool SystemCoreClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);

/**
 * @brief	Select low frequency clock oscillator type
 *
 * @param	ClkSrc : Clock source selection
 *						OSC_TYPE_RC - Internal RC
 *						OSC_TYPE_XTAL - External crystal
 *						OSC_TYPE_CTXO -	External oscillator
 * @param	OscFreq : Oscillator frequency
 *
 * @return	true - success
 *
 */
bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq);

/**
 * @brief	Get core clock frequency
 *
 * @return	Core frequency in Hz.
 */
uint32_t SystemCoreClockGet(void);

/**
 * @brief	Get peripheral clock frequency (PCLK)
 *
 * Most often the PCLK numbering starts from 1 (PCLK1, PCLK2,...).
 * Therefore the clock Idx parameter = 0 indicate PCK1, 1 indicate PCLK2
 *
 * @param	Idx : Zero based peripheral clock number. Many processors can
 * 				  have more than 1 peripheral clock settings.
 *
 * @return	Peripheral clock frequency in Hz.
 * 			0 - Bad clock number
 */
uint32_t SystemPeriphClockGet(int Idx);

/**
 * @brief	Set peripheral clock (PCLK) frequency
 *
 * Most often the PCLK numbering starts from 1 (PCLK1, PCLK2,...).
 * Therefore the clock Idx parameter = 0 indicate PCK1, 1 indicate PCLK2
 *
 * @param	Idx  : Zero based peripheral clock number. Many processors can
 * 				   have more than 1 peripheral clock settings.
 * @param	Freq : Clock frequency in Hz.
 *
 * @return	Actual frequency set in Hz.
 * 			0 - Failed
 */
uint32_t SystemPeriphClockSet(int Idx, uint32_t Freq);

/**
 * @brief	Initialize oscillator
 *
 * This function is MCU dependent. It is call to configure the oscillators.
 * For example set internal load capacitance value
 *
 */
void SystemOscInit(void);

/**
 * @brief	Get RAM start address
 *
 * @return	Pointer to start of RAM
 */
void *SystemRamStart(void);

/**
 * @brief	Reasons of the last reset
 *
 * Where the hardware adds the reasons of successive resets until they are
 * cleared, the first call reads and clears them, later calls return the same
 * value.
 *
 * @return	SYSTEM_RESET_xxx reasons, ORed
 */
uint32_t SystemResetReason(void);

/**
 * @brief	Power off
 *
 * Enters the lowest power mode. Only a reset leaves it: a wake up source set
 * before the call, the reset pin or a power cycle. RAM is lost except for
 * the parts kept with SystemRamRetain. The function does not return, except
 * on a core that does not control power off.
 */
void SystemPowerOff(void);

/**
 * @brief	Number of retention registers
 *
 * Retention registers keep their value through power off and software
 * resets, not through a power cycle.
 *
 * @return	Number of registers, 0 for none
 */
int SystemRetRegCount(void);

/**
 * @brief	Read a retention register
 *
 * @param	Idx : Register index, 0 to SystemRetRegCount() - 1
 *
 * @return	Register value, 0 for an invalid index
 */
uint32_t SystemRetRegRead(int Idx);

/**
 * @brief	Write a retention register
 *
 * Bits above the register width are dropped.
 *
 * @param	Idx : Register index, 0 to SystemRetRegCount() - 1
 * @param	Val : Value to keep
 *
 * @return	true - success
 */
bool SystemRetRegWrite(int Idx, uint32_t Val);

/**
 * @brief	Keep or drop a RAM range in power off
 *
 * The hardware works on RAM sections, every section the range touches is
 * changed.
 *
 * @param	Addr	: Start address
 * @param	Size	: Size in bytes
 * @param	bRetain	: true - keep the content in power off
 *
 * @return	true - success, false - the range is not RAM
 */
bool SystemRamRetain(uintptr_t Addr, uint32_t Size, bool bRetain);

#ifdef __cplusplus
}
#endif

#endif // __SYSTEM_CORE_CLOCK_H__
