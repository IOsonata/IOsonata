/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control for Espressif ESP32 RISC-V family

Common to all Espressif RISC-V targets (ESP32-C3, C5, C6, H2, H4).
GPIO base address 0x60004000 and W1TS/W1TC/ENABLE/IN offsets are
identical across the entire family; only the valid pin range differs:
  C3: GPIO[0..21]   C5: GPIO[0..28]   C6: GPIO[0..29]
  H2: GPIO[0..18]   H4: GPIO[0..35]

This file must be named iopinctrl.h regardless of target -- it is the
target-specific half of the IOsonata GPIO abstraction.  The other half,
iopincfg.h, contains the portable data types (IOPINDIR, etc.) and the
declaration of IOPinCfg().

ESP32-C6 GPIO peripheral notes (TRM Rev 0.6, Section 5):

  - Single port -- PortNo is always 0.  PinNo in [0, 29] (30 usable GPIOs).
  - Two atomically-safe register pairs for output:
      GPIO_OUT_W1TS_REG  (0x60004008) -- write 1-to-set,  no RMW needed
      GPIO_OUT_W1TC_REG  (0x6000400C) -- write 1-to-clear, no RMW needed
  - Output enable:
      GPIO_ENABLE_W1TS_REG (0x60004024)
      GPIO_ENABLE_W1TC_REG (0x60004028)
  - Input read:
      GPIO_IN_REG (0x6000403C) -- current sampled pad state
  - Current output latch (readable shadow):
      GPIO_OUT_REG (0x60004004)

IOPinToggle reads GPIO_OUT_REG to determine current driven level, then
writes to W1TS or W1TC -- avoiding a non-atomic RMW on the output register.

@author	Hoang Nguyen Hoan
@date	Mar. 5, 2026

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>

#include "coredev/iopincfg.h"

/*---------------------------------------------------------------------------
 * Espressif RISC-V GPIO register map (base 0x60004000)
 * Reference: ESP32-C6 TRM / ESP32-C3 TRM / ESP32-H2 TRM — Section "GPIO Register Summary" "Register Summary"
 *---------------------------------------------------------------------------*/
#define ESP32_GPIO_BASE           0x60004000UL

/** Output latch shadow (readable -- reflects what we are driving). */
#define GPIO_OUT_REG                (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x004U))

/** Atomic write-1-to-set: drives selected pins high. */
#define GPIO_OUT_W1TS_REG           (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x008U))

/** Atomic write-1-to-clear: drives selected pins low. */
#define GPIO_OUT_W1TC_REG           (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x00CU))

/** Output-enable register: bit n = 1 means GPIO n is an output. */
#define GPIO_ENABLE_REG             (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x020U))

/** Write-1-to-set output-enable (enable output driver). */
#define GPIO_ENABLE_W1TS_REG        (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x024U))

/** Write-1-to-clear output-enable (tri-state / input mode). */
#define GPIO_ENABLE_W1TC_REG        (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x028U))

/** Input register: current sampled pad level for all GPIOs. */
#define GPIO_IN_REG                 (*(volatile uint32_t *)(ESP32_GPIO_BASE + 0x03CU))

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Set GPIO pin direction without altering other configuration.
 *
 * Fast direction switch -- does not touch IO_MUX, pull resistors, or drive
 * strength.  Use IOPinCfg() (iopincfg_esp32c6.c) for full pin setup.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 * @param	PinNo   : GPIO pin number [0..29]
 * @param	Dir     : IOPINDIR_OUTPUT or IOPINDIR_INPUT
 */
static inline __attribute__((always_inline))
void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	(void)PortNo;
	uint32_t mask = 1UL << (unsigned)PinNo;

	if (Dir == IOPINDIR_OUTPUT)
	{
		GPIO_ENABLE_W1TS_REG = mask;
	}
	else
	{
		GPIO_ENABLE_W1TC_REG = mask;
	}
}

/**
 * @brief	Read current pin state.
 *
 * Reads GPIO_IN_REG which reflects the actual pad level.  For output pins
 * this normally matches the driven value; for input pins it reflects the
 * externally applied signal.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 * @param	PinNo   : GPIO pin number [0..29]
 *
 * @return	1 if the pin is high, 0 if low
 */
static inline __attribute__((always_inline))
int IOPinRead(int PortNo, int PinNo)
{
	(void)PortNo;
	return (int)((GPIO_IN_REG >> (unsigned)PinNo) & 1UL);
}

/**
 * @brief	Drive pin to logic 1 (high).
 *
 * Uses GPIO_OUT_W1TS_REG -- fully atomic, safe from ISR context.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 * @param	PinNo   : GPIO pin number [0..29]
 */
static inline __attribute__((always_inline))
void IOPinSet(int PortNo, int PinNo)
{
	(void)PortNo;
	GPIO_OUT_W1TS_REG = 1UL << (unsigned)PinNo;
}

/**
 * @brief	Drive pin to logic 0 (low).
 *
 * Uses GPIO_OUT_W1TC_REG -- fully atomic, safe from ISR context.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 * @param	PinNo   : GPIO pin number [0..29]
 */
static inline __attribute__((always_inline))
void IOPinClear(int PortNo, int PinNo)
{
	(void)PortNo;
	GPIO_OUT_W1TC_REG = 1UL << (unsigned)PinNo;
}

/**
 * @brief	Toggle pin output state.
 *
 * Reads GPIO_OUT_REG (output latch, not input pad) to determine what is
 * currently being driven, then issues a single atomic W1TS or W1TC write.
 * Only the one bit for PinNo is touched, so concurrent access to other pins
 * from interrupt context is safe.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 * @param	PinNo   : GPIO pin number [0..29]
 */
static inline __attribute__((always_inline))
void IOPinToggle(int PortNo, int PinNo)
{
	(void)PortNo;
	uint32_t mask = 1UL << (unsigned)PinNo;

	if (GPIO_OUT_REG & mask)
	{
		GPIO_OUT_W1TC_REG = mask;	/* currently high -- drive low */
	}
	else
	{
		GPIO_OUT_W1TS_REG = mask;	/* currently low  -- drive high */
	}
}

/**
 * @brief	Read all GPIO pins simultaneously.
 *
 * Returns GPIO_IN_REG (bits [29:0]); upper bits are reserved and read as 0.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 *
 * @return	Bit-field: bit n = current state of GPIO n
 */
static inline __attribute__((always_inline))
uint32_t IOPinReadPort(int PortNo)
{
	(void)PortNo;
	return GPIO_IN_REG;
}

/**
 * @brief	Write an entire port value atomically.
 *
 * Uses W1TS to set bits that are 1 in Data and W1TC to clear bits that
 * are 0 in Data.  Both writes happen in two instructions -- as close to
 * atomic as possible without a hardware shadow-write register.
 *
 * Input-only pins (GPIO_ENABLE_REG bit = 0) are unaffected at the pad.
 *
 * @param	PortNo  : Port number (always 0 on ESP32-C6, ignored)
 * @param	Data    : Desired pin state bit-field (bit n = desired GPIO n level)
 */
static inline __attribute__((always_inline))
void IOPinWritePort(int PortNo, uint32_t Data)
{
	(void)PortNo;
	Data &= 0x3FFFFFFFUL;			/* mask to valid GPIO[29:0] range */
	GPIO_OUT_W1TS_REG = Data;
	GPIO_OUT_W1TC_REG = (~Data) & 0x3FFFFFFFUL;
}

#ifdef __cplusplus
}
#endif

#endif	/* __IOPINCTRL_H__ */
