/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control for Espressif ESP32 RISC-V family

Common to all Espressif RISC-V targets currently supported by IOsonata
(ESP32-C3, C5, C6, with H2/H4 to follow).  Driver code in this file
goes through the chip-aware raw register macros declared in
esp32xx_gpio.h, so the same source compiles for every chip in the
family even though the register offsets differ on C5.  Per-chip pad
ranges are provided by the chip header (ESP32_GPIO_PIN_COUNT,
ESP32_GPIO_PIN_MAX).

This file must be named iopinctrl.h regardless of target -- it is the
target-specific half of the IOsonata GPIO abstraction.  The other half,
iopincfg.h, contains the portable data types (IOPINDIR, etc.) and the
declaration of IOPinConfig().

GPIO peripheral notes (TRMs and ESP-IDF v5.3 register headers):

  - Single port -- PortNo is always 0.  PinNo in [0, ESP32_GPIO_PIN_MAX].
  - Two atomically-safe register pairs for output:
      ESP32_GPIO_OUT_W1TS_REG32   write 1-to-set,  no RMW needed
      ESP32_GPIO_OUT_W1TC_REG32   write 1-to-clear, no RMW needed
  - Output enable:
      ESP32_GPIO_ENABLE_W1TS_REG32
      ESP32_GPIO_ENABLE_W1TC_REG32
  - Input read:
      ESP32_GPIO_IN_REG32         current sampled pad state
  - Current output latch (readable shadow):
      ESP32_GPIO_OUT_REG32

IOPinToggle reads ESP32_GPIO_OUT_REG32 to determine current driven level,
then writes to W1TS or W1TC -- avoiding a non-atomic RMW on the output
register.

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

#include "esp32xx.h"
#include "esp32xx_gpio.h"
#include "coredev/iopincfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Set GPIO pin direction without altering other configuration.
 *
 * Fast direction switch -- does not touch IO_MUX, pull resistors, or drive
 * strength.  Use IOPinConfig() (iopincfg_esp32.c) for full pin setup.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 * @param	PinNo   : GPIO pin number, 0 .. ESP32_GPIO_PIN_MAX
 * @param	Dir     : IOPINDIR_OUTPUT or IOPINDIR_INPUT
 */
static inline __attribute__((always_inline))
void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	(void)PortNo;
	uint32_t mask = 1UL << (unsigned)PinNo;

	if (Dir == IOPINDIR_OUTPUT)
	{
		ESP32_GPIO_ENABLE_W1TS_REG32 = mask;
	}
	else
	{
		ESP32_GPIO_ENABLE_W1TC_REG32 = mask;
	}
}

/**
 * @brief	Read current pin state.
 *
 * Reads ESP32_GPIO_IN_REG32 which reflects the actual pad level.  For output
 * pins this normally matches the driven value; for input pins it reflects
 * the externally applied signal.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 * @param	PinNo   : GPIO pin number, 0 .. ESP32_GPIO_PIN_MAX
 *
 * @return	1 if the pin is high, 0 if low
 */
static inline __attribute__((always_inline))
int IOPinRead(int PortNo, int PinNo)
{
	(void)PortNo;
	return (int)((ESP32_GPIO_IN_REG32 >> (unsigned)PinNo) & 1UL);
}

/**
 * @brief	Drive pin to logic 1 (high).
 *
 * Uses ESP32_GPIO_OUT_W1TS_REG32 -- fully atomic, safe from ISR context.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 * @param	PinNo   : GPIO pin number, 0 .. ESP32_GPIO_PIN_MAX
 */
static inline __attribute__((always_inline))
void IOPinSet(int PortNo, int PinNo)
{
	(void)PortNo;
	ESP32_GPIO_OUT_W1TS_REG32 = 1UL << (unsigned)PinNo;
}

/**
 * @brief	Drive pin to logic 0 (low).
 *
 * Uses ESP32_GPIO_OUT_W1TC_REG32 -- fully atomic, safe from ISR context.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 * @param	PinNo   : GPIO pin number, 0 .. ESP32_GPIO_PIN_MAX
 */
static inline __attribute__((always_inline))
void IOPinClear(int PortNo, int PinNo)
{
	(void)PortNo;
	ESP32_GPIO_OUT_W1TC_REG32 = 1UL << (unsigned)PinNo;
}

/**
 * @brief	Toggle pin output state.
 *
 * Reads ESP32_GPIO_OUT_REG32 (output latch, not input pad) to determine
 * what is currently being driven, then issues a single atomic W1TS or
 * W1TC write.  Only the one bit for PinNo is touched, so concurrent
 * access to other pins from interrupt context is safe.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 * @param	PinNo   : GPIO pin number, 0 .. ESP32_GPIO_PIN_MAX
 */
static inline __attribute__((always_inline))
void IOPinToggle(int PortNo, int PinNo)
{
	(void)PortNo;
	uint32_t mask = 1UL << (unsigned)PinNo;

	if (ESP32_GPIO_OUT_REG32 & mask)
	{
		ESP32_GPIO_OUT_W1TC_REG32 = mask;  // currently high -- drive low
	}
	else
	{
		ESP32_GPIO_OUT_W1TS_REG32 = mask;  // currently low  -- drive high
	}
}

/**
 * @brief	Read all GPIO pins simultaneously.
 *
 * Returns ESP32_GPIO_IN_REG32; bits beyond ESP32_GPIO_PIN_MAX are reserved
 * and read as 0.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 *
 * @return	Bit-field: bit n = current state of GPIO n
 */
static inline __attribute__((always_inline))
uint32_t IOPinReadPort(int PortNo)
{
	(void)PortNo;
	return ESP32_GPIO_IN_REG32;
}

/**
 * @brief	Write an entire port value.
 *
 * Drives every output pin in the valid range to the level specified
 * by the corresponding bit in @p Data.  Bits beyond ESP32_GPIO_PIN_COUNT
 * are masked off.
 *
 * IMPORTANT: this is a "whole-port" semantic.  Output pins NOT set in
 * @p Data are driven LOW, including pins not currently meant to change.
 * Pins configured as inputs (ESP32_GPIO_ENABLE bit clear) are unaffected
 * at the pad regardless.
 *
 * The two W1TS/W1TC writes are issued back-to-back but are not a single
 * atomic transaction; an interrupt between them can briefly observe an
 * inconsistent port state.
 *
 * @param	PortNo  : Port number (always 0 on Espressif RISC-V, ignored)
 * @param	Data    : Desired pin state bit-field (bit n = desired GPIO n level)
 */
static inline __attribute__((always_inline))
void IOPinWritePort(int PortNo, uint32_t Data)
{
	(void)PortNo;
	uint32_t validMask = (ESP32_GPIO_PIN_COUNT >= 32U)
	                   ? 0xFFFFFFFFUL
	                   : ((1UL << ESP32_GPIO_PIN_COUNT) - 1UL);
	Data &= validMask;
	ESP32_GPIO_OUT_W1TS_REG32 = Data;
	ESP32_GPIO_OUT_W1TC_REG32 = (~Data) & validMask;
}

#ifdef __cplusplus
}
#endif

#endif // __IOPINCTRL_H__
