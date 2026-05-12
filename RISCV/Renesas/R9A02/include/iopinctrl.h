/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control for Renesas R9A02G021 (RISC-V Andes N22).

This file MUST be named iopinctrl.h regardless of target -- it is the
target-specific half of the IOsonata GPIO abstraction.  The other half,
iopincfg.h in coredev/, contains the portable data types (IOPINDIR, etc.)
and the declaration of IOPinConfig() (implemented in iopincfg_r9a02.c).

GPIO peripheral notes (R9A02G021 Group Hardware Manual / RA-family TRM):

  - Each port has a 32-bit PCNTR1 register: high 16 bits = direction
    (PDR), low 16 bits = output data (PODR).  RMW on PCNTR1 is the
    simple way to change one pin's direction; not atomic with respect
    to interrupts touching the same port.
  - Atomic set/clear is provided through PCNTR3: writing 1 to bit n of
    POSR (low 16 bits) sets pin n; writing 1 to bit n of PORR (high
    16 bits, accessed as the high half-word of PCNTR3) clears it.
    POSR/PORR are write-only; reading PCNTR3 returns 0.
  - For atomic toggle the only sane option is to read the current
    PODR shadow from PCNTR1, then issue a single POSR or PORR write.
    A direct XOR against PODR is *not* atomic across interrupts and
    is intentionally avoided here.
  - PIDR (input data) is in PCNTR2; reading PCNTR1.PODR reflects what
    is being DRIVEN, not the actual pad level.

The macros R9A02_PORT(n), PORT0, PORT1, ..., PORT9 and the PORT0_Type
register block (PCNTR1, PCNTR2, PCNTR3, plus the PODR/PDR/PIDR/POSR/PORR
half-word aliases) are defined in r9a02g021.h.

@author	Hoang Nguyen Hoan
@date	May. 11, 2026

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

#include "r9a02g021.h"
#include "coredev/iopincfg.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Set GPIO pin direction without altering other configuration.
 *
 * Fast direction switch through PCNTR1.PDR.  Does not touch PFS, so pull
 * resistors and drive strength are preserved.  Use IOPinConfig() (in
 * iopincfg_r9a02.c) for full pin setup the first time.
 *
 * RMW on the 16-bit PDR half-word; not atomic across interrupts that may
 * also be reconfiguring direction on the same port.
 *
 * @param	PortNo  : Port number (0..R9A02_MAX_PORT-1)
 * @param	PinNo   : Pin number (0..R9A02_PINS_PER_PORT-1)
 * @param	Dir     : IOPINDIR_OUTPUT or IOPINDIR_INPUT
 */
static inline __attribute__((always_inline))
void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	uint16_t mask = (uint16_t)(1U << (unsigned)PinNo);

	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->PDR = (uint16_t)(reg->PDR | mask);
	}
	else
	{
		reg->PDR = (uint16_t)(reg->PDR & ~mask);
	}
}

/**
 * @brief	Read current pin state from the pad input register (PIDR).
 *
 * Reflects the actual pad level, not what the output latch is driving.
 *
 * @param	PortNo  : Port number
 * @param	PinNo   : Pin number
 *
 * @return	1 if the pin is high, 0 if low
 */
static inline __attribute__((always_inline))
int IOPinRead(int PortNo, int PinNo)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	return (int)((reg->PIDR >> (unsigned)PinNo) & 1U);
}

/**
 * @brief	Drive pin to logic 1 (high).
 *
 * Uses POSR (write-1-to-set) -- atomic single-pin operation, safe from
 * ISR context concurrent with other pins on the same port.
 *
 * @param	PortNo  : Port number
 * @param	PinNo   : Pin number
 */
static inline __attribute__((always_inline))
void IOPinSet(int PortNo, int PinNo)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	reg->POSR = (uint16_t)(1U << (unsigned)PinNo);
}

/**
 * @brief	Drive pin to logic 0 (low).
 *
 * Uses PORR (write-1-to-clear) -- atomic single-pin operation, safe from
 * ISR context concurrent with other pins on the same port.
 *
 * @param	PortNo  : Port number
 * @param	PinNo   : Pin number
 */
static inline __attribute__((always_inline))
void IOPinClear(int PortNo, int PinNo)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	reg->PORR = (uint16_t)(1U << (unsigned)PinNo);
}

/**
 * @brief	Toggle pin output state.
 *
 * Reads PODR (output latch in PCNTR1, NOT the input pad in PCNTR2) to see
 * what is currently being driven, then issues a single atomic POSR or PORR
 * write.  Only the one bit for PinNo is touched, so concurrent access to
 * other pins from interrupt context is safe.
 *
 * Note: this is not atomic in the read-decide-write sense -- if another
 * context toggles the same pin between the read and the write, the result
 * is undefined.  RA-family IOPORT has no single-instruction XOR primitive,
 * so this is the best you can do without disabling interrupts.
 *
 * @param	PortNo  : Port number
 * @param	PinNo   : Pin number
 */
static inline __attribute__((always_inline))
void IOPinToggle(int PortNo, int PinNo)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	uint16_t mask = (uint16_t)(1U << (unsigned)PinNo);

	if (reg->PODR & mask)
	{
		reg->PORR = mask;       /* currently high -- drive low */
	}
	else
	{
		reg->POSR = mask;       /* currently low  -- drive high */
	}
}

/**
 * @brief	Read all pins on a port (input data).
 *
 * Returns PIDR (16 bits, sampled pad state).  Bits beyond the populated
 * pins on a given package read as 0.
 *
 * @param	PortNo  : Port number
 *
 * @return	Bit field of input pin states
 */
static inline __attribute__((always_inline))
uint32_t IOPinReadPort(int PortNo)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	return (uint32_t)reg->PIDR;
}

/**
 * @brief	Write the entire output port at once.
 *
 * Drives every output pin in PODR to the level in Data.  Pins configured as
 * inputs are unaffected at the pad regardless of what is written.  Bits
 * beyond pin 15 are ignored (PODR is a 16-bit register).
 *
 * IMPORTANT: this is a "whole-port" semantic.  Pins NOT set in Data are
 * driven LOW, including pins not currently meant to change.  If you only
 * want to touch one pin atomically, use IOPinSet / IOPinClear instead --
 * those go through POSR/PORR and leave the rest of the port alone.
 *
 * @param	PortNo  : Port number
 * @param	Data    : Desired pin state bit-field (bit n = desired level on pin n)
 */
static inline __attribute__((always_inline))
void IOPinWritePort(int PortNo, uint32_t Data)
{
	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	reg->PODR = (uint16_t)(Data & 0xFFFFU);
}

#ifdef __cplusplus
}
#endif

#endif	// __IOPINCTRL_H__
