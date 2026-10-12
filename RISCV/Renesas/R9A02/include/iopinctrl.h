/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	General I/O pin control for the Renesas R9A02G021

Target half of the IOsonata GPIO abstraction, coredev/iopincfg.h holds the
portable half. IOPinConfig and the pin interrupt functions are in
iopincfg_r9a02.c.

I/O port registers, R9A02G021 User's Manual R01UH1036EJ0110, 16.2:
PCNTR1 is PDR [15:0] and PODR [31:16], PCNTR2 is PIDR [15:0], PCNTR3 is
POSR [15:0] and PORR [31:16]. Writing 1 to a POSR or PORR bit sets or
clears that pin only, so IOPinSet and IOPinClear are atomic.

Pin numbering is the MCU port and pin: P302 is port 3, pin 2. The pins that
exist depend on the package, R9A02_PACKAGE_PINS selects it (48, 32, 24 or
16, default 48 for the FPB-R9A02G021). P200 is input only. Functions here
ignore a pin that does not exist.

@author	Hoang Nguyen Hoan
@date	May. 11, 2026

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>
#include <stdbool.h>

#include "r9a02g021.h"
#include "coredev/iopincfg.h"
#include "coredev/interrupt.h"

#ifndef R9A02_PACKAGE_PINS
#define R9A02_PACKAGE_PINS		48
#endif

// Pins of each port by package, manual tables 16.6 to 16.10
static inline uint32_t R9a02PortMask(int PortNo)
{
#if R9A02_PACKAGE_PINS == 48
	static const uint16_t masks[R9A02_PORT_COUNT] = {
		R9A02_PORT0_PIN_MSK, R9A02_PORT1_PIN_MSK, R9A02_PORT2_PIN_MSK, R9A02_PORT3_PIN_MSK, R9A02_PORT4_PIN_MSK
	};
#elif R9A02_PACKAGE_PINS == 32
	static const uint16_t masks[R9A02_PORT_COUNT] = { 0x00CF, 0x0FFF, 0x000F, 0x000F, 0 };
#elif R9A02_PACKAGE_PINS == 24
	static const uint16_t masks[R9A02_PORT_COUNT] = { 0x000F, 0x00FF, 0x0009, 0x000F, 0 };
#elif R9A02_PACKAGE_PINS == 16
	static const uint16_t masks[R9A02_PORT_COUNT] = { 0x000F, 0x0081, 0x0009, 0x000F, 0 };
#else
#error "R9A02_PACKAGE_PINS must be 48, 32, 24 or 16"
#endif

	return (unsigned)PortNo < R9A02_PORT_COUNT ? masks[PortNo] : 0U;
}

static inline bool R9a02PinValid(int PortNo, int PinNo)
{
	return (unsigned)PinNo < R9A02_PORT_PIN_COUNT && (R9a02PortMask(PortNo) & (1UL << (unsigned)PinNo)) != 0U;
}

// Pins that can drive an output, P200 is input only
static inline uint32_t R9a02OutputMask(int PortNo)
{
	uint32_t mask = R9a02PortMask(PortNo);

	return PortNo == 2 ? mask & ~1UL : mask;
}

/**
 * @brief	Set GPIO pin direction.
 *
 * Writes the PDR half of PCNTR1 only, pull-up and function settings stay.
 * Configure the pin with IOPinConfig first.
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 * @param	Dir		: IOPINDIR_INPUT or IOPINDIR_OUTPUT
 */
static inline __attribute__((always_inline)) void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	if (!R9a02PinValid(PortNo, PinNo) || (Dir != IOPINDIR_INPUT && Dir != IOPINDIR_OUTPUT))
	{
		return;
	}

	uint16_t bit = (uint16_t)(1U << (unsigned)PinNo);

	if (Dir == IOPINDIR_OUTPUT && (R9a02OutputMask(PortNo) & bit) == 0U)
	{
		return;
	}

	R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
	uintptr_t state = DisableInterrupt();

	if (Dir == IOPINDIR_OUTPUT)
	{
		reg->PDR = (uint16_t)(reg->PDR | bit);
	}
	else
	{
		reg->PDR = (uint16_t)(reg->PDR & ~bit);
	}

	EnableInterrupt(state);
}

/**
 * @brief	Read pin state (PIDR).
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 *
 * @return	1 if the pin is high, 0 if low
 */
static inline __attribute__((always_inline)) int IOPinRead(int PortNo, int PinNo)
{
	if (!R9a02PinValid(PortNo, PinNo))
	{
		return 0;
	}

	return (int)((R9A02_PORT(PortNo)->PCNTR2 >> (unsigned)PinNo) & 1U);
}

/**
 * @brief	Drive pin high through POSR.
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo)
{
	if (R9a02PinValid(PortNo, PinNo))
	{
		uint32_t bit = (1UL << (unsigned)PinNo) & R9a02OutputMask(PortNo);

		if (bit)
		{
			R9A02_PORT(PortNo)->PCNTR3 = bit;
		}
	}
}

/**
 * @brief	Drive pin low through PORR.
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo)
{
	if (R9a02PinValid(PortNo, PinNo))
	{
		uint32_t bit = (1UL << (unsigned)PinNo) & R9a02OutputMask(PortNo);

		if (bit)
		{
			R9A02_PORT(PortNo)->PCNTR3 = bit << 16;
		}
	}
}

/**
 * @brief	Toggle pin output.
 *
 * Reads PODR and writes POSR or PORR with interrupts disabled.
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 */
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo)
{
	if (!R9a02PinValid(PortNo, PinNo))
	{
		return;
	}

	uint32_t bit = (1UL << (unsigned)PinNo) & R9a02OutputMask(PortNo);

	if (bit)
	{
		R9A02_PORT_Type *reg = R9A02_PORT(PortNo);
		uintptr_t state = DisableInterrupt();

		reg->PCNTR3 = ((reg->PCNTR1 >> 16) & bit) ? bit << 16 : bit;
		EnableInterrupt(state);
	}
}

/**
 * @brief	Read all pins of a port (PIDR).
 *
 * @param	PortNo	: Port number
 *
 * @return	Pin states, bit n is pin n. Pins that do not exist read 0.
 */
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo)
{
	uint32_t mask = R9a02PortMask(PortNo);

	return mask ? R9A02_PORT(PortNo)->PCNTR2 & mask : 0U;
}

/**
 * @brief	Write all output pins of a port.
 *
 * Every output pin of the port takes the level of its bit in Data, through
 * POSR and PORR in one write.
 *
 * @param	PortNo	: Port number
 * @param	Data	: Pin levels, bit n is pin n
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data)
{
	uint32_t mask = R9a02OutputMask(PortNo);

	if (mask)
	{
		R9A02_PORT(PortNo)->PCNTR3 = (Data & mask) | ((~Data & mask) << 16);
	}
}

#endif
