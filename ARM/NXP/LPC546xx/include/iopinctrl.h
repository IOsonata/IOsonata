/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	LPC546xx fast I/O pin control

		Inline pin access on the LPC546xx GPIO block. The pin must first be
		configured with IOPinConfig, which turns its port clock on.

@author	Hoang Nguyen Hoan
@date	Oct. 10, 2026

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

#include "LPC546xx.h"
#include "coredev/iopincfg.h"

/**
 * @brief	Set pin direction.
 *
 * Read-modify-write of DIR. The NXP SDK does not use the DIRSET and DIRCLR
 * registers on this family, they are not relied on here either.
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 * @param	Dir		: IOPINDIR_INPUT or IOPINDIR_OUTPUT
 */
static inline __attribute__((always_inline)) void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	if (Dir == IOPINDIR_OUTPUT)
	{
		GPIO->DIR[PortNo] |= 1UL << (unsigned)PinNo;
	}
	else if (Dir == IOPINDIR_INPUT)
	{
		GPIO->DIR[PortNo] &= ~(1UL << (unsigned)PinNo);
	}
}

/**
 * @brief	Read pin state.
 *
 * @param	PortNo	: Port number
 * @param	PinNo	: Pin number
 *
 * @return	0 or 1
 */
static inline __attribute__((always_inline)) int IOPinRead(int PortNo, int PinNo)
{
	return GPIO->B[PortNo][PinNo];
}

static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo)
{
	GPIO->SET[PortNo] = 1UL << (unsigned)PinNo;
}

static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo)
{
	GPIO->CLR[PortNo] = 1UL << (unsigned)PinNo;
}

static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo)
{
	GPIO->NOT[PortNo] = 1UL << (unsigned)PinNo;
}

static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo)
{
	return GPIO->PIN[PortNo];
}

/**
 * @brief	Write all output pins of a port.
 *
 * Writing PIN sets the output state of every pin of the port.
 *
 * @param	PortNo	: Port number
 * @param	Data	: Pin states, bit n for pin n
 */
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data)
{
	GPIO->PIN[PortNo] = Data;
}

#endif
