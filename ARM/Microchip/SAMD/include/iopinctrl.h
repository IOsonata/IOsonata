/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	SAM D11/D21 GPIO pin access

Inline functions for the fast access to the PORT registers.

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>

#include "samdxxx.h"
#include "coredev/iopincfg.h"

static inline void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	if (Dir == IOPINDIR_OUTPUT)
	{
		PORT_REGS->GROUP[PortNo].PORT_DIRSET = 1UL << PinNo;
	}
	else
	{
		PORT_REGS->GROUP[PortNo].PORT_DIRCLR = 1UL << PinNo;
	}
}

static inline int IOPinRead(int PortNo, int PinNo)
{
	return (PORT_REGS->GROUP[PortNo].PORT_IN >> PinNo) & 1U;
}

static inline void IOPinSet(int PortNo, int PinNo)
{
	PORT_REGS->GROUP[PortNo].PORT_OUTSET = 1UL << PinNo;
}

static inline void IOPinClear(int PortNo, int PinNo)
{
	PORT_REGS->GROUP[PortNo].PORT_OUTCLR = 1UL << PinNo;
}

static inline void IOPinToggle(int PortNo, int PinNo)
{
	PORT_REGS->GROUP[PortNo].PORT_OUTTGL = 1UL << PinNo;
}

static inline uint32_t IOPinReadPort(int PortNo)
{
	return PORT_REGS->GROUP[PortNo].PORT_IN;
}

static inline void IOPinWritePort(int PortNo, uint32_t Data)
{
	PORT_REGS->GROUP[PortNo].PORT_OUT = Data;
}

#endif // __IOPINCTRL_H__
