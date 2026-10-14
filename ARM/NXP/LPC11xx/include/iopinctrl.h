/*--------------------------------------------------------------------------
File   : iopinctrl.h

Author : Hoang Nguyen Hoan          Nov. 20, 2011

Desc   : General I/O pin control implementation specific 
		 This file must be named iopinctrl.h no matter which target

		 This is LPC11xx implementation

Copyright (c) 2011, I-SYST inc., all rights reserved

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

----------------------------------------------------------------------------
Modified by          Date              Description

----------------------------------------------------------------------------*/
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__

#include <stdint.h>
#include "LPC11Uxx.h"
#include "coredev/iopincfg.h"

// P1.30 is reserved. Package bonding may expose fewer of the other pins.
static inline bool Lpc11GpioValidPin(int PortNo, int PinNo)
{
	return (PortNo == 0 && (unsigned)PinNo < 24U) ||
		   (PortNo == 1 && (unsigned)PinNo < 32U && PinNo != 30);
}

static inline void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir)
{
	if (!Lpc11GpioValidPin(PortNo, PinNo))
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t mask = 1UL << (unsigned)PinNo;

	if (Dir == IOPINDIR_OUTPUT)
	{
		LPC_GPIO->DIR[PortNo] |= mask;
	}
	else
	{
		LPC_GPIO->DIR[PortNo] &= ~mask;
	}
	EnableInterrupt(state);
}

static inline int IOPinRead(int PortNo, int PinNo)
{
	return Lpc11GpioValidPin(PortNo, PinNo) ?
		   (int)((LPC_GPIO->PIN[PortNo] >> (unsigned)PinNo) & 1U) : 0;
}

static inline void IOPinSet(int PortNo, int PinNo)
{
	if (Lpc11GpioValidPin(PortNo, PinNo))
	{
		LPC_GPIO->SET[PortNo] = 1UL << (unsigned)PinNo;
	}
}

static inline void IOPinClear(int PortNo, int PinNo)
{
	if (Lpc11GpioValidPin(PortNo, PinNo))
	{
		LPC_GPIO->CLR[PortNo] = 1UL << (unsigned)PinNo;
	}
}

static inline void IOPinToggle(int PortNo, int PinNo)
{
	if (Lpc11GpioValidPin(PortNo, PinNo))
	{
		LPC_GPIO->NOT[PortNo] = 1UL << (unsigned)PinNo;
	}
}

static inline uint32_t IOPinReadPort(int PortNo)
{
	return (unsigned)PortNo < 2U ? LPC_GPIO->PIN[PortNo] : 0U;
}

static inline void IOPinWritePort(int PortNo, uint32_t Data)
{
	if ((unsigned)PortNo < 2U)
	{
		LPC_GPIO->PIN[PortNo] = Data;
	}
}

// Narrow writes affect only the low pins and preserve the remaining outputs.
static inline void IOPinWrite8Port(int PortNo, uint8_t Data)
{
	if ((unsigned)PortNo < 2U)
	{
		LPC_GPIO->CLR[PortNo] = (~(uint32_t)Data) & 0xFFU;
		LPC_GPIO->SET[PortNo] = Data;
	}
}

static inline void IOPinWrite16Port(int PortNo, uint16_t Data)
{
	if ((unsigned)PortNo < 2U)
	{
		LPC_GPIO->CLR[PortNo] = (~(uint32_t)Data) & 0xFFFFU;
		LPC_GPIO->SET[PortNo] = Data;
	}
}

static inline void IOPinWrite32Port(int PortNo, uint32_t Data)
{
	IOPinWritePort(PortNo, Data);
}

#endif // __IOPINCTRL_H__
