/**-------------------------------------------------------------------------
@file	iopincfg_samd.c

@brief	SAM D11/D21 PORT pin configuration

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
#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"
#include "coredev/iopincfg.h"

static bool SamdPinValid(int PortNo, int PinNo)
{
	return PortNo >= 0 && PortNo < PORT_GROUP_NUMBER &&
		PinNo >= 0 && PinNo < 32;
}

bool IOPinAssignCore(int PortNo, int PinNo, int CoreNo)
{
	return CoreNo == 0 && SamdPinValid(PortNo, PinNo);
}

void IOPinConfig(int PortNo, int PinNo, int PinOp, IOPINDIR Dir,
	IOPINRES Resistor, IOPINTYPE Type)
{
	port_group_registers_t *pPort;
	uint32_t mask;
	uint8_t cfg;
	uint8_t mux;
	uint32_t idx;

	if (!SamdPinValid(PortNo, PinNo) || PinOp < IOPINOP_GPIO ||
		PinOp > IOPINOP_PERIPHH || Type != IOPINTYPE_NORMAL)
	{
		return;
	}

	pPort = &PORT_REGS->GROUP[PortNo];
	mask = 1UL << PinNo;
	cfg = pPort->PORT_PINCFG[PinNo] & PORT_PINCFG_DRVSTR_Msk;

	if (Resistor == IOPINRES_PULLUP || Resistor == IOPINRES_FOLLOW)
	{
		pPort->PORT_OUTSET = mask;
		cfg |= PORT_PINCFG_PULLEN_Msk;
	}
	else if (Resistor == IOPINRES_PULLDOWN)
	{
		pPort->PORT_OUTCLR = mask;
		cfg |= PORT_PINCFG_PULLEN_Msk;
	}

	if (PinOp != IOPINOP_GPIO)
	{
		idx = (uint32_t)PinNo >> 1;
		mux = pPort->PORT_PMUX[idx];
		if ((PinNo & 1) != 0)
		{
			mux = (mux & 0x0FU) | (uint8_t)((PinOp - 1) << 4);
		}
		else
		{
			mux = (mux & 0xF0U) | (uint8_t)(PinOp - 1);
		}
		pPort->PORT_PMUX[idx] = mux;
		cfg |= PORT_PINCFG_PMUXEN_Msk | PORT_PINCFG_INEN_Msk;
	}
	else if (Dir == IOPINDIR_OUTPUT)
	{
		pPort->PORT_DIRSET = mask;
	}
	else
	{
		pPort->PORT_DIRCLR = mask;
		cfg |= PORT_PINCFG_INEN_Msk;
	}

	pPort->PORT_PINCFG[PinNo] = cfg;
}

void IOPinDisable(int PortNo, int PinNo)
{
	port_group_registers_t *pPort;

	if (!SamdPinValid(PortNo, PinNo))
	{
		return;
	}

	pPort = &PORT_REGS->GROUP[PortNo];
	pPort->PORT_DIRCLR = 1UL << PinNo;
	pPort->PORT_PINCFG[PinNo] = 0;
}

void IOPinSetStrength(int PortNo, int PinNo, IOPINSTRENGTH Strength)
{
	port_group_registers_t *pPort;
	uint8_t cfg;

	if (!SamdPinValid(PortNo, PinNo))
	{
		return;
	}

	pPort = &PORT_REGS->GROUP[PortNo];
	cfg = pPort->PORT_PINCFG[PinNo];
	if (Strength == IOPINSTRENGTH_STRONG)
	{
		cfg |= PORT_PINCFG_DRVSTR_Msk;
	}
	else
	{
		cfg &= (uint8_t)~PORT_PINCFG_DRVSTR_Msk;
	}
	pPort->PORT_PINCFG[PinNo] = cfg;
}
