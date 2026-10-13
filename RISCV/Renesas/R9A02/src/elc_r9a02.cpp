/**-------------------------------------------------------------------------
@file	elc_r9a02.cpp

@brief	R9A02G021 event link controller (ELC)

See elc_r9a02.h. R9A02G021 User's Manual R01UH1036EJ0110, chapter 15.
Procedure 15.3.3: destination set up, ELSRn.ELS, ELCR.ELCON = 1, then the
source. ELCON is cleared before the module stop (15.4.3).

@author	Hoang Nguyen Hoan
@date	Oct. 13, 2026

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
#include <stdint.h>
#include <stddef.h>

#include "coredev/interrupt.h"
#include "system_r9a02.h"
#include "elc_r9a02.h"

static const uint8_t s_R9a02ElcDest[] = {
	R9A02_ELC_DEST_ADC, R9A02_ELC_DEST_PORT1, R9A02_ELC_DEST_PORT2,
	R9A02_ELC_DEST_DAC1, R9A02_ELC_DEST_DAC0, R9A02_ELC_DEST_TML32,
};

static bool R9a02ElcDestValid(uint32_t Dest)
{
	for (size_t i = 0; i < sizeof(s_R9a02ElcDest); i++)
	{
		if (s_R9a02ElcDest[i] == Dest)
		{
			return true;
		}
	}

	return false;
}

uint8_t R9a02ElcEvent(R9A02_ELC_DEST Dest)
{
	if (!R9a02ElcDestValid((uint32_t)Dest) || !R9a02ModuleRunning(R9A02_MSTPCR_C, R9A02_MSTPCRC_ELC))
	{
		return 0U;
	}

	return (uint8_t)R9A02_ELC_ELSR(Dest);
}

bool R9a02ElcLink(R9A02_ELC_DEST Dest, uint8_t EvtId)
{
	// SYSTEM_SNZREQ goes to ELSR8, 14, 15 and 23 only (table 15.3 note 2)
	if (!R9a02ElcDestValid((uint32_t)Dest) || EvtId > R9A02_EVT_MAX ||
		(EvtId == R9A02_EVT_SYSTEM_SNZREQ && (Dest == R9A02_ELC_DEST_DAC0 || Dest == R9A02_ELC_DEST_DAC1)))
	{
		return false;
	}

	uintptr_t state = DisableInterrupt();

	if (EvtId != 0U && !R9a02ModuleRunning(R9A02_MSTPCR_C, R9A02_MSTPCRC_ELC))
	{
		R9a02ModuleStart(R9A02_MSTPCR_C, R9A02_MSTPCRC_ELC);
		for (size_t i = 0; i < sizeof(s_R9a02ElcDest); i++)
		{
			R9A02_ELC_ELSR(s_R9a02ElcDest[i]) = 0U;
		}
	}

	if (!R9a02ModuleRunning(R9A02_MSTPCR_C, R9A02_MSTPCRC_ELC))
	{
		// Unlinking with the ELC stopped, nothing is linked
		EnableInterrupt(state);

		return EvtId == 0U;
	}

	R9A02_ELC_ELSR(Dest) = EvtId;

	bool used = false;

	for (size_t i = 0; i < sizeof(s_R9a02ElcDest); i++)
	{
		if (R9A02_ELC_ELSR(s_R9a02ElcDest[i]) & 0xFFU)
		{
			used = true;
		}
	}

	if (used)
	{
		R9A02_ELC_ELCR = R9A02_ELCR_ELCON;
	}
	else
	{
		R9A02_ELC_ELCR = 0U;
		(void)R9A02_ELC_ELCR;
		R9a02ModuleStop(R9A02_MSTPCR_C, R9A02_MSTPCRC_ELC);
	}

	EnableInterrupt(state);

	return true;
}
