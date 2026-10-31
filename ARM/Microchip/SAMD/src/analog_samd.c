/**-------------------------------------------------------------------------
@file	analog_samd.c

@brief	SAM D11/D21 analog pins and voltage references shared by the ADC and
		the DAC

See analog_samd.h.

@author	Hoang Nguyen Hoan
@date	Oct. 30, 2026

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
#include <stdbool.h>
#include <stdint.h>

#include "samdxxx.h"
#include "coredev/interrupt.h"
#include "coredev/iopincfg.h"
#include "analog_samd.h"

#define SAMD_ANALOG_VREF_MSK	(SYSCTRL_VREF_TSEN_Msk | SYSCTRL_VREF_BGOUTEN_Msk)

static uint32_t s_SamdAnalogPin[SAMD_ANALOG_USER_CNT][2];	// Pins taken, port A and B
static uint32_t s_SamdAnalogVref[SAMD_ANALOG_USER_CNT];		// VREF outputs needed

void SamdAnalogPin(int User, uint32_t Pin, bool bOn)
{
	uint32_t port = Pin >> 5;
	uint32_t bit = 1UL << (Pin & 31U);

	if (User < 0 || User >= SAMD_ANALOG_USER_CNT || port > 1U)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	bool used = false;

	if (bOn)
	{
		s_SamdAnalogPin[User][port] |= bit;
		IOPinConfig((int)port, (int)(Pin & 31U), IOPINOP_PERIPHB, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL);
		PORT_REGS->GROUP[port].PORT_PINCFG[Pin & 31U] &= (uint8_t)~PORT_PINCFG_INEN_Msk;
	}
	else
	{
		s_SamdAnalogPin[User][port] &= ~bit;
		for (int i = 0; i < SAMD_ANALOG_USER_CNT; i++)
		{
			used = used || (s_SamdAnalogPin[i][port] & bit) != 0U;
		}
		if (!used)
		{
			IOPinDisable((int)port, (int)(Pin & 31U));
		}
	}

	EnableInterrupt(state);
}

void SamdAnalogVref(int User, uint32_t Mask)
{
	if (User < 0 || User >= SAMD_ANALOG_USER_CNT)
	{
		return;
	}

	uint32_t state = DisableInterrupt();
	uint32_t need = 0;

	s_SamdAnalogVref[User] = Mask & SAMD_ANALOG_VREF_MSK;
	for (int i = 0; i < SAMD_ANALOG_USER_CNT; i++)
	{
		need |= s_SamdAnalogVref[i];
	}
	SYSCTRL_REGS->SYSCTRL_VREF = (SYSCTRL_REGS->SYSCTRL_VREF & ~SAMD_ANALOG_VREF_MSK) | need;

	EnableInterrupt(state);
}
