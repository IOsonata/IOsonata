/**-------------------------------------------------------------------------
@file system_clock_sam4e.cpp
@brief SAM4E slow clock selection.

The slow clock can switch from RC to crystal or bypass once per backup
supply cycle. It cannot switch back to RC in software. Preserve the board
configuration when the requested source is unavailable or fails to start.

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#include "sam4e.h"
#include "coredev/system_core_clock.h"

#define SAM4E_SLOW_CLOCK_POLLS 10000000U

bool SystemLowFreqClockSelect(OSC_TYPE ClkSrc, uint32_t OscFreq)
{
	uint32_t status;
	uint32_t mode;
	uint32_t i;
	bool bypass;

	if (ClkSrc != OSC_TYPE_RC && ClkSrc != OSC_TYPE_XTAL && ClkSrc != OSC_TYPE_TCXO)
	{
		return false;
	}
	if (ClkSrc != OSC_TYPE_RC && OscFreq != 0 && OscFreq != 32768)
	{
		return false;
	}
	(void)SystemResetReason();
	status = SAM4E_SUPC->SUPC_SR;
	mode = SAM4E_SUPC->SUPC_MR;
	bypass = ClkSrc == OSC_TYPE_TCXO;
	if (status & SUPC_SR_OSCSEL)
	{
		if (ClkSrc == OSC_TYPE_RC || bypass != ((mode & SUPC_MR_OSCBYPASS) != 0))
		{
			return false;
		}
	}
	else if (ClkSrc != OSC_TYPE_RC)
	{
		mode &= ~(SUPC_MR_KEY_Msk | SUPC_MR_OSCBYPASS);
		if (bypass)
		{
			mode |= SUPC_MR_OSCBYPASS;
		}
		SAM4E_SUPC->SUPC_MR = mode | SUPC_MR_KEY_PASSWD;
		SAM4E_SUPC->SUPC_CR = SUPC_CR_KEY_PASSWD | SUPC_CR_XTALSEL;
		for (i = 0; i < SAM4E_SLOW_CLOCK_POLLS; i++)
		{
			if (SAM4E_SUPC->SUPC_SR & SUPC_SR_OSCSEL)
			{
				break;
			}
		}
		if (i == SAM4E_SLOW_CLOCK_POLLS)
		{
			return false;
		}
	}
	g_McuOsc.LowPwrOsc.Type = ClkSrc;
	g_McuOsc.LowPwrOsc.Freq = ClkSrc == OSC_TYPE_RC ? 32000 : 32768;
	return true;
}

void SystemOscInit(void)
{
	(void)SystemLowFreqClockSelect(g_McuOsc.LowPwrOsc.Type, g_McuOsc.LowPwrOsc.Freq);
}
