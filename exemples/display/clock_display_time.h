/**-------------------------------------------------------------------------
@file	clock_display_time.h

@brief	Clock time and button adjustment

Button samples use a 20 Hz time base. A short press advances one minute.
Holding the button advances one hour each second.

@author	Hoang Nguyen Hoan
@date	Nov. 15, 2026

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

#ifndef __CLOCK_DISPLAY_TIME_H__
#define __CLOCK_DISPLAY_TIME_H__

#include <stdint.h>

typedef struct {
	uint32_t Offset;
	uint32_t LastSample;
	uint32_t PressedAt;
	unsigned Debounce;
	bool Pressed;
	bool Held;
} ClockTime_t;

static uint32_t ClockTimeUpdate(ClockTime_t *pTime, uint64_t Ticks, uint32_t Frequency, bool Down)
{
	uint32_t elapsed = (uint32_t)((Ticks / Frequency) % 86400U);
	uint32_t sample = (uint32_t)(Ticks * 20U / Frequency);

	if (sample != pTime->LastSample)
	{
		pTime->LastSample = sample;
		if (Down != pTime->Pressed)
		{
			if (++pTime->Debounce == 2)
			{
				pTime->Pressed = Down;
				pTime->Debounce = 0;
				if (pTime->Pressed)
				{
					pTime->PressedAt = sample;
					pTime->Held = false;
				}
				else if (!pTime->Held)
				{
					pTime->Offset = (pTime->Offset + 60U) % 86400U;
				}
			}
		}
		else
		{
			pTime->Debounce = 0;
		}
		if (pTime->Pressed && sample - pTime->PressedAt >= 20U)
		{
			pTime->Offset = (pTime->Offset + 3600U) % 86400U;
			pTime->PressedAt = sample;
			pTime->Held = true;
		}
	}

	return (elapsed + pTime->Offset) % 86400U;
}

#endif
