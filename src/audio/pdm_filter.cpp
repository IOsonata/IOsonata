/**-------------------------------------------------------------------------
@file	pdm_filter.cpp

@brief	64:1 third-order CIC PDM to PCM filter


@author	Hoang Nguyen Hoan
@date	Oct. 19, 2026

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

#include <string.h>
#include "audio/pdm_filter.h"

void PdmFilterReset(PdmFilter_t *pFilter)
{
	memset(pFilter, 0, sizeof(*pFilter));
}

bool PdmFilterByte(PdmFilter_t *pFilter, uint8_t Bits, int16_t *pSample)
{
	uint32_t value;
	int64_t sample;

	// Unsigned arithmetic supplies the CIC's required modulo-2^32 wrap.
	for (unsigned bit = 0; bit < 8; bit++)
	{
		pFilter->Integrator[0] += (Bits & 0x80) ? 1U : UINT32_MAX;
		pFilter->Integrator[1] += pFilter->Integrator[0];
		pFilter->Integrator[2] += pFilter->Integrator[1];
		Bits <<= 1;
	}
	if (++pFilter->Count != 8)
	{
		return false;
	}
	pFilter->Count = 0;
	value = pFilter->Integrator[2];
	for (unsigned stage = 0; stage < 3; stage++)
	{
		uint32_t previous = value;

		value -= pFilter->Delay[stage];
		pFilter->Delay[stage] = previous;
	}
	// CIC gain is 64^3. Convert the modulo result without an out-of-range
	// unsigned-to-signed cast, then scale to signed PCM and saturate.
	sample = value <= INT32_MAX ? int64_t(value) : int64_t(value) - 0x100000000LL;
	sample /= 8;
	if (sample > INT16_MAX)
	{
		sample = INT16_MAX;
	}
	if (sample < INT16_MIN)
	{
		sample = INT16_MIN;
	}
	*pSample = int16_t(sample);
	return true;
}
