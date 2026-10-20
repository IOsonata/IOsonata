/**-------------------------------------------------------------------------
@file	pdm_filter.h

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

#ifndef __PDM_FILTER_H__
#define __PDM_FILTER_H__

#include <stdint.h>
#include <stdbool.h>

// One independent filter per microphone. Zero-initialize or call Reset.
// Input is chronological MSB-first PDM, 0 = -1 and 1 = +1.
// Eight input bytes produce one signed 16-bit PCM sample. This is a CIC3
// decimator, without a passband compensation filter or microphone gain.
typedef struct __Pdm_Filter
{
	uint32_t Integrator[3];
	uint32_t Delay[3];
	unsigned Count;
} PdmFilter_t;

#ifdef __cplusplus
extern "C" {
#endif

void PdmFilterReset(PdmFilter_t *pFilter);
bool PdmFilterByte(PdmFilter_t *pFilter, uint8_t Bits, int16_t *pSample);

#ifdef __cplusplus
}
#endif
#endif
