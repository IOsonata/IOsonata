/**-------------------------------------------------------------------------
@file	rng_nrfx.c

@brief	Random number generator implementation on Nordic nRF series

This file implement Random Number Generator using Nordic nRF harwdare engine

@author	Hoang Nguyen Hoan
@date	Aug. 9, 2024

@license

MIT License

Copyright (c) 2024, I-SYST inc., all rights reserved

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

#include "nrf.h"

bool RngInit()
{
	
}

bool RngGet(uint8_t *pBuff, size_t Len)
{
#if defined(NRF54H20_XXAA) || defined(NRF54L15_XXAA)
	NRF_CRACEN_Type *reg = NRF_CRACEN_S;
	NRF_CRACENCORE_Type *regcore = NRF_CRACENCORE_S
	
	reg->ENABLE |= CRACEN_ENABLE_RNG_Enabled;

	while (reg->EVENTS_RNG == 0);

#else
#if defined(NRF91_SERIES) || defined(NRF53_SERIES)
#ifdef NRF5340_XXAA_NETWORK
	NRF_RNG_Type *reg = NRF_RNG_NS;
#else
	NRF_RNG_Type *reg = NRF_RNG_S;
#endif
#else
	NRF_RNG_Type *reg = NRF_RNG;
#endif

	reg->CONFIG = RNG_CONFIG_DERCEN_Enabled;

	reg->TASKS_START = 1;

	for (int i = 0; i < Len; i++)
	{
		while (reg->EVENTS_VALRDY == 0);

		pBuff[i] = reg->VALUE;
	}

	reg->TASKS_STOP = 1;

	reg->CONFIG = RNG_CONFIG_DERCEN_Disabled;
#endif
}