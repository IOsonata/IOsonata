/**-------------------------------------------------------------------------
@file	memcpy_fast.cpp

@brief	Fast memory copy.

Bulk half of memcpy_fast.h. The header handles the fixed word aligned sizes
without a call, this handles everything else.

@author	Hoang Nguyen Hoan
@date	Sep. 5, 2026

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
#include "memcpy_fast.h"

/// Bytes moved per iteration of the unrolled body. Four words is one LDM and
/// one STM on Cortex-M, and stays inside the register set on RISC-V.
#define MEMCPY_FAST_BLKSIZE			16U

void *memcpy_fast_bulk(void *pDst, const void *pSrc, size_t Size)
{
	uint8_t *pD = (uint8_t *)pDst;
	const uint8_t *pS = (const uint8_t *)pSrc;
	uint32_t *pDw;
	const uint32_t *pSw;

	if (pDst == NULL || pSrc == NULL || Size == 0U)
	{
		return pDst;
	}

	// Pointers that disagree on their offset within a word can never both be
	// brought to a word boundary, and not every core the library builds for
	// allows an unaligned word access. Those go one byte at a time.
	if (((((uintptr_t)pD) ^ ((uintptr_t)pS)) & 3U) != 0U)
	{
		while (Size-- > 0U)
		{
			*pD++ = *pS++;
		}

		return pDst;
	}

	// Leading bytes up to the first word boundary.
	while (Size > 0U && ((((uintptr_t)pD) & 3U) != 0U))
	{
		*pD++ = *pS++;
		Size--;
	}

	pDw = (uint32_t *)pD;
	pSw = (const uint32_t *)pS;

	// Body. Reading all four words before writing any of them is what lets
	// the compiler pair them into LDM and STM.
	while (Size >= MEMCPY_FAST_BLKSIZE)
	{
		uint32_t w0 = pSw[0];
		uint32_t w1 = pSw[1];
		uint32_t w2 = pSw[2];
		uint32_t w3 = pSw[3];

		pDw[0] = w0;
		pDw[1] = w1;
		pDw[2] = w2;
		pDw[3] = w3;

		pDw += 4;
		pSw += 4;
		Size -= MEMCPY_FAST_BLKSIZE;
	}

	while (Size >= sizeof(uint32_t))
	{
		*pDw++ = *pSw++;
		Size -= sizeof(uint32_t);
	}

	// Trailing bytes.
	pD = (uint8_t *)pDw;
	pS = (const uint8_t *)pSw;

	while (Size-- > 0U)
	{
		*pD++ = *pS++;
	}

	return pDst;
}
