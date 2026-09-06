/**-------------------------------------------------------------------------
@file	memcpy_fast.h

@brief	Fast memory copy.

Word wide copy for the sizes embedded code moves most often. Ported from the
queue item copy in TaktOS, where the fixed size cases are assigned as whole
word structs so the compiler emits LDM/STM instead of calling memcpy.

Two entry points:

	memcpy_fast		Inline. Handles 4, 8, 12, 16 and 32 byte word aligned
					copies without a call. Everything else goes to the bulk
					routine below.
	memcpy_fast_bulk	Out of line. Any size, any alignment.

Both return the destination pointer, so either can stand in for memcpy.

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

#ifndef __MEMCPY_FAST_H__
#define __MEMCPY_FAST_H__

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/** @addtogroup Utilities
  * @{
  */

// Attribute spelling only. Nothing here selects behaviour and nothing here
// is meant to be defined by the application.
#if defined(__GNUC__) || defined(__clang__) || defined(__ICCARM__)
#define MEMCPY_FAST_INLINE		static inline __attribute__((always_inline))
#define MEMCPY_FAST_LIKELY(x)	(__builtin_expect(!!(x), 1))
#else
#define MEMCPY_FAST_INLINE		static inline
#define MEMCPY_FAST_LIKELY(x)	(x)
#endif

/// True when both pointers sit on a word boundary, which is what the word
/// wide moves below require. One OR covers both.
#define MEMCPY_FAST_ALIGNED(Dst, Src) \
	(((((uintptr_t)(Dst)) | ((uintptr_t)(Src))) & 3U) == 0U)

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Copy any size at any alignment.
 *
 * Byte copies the leading bytes up to a word boundary, moves the body four
 * words at a time, then byte copies the tail. Source and destination that
 * are misaligned against each other are copied byte at a time, since not
 * every core the library builds for allows an unaligned word access.
 *
 * @param	pDst	: Destination
 * @param	pSrc	: Source
 * @param	Size	: Byte count
 *
 * @return	pDst
 */
void *memcpy_fast_bulk(void *pDst, const void *pSrc, size_t Size);

/**
 * @brief	Copy, taking the word wide path for the common fixed sizes.
 *
 * A size the compiler knows folds to a plain load store sequence with no
 * comparison left. A size only known at run time pays the compares, which
 * are still cheaper than the call the fallback would make. The order of the
 * tests is the order TaktOS measured, most frequent first.
 *
 * @param	pDst	: Destination
 * @param	pSrc	: Source
 * @param	Size	: Byte count
 *
 * @return	pDst
 */
MEMCPY_FAST_INLINE void *memcpy_fast(void *pDst, const void *pSrc, size_t Size)
{
	if (MEMCPY_FAST_ALIGNED(pDst, pSrc))
	{
		if (MEMCPY_FAST_LIKELY(Size == 16U))
		{
			typedef struct { uint32_t w[4]; } Blk16_t;
			*(Blk16_t *)pDst = *(const Blk16_t *)pSrc;
			return pDst;
		}
		else if (Size == 32U)
		{
			typedef struct { uint32_t w[8]; } Blk32_t;
			*(Blk32_t *)pDst = *(const Blk32_t *)pSrc;
			return pDst;
		}
		else if (Size == 8U)
		{
			typedef struct { uint32_t w[2]; } Blk8_t;
			*(Blk8_t *)pDst = *(const Blk8_t *)pSrc;
			return pDst;
		}
		else if (Size == 4U)
		{
			*(uint32_t *)pDst = *(const uint32_t *)pSrc;
			return pDst;
		}
		else if (Size == 12U)
		{
			typedef struct { uint32_t w[3]; } Blk12_t;
			*(Blk12_t *)pDst = *(const Blk12_t *)pSrc;
			return pDst;
		}
	}

	return memcpy_fast_bulk(pDst, pSrc, Size);
}

#ifdef __cplusplus
}
#endif


/** @} End of group Utilities */

#endif // __MEMCPY_FAST_H__
