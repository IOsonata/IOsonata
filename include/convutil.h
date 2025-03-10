/**-------------------------------------------------------------------------
@file convutil.h

@brief  Conversion utilities

@author Hoang Nguyen Hoan
@date Feb. 8, 2015

@license

MIT License

Copyright (c) 2015 I-SYST inc. All rights reserved.

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

#ifndef __CONVUTIL_H__
#define __CONVUTIL_H__

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

/** @addtogroup Utilities
  * @{
  */

#ifdef _MSC_VER
// Microsoft does not support C99 inline
#ifndef inline
#define inline __forceinline
#endif
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Conversion uSec & mSec to 1.25ms unit
 */
#define USEC_TO_1250(Val)	((uint16_t)(((Val) + 500UL) / 1250UL))
#define MSEC_TO_1_25(Val)	((uint16_t)((Val) / 1.250F))

static inline uint16_t uSecTo1250(uint32_t Val) {
	return (uint16_t)((Val + 500UL) / 1250UL);
};

static inline uint16_t mSecTo1_25(float Val) {
	return (uint16_t)(Val / 1.250F);
};

/**
 * @brief	Conversion uSec & mSec to 0.625ms unit
 */

#define USEC_TO_625(Val)	((uint16_t)(((Val) + 500UL) / 625UL))
#define MSEC_TO_0_625(Val)	((uint16_t)((Val) / 0.6250F))

static inline uint16_t uSecTo625(uint32_t Val) {
	return (uint16_t)((Val + 500UL) / 625UL);
};

static inline uint16_t mSecTo0_625(float Val) {
	return (uint16_t)(Val / 0.625F);
};

/**
 * @brief 16 bits endianess conversion.
 *
 * @param x : 16 bits number to covert.
 *
 * @return  converted data.
 */
static inline uint16_t EndianCvt16(uint16_t x) {
  return ((x >> 8) & 0xffUL) | ((x << 8) & 0xff00UL);
}

/**
 * @brief 32 bits endianess conversion.
 *
 * @param x : 32 bits number to covert.
 *
 * @return  converted data.
 */
static inline uint32_t EndianCvt32(uint32_t x) {
  return (((x >> 24UL) & 0xff) | ((x << 24UL) & 0xff000000) |
      ((x >> 8UL) & 0xff00) | ((x << 8UL) & 0xff0000));
}

/**
 * @brief 64 bits endianess conversion.
 *
 * @param x : 64 bits number to covert.
 *
 * @return  converted data.
 */
static inline uint64_t EndianCvt64(uint64_t x) {
  return (((x >> 56ULL) & 0xffULL) | ((x << 56ULL) & 0xff00000000000000ULL) |
      ((x >> 40ULL) & 0xff00ULL) | ((x << 40ULL) & 0xff000000000000ULL) |
      ((x >> 24ULL) & 0xff0000ULL) | ((x << 24ULL) & 0xff0000000000ULL) |
      ((x >> 8ULL) & 0xff000000ULL) | ((x << 8ULL) & 0xff00000000ULL));
}

/**
 * @brief Convert ASCII hex character to integer.
 *
 * @param c : Hex character to convert
 *
 * @return  converted value.  -1 if wrong character
 */
static inline int chex2i(char c) {
  if (c >= 'a')
    return (c - 'a' + 10);
  else if (c >= 'A')
    return (c - 'A' + 10);

  return (c - '0');
}
  
#ifdef __cplusplus
}
#endif

/** @} End of group Utilities */

#endif // __CONVUTIL_H__
