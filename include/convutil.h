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

// Milliseconds to 0.625 ms units, the unit every LE advertising and scan
// interval is expressed in.
//
// Integer arithmetic in 64 bits, and a 32 bit result. The previous form took a
// float and cast the quotient to uint16_t, so any interval from 40960 ms up
// produced a value the destination type cannot hold, which is undefined: the
// same input gave 65535 folded at compile time and 0 at run time. 40960 ms is
// well inside the extended advertising range of 0x000020 to 0xFFFFFF that
// Core Vol 4 Part E 7.8.53 defines, so this was reachable with a legal
// request. Callers apply the range their command allows.
static inline uint32_t mSecTo0_625(float Val) {
	// Single precision throughout. 0.625 is exact in binary and the largest
	// value any of these fields holds is 0xFFFFFF units, just under 2 to the
	// 24, so every interval in range converts exactly: 7.5 ms gives 12. On a
	// Cortex-M4F this is one VDIV.F32 and one VCVT, both hardware. Double
	// would force calls into the soft-float library on a part that has no
	// double unit.
	float units = Val / 0.625F;

	// Only the destination width is guarded, which the two compares above the
	// convert do in hardware as well. The old form cast to uint16_t, and a
	// float to integer conversion that does not fit is undefined: 40960 ms
	// gave 65535 folded at compile time and 0 at run time.
	if (units <= 0.0F)
	{
		return 0;
	}
	if (units >= 4294967296.0F)
	{
		return 0xFFFFFFFFU;
	}
	return (uint32_t)units;
};

/**
 * @brief	Conversion mSec to 10ms unit (advertising/scan duration).
 */
#define MSEC_TO_10MS(Val)	((uint16_t)(((Val) + 5UL) / 10UL))

// Milliseconds to 10 ms units, the unit of an advertising or scan duration.
// A 32 bit result for the same reason as above: truncating to uint16_t turned
// 660000 ms into 464 units, 4.64 s instead of 11 minutes. Callers apply the
// range their command allows, and note that a duration of zero means "until
// the Host disables it" in Core Vol 4 Part E 7.8.56, so a caller with a
// nonzero request must not let this round down to zero.
static inline uint32_t mSecTo10Ms(uint32_t Val) {
	return (uint32_t)(((uint64_t)Val + 5ULL) / 10ULL);
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
