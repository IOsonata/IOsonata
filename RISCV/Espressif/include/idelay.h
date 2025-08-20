/**-------------------------------------------------------------------------
@file	idelay.h

@brief	Delay loop functions.

ESP32 implementation PlatformIO

@author Hoang Nguyen Hoan
@date	Aug. 16, 2025

@license

MIT License

Copyright (c) 2025 I-SYST inc. All rights reserved.

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
#ifndef __IDELAY_H__
#define __IDELAY_H__

#include <stdint.h>

#include "esp_timer.h"

/** @addtogroup Utilities
  * @{
  */

/**
 * @brief	Microsecond delay.
 *
 * This function is based on a 16MHz clock. For higher clock
 * rate SystemMicroSecNopCnt needs to be adjusted.  Adjustment of this variable
 * should be done in the CMSIS SystemCoreCLockUpdate function.
 * This delay is only approximate, it is NOT 100% accurate.
 *
 * @param	cnt : microsecond delay count
 */
static inline __attribute__((always_inline)) void usDelay(uint32_t cnt) {
  uint64_t m = (uint64_t)esp_timer_get_time();
  if (cnt) {
    uint64_t e = (m + cnt);
    if (m > e) {  //overflow
      while ((uint64_t)esp_timer_get_time() > e) {
        asm volatile("nop");
      }
    }
    while ((uint64_t)esp_timer_get_time() < e) {
      asm volatile("nop");
    }
  }
}

/**
 * @brief	Nanosecond delay.
 *
 * This is highly inaccurate use at you own risk
 *
 * nsec delay cannot be achieved for low cpu clock.
 * this loop is 295ns on a 16MHz cpu
 *
 * @param	cnt : nanosecond count
 */
static inline __attribute__((always_inline)) void nsDelay(uint32_t cnt) {
}

static inline __attribute__((always_inline)) void msDelay(uint32_t ms) {
	usDelay(ms * 1000UL);
}

/** @} End of group Utilities */

#endif	// __IDELAY_H__

