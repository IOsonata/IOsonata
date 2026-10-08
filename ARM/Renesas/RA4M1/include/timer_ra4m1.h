/**-------------------------------------------------------------------------
@file	timer_ra4m1.h

@brief	RA4M1 native timer device mapping and diagnostics.

@license

MIT License

Copyright (c) 2026 I-SYST inc. All rights reserved.

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
#ifndef TIMER_RA4M1_H
#define TIMER_RA4M1_H
#include <stdint.h>
#define RA4M1_TIMER_AGT_COUNT 2
#define RA4M1_TIMER_GPT_COUNT 8
#define RA4M1_TIMER_COUNT 10
/* DevNo 0/1 = AGT0/1; DevNo 2..9 = GPT0..7. */
typedef enum {
    RA4M1_TIMER_OK,
    RA4M1_TIMER_CLOCK_ERROR,
    RA4M1_TIMER_ACCESS_ERROR,
    RA4M1_TIMER_STOP_TIMEOUT,
    RA4M1_TIMER_START_TIMEOUT,
    RA4M1_TIMER_COMPARE_ERROR,
    RA4M1_TIMER_IRQ_ERROR
} Ra4m1TimerError_t;
#ifdef __cplusplus
extern "C" {
#endif
/* Last hardware failure for each device; successful reset clears it. */
extern volatile uint32_t g_Ra4m1TimerError[RA4M1_TIMER_COUNT];
#ifdef __cplusplus
}
#endif
#endif
