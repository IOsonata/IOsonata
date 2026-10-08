/**-------------------------------------------------------------------------
@file	ra4m1_timer_regs.h

@brief	Private RA4M1 AGT/GPT register subset, R01UH0887EJ0110.

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
#ifndef RA4M1_TIMER_REGS_H
#define RA4M1_TIMER_REGS_H
#include "ra4m1_ioregs.h"
#include "ra4m1_startup_regs.h"
#define RA4M1_TIMER_MSTPCRD 0x40047008UL
#define RA4M1_AGT_BASE(n) (0x40084000UL + 0x100UL * (n))
#define RA4M1_GPT_BASE(n) (0x40078000UL + 0x100UL * (n))
/* AGT data registers are halfwords; control registers are bytes. */
#define RA4M1_AGT 0x00U
#define RA4M1_AGTCMA 0x02U
#define RA4M1_AGTCMB 0x04U
#define RA4M1_AGTCR 0x08U
#define RA4M1_AGTMR1 0x09U
#define RA4M1_AGTMR2 0x0AU
#define RA4M1_AGTIOC 0x0CU
#define RA4M1_AGTISR 0x0DU
#define RA4M1_AGTCMSR 0x0EU
#define RA4M1_AGTIOSEL 0x0FU
#define RA4M1_AGT_START 0x01U
#define RA4M1_AGT_STARTED 0x02U
#define RA4M1_AGT_UNDERFLOW 0x20U
/* All GPT registers require word accesses, including GPT16 channels. */
#define RA4M1_GTWP 0x00U
#define RA4M1_GTSSR 0x10U
#define RA4M1_GTPSR 0x14U
#define RA4M1_GTCSR 0x18U
#define RA4M1_GTUPSR 0x1CU
#define RA4M1_GTDNSR 0x20U
#define RA4M1_GTICASR 0x24U
#define RA4M1_GTICBSR 0x28U
#define RA4M1_GTCR 0x2CU
#define RA4M1_GTUDDTYC 0x30U
#define RA4M1_GTIOR 0x34U
#define RA4M1_GTINTAD 0x38U
#define RA4M1_GTST 0x3CU
#define RA4M1_GTBER 0x40U
#define RA4M1_GTCNT 0x48U
#define RA4M1_GTPR 0x64U
#define RA4M1_GTPBR 0x68U
#define RA4M1_GTDTCR 0x88U
#define RA4M1_GTDVU 0x8CU
#define RA4M1_GPT_OVERFLOW (1UL << 6)
#define RA4M1_GTWP_KEY 0xA500UL
/* Physical order is A, B, C, E, D, F -- NOT alphabetical. */
static const uint8_t s_Ra4m1GtccrOffset[6] = {0x4C, 0x50, 0x54, 0x5C, 0x58, 0x60};
#ifndef RA4M1_TIMER_READY_POLLS
#define RA4M1_TIMER_READY_POLLS 1000000UL
#endif
#endif
