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
#include <stddef.h>
#include "ra4m1_ioregs.h"
#define RA4M1_TIMER_MSTPCRD ((uintptr_t)&R_MSTP->MSTPCRD)
/* Clock-source selection values are specific to the RA4M1 system clock. */
#define RA4M1_LOCOCR  ((uintptr_t)&R_SYSTEM->LOCOCR)
#define RA4M1_SCKSCR  ((uintptr_t)&R_SYSTEM->SCKSCR)
#define RA4M1_CK_HOCO 0U
#define RA4M1_CK_MOCO 1U
#define RA4M1_CK_MOSC 3U
#define RA4M1_CK_PLL  5U
#define RA4M1_AGT_BASE(n) ((uintptr_t)((n) == 0U ? R_AGT0 : R_AGT1))
#define RA4M1_GPT_BASE(n) ((uintptr_t)R_GPT0 + (uintptr_t)(n) * (R_GPT1_BASE - R_GPT0_BASE))
/* AGT data registers are halfwords; control registers are bytes. */
#define RA4M1_AGT offsetof(R_AGTX0_Type, AGT16.AGT)
#define RA4M1_AGTCMA offsetof(R_AGTX0_Type, AGT16.AGTCMA)
#define RA4M1_AGTCMB offsetof(R_AGTX0_Type, AGT16.AGTCMB)
#define RA4M1_AGTCR offsetof(R_AGTX0_Type, AGT16.CTRL.AGTCR)
#define RA4M1_AGTMR1 offsetof(R_AGTX0_Type, AGT16.CTRL.AGTMR1)
#define RA4M1_AGTMR2 offsetof(R_AGTX0_Type, AGT16.CTRL.AGTMR2)
#define RA4M1_AGTIOC offsetof(R_AGTX0_Type, AGT16.CTRL.AGTIOC)
#define RA4M1_AGTISR offsetof(R_AGTX0_Type, AGT16.CTRL.AGTISR)
#define RA4M1_AGTCMSR offsetof(R_AGTX0_Type, AGT16.CTRL.AGTCMSR)
#define RA4M1_AGTIOSEL offsetof(R_AGTX0_Type, AGT16.CTRL.AGTIOSEL)
#define RA4M1_AGT_START 0x01U
#define RA4M1_AGT_STARTED 0x02U
#define RA4M1_AGT_UNDERFLOW 0x20U
/* All GPT registers require word accesses, including GPT16 channels. */
#define RA4M1_GTWP offsetof(R_GPT0_Type, GTWP)
#define RA4M1_GTSSR offsetof(R_GPT0_Type, GTSSR)
#define RA4M1_GTPSR offsetof(R_GPT0_Type, GTPSR)
#define RA4M1_GTCSR offsetof(R_GPT0_Type, GTCSR)
#define RA4M1_GTUPSR offsetof(R_GPT0_Type, GTUPSR)
#define RA4M1_GTDNSR offsetof(R_GPT0_Type, GTDNSR)
#define RA4M1_GTICASR offsetof(R_GPT0_Type, GTICASR)
#define RA4M1_GTICBSR offsetof(R_GPT0_Type, GTICBSR)
#define RA4M1_GTCR offsetof(R_GPT0_Type, GTCR)
#define RA4M1_GTUDDTYC offsetof(R_GPT0_Type, GTUDDTYC)
#define RA4M1_GTIOR offsetof(R_GPT0_Type, GTIOR)
#define RA4M1_GTINTAD offsetof(R_GPT0_Type, GTINTAD)
#define RA4M1_GTST offsetof(R_GPT0_Type, GTST)
#define RA4M1_GTBER offsetof(R_GPT0_Type, GTBER)
#define RA4M1_GTCNT offsetof(R_GPT0_Type, GTCNT)
#define RA4M1_GTPR offsetof(R_GPT0_Type, GTPR)
#define RA4M1_GTPBR offsetof(R_GPT0_Type, GTPBR)
#define RA4M1_GTDTCR offsetof(R_GPT0_Type, GTDTCR)
#define RA4M1_GTDVU offsetof(R_GPT0_Type, GTDVU)
#define RA4M1_GPT_OVERFLOW (1UL << 6)
#define RA4M1_GTWP_KEY 0xA500UL
/* Physical order is A, B, C, E, D, F -- NOT alphabetical. */
static const uint8_t s_Ra4m1GtccrOffset[6] = {
    (uint8_t)(offsetof(R_GPT0_Type, GTCCR) + 0U * sizeof(uint32_t)),
    (uint8_t)(offsetof(R_GPT0_Type, GTCCR) + 1U * sizeof(uint32_t)),
    (uint8_t)(offsetof(R_GPT0_Type, GTCCR) + 2U * sizeof(uint32_t)),
    (uint8_t)(offsetof(R_GPT0_Type, GTCCR) + 4U * sizeof(uint32_t)),
    (uint8_t)(offsetof(R_GPT0_Type, GTCCR) + 3U * sizeof(uint32_t)),
    (uint8_t)(offsetof(R_GPT0_Type, GTCCR) + 5U * sizeof(uint32_t))
};
#ifndef RA4M1_TIMER_READY_POLLS
#define RA4M1_TIMER_READY_POLLS 1000000UL
#endif
#endif
