/**-------------------------------------------------------------------------
@file	ra4m1_startup_regs.h

@brief	Private RA4M1 startup register subset, R01UH0887EJ0110.

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
#ifndef RA4M1_STARTUP_REGS_H
#define RA4M1_STARTUP_REGS_H

/* Native bootstrap subset, not a replacement for a full vendor device header.
 * Register access widths and addresses are from chapters 8, 10, 12, 13, 44
 * of the RA4M1 hardware manual. In particular PLLCCR2 is BYTE-wide at E02B.
 * Test accessors model these exact addresses; production is direct volatile IO.
 */
#ifndef RA4M1_RD8
#define RA4M1_RD8(a)        (*(volatile const uint8_t *)(uintptr_t)(a))
#define RA4M1_WR8(a, v)     (*(volatile uint8_t *)(uintptr_t)(a) = (uint8_t)(v))
#define RA4M1_RD16(a)       (*(volatile const uint16_t *)(uintptr_t)(a))
#define RA4M1_WR16(a, v)    (*(volatile uint16_t *)(uintptr_t)(a) = (uint16_t)(v))
#define RA4M1_RD32(a)       (*(volatile const uint32_t *)(uintptr_t)(a))
#define RA4M1_WR32(a, v)    (*(volatile uint32_t *)(uintptr_t)(a) = (uint32_t)(v))
#endif

#define RA4M1_HOCOCR2       0x4001E000UL
#define RA4M1_SCKDIVCR      0x4001E020UL
#define RA4M1_SCKSCR        0x4001E026UL
#define RA4M1_PLLCR         0x4001E02AUL
#define RA4M1_PLLCCR2       0x4001E02BUL
#define RA4M1_MEMWAIT       0x4001E031UL
#define RA4M1_MOSCCR        0x4001E032UL
#define RA4M1_SOSCCR        0x4001E033UL
#define RA4M1_LOCOCR        0x4001E034UL
#define RA4M1_HOCOCR        0x4001E036UL
#define RA4M1_MOCOCR        0x4001E038UL
#define RA4M1_OSCSF         0x4001E03CUL
#define RA4M1_OSTDCR        0x4001E040UL
#define RA4M1_OSTDSR        0x4001E041UL
#define RA4M1_HOCOUTCR      0x4001E062UL
#define RA4M1_OPCCR         0x4001E0A0UL
#define RA4M1_MOSCWTCR      0x4001E0A2UL
#define RA4M1_HOCOWTCR      0x4001E0A5UL
#define RA4M1_SOPCCR        0x4001E0AAUL
#define RA4M1_USBCKCR       0x4001E0D0UL
#define RA4M1_PRCR          0x4001E3FEUL
#define RA4M1_MOMCR         0x4001E413UL
#define RA4M1_SOMCR         0x4001E481UL
#define RA4M1_FCACHEE       0x4001C100UL
#define RA4M1_FCACHEIV      0x4001C104UL
#define RA4M1_IELSR(n)      (0x40006300UL + 4UL * (n))

#define RA4M1_PRCR_KEY      0xA500U
#define RA4M1_PRCR_PRC0     0x01U
#define RA4M1_PRCR_PRC1     0x02U
#define RA4M1_OSCSF_HOCO    0x01U
#define RA4M1_OSCSF_MOSC    0x08U
#define RA4M1_OSCSF_PLL     0x20U
#define RA4M1_MODE_BUSY     0x10U
#define RA4M1_CK_HOCO       0U
#define RA4M1_CK_MOCO       1U
#define RA4M1_CK_LOCO       2U
#define RA4M1_CK_MOSC       3U
#define RA4M1_CK_SOSC       4U
#define RA4M1_CK_PLL        5U
#define RA4M1_MOCO_HZ       8000000UL
#define RA4M1_LOCO_HZ       32768UL
#define RA4M1_RESET_DIV     0x44044444UL
#define RA4M1_FAST_DIV      0x10010100UL
#define RA4M1_SLOW_DIV      0x00000000UL

#endif
