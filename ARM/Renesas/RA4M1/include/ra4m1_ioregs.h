/**-------------------------------------------------------------------------
@file	ra4m1_ioregs.h

@brief	RA4M1 GPIO and ICU register subset (not a complete device header).

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
#ifndef RA4M1_IOREGS_H
#define RA4M1_IOREGS_H
#include <stdint.h>
#include <stdbool.h>
#include "ra4m1xxx.h"

/* R01UH0887EJ0110 chapters 13 and 19. Word accesses avoid the overlapping
 * halfword register aliases. Tests override only the accessors, not addresses.
 */
#ifndef RA4M1_RD8
#define RA4M1_RD8(a)       (*(volatile const uint8_t *)(uintptr_t)(a))
#define RA4M1_WR8(a, v)    (*(volatile uint8_t *)(uintptr_t)(a) = (uint8_t)(v))
#define RA4M1_RD16(a)      (*(volatile const uint16_t *)(uintptr_t)(a))
#define RA4M1_WR16(a, v)   (*(volatile uint16_t *)(uintptr_t)(a) = (uint16_t)(v))
#define RA4M1_RD32(a)      (*(volatile const uint32_t *)(uintptr_t)(a))
#define RA4M1_WR32(a, v)   (*(volatile uint32_t *)(uintptr_t)(a) = (uint32_t)(v))
#endif
#define RA4M1_PCNTR1(p)    (0x40040000UL + 0x20UL * (p))
#define RA4M1_PCNTR2(p)    (0x40040004UL + 0x20UL * (p))
#define RA4M1_PCNTR3(p)    (0x40040008UL + 0x20UL * (p))
#define RA4M1_PCNTR4(p)    (0x4004000CUL + 0x20UL * (p))
#define RA4M1_PFS(p, n)    (0x40040800UL + 0x40UL * (p) + 4UL * (n))
#define RA4M1_PWPR        0x40040D03UL
#define RA4M1_IRQCR(n)    (0x40006000UL + (n))
#define RA4M1_WUPEN       0x400061A0UL
#define RA4M1_DELSR(n)    (0x40006280UL + 4UL * (n))
#define RA4M1_IELSR(n)    (0x40006300UL + 4UL * (n))
#define RA4M1_IELS_MASK   0xFFUL
#define RA4M1_IELS_IR     (1UL << 16)
#define RA4M1_IELS_DTCE   (1UL << 24)
#define RA4M1_PFS_PODR    (1UL << 0)
#define RA4M1_PFS_PIDR    (1UL << 1)
#define RA4M1_PFS_PDR     (1UL << 2)
#define RA4M1_PFS_PCR     (1UL << 4)
#define RA4M1_PFS_NCODR   (1UL << 6)
#define RA4M1_PFS_DSCR    (1UL << 10)
#define RA4M1_PFS_DSCR1   (1UL << 11) /* P408 only */
#define RA4M1_PFS_EOR     (1UL << 12)
#define RA4M1_PFS_EOF     (1UL << 13)
#define RA4M1_PFS_ISEL    (1UL << 14)
#define RA4M1_PFS_ASEL    (1UL << 15)
#define RA4M1_PFS_PMR     (1UL << 16)
#define RA4M1_PFS_PSEL    (0x1FUL << 24)
#define RA4M1_PWPR_B0WI   0x80U
#define RA4M1_PWPR_PFSWE  0x40U

/* Physical MCU package, not application wiring. Unspecified means the group
 * superset (100-pin). The caller must still use a pin bonded on its device.
 */
#ifndef RA4M1_PACKAGE_PINS
#define RA4M1_PACKAGE_PINS 100
#endif
static inline uint32_t Ra4m1PortMask(int PortNo)
{
#if RA4M1_PACKAGE_PINS == 100
    static const uint16_t masks[10] = {
        0xFDFF, 0xFFFF, 0xF07F, 0x00FF, 0xFFFF, 0x003F, 0x070F, 0x0100, 0x0300, 0xC000
    };
#elif RA4M1_PACKAGE_PINS == 64
    static const uint16_t masks[10] = {
        0xFC1F, 0x3FFF, 0xF073, 0x001F, 0x0F87, 0x0007, 0, 0, 0, 0xC000
    };
#elif RA4M1_PACKAGE_PINS == 48
    static const uint16_t masks[10] = {
        0xFC07, 0x1F1F, 0xF043, 0x0007, 0x0381, 0x0001, 0, 0, 0, 0xC000
    };
#elif RA4M1_PACKAGE_PINS == 40
    static const uint16_t masks[10] = {
        0xFC03, 0x1F07, 0xF003, 0x0003, 0x0180, 0, 0, 0, 0, 0xC000
    };
#else
#error "RA4M1_PACKAGE_PINS must be 40, 48, 64, or 100"
#endif
    return (unsigned)PortNo < 10U ? masks[PortNo] : 0U;
}
static inline bool Ra4m1PinValid(int PortNo, int PinNo)
{
    return (unsigned)PinNo < 16U &&
        (Ra4m1PortMask(PortNo) & (1UL << (unsigned)PinNo)) != 0U;
}
static inline uint32_t Ra4m1OutputMask(int PortNo)
{
    /* P200/P214/P215 are input-only. USB pair is owned by the future USB
     * backend: its one-time paired mux transition is not a per-pin operation.
     */
    uint32_t mask = Ra4m1PortMask(PortNo);
    if (PortNo == 2) mask &= ~0xC001UL;
    if (PortNo == 9) mask = 0;
    if (PortNo >= 1 && PortNo <= 4) {
        uint32_t elc = RA4M1_RD32(RA4M1_PCNTR4(PortNo));
        mask &= ~(elc | (elc >> 16));
    }
    return mask;
}
#endif
