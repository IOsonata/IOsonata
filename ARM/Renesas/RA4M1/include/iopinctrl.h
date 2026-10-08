/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	Native RA4M1 fast GPIO control; the target header is always iopinctrl.h.

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__
#include "ra4m1_ioregs.h"
#include "coredev/iopincfg.h"

/* IOPinConfig must first establish GPIO operation. These fast controls do
 * not steal pins from an oscillator/peripheral, or initialize VBATT buffers.
 * Changing direction also needs PFS write protection, so it is out of line.
 */
#ifdef __cplusplus
extern "C" {
#endif
void IOPinSetDir(int PortNo, int PinNo, IOPINDIR Dir);
#ifdef __cplusplus
}
#endif
static inline __attribute__((always_inline)) int IOPinRead(int PortNo, int PinNo)
{
    return Ra4m1PinValid(PortNo, PinNo) ?
        (int)((RA4M1_RD32(RA4M1_PCNTR2(PortNo)) >> PinNo) & 1U) : 0;
}
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo)
{
    if (Ra4m1PinValid(PortNo, PinNo)) {
        uint32_t bit = (1UL << PinNo) & Ra4m1OutputMask(PortNo);
        if (bit) RA4M1_WR32(RA4M1_PCNTR3(PortNo), bit);
    }
}
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo)
{
    if (Ra4m1PinValid(PortNo, PinNo)) {
        uint32_t bit = (1UL << PinNo) & Ra4m1OutputMask(PortNo);
        if (bit) RA4M1_WR32(RA4M1_PCNTR3(PortNo), bit << 16);
    }
}
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo)
{
    if (!Ra4m1PinValid(PortNo, PinNo)) return;
    uint32_t state = DisableInterrupt();
    uint32_t bit = (1UL << PinNo) & Ra4m1OutputMask(PortNo);
    uint32_t high = RA4M1_RD32(RA4M1_PCNTR1(PortNo)) >> 16;
    if (bit) RA4M1_WR32(RA4M1_PCNTR3(PortNo), (high & bit) ? bit << 16 : bit);
    EnableInterrupt(state);
}
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo)
{
    uint32_t mask = Ra4m1PortMask(PortNo);
    return mask ? RA4M1_RD32(RA4M1_PCNTR2(PortNo)) & mask : 0U;
}
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data)
{
    uint32_t mask = Ra4m1OutputMask(PortNo);
    if (mask) RA4M1_WR32(RA4M1_PCNTR3(PortNo),
        (Data & mask) | ((~Data & mask) << 16));
}
#endif /* __IOPINCTRL_H__ */
