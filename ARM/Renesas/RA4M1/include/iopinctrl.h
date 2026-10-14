/**-------------------------------------------------------------------------
@file	iopinctrl.h

@brief	Native RA4M1 fast GPIO control; the target header is always iopinctrl.h.

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
#ifndef __IOPINCTRL_H__
#define __IOPINCTRL_H__
#include "ra4m1xxx.h"
#include <stdbool.h>
#include "coredev/iopincfg.h"

#define RA4M1_IELS_MASK   0xFFUL
#define RA4M1_IELS_IR     (1UL << 16)
#define RA4M1_IELS_DTCE   (1UL << 24)
#define RA4M1_PFS_PODR    (1UL << 0)
#define RA4M1_PFS_PIDR    (1UL << 1)
#define RA4M1_PFS_PDR     (1UL << 2)
#define RA4M1_PFS_PCR     (1UL << 4)
#define RA4M1_PFS_NCODR   (1UL << 6)
#define RA4M1_PFS_DSCR    (1UL << 10)
#define RA4M1_PFS_DSCR1   (1UL << 11) // P408 only
#define RA4M1_PFS_EOR     (1UL << 12)
#define RA4M1_PFS_EOF     (1UL << 13)
#define RA4M1_PFS_ISEL    (1UL << 14)
#define RA4M1_PFS_ASEL    (1UL << 15)
#define RA4M1_PFS_PMR     (1UL << 16)
#define RA4M1_PFS_PSEL    (0x1FUL << 24)
#define RA4M1_PWPR_B0WI   0x80U
#define RA4M1_PWPR_PFSWE  0x40U


#ifndef RA4M1_PACKAGE_PINS
#define RA4M1_PACKAGE_PINS 100
#endif
static inline R_PORT0_Type *Ra4m1PortRegs(int PortNo)
{
	static R_PORT0_Type * const ports[10] = {
		R_PORT0, R_PORT1, R_PORT2, R_PORT3, R_PORT4,
		R_PORT5, R_PORT6, R_PORT7, R_PORT8, R_PORT9
	};
	return (unsigned)PortNo < 10U ? ports[PortNo] : (R_PORT0_Type *)0;
}

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
	// P200/P214/P215 are input-only. USB pair is owned by the future USB
	// driver: its one-time paired mux transition is not a per-pin operation.
	uint32_t mask = Ra4m1PortMask(PortNo);
	if (PortNo == 2)
	{
		mask &= ~0xC001UL;
	}
	if (PortNo == 9)
	{
		mask = 0;
	}
	if (PortNo >= 1 && PortNo <= 4) {
		uint32_t elc = Ra4m1PortRegs(PortNo)->PCNTR4;
		mask &= ~(elc | (elc >> 16));
	}
	return mask;
}

// IOPinConfig must first establish GPIO operation. These fast controls do
// not steal pins from an oscillator/peripheral, or initialize VBATT buffers.
// Changing direction also needs PFS write protection, so it is out of line.
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
		(int)((Ra4m1PortRegs(PortNo)->PCNTR2 >> PinNo) & 1U) : 0;
}
static inline __attribute__((always_inline)) void IOPinSet(int PortNo, int PinNo)
{
	if (Ra4m1PinValid(PortNo, PinNo)) {
		uint32_t bit = (1UL << PinNo) & Ra4m1OutputMask(PortNo);
		if (bit) Ra4m1PortRegs(PortNo)->PCNTR3 =  bit;
	}
}
static inline __attribute__((always_inline)) void IOPinClear(int PortNo, int PinNo)
{
	if (Ra4m1PinValid(PortNo, PinNo)) {
		uint32_t bit = (1UL << PinNo) & Ra4m1OutputMask(PortNo);
		if (bit) Ra4m1PortRegs(PortNo)->PCNTR3 =  bit << 16;
	}
}
static inline __attribute__((always_inline)) void IOPinToggle(int PortNo, int PinNo)
{
	if (!Ra4m1PinValid(PortNo, PinNo)) return;
	uint32_t state = DisableInterrupt();
	uint32_t bit = (1UL << PinNo) & Ra4m1OutputMask(PortNo);
	uint32_t high = Ra4m1PortRegs(PortNo)->PCNTR1 >> 16;
	if (bit) Ra4m1PortRegs(PortNo)->PCNTR3 =  (high & bit) ? bit << 16 : bit;
	EnableInterrupt(state);
}
static inline __attribute__((always_inline)) uint32_t IOPinReadPort(int PortNo)
{
	uint32_t mask = Ra4m1PortMask(PortNo);
	return mask ? Ra4m1PortRegs(PortNo)->PCNTR2 & mask : 0U;
}
static inline __attribute__((always_inline)) void IOPinWritePort(int PortNo, uint32_t Data)
{
	uint32_t mask = Ra4m1OutputMask(PortNo);
	if (mask) Ra4m1PortRegs(PortNo)->PCNTR3 = 
		(Data & mask) | ((~Data & mask) << 16);
}
#endif
