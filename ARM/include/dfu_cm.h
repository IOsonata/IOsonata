/**-------------------------------------------------------------------------
@file	dfu_cm.h

@brief	Cortex-M part of the DFU target layer: entry check, quiesce and
		start. Inline, included by a dfu_<mcu> file after its device
		header, which is what brings in the core definitions.

@author	Hoang Nguyen Hoan
@date	Sep. 21, 2026

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
#ifndef __DFU_CM_H__
#define __DFU_CM_H__

#include <stdint.h>
#include <stdbool.h>

#include "dfu/dfu_image.h"

/** @addtogroup DFU
  * @{
  */

/// SRAM region of the Cortex-M memory map, where the initial stack pointer
/// of most parts points. A part with RAM elsewhere (LPC11U, LPC17 local
/// SRAM) passes its own range.
#define DFU_CM_SRAM_START		0x20000000UL
#define DFU_CM_SRAM_END			0x40000000UL

#if defined(__ARM_ARCH_7M__) || defined(__ARM_ARCH_7EM__) || \
	defined(__ARM_ARCH_8M_MAIN__) || defined(__ARM_ARCH_8_1M_MAIN__)
#define DFU_CM_MAINLINE			1	//!< BASEPRI and FAULTMASK exist
#else
#define DFU_CM_MAINLINE			0
#endif

#if (defined(__VTOR_PRESENT) && __VTOR_PRESENT) || DFU_CM_MAINLINE || \
	defined(__ARM_ARCH_8M_BASE__)
#define DFU_CM_HAS_VTOR			1
#else
#define DFU_CM_HAS_VTOR			0	//!< Cortex-M0: vectors stay at 0
#endif

/// Entry check of a Cortex-M image: its vector table.
static inline bool DfuCmEntryValid(const void *pImg, uintptr_t RunAddr,
								   uint32_t ImgSize, uintptr_t RamStart,
								   uintptr_t RamEnd)
{
	return DfuImgVectorValid((const uint32_t *)pImg, RunAddr, ImgSize,
							 RamStart, RamEnd);
}

/// Every NVIC interrupt off and not pending, SysTick stopped. PRIMASK is
/// left clear: a call into an MBR or ROM through SVC faults with it set.
static inline void DfuCmQuiesce(void)
{
	for (unsigned i = 0; i < sizeof(NVIC->ICER) / sizeof(NVIC->ICER[0]); i++)
	{
		NVIC->ICER[i] = 0xFFFFFFFFUL;
		NVIC->ICPR[i] = 0xFFFFFFFFUL;
	}
	SysTick->CTRL = 0;
	SCB->ICSR = SCB_ICSR_PENDSTCLR_Msk;
}

// Load the stack pointer and branch, with nothing of the caller's frame used
// after the stack moves.
__attribute__((noreturn, naked, unused)) static void DfuCmJump(
	uint32_t Sp, uint32_t Pc)
{
	(void)Sp;
	(void)Pc;
	__asm volatile(
		"msr msp, r0	\n"
		"bx r1			\n"
	);
}

/**
 * @brief	Start the image whose vector table is at pVec.
 *
 * @param	pVec  : Vector table to take the stack pointer and reset from.
 * @param	bVtor : Point VTOR at it. False where something else forwards the
 * 			interrupts (nRF52 MBR and SoftDevice) or where there is no VTOR
 * 			and the target remapped the vectors itself.
 */
__attribute__((noreturn, unused)) static void DfuCmStart(const uint32_t *pVec,
														 bool bVtor)
{
#if DFU_CM_HAS_VTOR
	if (bVtor)
	{
		SCB->VTOR = (uint32_t)(uintptr_t)pVec;
	}
#else
	(void)bVtor;
#endif

	__DSB();
	__ISB();

	__set_CONTROL(0);
#if DFU_CM_MAINLINE
	__set_BASEPRI(0);
	__set_FAULTMASK(0);
#endif
	__ISB();

	DfuCmJump(pVec[0], pVec[1]);
}

/** @} */

#endif
