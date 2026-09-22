/**-------------------------------------------------------------------------
@file	dfu_rv.h

@brief	RISC-V part of the DFU target layer: entry check, quiesce and
		start. Inline, included by a dfu_<mcu> file after its device header.

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
#ifndef __DFU_RV_H__
#define __DFU_RV_H__

#include <stdint.h>
#include <stdbool.h>

/** @addtogroup DFU
  * @{
  */

/**
 * @brief	Entry check of a RISC-V image.
 *
 * There is no vector table to read, so the check is on the entry itself:
 * the first instruction word of the payload is programmed (neither erased
 * nor zero) and its low bits make a valid instruction length. The target
 * says where the entry is, from the start of the payload.
 *
 * @param	pImg     : Payload.
 * @param	ImgSize  : Payload size.
 * @param	EntryOff : Offset of the entry in the payload.
 */
static inline bool DfuRvEntryValid(const void *pImg, uint32_t ImgSize,
								   uint32_t EntryOff)
{
	if (pImg == 0 || ImgSize < EntryOff + 4)
	{
		return false;
	}

	const uint8_t *p = (const uint8_t *)pImg + EntryOff;
	uint32_t w = (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
				 ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);

	return w != 0xFFFFFFFFUL && w != 0UL;
}

/// Machine interrupts off, globally and per source.
static inline void DfuRvQuiesce(void)
{
	__asm volatile("csrci mstatus, 8" ::: "memory");
	__asm volatile("csrw mie, zero" ::: "memory");
}

/**
 * @brief	Jump to an application entry.
 *
 * The application's reset entry sets its own stack, global pointer and trap
 * vector, so all this does is make sure no interrupt comes in between and
 * that the instruction fetch sees what was written.
 */
__attribute__((noreturn, unused)) static void DfuRvStart(uintptr_t Entry)
{
	DfuRvQuiesce();
	// fence.i, as a word: not every -march this builds with has Zifencei.
	__asm volatile(".word 0x0000100f" ::: "memory");

	((void (*)(void))Entry)();

	for (;;)
	{
	}
}

/** @} */

#endif
