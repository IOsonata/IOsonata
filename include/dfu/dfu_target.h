/**-------------------------------------------------------------------------
@file	dfu_target.h

@brief	DFU target layer: what the generic DFU code needs from an MCU port.

The generic layer (image checks, stage 0 boot, SMP server, transports) is
the same on every MCU. What differs is in one file per MCU family,
dfu_<mcu>, plus the flash layout in the target's dfu_layout_*.ld:

	internal memory	geometry, erase, program
	start			hand the core to the application
	reset			restart the MCU
	entry check		does an image look like it runs at its slot

The internal memory is read memory mapped on every target, so there is no
read call. A target whose memory sits behind a cache (Espressif, L4 flash
cache) keeps the mapped view current itself after an erase or a write.

The functions run from the stage 0 boot, before any stack, and from the
application when no radio stack owns the memory. Where a stack arbitrates
the memory (nRF52 SoftDevice, nRF54L MPSL), the application writes slot 1
through Nvm and NvmIntrf instead and uses only the pure functions here:
DfuTgtWriteUnit and DfuTgtEntryValid.

Each library configuration links exactly one dfu_<mcu> file.

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
#ifndef __DFU_TARGET_H__
#define __DFU_TARGET_H__

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/** @addtogroup DFU
  * @{
  */

/// Largest program unit a target may report. LPC IAP programs 256 bytes at
/// a time, SAM4 a 512 byte page.
#define DFU_TGT_WRITE_UNIT_MAX		512

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Program unit of the internal memory.
 *
 * What one program operation takes, whole and aligned: 4 on NVMC and RRAMC,
 * 2 on STM32F0, 8 or 16 on the ECC flash of STM32L4 and WBA, 256 on LPC,
 * 512 on SAM4. A unit is programmed once between two erases.
 *
 * @return	Power of 2, at most DFU_TGT_WRITE_UNIT_MAX.
 */
uint32_t DfuTgtWriteUnit(void);

/**
 * @brief	Erase unit holding Addr.
 *
 * Uniform on most parts. On a part with sectors of several sizes (STM32F4)
 * it is the size of the sector holding Addr.
 *
 * @param	Addr : Mapped address.
 *
 * @return	Unit size, 0 when Addr is not internal memory.
 */
uint32_t DfuTgtEraseUnit(uintptr_t Addr);

/**
 * @brief	Erase the unit starting at Addr.
 *
 * Afterwards the unit reads as all ones. On a memory with no erase (RRAM,
 * MRAM) ones are written over it.
 *
 * @param	Addr : Mapped address, start of a unit.
 *
 * @return	true on success.
 */
bool DfuTgtErase(uintptr_t Addr);

/**
 * @brief	Program erased memory.
 *
 * @param	Addr  : Mapped address, a multiple of DfuTgtWriteUnit.
 * @param	pData : Data, any alignment, may be in the internal memory.
 * @param	Len   : A multiple of DfuTgtWriteUnit.
 *
 * @return	true on success.
 */
bool DfuTgtWrite(uintptr_t Addr, const void *pData, uint32_t Len);

/**
 * @brief	Does the start of an image look like code that runs at RunAddr.
 *
 * On Cortex-M the vector table: stack pointer in RAM, reset handler inside
 * the image. On RISC-V the entry the target starts. Checked before anything
 * is marked or copied, so an image built for another slot is refused.
 *
 * @param	pImg    : First bytes of the payload, at least 8, word aligned.
 * @param	RunAddr : Where the payload runs.
 * @param	ImgSize : Payload size.
 *
 * @return	true when it can be started there.
 */
bool DfuTgtEntryValid(const void *pImg, uintptr_t RunAddr, uint32_t ImgSize);

/**
 * @brief	Hand the core to the application whose payload is at RunAddr.
 *
 * Peripherals the boot used are left for the application to set up again;
 * interrupts are off at the NVIC or interrupt matrix.
 *
 * @param	RunAddr : Slot 0.
 */
void DfuTgtStart(uintptr_t RunAddr) __attribute__((noreturn));

/// Restart the MCU the way a reset pin would, RAM kept.
void DfuTgtReset(void) __attribute__((noreturn));

#ifdef __cplusplus
}
#endif

/** @} */

#endif
