/**-------------------------------------------------------------------------
@file	nvm_mcu.h

@brief	MCU internal flash / NVM driver on the NvmIO base.

MCU internal memory behaves like a serial flash chip. It has a write enable
step, an erase command, a program step and a ready poll, and it programs by
clearing bits only so a page must be erased before it is rewritten. The
differences are the mechanism and the granularity:

	Serial flash			MCU internal
	WREN command			write enable in the memory controller
	sector erase command	page erase in the memory controller
	program command			word stores to the mapped address
	poll status WIP			poll the controller ready flag
	read command			direct read, the memory is mapped

Reads are therefore a plain copy from the mapped address and need no transport
at all, which is why this driver takes no DeviceIntrf.

The one part that varies per target is who schedules the program and erase.
Internal memory shares the memory bus with the running code, so the controller
stalls instruction fetch for the duration of the operation. A page erase is far
longer than a radio event can tolerate, so on a wireless part the operation has
to be scheduled around radio activity. That scheduling differs completely
between targets: a full SoftDevice performs it and reports through a system
event, an MPSL timeslot implementation slices the erase across timeslots with
the code running from RAM, and a target with no stack simply performs the
operation and waits.

For that reason the program and erase operations are supplied in an NvmMcuOp_t,
the same way NvmFlash takes its read and write command pair in its config. Each
operation includes its own arbitration and returns once the data is committed,
so the caller sees the same uniform NvmIO verbs on every target.

@author	Hoang Nguyen Hoan
@date	Jul. 22, 2026

@license

MIT License

Copyright (c) 2026, I-SYST, all rights reserved

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
#ifndef __NVM_MCU_H__
#define __NVM_MCU_H__

#include <stdint.h>

#include "storage/nvmio.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/**
 * @brief	Program words into internal memory.
 *
 * The implementation performs the write enable, the word stores and the ready
 * wait, and includes whatever arbitration the target needs against the radio
 * or any other user of the memory. It returns once the data is committed.
 *
 * @param	Addr		: Absolute address, aligned to the write granularity
 * @param	pSrc		: Source words
 * @param	WordCnt		: Number of words to program
 * @param	pCtx		: Target context from NvmMcuOp_t
 *
 * @return	0 on success, negative errno on failure.
 */
typedef int (*NvmMcuProg_t)(uintptr_t Addr, const uint32_t *pSrc,
							uint32_t WordCnt, void *pCtx);

/**
 * @brief	Erase the page starting at Addr.
 *
 * Same arbitration note as the program operation. Returns once the page reads
 * back erased.
 *
 * @param	Addr	: Absolute page start address
 * @param	pCtx	: Target context from NvmMcuOp_t
 *
 * @return	0 on success, negative errno on failure.
 */
typedef int (*NvmMcuErase_t)(uintptr_t Addr, void *pCtx);

/**
 * @brief	Set or clear the memory controller region write protect.
 *
 * Optional. A target without region protect leaves this NULL and the driver
 * reports the operation as not supported.
 *
 * @param	Addr	: Absolute start address of the protected range
 * @param	Len		: Length of the range in bytes
 * @param	bEnable	: true to protect, false to unprotect
 * @param	pCtx	: Target context from NvmMcuOp_t
 *
 * @return	0 on success, negative errno on failure.
 */
typedef int (*NvmMcuProt_t)(uintptr_t Addr, uint32_t Len, bool bEnable,
							void *pCtx);

/// Target operations. The equivalent of the command pair a serial flash config
/// supplies, for a memory whose commands are controller operations.
typedef struct __Nvm_Mcu_Op {
	NvmMcuProg_t	Program;		//!< Program words, arbitration included
	NvmMcuErase_t	ErasePage;		//!< Erase one page
	NvmMcuProt_t	SetProtect;		//!< Region write protect, may be NULL
	uint32_t		MaxProgWords;	//!< Largest word count one Program call
									//!< accepts. 0 : no limit. A stack that
									//!< limits the size of a single program
									//!< request sets this and the driver splits
									//!< the write to match.
	void			*pCtx;			//!< Passed unchanged to each operation
} NvmMcuOp_t;

/// @brief	MCU internal flash / NVM device on the NvmIO API.
///
/// The memory is mapped, so there is no address auto increment window and
/// PageSize() stays 0. A write is split to the largest word count the target
/// program operation accepts, which is a different limit from a page.
class NvmMcu : public NvmIO {
public:
	NvmMcu();
	virtual ~NvmMcu() {}
	NvmMcu(NvmMcu&) = delete;

	/**
	 * @brief	Initialize the device and set the region window.
	 *
	 * @param	Cfg			: Device configuration. BaseAddr is the mapped base
	 *						  address of the memory, EraseSize the page size,
	 *						  WriteGran the word size.
	 * @param	Op			: Target program, erase and protect operations
	 * @param	RegionOff	: Region start relative to BaseAddr in bytes
	 * @param	RegionSize	: Region size in bytes, 0 : to the end of device
	 *
	 * @return	true on success.
	 */
	bool Init(const NvmCfg_t &Cfg, const NvmMcuOp_t &Op,
			  uint64_t RegionOff = 0, uint64_t RegionSize = 0);

	// *** NvmIO ***

	uint32_t EraseSize(void) const override { return vPageSize; }
	uint32_t WriteGran(void) const override { return vWrGran; }
	int Read(uint64_t Off, void *pBuf, uint32_t Len) override;
	int Write(uint64_t Off, const void *pData, uint32_t Len) override;
	int Erase(uint64_t Off, uint32_t Len) override;
	int SetWriteProtect(uint64_t Off, uint32_t Len, bool bEnable) override;

	// *** Device ***

	bool Enable(void) override { return true; }
	void Disable(void) override {}
	void Reset(void) override {}

private:
	uint64_t		vBaseAddr;		//!< Mapped base address of the memory
	uint64_t		vDevSize;		//!< Whole device size in bytes
	uint32_t		vPageSize;		//!< Erase page size in bytes
	uint32_t		vWrGran;		//!< Word size in bytes
	uint32_t		vMaxProgWords;	//!< Word limit of one program call
	NvmMcuProg_t	vProgram;		//!< Target program operation
	NvmMcuErase_t	vErasePage;		//!< Target erase operation
	NvmMcuProt_t	vSetProtect;	//!< Target protect operation, may be NULL
	void			*vpCtx;			//!< Target context
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __NVM_MCU_H__
