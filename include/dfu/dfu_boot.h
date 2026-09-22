/**-------------------------------------------------------------------------
@file	dfu_boot.h

@brief	DFU flash layout, slot 1 trailer, slot 0 record and the stage 0 boot.

The layout is what the linker script declares, as absolute symbols from the
target's dfu_layout_*.ld, so the boot and the application agree on it by
including the same file. No C define repeats an address.

	slot 0		application payload, at its link address
	record		header and TLV areas of the image in slot 0
	slot 1		a complete signed image, as uploaded, then the trailer.
				Only where the application updates itself (OTA). A part
				updated by wire only has no slot 1: stage 0 writes slot 0
				and the record directly.

State changes are single program units into erased memory, never erases,
so a power loss between two of them leaves one of the states below and
nothing else:

	trailer Pending	trailer Done	meaning
	erased			any				nothing to install
	PENDING			erased			install slot 1 at the next boot
	PENDING			not erased		installed, or refused, leave it alone.
									A Done write cut by power loss counts,
									as ECC flash cannot take it again.

The boot writes Done only after slot 0 verifies from the record, so power
lost during the copy leaves slot 1 pending and the next boot copies again.

Each state word sits alone in its own state unit, DfuStateUnit bytes: at
least 16, and at least the target program unit. On ECC flash (STM32L4, WBA)
a program unit takes one program between erases, so two words sharing one
could not be written at different times. The record magic has its unit to
itself for the same reason, and is written last.

	trailer		[end - 2 units] Pending, [end - 1 unit] Done
	record		[unit 0] magic, [unit 1] DfuRecInfo_t, image header, TLVs

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
#ifndef __DFU_BOOT_H__
#define __DFU_BOOT_H__

#include <stdint.h>
#include <stddef.h>

#include "dfu/dfu_image.h"
#include "dfu/dfu_target.h"

/** @addtogroup DFU
  * @{
  */

#define DFU_TRAILER_PENDING		0x444E4550UL	//!< "PEND", install asked
#define DFU_TRAILER_DONE		0x454E4F44UL	//!< "DONE", installed or refused
#define DFU_ERASED_WORD			0xFFFFFFFFUL

#define DFU_REC_MAGIC			0x43455244UL	//!< "DREC"

/// Smallest state unit, whatever the program unit.
#define DFU_STATE_UNIT_MIN		16

/// Largest state unit.
#define DFU_STATE_UNIT_MAX		DFU_TGT_WRITE_UNIT_MAX

/// Recovery request word value, see DfuRecoveryRequest.
#define DFU_FLAG_RECOVERY		0x56434552UL	//!< "RECV"

#pragma pack(push, 4)

/// Record, after its magic unit. The image header follows, HdrLen bytes,
/// then its TLV areas, TlvLen bytes.
typedef struct __Dfu_Rec_Info {
	uint16_t HdrLen;			//!< Image header size, its HdrSize
	uint16_t TlvLen;			//!< Both TLV areas
} DfuRecInfo_t;

#pragma pack(pop)

/// Where things are, from the linker symbols.
typedef struct __Dfu_Layout {
	uintptr_t Slot0;			//!< Application run address
	uint32_t Slot0Size;
	uintptr_t Slot1;			//!< Upload slot, when there is one
	uint32_t Slot1Size;			//!< Trailer included, 0 with no slot 1
	uintptr_t Rec;				//!< Slot 0 record
	uint32_t RecSize;
	uint32_t Unit;				//!< State unit, DfuStateUnit
} DfuLayout_t;

/// Memory mapped region, the context of DfuMapRead.
typedef struct __Dfu_Map {
	uintptr_t Addr;
	uint32_t Size;
} DfuMap_t;

#ifdef __cplusplus

/// State unit of this target: max(DFU_STATE_UNIT_MIN, DfuTgtWriteUnit()).
uint32_t DfuStateUnit(void);

/// Largest record, magic unit included, for a state unit.
static inline uint32_t DfuRecMax(uint32_t Unit) {
	return Unit + sizeof(DfuRecInfo_t) + DFU_IMG_HDR_MAX + DFU_IMG_TLV_MAX;
}

/// Trailer size, the two state units at the end of slot 1.
static inline uint32_t DfuTrailerSize(const DfuLayout_t &Lay) {
	return 2 * Lay.Unit;
}

/// Room for an image in slot 1, trailer excluded.
static inline uint32_t DfuSlot1Room(const DfuLayout_t &Lay) {
	return Lay.Slot1Size > DfuTrailerSize(Lay) ?
		   Lay.Slot1Size - DfuTrailerSize(Lay) : 0;
}

/// Offset in slot 1 of the Pending unit, the Done unit is the next one.
static inline uint32_t DfuTrailerOff(const DfuLayout_t &Lay) {
	return Lay.Slot1Size - DfuTrailerSize(Lay);
}

/**
 * @brief	The layout the linker script declares.
 *
 * @param	pLay : Filled in.
 *
 * @return	false when the project declares no DFU layout, or one that does
 * 			not fit the target's units.
 */
bool DfuLayoutGet(DfuLayout_t *pLay);

/**
 * @brief	DfuImgRead_t over a memory mapped region.
 *
 * @param	pCtx : A DfuMap_t.
 */
bool DfuMapRead(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len);

/**
 * @brief	The record of slot 0, when it holds one that fits the layout.
 *
 * @param	Lay : Layout.
 *
 * @return	Record information in memory, the image header right after it,
 * 			or null.
 */
const DfuRecInfo_t *DfuRecGet(const DfuLayout_t &Lay);

/**
 * @brief	DfuImgRead_t over an image split in a record and a payload.
 *
 * Header and TLV areas come from the record, the payload from its slot, so
 * the offsets are those of the image as it was uploaded.
 *
 * @param	pCtx : A DfuSplit_t.
 */
bool DfuSplitRead(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len);

/// Context of DfuSplitRead.
typedef struct __Dfu_Split {
	const DfuRecInfo_t *pRec;	//!< Record information, header after it
	uintptr_t Payload;			//!< Payload, memory mapped
} DfuSplit_t;

/**
 * @brief	DfuImgRead_t over the image in slot 0.
 *
 * @param	pCtx : The DfuLayout_t.
 */
bool DfuSlot0Read(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len);

/**
 * @brief	Parse the image in slot 0 through its record.
 *
 * @param	Lay   : Layout.
 * @param	pInfo : Filled in.
 *
 * @return	DFU_IMG_OK, DFU_IMG_ERR_MAGIC when there is no record, or another
 * 			DFU_IMG_ERR_ value.
 */
int DfuSlot0Parse(const DfuLayout_t &Lay, DfuImgInfo_t *pInfo);

/// Pending word of the slot 1 trailer, memory mapped.
static inline uint32_t DfuTrailerPending(const DfuLayout_t &Lay) {
	return *(const volatile uint32_t *)(Lay.Slot1 + DfuTrailerOff(Lay));
}

/// Done word of the slot 1 trailer, memory mapped.
static inline uint32_t DfuTrailerDone(const DfuLayout_t &Lay) {
	return *(const volatile uint32_t *)(Lay.Slot1 + DfuTrailerOff(Lay) +
										Lay.Unit);
}

/**
 * @brief	Erase the units of the internal memory covering a range.
 *
 * Through the target layer, one unit at a time at its own size, so a range
 * over sectors of several sizes erases each once. A unit is erased even
 * when it reads as all ones: on ECC flash ones may have been programmed.
 *
 * @param	Addr : Start, rounded down to the unit holding it.
 * @param	Len  : Length, rounded up to whole units.
 *
 * @return	true when the range reads as all ones.
 */
bool DfuEraseRange(uintptr_t Addr, uint32_t Len);

/**
 * @brief	Write a state word alone in its unit, ones around it.
 *
 * @param	Addr  : Start of an erased state unit.
 * @param	Unit  : State unit.
 * @param	Value : The word.
 */
bool DfuWriteState(uintptr_t Addr, uint32_t Unit, uint32_t Value);

/// Largest record body: information, image header, TLV areas.
#define DFU_REC_BODY_MAX		(sizeof(DfuRecInfo_t) + DFU_IMG_HDR_MAX + \
								 DFU_IMG_TLV_MAX)

/**
 * @brief	The one record body buffer, where a record is built in RAM.
 *
 * DFU_REC_BODY_MAX bytes plus a program unit of padding, word aligned. The
 * boot install and the direct upload both build here, never at once.
 */
uint8_t *DfuRecBody(void);

/**
 * @brief	Write the record of an image whose payload is already in slot 0.
 *
 * The body is what DfuRecBody holds: DfuRecInfo_t, image header, TLVs. The
 * record region is erased, the body written, then the magic unit last.
 * Until the magic is in, there is no record.
 *
 * @param	Lay     : Layout.
 * @param	BodyLen : Bytes of DfuRecBody in use.
 */
bool DfuRecWrite(const DfuLayout_t &Lay, uint32_t BodyLen);

/// Erase the record: slot 0 no longer holds a startable image.
bool DfuRecClear(const DfuLayout_t &Lay);

/**
 * @brief	Ask stage 0 to stay in recovery after the next reset.
 *
 * Sets the request word in RAM both the boot and the application leave
 * alone (__dfu_flag in the layout), then resets. The boot clears it.
 */
void DfuRecoveryRequest(void) __attribute__((noreturn));

/**
 * @brief	Whether the application asked for recovery. Clears the request.
 *
 * @return	true once per request.
 */
bool DfuRecoveryAsked(void);

/// Stage 0 configuration.
typedef struct __Dfu_Boot_Cfg {
	HashEngine *pHash;			//!< SHA-256
	SignEngine *pSign;			//!< P-256 verify
	const DfuImgKey_t *pKey;	//!< Keys an image may be signed with
	int NbKey;
	bool bVerifySlot0;			//!< Verify slot 0 at every boot, not only after
								//!< an install
	bool bAllowNoRec;			//!< Start slot 0 with no record. Debugger
								//!< development only: it starts unsigned code
} DfuBootCfg_t;

/**
 * @brief	Verify an image with each key until one takes it.
 *
 * @return	DFU_IMG_OK or the DFU_IMG_ERR_ value of the last key tried.
 */
int DfuBootVerify(const DfuBootCfg_t &Cfg, DfuImgRead_t Read, void *pCtx,
				  const DfuImgInfo_t &Info);

/**
 * @brief	Stage 0: install slot 1 when it is pending, then start slot 0.
 *
 * @param	Cfg : Configuration.
 *
 * @return	Only when slot 0 is not started: DFU_IMG_OK when the application
 * 			asked for recovery (DfuRecoveryRequest), else the DFU_IMG_ERR_
 * 			value that says why there is nothing to start. The boot then
 * 			serves recovery.
 */
int DfuBootRun(const DfuBootCfg_t &Cfg);

#endif

/** @} */

#endif
