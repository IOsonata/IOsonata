/**-------------------------------------------------------------------------
@file	dfu_writer.h

@brief	Where an uploaded image goes, and writing it there as it arrives.

A store is the memory an image is written to, seen through four calls. Two
are provided:

	DfuStoreTgt		internal memory through the target layer, dfu_target.h.
					Used by stage 0, and by an application on a part where
					no radio stack owns the memory.
	DfuStoreNvm		an Nvm, dfu_store_nvm.cpp. Used by an application where
					a stack arbitrates the memory (nRF52 SoftDevice, nRF54L
					MPSL): its NvmIntrf asks the stack for each operation.

The writer takes an image in order, the way every transport delivers one,
in either of two modes:

	slot 1		the image as it is, into slot 1 (OTA from the application).
				Marked pending once checked; stage 0 installs it.
	direct		header and TLV areas into the record, built in RAM, the
				payload straight into slot 0 (stage 0 recovery). The record
				goes in last, once the signature checks, so slot 0 is never
				startable half written.

Both modes erase ahead of the write position one unit at a time, so no
single call waits for a whole slot, and stage a part of a program unit left
over by one call until the next.

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
#ifndef __DFU_WRITER_H__
#define __DFU_WRITER_H__

#include <stdint.h>

#include "dfu/dfu_boot.h"

/** @addtogroup DFU
  * @{
  */

/// A memory an image is written to. Offsets are from its start.
typedef struct __Dfu_Store {
	uint32_t Size;				//!< Bytes
	uint32_t EraseSize;			//!< Erase ahead step, 0 when nothing erases
	uint32_t WriteGran;			//!< Program unit
	uintptr_t Addr;				//!< Mapped start, for DfuStoreTgt
	bool (*Read)(const struct __Dfu_Store *pSt, uint32_t Off, void *pBuf,
				 uint32_t Len);
	bool (*Write)(const struct __Dfu_Store *pSt, uint32_t Off,
				  const void *pData, uint32_t Len);
	/// Erase whole units covering Off, Len. A unit that reads as ones is
	/// still erased where the memory has ECC.
	bool (*Erase)(const struct __Dfu_Store *pSt, uint32_t Off, uint32_t Len);
	/// Size of the erase unit holding Off, where units differ in size
	/// (STM32F4 sectors). Null when every unit is EraseSize.
	uint32_t (*UnitAt)(const struct __Dfu_Store *pSt, uint32_t Off);
	void *pCtx;					//!< Store specific
} DfuStore_t;

#ifdef __cplusplus

class Nvm;

/**
 * @brief	A store over internal memory, through the target layer.
 *
 * @param	pSt  : Filled in. Outlives the writer using it.
 * @param	Addr : Mapped start.
 * @param	Size : Bytes.
 *
 * @return	false when the region does not fit the target's units.
 */
bool DfuStoreTgt(DfuStore_t *pSt, uintptr_t Addr, uint32_t Size);

/**
 * @brief	A store over an Nvm initialised over the region.
 *
 * @param	pSt  : Filled in. Outlives the writer using it.
 * @param	pNvm : The Nvm.
 *
 * @return	false when the Nvm's program unit is larger than the writer
 * 			stages.
 */
bool DfuStoreNvm(DfuStore_t *pSt, Nvm *pNvm);

/// Writes an image into its store, as it arrives.
class DfuWriter {
public:
	DfuWriter();

	/**
	 * @brief	Set up.
	 *
	 * @param	Lay     : Layout.
	 * @param	pStore  : Slot 1 in slot 1 mode, slot 0 in direct mode.
	 * @param	bDirect : Direct mode.
	 *
	 * @return	false when the store is not the region the layout says.
	 */
	bool Init(const DfuLayout_t &Lay, const DfuStore_t *pStore, bool bDirect);

	/// Largest image this takes.
	uint32_t MaxLen(void) const;

	/**
	 * @brief	Start an image of Len bytes, dropping any other.
	 *
	 * In slot 1 mode the trailer is erased: nothing is pending from here on.
	 * In direct mode the record is erased: slot 0 is not startable until
	 * Commit.
	 *
	 * @return	DFU_IMG_OK, or DFU_IMG_ERR_SIZE, DFU_IMG_ERR_READ.
	 */
	int Begin(uint32_t Len);

	/**
	 * @brief	The next bytes of the image.
	 *
	 * @return	DFU_IMG_OK, DFU_IMG_ERR_SIZE past the announced length or
	 * 			with a header that does not fit, DFU_IMG_ERR_HDR,
	 * 			DFU_IMG_ERR_READ when the memory failed.
	 */
	int Write(const uint8_t *pData, uint32_t Len);

	/// Bytes taken so far.
	uint32_t Offset(void) const { return vOff; }

	/// Announced length, 0 when no image is under way.
	uint32_t Length(void) const { return vLen; }

	/// True while an image is under way.
	bool Active(void) const { return vLen != 0; }

	/// Drop the image under way. What was written stays, unmarked.
	void Abort(void) { vLen = 0; vStageLen = 0; }

	/**
	 * @brief	All bytes in: write what is staged, check the image.
	 *
	 * Parses it, checks its SHA-256 TLV, its size against slot 0 and the
	 * record, and its entry for slot 0. The signature is not checked.
	 *
	 * @param	pHash : SHA-256.
	 * @param	pInfo : Filled in.
	 *
	 * @return	DFU_IMG_OK or a DFU_IMG_ERR_ value.
	 */
	int Finish(HashEngine *pHash, DfuImgInfo_t *pInfo);

	/// Read the image written, at its upload offsets. Valid after Finish in
	/// direct mode, any time in slot 1 mode.
	static bool Read(void *pCtx, uint32_t Off, void *pBuf, uint32_t Len);

	/**
	 * @brief	Make the checked image count.
	 *
	 * Slot 1 mode: mark it pending. Direct mode: write the record.
	 */
	bool Commit(void);

	/// Slot 1 mode: parse what slot 1 holds.
	int Slot1Parse(DfuImgInfo_t *pInfo);

	/// Slot 1 mode: the trailer words.
	bool Slot1State(uint32_t *pPending, uint32_t *pDone);

	/// Slot 1 mode: erase the first unit and the trailer.
	bool Slot1Erase(void);

	bool Direct(void) const { return vbDirect; }

private:
	bool EraseTo(uint32_t End);
	bool Put(uint32_t Off, const uint8_t *pData, uint32_t Len);
	bool Flush(void);
	bool ClearTrailer(void);

	DfuLayout_t vLay;
	const DfuStore_t *vpStore;
	bool vbDirect;
	bool vbInit;

	uint32_t vLen;				//!< Announced length, 0 when idle
	uint32_t vOff;				//!< Bytes taken
	uint32_t vErasedEnd;		//!< Store erased up to here

	// Direct mode: where the three parts are, known once 16 bytes are in.
	uint32_t vHdrSize;			//!< 0 until known
	uint32_t vImgSize;
	uint32_t vTlvLen;			//!< TLV areas, once Finish parsed them

	// Payload bytes short of a program unit, and where they go. Units up to
	// the state unit minimum are staged here; a larger unit (LPC, SAM4) only
	// occurs on parts updated by wire, in direct mode, which stages in the
	// spare end of the record body buffer instead, so a small RAM part does
	// not carry a 512 byte buffer for a 2 byte unit.
	uint32_t vStageBuf[DFU_STATE_UNIT_MIN / 4];
	uint32_t *vStage;
	uint32_t vStageSize;
	uint32_t vStageLen;
	uint32_t vStageOff;			//!< Store offset of vStage
};

#endif

/** @} */

#endif
