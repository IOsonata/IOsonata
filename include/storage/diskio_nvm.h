/**-------------------------------------------------------------------------
@file	diskio_nvm.h

@brief	Block device over any Nvm.

		A filesystem wants sectors. Nvm offers byte addressed read, write and
		erase plus its geometry, which is all a sector needs, so this adapter
		is thin and knows nothing about the medium underneath. The same FatFS
		or littlefs volume therefore runs on serial flash, on an EEPROM, on the
		MCU internal memory, or on a medium reached over a link, and cannot
		tell which it is.

		Two medium differences are handled here rather than by the caller. A
		medium with an erase step needs the sector erased before it is written,
		while one that overwrites directly does not, so a sector write erases
		only where EraseSize is non zero. And a medium with no erase step
		reports no logical sector, so those need a sector size given to Init.

		This is new development on the Nvm API. It does not replace
		diskio_flash.h and FlashDiskIO, which remain for existing applications.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

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
#ifndef __DISKIO_NVM_H__
#define __DISKIO_NVM_H__

#include "storage/diskio.h"
#include "storage/nvm.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Block device over any Nvm.
class NvmDiskIO : public DiskIO {
public:
	NvmDiskIO();
	virtual ~NvmDiskIO() {}
	NvmDiskIO(NvmDiskIO&) = delete;

	/**
	 * @brief	Attach to a memory.
	 *
	 * @param	Nvm			: The memory, already initialized
	 * @param	SectSize	: Sector size in bytes. 0 : take the logical sector
	 * 						  the medium reports, which a medium with no erase
	 * 						  step gives as 0, so those need a size here.
	 *
	 * @return	true on success. Fails when the sector size is absent, does not
	 * 			divide the region, or is not a whole number of erase units.
	 */
	bool Init(Nvm &Mem, uint32_t SectSize = 0);

	// *** DiskIO ***

	uint16_t GetSectSize(void) override { return (uint16_t)vSectSize; }
	uint32_t GetSize(void) override;
	uint32_t GetMinEraseSize(void) override;
	bool SectRead(uint32_t SectNo, uint8_t *pBuff) override;
	bool SectWrite(uint32_t SectNo, uint8_t *pData) override;
	void Erase(void) override;
	void EraseSector(uint32_t SectNo, int NbSect) override;

private:
	Nvm			*vpNvm;			//!< The memory this disk sits on
	uint32_t	vSectSize;		//!< Sector size in bytes
};

#endif	// __cplusplus

/** @} End of group Storage */

#endif	// __DISKIO_NVM_H__
