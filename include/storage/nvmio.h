/**-------------------------------------------------------------------------
@file	nvmio.h

@brief	Non volatile memory device base class.

Common base class for all persistent memory devices: SPI/QSPI NOR Flash,
I2C/SPI EEPROM, FRAM/NVRAM, SD card, MCU internal Flash or RRAM. It extends
the generic Device class with a byte addressed persistent storage interface.

Bus binding comes from the Device base. External parts are initialized with
a DeviceIntrf (SPI, QSPI, I2C, ...) and a device address (chip select index
or I2C address). MCU internal memory uses no interface (DEVINTRF_TYPE_NULL)
with the memory mapped base address as device address.

Each instance covers one region of the physical medium, defined by a region
offset and size at initialization. All offsets passed to Read, Write and
Erase are relative to the region and bounds checked against its size. Several
instances may map disjoint regions of the same physical device and share the
same DeviceIntrf; transaction serialization and power reference counting are
provided by the interface layer.

Write semantics depend on the erase property:

- EraseSize() == 0 : direct write medium (EEPROM, FRAM, SD). Write freely
  overwrites existing data. Erase is not required and succeeds as a no-op.

- EraseSize() > 0 : erase-write medium (NOR Flash, MCU Flash, RRAM with
  erase blocks). Write only programs erased cells and never performs a
  read-modify-write on the caller's behalf. Erase discipline belongs to the
  caller. Within one Write call, memory cells are programmed in ascending
  address order: when a write is torn by power loss, no cell of a later
  write unit is programmed while a cell of an earlier unit of the same call
  remains erased. Log structured consumers rely on this ordering for their
  power loss recovery.

All operations are synchronous: when a call returns success, the medium
holds the committed result. Implementations over asynchronous services must
wait for completion internally. Operations must be called from a single
execution context.

@author	Hoang Nguyen Hoan
@date	Jul 19, 2026

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
#ifndef __NVMIO_H__
#define __NVMIO_H__

#include <stdint.h>
#include <errno.h>

#include "device.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Non volatile memory device base class.
///
/// Abstract base for persistent memory devices. Subclasses implement the
/// medium specific access and fill the region window via Region() during
/// their Init.
class NvmIO : virtual public Device {
public:
	// Keep the Device register access overloads visible alongside the
	// offset based storage operations below.
	using Device::Read;
	using Device::Write;

	/**
	 * @brief	Get the region size in bytes.
	 *
	 * This is the size visible to the caller, not the physical device size.
	 *
	 * @return	Region size in bytes.
	 */
	virtual uint64_t Size(void) const { return vRegionSize; }

	/**
	 * @brief	Get the erase unit size in bytes.
	 *
	 * @return	Erase unit in bytes.
	 * 			0 : direct write medium, no erase required.
	 */
	virtual uint32_t EraseSize(void) const = 0;

	/**
	 * @brief	Get the minimum write unit in bytes.
	 *
	 * Offsets and lengths passed to Write must be multiples of this value.
	 * Typical values: 1 for EEPROM/FRAM, 4 for MCU Flash word programming,
	 * 16 for STM32WBA quad word programming.
	 *
	 * @return	Write granularity in bytes.
	 */
	virtual uint32_t WriteGran(void) const = 0;

	/**
	 * @brief	Get the logical sector size in bytes.
	 *
	 * A log-structured consumer partitions the region into sectors for
	 * garbage collection. On erase-write media the sector is the physical
	 * erase unit, so this defaults to EraseSize(). On a rewritable medium
	 * whose physical unit is smaller than the sector a consumer wants
	 * (for example RRAM programmed in words but collected in kilobyte
	 * sectors), the subclass reports the larger logical sector here while
	 * EraseSize() reports the physical erase or program unit. The logical
	 * sector must be a whole multiple of EraseSize() so a sector spans
	 * complete physical erase units.
	 *
	 * @return	Logical sector size in bytes.
	 */
	virtual uint32_t LogicalSectorSize(void) const { return EraseSize(); }

	/**
	 * @brief	Get the page size in bytes.
	 *
	 * The page is the address auto increment window of the medium. During a
	 * write burst the address counter advances only within the current page;
	 * on reaching the page boundary it wraps to the start of the same page
	 * rather than moving to the next page. A single write must therefore not
	 * cross a page boundary. Reads and writes may be any size; the
	 * implementation splits a burst at page boundaries and reissues the
	 * address for each page. EEPROM and NOR Flash have a page; RAM based
	 * media and byte addressable media report 0 (no page boundary).
	 *
	 * @return	Page size in bytes. 0 : no page boundary.
	 */
	virtual uint32_t PageSize(void) const { return 0; }

	/**
	 * @brief	Read data from the region.
	 *
	 * @param	Off		: Region relative offset
	 * @param	pBuf	: Buffer to receive the data
	 * @param	Len		: Number of bytes to read
	 *
	 * @return	Len on success, negative errno code on failure.
	 */
	virtual int Read(uint64_t Off, void *pBuf, uint32_t Len) = 0;

	/**
	 * @brief	Write data to the region.
	 *
	 * On an erase-write medium (EraseSize() > 0), the destination cells
	 * must be in the erased state and Off and Len must be multiples of
	 * WriteGran(). Cells are programmed in ascending address order. No
	 * read-modify-write is performed.
	 *
	 * On a direct write medium (EraseSize() == 0), existing data is
	 * overwritten freely.
	 *
	 * The call is synchronous: on success the data is committed to the
	 * medium.
	 *
	 * @param	Off		: Region relative offset
	 * @param	pData	: Data to write
	 * @param	Len		: Number of bytes to write
	 *
	 * @return	Len on success, negative errno code on failure.
	 */
	virtual int Write(uint64_t Off, const void *pData, uint32_t Len) = 0;

	/**
	 * @brief	Erase a range of the region.
	 *
	 * Off and Len must be multiples of EraseSize(). An interrupted erase
	 * may leave the range in a partially erased state; consumers must
	 * treat unreadable content after a power loss accordingly.
	 *
	 * The default implementation succeeds as a no-op on a direct write
	 * medium and fails on an erase-write medium; subclasses of erase-write
	 * media must override.
	 *
	 * @param	Off		: Region relative offset
	 * @param	Len		: Number of bytes to erase
	 *
	 * @return	0 on success, negative errno code on failure.
	 */
	virtual int Erase(uint64_t Off, uint32_t Len) {
		if (!RangeValid(Off, Len))
		{
			return -EINVAL;
		}
		return EraseSize() == 0 ? 0 : -ENOTSUP;
	}

	/**
	 * @brief	Flush any buffered state to the medium.
	 *
	 * Most implementations are unbuffered and inherit this no-op.
	 *
	 * @return	0 on success, negative errno code on failure.
	 */
	virtual int Sync(void) { return 0; }

	/**
	 * @brief	Get the region offset on the physical medium.
	 *
	 * @return	Absolute offset of the region in bytes.
	 */
	uint64_t RegionOffset(void) const { return vRegionOffset; }

protected:
	NvmIO() : vRegionOffset(0), vRegionSize(0) {}

	/**
	 * @brief	Set the region window. Called by subclass Init.
	 *
	 * @param	Offset	: Absolute offset of the region on the medium
	 * @param	Size	: Region size in bytes
	 */
	void Region(uint64_t Offset, uint64_t Size) {
		vRegionOffset = Offset;
		vRegionSize = Size;
	}

	/**
	 * @brief	Validate a region relative range.
	 *
	 * @param	Off	: Region relative offset
	 * @param	Len	: Range length in bytes
	 *
	 * @return	true when the range lies within the region.
	 */
	bool RangeValid(uint64_t Off, uint32_t Len) const {
		return Off <= vRegionSize && Len <= vRegionSize - Off;
	}

private:
	uint64_t vRegionOffset;		//!< Absolute region offset on the medium
	uint64_t vRegionSize;		//!< Region size in bytes
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __NVMIO_H__
