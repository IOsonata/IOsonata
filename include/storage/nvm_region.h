/**-------------------------------------------------------------------------
@file	nvm_region.h

@brief	Where a storage region lives, as the linker script declares it.

MCU internal memory is shared with code, and only the linker knows where the
image ends. A MEMORY region is how it says so: the region is carved out of
the part, FLASH shrinks to fit, and an image that grows into the store fails
to link instead of quietly overlapping it.

The region is the device. Nvm sees a base and a size and does not care that
the same silicon holds code elsewhere, so an internal store reads exactly
like an external chip.

A project declares one like this, in its own linker script:

	MEMORY
	{
		FLASH (rx) : ORIGIN = 0x00000000, LENGTH = 0x7D000
		NVM0  (rw) : ORIGIN = 0x0007D000, LENGTH = 0x00002000
		RAM   (rwx): ORIGIN = 0x20000000, LENGTH = 0x00010000
	}

	SECTIONS
	{
		nvm0 (NOLOAD) :
		{
			__start_nvm0 = .;
			KEEP(*(nvm0))
			. = ORIGIN(NVM0) + LENGTH(NVM0);
			__stop_nvm0 = .;
		} > NVM0
	}

Naming the region as a section puts it in the map file with its size, and
gives the bounds as __start_ and __stop_ symbols, which is what the SDK uses
for its own section variables. The size is the distance between them, so
nothing states it twice.

The symbols are weak here, so a project that declares no region resolves
them to null and the size reads 0. Nothing guesses an address.

@author	Hoang Nguyen Hoan
@date	Jul. 27, 2026

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
#ifndef __NVM_REGION_H__
#define __NVM_REGION_H__

#include <stdint.h>
#include <stddef.h>

/** @addtogroup Storage
  * @{
  */

/// How many regions a project may declare. Raise it and add the matching
/// symbols; nothing else changes.
#ifndef NVM_REGION_MAX
#define NVM_REGION_MAX				2
#endif

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief	Where region RegionNo starts, or 0 when the project declared none.
 */
uintptr_t NvmRegionAddr(int RegionNo);

/**
 * @brief	How big region RegionNo is, or 0 when the project declared none.
 */
size_t NvmRegionSize(int RegionNo);

#ifdef __cplusplus
}
#endif

/** @} end group Storage */

#endif // __NVM_REGION_H__
