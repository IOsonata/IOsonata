/**-------------------------------------------------------------------------
@file	nvm_region.cpp

@brief	The linker declared storage regions.

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

	PROVIDE(__nvm0_start__ = ORIGIN(NVM0));
	PROVIDE(__nvm0_size__  = LENGTH(NVM0));

The symbols are weak here, so a project that declares no region resolves
them to null and NvmRegion reports a size of 0. Nothing guesses an address.

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
#include "storage/nvm_region.h"

// The bounds of each region section, as the project's linker script places
// them. Same __start_ and __stop_ naming the SDK uses for its own section
// variables, so the section is the declaration and the size is the distance
// between the two.
//
// Weak, so a project that declares no region links and reports a size of 0
// rather than failing to build or, worse, guessing an address the way FDS
// does from CODESIZE. A linker symbol's value is its address, which is why
// these are arrays and are read as such.
extern "C" {
extern char __start_nvm0[] __attribute__((weak));
extern char __stop_nvm0[] __attribute__((weak));
extern char __start_nvm1[] __attribute__((weak));
extern char __stop_nvm1[] __attribute__((weak));
}

typedef struct {
	const char	*pStart;
	const char	*pStop;
} NvmRegionSym_t;

static const NvmRegionSym_t s_Region[NVM_REGION_MAX] = {
	{ __start_nvm0, __stop_nvm0 },
#if NVM_REGION_MAX > 1
	{ __start_nvm1, __stop_nvm1 },
#endif
};

uintptr_t NvmRegionAddr(int RegionNo)
{
	if (RegionNo < 0 || RegionNo >= NVM_REGION_MAX)
	{
		return 0;
	}

	return (uintptr_t)s_Region[RegionNo].pStart;
}

size_t NvmRegionSize(int RegionNo)
{
	if (RegionNo < 0 || RegionNo >= NVM_REGION_MAX)
	{
		return 0;
	}

	const NvmRegionSym_t *p = &s_Region[RegionNo];

	if (p->pStart == nullptr || p->pStop == nullptr || p->pStop <= p->pStart)
	{
		return 0;
	}

	return (size_t)(p->pStop - p->pStart);
}
