/**-------------------------------------------------------------------------
@file	nvm_mcu_region.cpp

@brief	Initialize MCU NVM inside a linker-reserved region.

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
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"

bool NvmMcuRegionInit(Nvm &Mem, NvmIntrf &Intrf, int RegionNo)
{
	NvmCfg_t cfg = {};
	uint64_t start;
	uint64_t size;
	uint64_t ceiling;
	uint64_t offset;
	uint32_t unit;
	uint32_t writeGran;

	if (RegionNo < 0 || RegionNo >= NVM_REGION_MAX)
	{
		return false;
	}

	NvmMcuCfg(cfg);
	start = NvmRegionAddr(RegionNo);
	size = NvmRegionSize(RegionNo);
	ceiling = NvmMcuCeiling();
	unit = cfg.EraseSize != 0U ? cfg.EraseSize : cfg.SectorSize;
	writeGran = cfg.WriteGran != 0U ? cfg.WriteGran : 1U;

	if (size == 0U || unit == 0U || cfg.PageSize == 0U)
	{
		return false;
	}

	if (start < cfg.BaseAddr || start > ceiling)
	{
		return false;
	}

	offset = start - cfg.BaseAddr;

	// Check the reserved region without overflowing the address arithmetic.
	if (offset > cfg.TotalSize || size > cfg.TotalSize - offset)
	{
		return false;
	}

	if (size > ceiling - start)
	{
		return false;
	}

	if ((start % unit) != 0U || (size % unit) != 0U)
	{
		return false;
	}

	if ((start % writeGran) != 0U || (size % writeGran) != 0U)
	{
		return false;
	}

	if (Intrf.Init() == false)
	{
		return false;
	}

	return Mem.Init(cfg, &Intrf, offset, size);
}
