/**-------------------------------------------------------------------------
@file	diskio_nvm_mcu.cpp

@brief	Initialize an NvmDiskIO from an MCU linker-reserved NVM region.

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
#include "storage/diskio_nvm.h"
#include "storage/nvm_intrf.h"
#include "storage/nvm_region.h"

bool NvmDiskIO::InitMcu(Nvm &Mem, NvmIntrf &Intrf, int RegionNo)
{
	NvmCfg_t cfg = {};
	uint64_t start;
	uint64_t size;
	uint64_t ceiling;
	uint64_t offset;
	uint32_t unit;
	uint32_t writeGran;

	// A failed reinitialization must not leave a previously attached medium
	// available through this disk.
	vpNvm = nullptr;
	vSectSize = 0;

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

	// The linker reservation must lie wholly inside the target's memory and
	// below the application ceiling. Both bounds are checked without overflow.
	if (size == 0U || unit == 0U || cfg.PageSize == 0U ||
		start < cfg.BaseAddr || size > cfg.TotalSize ||
		start - cfg.BaseAddr > cfg.TotalSize - size ||
		start > ceiling || size > ceiling - start ||
		(start % unit) != 0U || (size % unit) != 0U ||
		(start % writeGran) != 0U || (size % writeGran) != 0U)
	{
		return false;
	}

	offset = start - cfg.BaseAddr;
	return Intrf.Init() && Mem.Init(cfg, &Intrf, offset, size) && Init(Mem);
}
