/**-------------------------------------------------------------------------
@file	diskio_nvm.cpp

@brief	Block device over any Nvm.

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
#include <string.h>

#include "storage/diskio_nvm.h"

NvmDiskIO::NvmDiskIO()
{
	vpNvm = nullptr;
	vSectSize = 0;
	vpEraseWork = nullptr;
	vCacheOff = 0;
	vbCacheValid = false;
	vbCacheDirty = false;
	vbWriteCache = false;
	vStats = {};
}

bool NvmDiskIO::Init(Nvm &Mem, uint32_t SectSize,
					uint8_t *pEraseWork, uint32_t WorkSize, bool bWriteCache)
{
	uint32_t sect = SectSize;
	uint32_t esize = Mem.EraseSize();

	vpNvm = nullptr;
	vSectSize = 0;
	vpEraseWork = nullptr;
	vCacheOff = 0;
	vbCacheValid = false;
	vbCacheDirty = false;
	vbWriteCache = false;
	vStats = {};

	if (sect == 0)
	{
		sect = Mem.LogicalSectorSize();
	}
	if (sect == 0 || sect > 0xFFFF)
	{
		return false;
	}
	if (esize != 0 && (sect % esize) != 0)
	{
		if ((esize % sect) != 0 || pEraseWork == nullptr ||
			WorkSize < esize || (Mem.Size() % esize) != 0)
		{
			return false;
		}
		vpEraseWork = pEraseWork;
	}
	if (Mem.Size() < sect || (Mem.Size() % sect) != 0 ||
		(Mem.Size() / sect) > 0xFFFFFFFFULL)
	{
		return false;
	}
	if (bWriteCache && (esize == 0 || sect >= esize ||
		vpEraseWork == nullptr))
	{
		return false;
	}
	vpNvm = &Mem;
	vSectSize = sect;
	vbWriteCache = bWriteCache;
	return true;
}

bool NvmDiskIO::WriteThrough(uint64_t Off, const uint8_t *pData, uint32_t Len)
{
	if (vpNvm->EraseSize() != 0 && vpNvm->Erase(Off, Len) != 0)
	{
		return false;
	}
	// Write waits for a pending erase. Sync finishes the write before the
	// caller can reuse the sector buffer.
	if (vpNvm->Write(Off, pData, Len) != (int)Len)
	{
		return false;
	}
	return vpNvm->Sync() == 0;
}

bool NvmDiskIO::WritePage(void)
{
	if (!vbCacheDirty)
	{
		return true;
	}
	if (!WriteThrough(vCacheOff, vpEraseWork, vpNvm->EraseSize()))
	{
		return false;
	}
	vbCacheDirty = false;
	vStats.PagePrograms++;
	return true;
}

bool NvmDiskIO::LoadPage(uint64_t PageOff)
{
	if (vbCacheValid && vCacheOff == PageOff)
	{
		vStats.CacheHits++;
		return true;
	}
	if (vbCacheValid)
	{
		vStats.PageEvictions++;
	}
	if (!WritePage())
	{
		return false;
	}
	if (vpNvm->Read(PageOff, vpEraseWork, vpNvm->EraseSize()) !=
		(int)vpNvm->EraseSize())
	{
		vbCacheValid = false;
		return false;
	}
	vStats.PageLoads++;
	vCacheOff = PageOff;
	vbCacheValid = true;
	vbCacheDirty = false;
	return true;
}

bool NvmDiskIO::Commit(void)
{
	vStats.SyncCalls++;
	if (!DiskIO::Commit())
	{
		return false;
	}
	return WritePage();
}

uint32_t NvmDiskIO::GetNbSect(void)
{
	if (vpNvm == nullptr || vSectSize == 0U)
	{
		return 0;
	}
	return (uint32_t)(vpNvm->Size() / vSectSize);
}

uint32_t NvmDiskIO::GetSize(void)
{
	if (vpNvm == nullptr)
	{
		return 0;
	}

	// DiskIO reports the size in KBytes.
	return (uint32_t)(vpNvm->Size() >> 10);
}

uint32_t NvmDiskIO::GetMinEraseSize(void)
{
	if (vpNvm == nullptr)
	{
		return 0;
	}

	uint32_t esize = vpNvm->EraseSize();

	// A medium that overwrites directly has no erase unit; the sector is the
	// smallest thing a consumer replaces.
	return esize != 0 ? esize : vSectSize;
}

bool NvmDiskIO::SectRead(uint32_t SectNo, uint8_t *pBuff)
{
	vStats.SectReads++;
	if (vpNvm == nullptr || pBuff == nullptr || SectNo >= GetNbSect())
	{
		return false;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;
	if (vbWriteCache)
	{
		uint32_t eraseSize = vpNvm->EraseSize();
		uint64_t pageOff = off - off % eraseSize;
		if (vbCacheValid && vCacheOff == pageOff)
		{
			vStats.CacheHits++;
			memcpy(pBuff, vpEraseWork + (uint32_t)(off - pageOff),
				vSectSize);
			return true;
		}
	}

	// Reading another page must not evict or program the dirty write
	// cache. Read that sector directly from the non-overlapping NVM page.
	return vpNvm->Read(off, pBuff, vSectSize) == (int)vSectSize;
}

bool NvmDiskIO::SectWrite(uint32_t SectNo, uint8_t *pData)
{
	vStats.SectWrites++;
	if (vpNvm == nullptr || pData == nullptr || SectNo >= GetNbSect())
	{
		return false;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;
	uint32_t eraseSize = vpNvm->EraseSize();
	if (vSectSize >= eraseSize)
	{
		return WriteThrough(off, pData, vSectSize);
	}

	uintptr_t src = (uintptr_t)pData;
	uintptr_t work = (uintptr_t)vpEraseWork;
	if ((src >= work && src - work < eraseSize) ||
		(src < work && work - src < vSectSize))
	{
		return false;
	}

	uint64_t pageOff = off - off % eraseSize;
	if (vbWriteCache)
	{
		if (!LoadPage(pageOff))
		{
			return false;
		}
	}
	else if (vpNvm->Read(pageOff, vpEraseWork, eraseSize) != (int)eraseSize)
	{
		return false;
	}

	uint8_t *pSector = vpEraseWork + (uint32_t)(off - pageOff);
	if (memcmp(pSector, pData, vSectSize) == 0)
	{
		return true;
	}
	memcpy(pSector, pData, vSectSize);
	if (vbWriteCache)
	{
		vbCacheDirty = true;
		return true;
	}
	return WriteThrough(pageOff, vpEraseWork, eraseSize);
}


void NvmDiskIO::Erase(void)
{
	if (!Commit())
	{
		return;
	}
	vbCacheValid = false;
	vbCacheDirty = false;
	if (vpNvm == nullptr || vpNvm->EraseSize() == 0 ||
		vpNvm->Size() > 0xFFFFFFFFULL)
	{
		return;
	}

	if (vpNvm->Erase(0, (uint32_t)vpNvm->Size()) == 0)
	{
		// Nothing here answers, so the wait is the only way the caller can
		// be sure the medium is done before it reads the sectors back.
		(void)vpNvm->Sync();
	}
}

void NvmDiskIO::EraseSector(uint32_t SectNo, int NbSect)
{
	if (!Commit())
	{
		return;
	}
	vbCacheValid = false;
	vbCacheDirty = false;
	if (vpNvm == nullptr || vpNvm->EraseSize() == 0 || NbSect <= 0 ||
		SectNo >= GetNbSect() || (uint32_t)NbSect > GetNbSect() - SectNo ||
		(uint64_t)NbSect * vSectSize > 0xFFFFFFFFULL)
	{
		return;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;
	uint32_t len = vSectSize * (uint32_t)NbSect;
	uint32_t eraseSize = vpNvm->EraseSize();

	// Never erase a neighbouring logical sector through a partial TRIM.
	if ((off % eraseSize) != 0 || (len % eraseSize) != 0)
	{
		return;
	}

	if (vpNvm->Erase(off, len) == 0)
	{
		(void)vpNvm->Sync();
	}
}
