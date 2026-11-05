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
}

bool NvmDiskIO::Init(Nvm &Mem, uint32_t SectSize,
					uint8_t *pEraseWork, uint32_t WorkSize)
{
	uint32_t sect = SectSize;
	uint32_t esize = Mem.EraseSize();

	vpNvm = nullptr;
	vSectSize = 0;
	vpEraseWork = nullptr;

	if (sect == 0)
	{
		// A medium with an erase step reports a logical sector. One that
		// overwrites directly reports none, so it needs a size given.
		sect = Mem.LogicalSectorSize();
	}

	if (sect == 0 || sect > 0xFFFF)
	{
		return false;
	}

	// Normal writes replace complete erase units. Smaller logical sectors
	// require a caller-owned erase-unit buffer so neighbouring sectors survive.
	if (esize != 0 && (sect % esize) != 0)
	{
		if ((esize % sect) != 0 || pEraseWork == nullptr ||
			WorkSize < esize || (Mem.Size() % esize) != 0)
		{
			return false;
		}
		vpEraseWork = pEraseWork;
	}

	// DiskIO exposes only whole sectors. A tail shorter than one sector would
	// be unreachable and would make the reported geometry ambiguous.
	if (Mem.Size() < sect || (Mem.Size() % sect) != 0 ||
		(Mem.Size() / sect) > 0xFFFFFFFFULL)
	{
		return false;
	}

	vpNvm = &Mem;
	vSectSize = sect;

	return true;
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
	if (vpNvm == nullptr || pBuff == nullptr || SectNo >= GetNbSect())
	{
		return false;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;

	return vpNvm->Read(off, pBuff, vSectSize) == (int)vSectSize;
}

bool NvmDiskIO::SectWrite(uint32_t SectNo, uint8_t *pData)
{
	if (vpNvm == nullptr || pData == nullptr || SectNo >= GetNbSect())
	{
		return false;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;
	uint64_t writeOff = off;
	uint32_t writeLen = vSectSize;
	const uint8_t *pWriteData = pData;
	uint32_t eraseSize = vpNvm->EraseSize();

	if (eraseSize != 0 && vSectSize < eraseSize)
	{
		// Preserve all the other logical sectors of the same physical
		// erase unit. The caller must keep its sector input separate from
		// the scratch buffer, which Nvm reads into before the copy.
		if (vpEraseWork == nullptr)
		{
			return false;
		}

		// Reject overlapping work/input memory before the read overwrites
		// part of the sector being supplied by the caller.
		uintptr_t src = (uintptr_t)pData;
		uintptr_t work = (uintptr_t)vpEraseWork;

		if ((src >= work && src - work < eraseSize) ||
			(src < work && work - src < vSectSize))
		{
			return false;
		}

		writeOff = off - off % eraseSize;
		writeLen = eraseSize;

		if (vpNvm->Read(writeOff, vpEraseWork, eraseSize) != (int)eraseSize)
		{
			return false;
		}
		memcpy(vpEraseWork + (uint32_t)(off - writeOff), pData, vSectSize);
		pWriteData = vpEraseWork;
	}

	// On erase-write media replace the full aligned erase region. Media
	// without an erase step are simply written over.
	if (eraseSize != 0 && vpNvm->Erase(writeOff, writeLen) != 0)
	{
		return false;
	}

	if (vpNvm->Write(writeOff, pWriteData, writeLen) != (int)writeLen)
	{
		return false;
	}

	// A filesystem reuses its sector buffer the moment this answers, and an
	// interrupt driven memory returns as soon as the first chunk is handed
	// over, with the rest still to be read out of pData. Wait for the medium
	// here so what is reported is what reached it. Sync returns at once on a
	// memory that finishes in the call.
	//
	// The erase above needs no wait of its own: the write that follows it
	// drains whatever is outstanding before it starts, and answers the
	// erase's error if it had one.
	return vpNvm->Sync() == 0;
}

void NvmDiskIO::Erase(void)
{
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
