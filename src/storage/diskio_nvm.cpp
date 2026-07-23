/**-------------------------------------------------------------------------
@file	diskio_nvm.cpp

@brief	Block device over any NvmIO.

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
}

bool NvmDiskIO::Init(NvmIO &Nvm, uint32_t SectSize)
{
	uint32_t sect = SectSize;

	if (sect == 0)
	{
		// A medium with an erase step reports a logical sector. One that
		// overwrites directly reports none, so it needs a size given.
		sect = Nvm.LogicalSectorSize();
	}

	if (sect == 0 || sect > 0xFFFF)
	{
		return false;
	}

	// The sector has to be a whole number of erase units, or an erase would
	// take out more than the sector being written.
	uint32_t esize = Nvm.EraseSize();
	if (esize != 0 && (sect % esize) != 0)
	{
		return false;
	}

	if (Nvm.Size() < sect)
	{
		return false;
	}

	vpNvm = &Nvm;
	vSectSize = sect;

	return true;
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
	if (vpNvm == nullptr || pBuff == nullptr)
	{
		return false;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;

	return vpNvm->Read(off, pBuff, vSectSize) == (int)vSectSize;
}

bool NvmDiskIO::SectWrite(uint32_t SectNo, uint8_t *pData)
{
	if (vpNvm == nullptr || pData == nullptr)
	{
		return false;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;

	// Only an erase-write medium needs the sector cleared first. One that
	// overwrites directly is written as it is.
	if (vpNvm->EraseSize() != 0)
	{
		if (vpNvm->Erase(off, vSectSize) != 0)
		{
			return false;
		}
	}

	return vpNvm->Write(off, pData, vSectSize) == (int)vSectSize;
}

void NvmDiskIO::Erase(void)
{
	if (vpNvm == nullptr || vpNvm->EraseSize() == 0)
	{
		return;
	}

	vpNvm->Erase(0, (uint32_t)vpNvm->Size());
}

void NvmDiskIO::EraseSector(uint32_t SectNo, int NbSect)
{
	if (vpNvm == nullptr || vpNvm->EraseSize() == 0 || NbSect <= 0)
	{
		return;
	}

	uint64_t off = (uint64_t)SectNo * vSectSize;

	vpNvm->Erase(off, vSectSize * (uint32_t)NbSect);
}
