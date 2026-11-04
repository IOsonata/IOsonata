/**-------------------------------------------------------------------------
@file	fatfs_format.cpp

@brief	FAT12 initialization of a blank DiskIO volume.

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

#include "storage/diskio.h"
#include "storage/fatfs.h"
#include "storage/fatfs_format.h"

static void Fat12Put16(uint8_t *pData, uint16_t Value)
{
	pData[0] = (uint8_t)Value;
	pData[1] = (uint8_t)(Value >> 8);
}

static void Fat12Put32(uint8_t *pData, uint32_t Value)
{
	pData[0] = (uint8_t)Value;
	pData[1] = (uint8_t)(Value >> 8);
	pData[2] = (uint8_t)(Value >> 16);
	pData[3] = (uint8_t)(Value >> 24);
}

static bool Fat12Layout(uint32_t Count, uint16_t SectorSize,
						uint32_t *pFatSectors, uint32_t *pRootSector)
{
	uint32_t fatSectors;
	uint32_t clusters;
	uint32_t needed;
	uint32_t rootSector;

	if (Count < 5U || Count > 65535U)
	{
		return false;
	}

	fatSectors = 1U;
	for (;;)
	{
		if (Count <= 2U + FATFS_NBFAT * fatSectors)
		{
			return false;
		}
		clusters = Count - 2U - FATFS_NBFAT * fatSectors;
		needed = (uint32_t)((((uint64_t)(clusters + 2U) * 3U + 1U) / 2U +
			SectorSize - 1U) / SectorSize);
		if (needed <= fatSectors)
		{
			break;
		}
		fatSectors = needed;
	}

	// FAT12 requires fewer than 4085 data clusters.
	if (clusters == 0U || clusters >= 4085U)
	{
		return false;
	}
	rootSector = FATFS_RSVDSECCNT_FAT16 + FATFS_NBFAT * fatSectors;
	if (rootSector >= Count - 1U || fatSectors > 65535U)
	{
		return false;
	}
	*pFatSectors = fatSectors;
	*pRootSector = rootSector;
	return true;
}

static bool Fat12Blank(const uint8_t *pSector, uint16_t SectorSize)
{
	uint8_t value;
	uint32_t i;

	value = pSector[0];
	if (value != 0xFFU && value != 0U)
	{
		return false;
	}
	for (i = 1U; i < SectorSize; i++)
	{
		if (pSector[i] != value)
		{
			return false;
		}
	}
	return true;
}

static bool Fat12Recoverable(DiskIO &Disk, uint8_t *pSector,
						uint32_t FatSectors, uint32_t RootSector)
{
	static const uint8_t s_Header[] = {
		0xEBU, 0x3CU, 0x90U, 'I', 'O', 's', 'o', 'n', 'a', 't', 'a'
	};
	uint32_t i;

	// Flash programs bits from 1 to 0. A reset may leave only part of the
	// first program word committed, so accept a partially programmed header
	// provided that no bits contradict the expected IOsonata header. The
	// FATs and root must also match: they were written before sector zero.
	for (i = 0U; i < sizeof(s_Header); i++)
	{
		if ((pSector[i] & s_Header[i]) != s_Header[i])
		{
			return false;
		}
	}
	for (i = 0U; i < FATFS_NBFAT; i++)
	{
		if (!Disk.SectRead(FATFS_RSVDSECCNT_FAT16 + i * FatSectors,
			pSector) || pSector[0] != FATFS_MEDIA_FIXED ||
			pSector[1] != 0xFFU || pSector[2] != 0xFFU)
		{
			return false;
		}
	}
	if (!Disk.SectRead(RootSector, pSector) ||
		memcmp(pSector, "IOSONATA   ", 11U) != 0 ||
		pSector[11] != FATFS_DIRATTR_VOLUME_ID)
	{
		return false;
	}
	return true;
}

static bool Fat12WriteBoot(DiskIO &Disk, uint8_t *pSector,
						uint16_t SectorSize, uint32_t Count, uint32_t FatSectors)
{
	memset(pSector, 0, SectorSize);
	pSector[0] = 0xEBU;
	pSector[1] = 0x3CU;
	pSector[2] = 0x90U;
	memcpy(&pSector[3], "IOsonata", 8U);
	Fat12Put16(&pSector[11], SectorSize);
	pSector[13] = 1U;
	Fat12Put16(&pSector[14], FATFS_RSVDSECCNT_FAT16);
	pSector[16] = FATFS_NBFAT;
	Fat12Put16(&pSector[17], SectorSize / 32U);
	Fat12Put16(&pSector[19], (uint16_t)Count);
	pSector[21] = FATFS_MEDIA_FIXED;
	Fat12Put16(&pSector[22], (uint16_t)FatSectors);
	Fat12Put16(&pSector[24], 32U);
	Fat12Put16(&pSector[26], 1U);
	pSector[36] = 0x80U;
	pSector[38] = 0x29U;
	Fat12Put32(&pSector[39], 0x4E534F49U);
	memcpy(&pSector[43], "IOSONATA   ", 11U);
	memcpy(&pSector[54], "FAT12   ", 8U);
	pSector[510] = 0x55U;
	pSector[511] = 0xAAU;
	pSector[SectorSize - 2U] = 0x55U;
	pSector[SectorSize - 1U] = 0xAAU;

	return Disk.SectWrite(0U, pSector);
}

bool FatFsFormatIfBlank(DiskIO &Disk, uint8_t *pSector, uint32_t BufferSize)
{
	uint16_t sectorSize;
	uint32_t sectorCount;
	uint32_t fatSectors;
	uint32_t rootSector;
	uint32_t i;
	bool blank;

	sectorSize = Disk.GetSectSize();
	sectorCount = Disk.GetNbSect();
	if (pSector == nullptr || sectorCount == 0U ||
		(sectorSize != 512U && sectorSize != 1024U &&
		 sectorSize != 2048U && sectorSize != 4096U) ||
		BufferSize < sectorSize || !Disk.SectRead(0U, pSector))
	{
		return false;
	}

	// A volume or an MBR already exists. Do not impose our FAT12 layout on it.
	if (pSector[510] == 0x55U && pSector[511] == 0xAAU)
	{
		return true;
	}

	blank = Fat12Blank(pSector, sectorSize);
	if (!Fat12Layout(sectorCount, sectorSize, &fatSectors, &rootSector))
	{
		return false;
	}
	if (!blank)
	{
		// An interrupted final boot write can be repaired without touching
		// any filesystem sectors. Unknown nonblank contents are left alone.
		if (!Fat12Recoverable(Disk, pSector, fatSectors, rootSector))
		{
			return false;
		}
		return Fat12WriteBoot(Disk, pSector, sectorSize, sectorCount,
			fatSectors);
	}

	// Write the FATs and root first. An interruption before the boot write
	// leaves sector zero blank, so a later start can safely repeat formatting.
	for (i = 0U; i < FATFS_NBFAT * fatSectors; i++)
	{
		memset(pSector, 0, sectorSize);
		if ((i % fatSectors) == 0U)
		{
			pSector[0] = FATFS_MEDIA_FIXED;
			pSector[1] = 0xFFU;
			pSector[2] = 0xFFU;
		}
		if (!Disk.SectWrite(FATFS_RSVDSECCNT_FAT16 + i, pSector))
		{
			return false;
		}
	}

	memset(pSector, 0, sectorSize);
	memcpy(pSector, "IOSONATA   ", 11U);
	pSector[11] = FATFS_DIRATTR_VOLUME_ID;
	if (!Disk.SectWrite(rootSector, pSector))
	{
		return false;
	}

	return Fat12WriteBoot(Disk, pSector, sectorSize, sectorCount, fatSectors);
}
