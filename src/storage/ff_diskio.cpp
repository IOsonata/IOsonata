/**-------------------------------------------------------------------------
@file	ff_diskio.cpp

@brief	Implementation disk I/O functions for fatfs

@author	Hoang Nguyen Hoan

@date	Feb. 11, 2023

@license

MIT License

Copyright (c) 2023, I-SYST, all rights reserved

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

#include <stdint.h>
#include <string.h>

#include "ff.h"
#include "ff_diskio.h"
#include "storage/diskio.h"

static DiskIO *s_pFFDiskIO = nullptr;

static bool FFDiskReady(void)
{
	if (s_pFFDiskIO == nullptr)
	{
		return false;
	}

	uint32_t size = s_pFFDiskIO->GetSectSize();

	return size >= FF_MIN_SS && size <= FF_MAX_SS &&
		(size & (size - 1U)) == 0U && s_pFFDiskIO->GetNbSect() != 0U;
}

void FFDiskSetDevice(DiskIO * const pDev)
{
	s_pFFDiskIO = pDev;
}

DSTATUS disk_status(BYTE pdrv)
{
	return pdrv == 0U && FFDiskReady() ? 0U : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
	return disk_status(pdrv);
}

DRESULT disk_read(BYTE pdrv, BYTE *pBuff, LBA_t Sector, UINT Count)
{
	if (pdrv != 0U || pBuff == nullptr || Count == 0U)
	{
		return RES_PARERR;
	}
	if (!FFDiskReady())
	{
		return RES_NOTRDY;
	}

	uint32_t total = s_pFFDiskIO->GetNbSect();

	if ((uint64_t)Sector >= total ||
		(uint64_t)Count > total - (uint64_t)Sector)
	{
		return RES_PARERR;
	}

	uint16_t size = s_pFFDiskIO->GetSectSize();

	while (Count > 0U)
	{
		if (!s_pFFDiskIO->SectRead((uint32_t)Sector, pBuff))
		{
			return RES_ERROR;
		}
		pBuff += size;
		Sector++;
		Count--;
	}

	return RES_OK;
}

#if FF_FS_READONLY == 0
DRESULT disk_write(BYTE pdrv, const BYTE *pBuff, LBA_t Sector, UINT Count)
{
	if (pdrv != 0U || pBuff == nullptr || Count == 0U)
	{
		return RES_PARERR;
	}
	if (!FFDiskReady())
	{
		return RES_NOTRDY;
	}

	uint32_t total = s_pFFDiskIO->GetNbSect();

	if ((uint64_t)Sector >= total ||
		(uint64_t)Count > total - (uint64_t)Sector)
	{
		return RES_PARERR;
	}

	uint16_t size = s_pFFDiskIO->GetSectSize();

	while (Count > 0U)
	{
		// DiskIO's historical pointer type is non-const; SectWrite
		// consumes the input and must not change the data.
		if (!s_pFFDiskIO->SectWrite((uint32_t)Sector,
				const_cast<uint8_t *>(pBuff)))
		{
			return RES_ERROR;
		}
		pBuff += size;
		Sector++;
		Count--;
	}

	return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE Cmd, void *pBuff)
{
	if (pdrv != 0U)
	{
		return RES_PARERR;
	}

	if (Cmd == SET_DEVICE)
	{
		FFDiskSetDevice(static_cast<DiskIO *>(pBuff));
		return FFDiskReady() ? RES_OK : RES_PARERR;
	}
	if (!FFDiskReady())
	{
		return RES_NOTRDY;
	}

	switch (Cmd)
	{
		case CTRL_SYNC:
			return s_pFFDiskIO->Commit() ? RES_OK : RES_ERROR;

		case GET_SECTOR_COUNT:
			if (pBuff == nullptr)
			{
				return RES_PARERR;
			}
			*static_cast<LBA_t *>(pBuff) =
				(LBA_t)s_pFFDiskIO->GetNbSect();
			return RES_OK;

		case GET_SECTOR_SIZE:
			if (pBuff == nullptr)
			{
				return RES_PARERR;
			}
			*static_cast<WORD *>(pBuff) = s_pFFDiskIO->GetSectSize();
			return RES_OK;

		case GET_BLOCK_SIZE:
			if (pBuff == nullptr)
			{
				return RES_PARERR;
			}
			{
				uint32_t sector = s_pFFDiskIO->GetSectSize();
				uint32_t erase = s_pFFDiskIO->GetMinEraseSize();
				*static_cast<DWORD *>(pBuff) =
					erase >= sector ? erase / sector : 1U;
			}
			return RES_OK;

		// Avoid erasing a partially used physical sector. DiskIO erase
		// methods return void, so failures cannot be propagated here.
		case CTRL_TRIM:
			return RES_PARERR;

		default:
			return RES_PARERR;
	}
}

static FRESULT FatFsDiskIsBlank(void *pWork, UINT WorkSize, bool *pBlank,
									bool bBootOnly)
{
	if (!FFDiskReady() || pBlank == nullptr || pWork == nullptr ||
		WorkSize < s_pFFDiskIO->GetSectSize())
	{
		return FR_INVALID_PARAMETER;
	}

	uint8_t *pSector = static_cast<uint8_t *>(pWork);
	uint32_t count = s_pFFDiskIO->GetNbSect();
	uint32_t size = s_pFFDiskIO->GetSectSize();

	*pBlank = false;

	for (uint32_t i = 0; i < count; i++)
	{
		if (!s_pFFDiskIO->SectRead(i, pSector))
		{
			return FR_DISK_ERR;
		}
		for (uint32_t j = 0; j < size; j++)
		{
			if (pSector[j] != 0xFFU)
			{
				return FR_OK;
			}
		}
		if (bBootOnly)
		{
			break;
		}
	}

	*pBlank = true;

	return FR_OK;
}

// FatFs f_mount verifies boot geometry. On request, also traverse the
// allocation table so a readable boot sector alone is not considered a
// validated volume. Force a scan rather than trusting cached FSINFO counts.
static FRESULT FatFsValidate(const TCHAR *pPath, FATFS *pFs)
{
	DWORD freeClusters = 0;
	FATFS *pMounted = nullptr;

	pFs->free_clst = 0xFFFFFFFFUL;
	FRESULT res = f_getfree(pPath, &freeClusters, &pMounted);
	if (res == FR_OK && pMounted != pFs)
	{
		return FR_INT_ERR;
	}
	return res;
}

FRESULT FatFsMount(const FatFsMountCfg_t &Cfg)
{
	FRESULT res;
	bool blank = false;
	TCHAR label[FF_MAX_LFN + 1];
	const TCHAR *pPath;
	const MKFS_PARM defaults = { FM_ANY | FM_SFD, 0, 0, 0, 0 };
	const MKFS_PARM *pFormat;

	if (Cfg.pDisk == nullptr || Cfg.pFatFs == nullptr)
	{
		return FR_INVALID_PARAMETER;
	}

	if ((Cfg.bFormatIfBlank || Cfg.bFormatIfBootBlank ||
		 Cfg.bFormatIfInvalid) &&
		(Cfg.pWork == nullptr || Cfg.WorkSize < Cfg.pDisk->GetSectSize()))
	{
		return FR_INVALID_PARAMETER;
	}

	pPath = Cfg.pPath != nullptr ? Cfg.pPath : "0:";
	FFDiskSetDevice(Cfg.pDisk);

	if (!FFDiskReady())
	{
		return FR_NOT_READY;
	}

	res = f_mount(Cfg.pFatFs, pPath, 1);
	if (res == FR_OK && Cfg.bValidateOnMount)
	{
		res = FatFsValidate(pPath, Cfg.pFatFs);
	}

	// Never erase a volume after a read or write failure. Recovery is
	// deliberately opt-in and distinct from checking an erased sector.
	if (res == FR_NO_FILESYSTEM || (res == FR_INT_ERR && Cfg.bFormatIfInvalid))
	{
		if (Cfg.bFormatIfInvalid)
		{
			blank = true;
		}
		else if (Cfg.bFormatIfBlank || Cfg.bFormatIfBootBlank)
		{
			res = FatFsDiskIsBlank(Cfg.pWork, Cfg.WorkSize, &blank,
				Cfg.bFormatIfBootBlank);
			if (res != FR_OK)
			{
				return res;
			}
		}

		if (blank)
		{
			pFormat = Cfg.pFormatOptions != nullptr ?
				Cfg.pFormatOptions : &defaults;

			res = f_mkfs(pPath, pFormat, Cfg.pWork, Cfg.WorkSize);
			if (res == FR_OK)
			{
				res = f_mount(Cfg.pFatFs, pPath, 1);
				if (res == FR_OK && Cfg.bValidateOnMount)
				{
					res = FatFsValidate(pPath, Cfg.pFatFs);
				}
			}
		}
	}

	// FatFs mkfs creates an unnamed volume ("NO NAME" on the PC).
	// Adding a label is independent of formatting and is also useful for
	// disks formatted by older firmware. Never replace a custom label.
	if (res == FR_OK && Cfg.pLabelIfUnnamed != nullptr)
	{
		res = f_getlabel(pPath, label, nullptr);

		if (res == FR_OK && label[0] == 0)
		{
			res = f_setlabel(Cfg.pLabelIfUnnamed);
		}
	}

	return res;
}

FRESULT FatFsUnmount(const TCHAR *pPath)
{
	FRESULT res = f_mount(nullptr, pPath != nullptr ? pPath : "0:", 0);

	if (res == FR_OK)
	{
		FFDiskSetDevice(nullptr);
	}
	return res;
}
