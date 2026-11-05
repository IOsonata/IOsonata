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

// The FatFs logical sector is FF_MIN_SS (512 bytes). The physical disk can
// use larger sectors; the application provides one physical-sector buffer.
// Sector writes on erase-before-program media are handled by DiskIO.
static DiskIO *s_pFFDisk = nullptr;
static uint8_t *s_pFFSectorBuffer = nullptr;
static uint32_t s_FFPhysicalSize = 0;
static uint32_t s_FFPhysicalCount = 0;
static uint32_t s_FFLogicalCount = 0;
static uint32_t s_FFLogicalPerPhysical = 0;

static bool FFDiskBind(DiskIO *pDisk, uint8_t *pBuffer, uint32_t BufferSize)
{
	s_pFFDisk = nullptr;
	s_pFFSectorBuffer = nullptr;
	s_FFPhysicalSize = 0;
	s_FFPhysicalCount = 0;
	s_FFLogicalCount = 0;
	s_FFLogicalPerPhysical = 0;

	if (pDisk == nullptr)
	{
		return false;
	}

	uint32_t size = pDisk->GetSectSize();
	uint32_t count = pDisk->GetNbSect();
	uint32_t eraseSize = pDisk->GetMinEraseSize();

	if (size < FF_MIN_SS || (size % FF_MIN_SS) != 0 ||
		count == 0 || ((uint64_t)size / FF_MIN_SS) * count > UINT32_MAX ||
		(eraseSize != 0 && (eraseSize % FF_MIN_SS) != 0) ||
		(size > FF_MIN_SS && (pBuffer == nullptr || BufferSize < size)) ||
		(pBuffer != nullptr && BufferSize < size))
	{
		return false;
	}

	s_pFFDisk = pDisk;
	s_pFFSectorBuffer = pBuffer;
	s_FFPhysicalSize = size;
	s_FFPhysicalCount = count;
	s_FFLogicalPerPhysical = size / FF_MIN_SS;
	s_FFLogicalCount = count * s_FFLogicalPerPhysical;

	return true;
}

void FFDiskSetDevice(DiskIO * const pDev)
{
	(void)FFDiskBind(pDev, nullptr, 0);
}

DSTATUS disk_status(BYTE pdrv)
{
	return pdrv == 0 && s_pFFDisk != nullptr ? 0 : STA_NOINIT;
}

DSTATUS disk_initialize(BYTE pdrv)
{
	return disk_status(pdrv);
}

DRESULT disk_read(BYTE pdrv, BYTE *buff, LBA_t sector, UINT count)
{
	if (pdrv != 0 || buff == nullptr || count == 0)
	{
		return RES_PARERR;
	}
	if (s_pFFDisk == nullptr)
	{
		return RES_NOTRDY;
	}
	if ((uint64_t)sector >= s_FFLogicalCount ||
		(uint64_t)count > s_FFLogicalCount - (uint64_t)sector)
	{
		return RES_PARERR;
	}

	while (count > 0)
	{
		uint32_t physical = (uint32_t)((uint64_t)sector / s_FFLogicalPerPhysical);
		uint32_t skip = (uint32_t)((uint64_t)sector % s_FFLogicalPerPhysical);
		uint32_t nb = s_FFLogicalPerPhysical - skip;

		if (nb > count)
		{
			nb = count;
		}

		if (s_FFLogicalPerPhysical == 1)
		{
			if (!s_pFFDisk->SectRead(physical, buff))
			{
				return RES_ERROR;
			}
		}
		else
		{
			if (!s_pFFDisk->SectRead(physical, s_pFFSectorBuffer))
			{
				return RES_ERROR;
			}
			memcpy(buff, s_pFFSectorBuffer + skip * FF_MIN_SS, nb * FF_MIN_SS);
		}
		buff += nb * FF_MIN_SS;
		sector += nb;
		count -= nb;
	}

	return RES_OK;
}

#if FF_FS_READONLY == 0

DRESULT disk_write(BYTE pdrv, const BYTE *buff, LBA_t sector, UINT count)
{
	if (pdrv != 0 || buff == nullptr || count == 0)
	{
		return RES_PARERR;
	}
	if (s_pFFDisk == nullptr)
	{
		return RES_NOTRDY;
	}
	if ((uint64_t)sector >= s_FFLogicalCount ||
		(uint64_t)count > s_FFLogicalCount - (uint64_t)sector)
	{
		return RES_PARERR;
	}

	while (count > 0)
	{
		uint32_t physical = (uint32_t)((uint64_t)sector / s_FFLogicalPerPhysical);
		uint32_t skip = (uint32_t)((uint64_t)sector % s_FFLogicalPerPhysical);
		uint32_t nb = s_FFLogicalPerPhysical - skip;
		uint8_t *pWrite = s_pFFSectorBuffer;

		if (nb > count)
		{
			nb = count;
		}

		if (s_FFLogicalPerPhysical == 1 && pWrite == nullptr)
		{
			// DiskIO's historical signature is non-const, but SectWrite
			// consumes the buffer and must not modify it.
			pWrite = const_cast<uint8_t *>(buff);
		}
		else
		{
			if ((skip != 0 || nb != s_FFLogicalPerPhysical) &&
				!s_pFFDisk->SectRead(physical, pWrite))
			{
				return RES_ERROR;
			}
			memcpy(pWrite + skip * FF_MIN_SS, buff, nb * FF_MIN_SS);
		}

		if (!s_pFFDisk->SectWrite(physical, pWrite))
		{
			return RES_ERROR;
		}

		buff += nb * FF_MIN_SS;
		sector += nb;
		count -= nb;
	}

	return RES_OK;
}
#endif

DRESULT disk_ioctl(BYTE pdrv, BYTE cmd, void *buff)
{
	if (pdrv != 0)
	{
		return RES_PARERR;
	}

	if (cmd == SET_DEVICE)
	{
		// Existing single-drive users can attach a 512-byte DiskIO.
		// A larger physical sector requires a scratch buffer.
		return FFDiskBind(static_cast<DiskIO *>(buff), nullptr, 0) ?
			RES_OK : RES_PARERR;
	}
	if (s_pFFDisk == nullptr)
	{
		return RES_NOTRDY;
	}

	switch (cmd)
	{
		case CTRL_SYNC:
			s_pFFDisk->Flush();
			return RES_OK;

		case GET_SECTOR_COUNT:
			if (buff == nullptr)
			{
				return RES_PARERR;
			}
			*static_cast<LBA_t *>(buff) = (LBA_t)s_FFLogicalCount;
			return RES_OK;

		case GET_SECTOR_SIZE:
			if (buff == nullptr)
			{
				return RES_PARERR;
			}
			*static_cast<WORD *>(buff) = FF_MIN_SS;
			return RES_OK;

		case GET_BLOCK_SIZE:
			if (buff == nullptr)
			{
				return RES_PARERR;
			}
			{
				uint32_t eraseSize = s_pFFDisk->GetMinEraseSize();
				*static_cast<DWORD *>(buff) = eraseSize > FF_MIN_SS ?
					eraseSize / FF_MIN_SS : 1;
			}
			return RES_OK;

		// Never erase a partial flash block through TRIM. The DiskIO
		// erase methods return void and cannot propagate failures.
		case CTRL_TRIM:
			return RES_PARERR;

		default:
			return RES_PARERR;
	}
}

static FRESULT FFDiskIsErased(bool *pErased)
{
	alignas(4) uint8_t local[FF_MIN_SS];
	uint8_t *pData = s_pFFSectorBuffer != nullptr ? s_pFFSectorBuffer : local;

	*pErased = false;

	for (uint32_t i = 0; i < s_FFPhysicalCount; i++)
	{
		if (!s_pFFDisk->SectRead(i, pData))
		{
			return FR_DISK_ERR;
		}
		for (uint32_t j = 0; j < s_FFPhysicalSize; j++)
		{
			if (pData[j] != 0xFF)
			{
				return FR_OK;
			}
		}
	}

	*pErased = true;
	return FR_OK;
}

FRESULT FatFsMount(const FatFsMountCfg_t &Cfg)
{
	if (Cfg.pDisk == nullptr || Cfg.pFatFs == nullptr)
	{
		return FR_INVALID_PARAMETER;
	}
	if (Cfg.bFormatIfBlank &&
		(Cfg.pFormatWork == nullptr || Cfg.FormatWorkSize < FF_MIN_SS ||
		 Cfg.pFormatWork == Cfg.pSectorBuffer))
	{
		return FR_INVALID_PARAMETER;
	}
	if (!FFDiskBind(Cfg.pDisk, Cfg.pSectorBuffer, Cfg.SectorBufferSize))
	{
		return FR_INVALID_PARAMETER;
	}

	const TCHAR *pPath = Cfg.pPath != nullptr ? Cfg.pPath : "0:";
	FRESULT res = f_mount(Cfg.pFatFs, pPath, 1);

	if (res != FR_NO_FILESYSTEM || !Cfg.bFormatIfBlank)
	{
		return res;
	}

	bool erased = false;
	res = FFDiskIsErased(&erased);

	if (res != FR_OK)
	{
		return res;
	}
	if (!erased)
	{
		return FR_NO_FILESYSTEM;
	}

	const MKFS_PARM defaults = { FM_ANY | FM_SFD, 0, 0, 0, 0 };
	const MKFS_PARM *pFormat = Cfg.pFormatOptions != nullptr ?
		Cfg.pFormatOptions : &defaults;

	res = f_mkfs(pPath, pFormat, Cfg.pFormatWork, Cfg.FormatWorkSize);
	if (res == FR_OK)
	{
		res = f_mount(Cfg.pFatFs, pPath, 1);
	}
	return res;
}

FRESULT FatFsUnmount(const TCHAR *pPath)
{
	FRESULT res = f_mount(nullptr, pPath != nullptr ? pPath : "0:", 0);

	if (res == FR_OK)
	{
		(void)FFDiskBind(nullptr, nullptr, 0);
	}
	return res;
}
