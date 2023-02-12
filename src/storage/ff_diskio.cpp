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

#include <string.h>
#include <stdio.h>

#include "ff.h"			/* Obtains integer types */
#include "ff_diskio.h"		/* Declarations of device I/O functions */

#include "istddef.h"
#include "crc.h"
#include "storage/diskio.h"

static DiskIO *s_pFFDiskIO = nullptr;

void FFDiskSetDevice(DiskIO * const pDev)
{
	s_pFFDiskIO = pDev;
}

/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	DSTATUS stat;
	int result;

	if (pdrv > 0)
	{
		return RES_PARERR;
	}

	return RES_OK;

	switch (pdrv) {
	case DEV_RAM :
		//result = RAM_disk_status();

		// translate the reslut code here

		return stat;

	case DEV_MMC :
		//result = MMC_disk_status();

		// translate the reslut code here

		return stat;

	case DEV_USB :
		//result = USB_disk_status();

		// translate the reslut code here

		return stat;
	}
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	static bool s_bInitialized = false;

	if (pdrv > 0)
	{
		return RES_PARERR;
	}

	if (s_bInitialized == false)
	{
		if (s_pFFDiskIO == nullptr)
		{
			return STA_NOINIT;
		}

		s_bInitialized = true;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA */
	UINT count		/* Number of sectors to read */
)
{
	DRESULT res;
	int result;

	if (pdrv > 0)
	{
		return RES_PARERR;
	}

	uint64_t offset = (uint64_t)sector * FF_MIN_SS;

	while (count > 0)
	{
		int size = FF_MIN_SS;

		while (size > 0)
		{
			int l = s_pFFDiskIO->Read(offset, (uint8_t*)buff, size);
			buff += l;
			offset += l;
			size -= l;
		}
		count--;
	}

	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	DRESULT res;
	int result;

	if (pdrv > 0)
	{
		return RES_PARERR;
	}

	if (count <= 0)
	{
		return RES_OK;
	}

	uint32_t offs = sector * FF_MIN_SS;

	s_pFFDiskIO->Write(offs, (uint8_t*)buff, count * FF_MIN_SS);

	return RES_OK;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	DRESULT res = RES_OK;
	int result;

	if (pdrv > 0)
	{
		return RES_PARERR;
	}

	switch (cmd)
	{
		case CTRL_SYNC :

			s_pFFDiskIO->Flush();

			return RES_OK;

		case GET_SECTOR_COUNT :

			// Process of the command for the MMC/SD card
			*(uint32_t *)buff = s_pFFDiskIO->GetSize() * 1024U / FF_MIN_SS;

			return RES_OK;

		case GET_SECTOR_SIZE :
			*(uint32_t *)buff = FF_MIN_SS;//s_pFFDiskIO->GetSectSize();
			return RES_OK;
		case GET_BLOCK_SIZE:
			*(uint32_t *)buff = s_pFFDiskIO->GetSectSize();
			return RES_OK;
		case CTRL_TRIM:
			{
				uint32_t *p = (uint32_t*)buff;
				uint32_t sz = s_pFFDiskIO->GetSectSize();
				uint32_t st = p[0] * FF_MIN_SS / sz;
				uint32_t end = p[1] * FF_MIN_SS / sz;

				s_pFFDiskIO->EraseSector(st, end - st);
			}

			return RES_OK;

		case SET_DEVICE:

			s_pFFDiskIO = (DiskIO*)buff;

			return RES_OK;
	}

	return RES_PARERR;
}
