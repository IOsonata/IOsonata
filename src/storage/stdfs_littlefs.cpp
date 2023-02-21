/**-------------------------------------------------------------------------
@file	stdfs_littlefs.cpp

@brief	Implement default std filesystem using LittleFS

@author	Hoang Nguyen Hoan
@date	Feb. 18, 2023

@license

MIT License

Copyright (c) 2023, I-SYST inc., all rights reserved

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
#include <stdlib.h>
#include <fcntl.h>

#include "lfs.h"

#include "stddev.h"
#include "storage/diskio_flash.h"
#include "storage/stdfs_littlefs.h"

uint32_t r  = O_RDONLY;

#define FDIDX_MASK		0xFFL

// open
static int FsOpen(void * const pDevObj, const char *pDevName, int Flags, int Mode);
// close
static int FsClose(void * const pDevObj, int Handle);
// Read/Write
static int FsRead(void * const pDevObj, int Handle, uint8_t *pBuff, size_t Len);
static int FsWrite(void * const pDevObj, int Handle, uint8_t *pBuff, size_t Len);
// seek
static int FsSeek(void * const pDevObj, int Handle, int Offset);

static int LittleFsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
static int LittleFsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
static int LittleFsErase(const struct lfs_config *c, lfs_block_t block);
static int LittleFsSync(const struct lfs_config *c);

typedef struct __FS_Dev_Config {

} FsDevCfg_t;

#define FILE_OPEN_MAX_COUNT			4

typedef struct __FS_FIle_Open {
	bool bOpen;
	lfs_file_t File;
} FSFile_t;

typedef struct __FS_Device {
	DiskIO *pDiskIO;
	lfs_t LtleFs;
	FSFile_t Files[FILE_OPEN_MAX_COUNT];
	uint32_t Handle;
} FSDev_t;

static StdDev_t s_LittleFsStdDev = {
	"LtleFS",
	nullptr,
	FsOpen,
	FsClose,
	FsRead,
	FsWrite,
	FsSeek
};

static struct lfs_config s_LittelFSCfg = {
    // block device operations
    .read  = LittleFsRead,
    .prog  = LittleFsProg,
    .erase = LittleFsErase,
    .sync  = LittleFsSync,

    // block device configuration
    .read_size = 1,
    .prog_size = 1,
    .block_size = 4096,
    .block_count = 128,
    .block_cycles = 500,
    .cache_size = 16,
    .lookahead_size = 16,
};

static FSDev_t s_FSDevice = {
	nullptr,
};

bool StdFSInit(DiskIO * const pDiskIO)
{
	if (pDiskIO == NULL)
	{
		return false;
	}

	s_FSDevice.pDiskIO = pDiskIO;

	s_LittelFSCfg.block_size = pDiskIO->GetSectSize();
	s_LittelFSCfg.block_count = pDiskIO->GetNbSect();

    // mount the filesystem
    int err = lfs_mount(&s_FSDevice.LtleFs, &s_LittelFSCfg);

    if (err == LFS_ERR_OK)
    {
    	memset(s_FSDevice.Files, 0, sizeof(FSFile_t) * FILE_OPEN_MAX_COUNT);

    	s_FSDevice.Handle = InstallBlkDev(&s_LittleFsStdDev, STDFS_FILENO);

    	return true;
    }

    return false;
}

// open
static int FsOpen(void * const pDevObj, const char *pName, int Flags, int Mode)
{
	int retval = -1;

	for (int i = 0; i < FILE_OPEN_MAX_COUNT; i++)
	{
		if (s_FSDevice.Files[i].bOpen == false)
		{
			retval = i;
			break;
		}
	}

	if (retval >= 0)
	{
		int lflags = 0;
		if (Flags == 0)
		{
			lflags |= LFS_O_RDONLY;
		}
		if (Flags & O_WRONLY)
		{
			lflags |= LFS_O_WRONLY;
		}
		if (Flags & O_RDWR)
		{
			lflags |= LFS_O_RDWR;
		}
		if (Flags & O_APPEND)
		{
			lflags |= LFS_O_APPEND;
		}
		if (Flags & O_CREAT)
		{
			lflags |= LFS_O_CREAT;
		}
		if (Flags & O_TRUNC)
		{
			lflags |= LFS_O_TRUNC;
		}
		if (Flags & O_EXCL)
		{
			lflags |= LFS_O_EXCL;
		}

		int err = lfs_file_open(&s_FSDevice.LtleFs, &s_FSDevice.Files[retval].File, pName, lflags & (LFS_O_RDWR | LFS_O_CREAT));

		if (err == LFS_ERR_OK)
		{
			s_FSDevice.Files[retval].bOpen = true;
		}
	}

	return retval;
}

// close
static int FsClose(void * const pDevObj, int Handle)
{
	if (Handle < 0 || Handle >= FILE_OPEN_MAX_COUNT)
	{
		return -1;
	}

    lfs_file_close(&s_FSDevice.LtleFs, &s_FSDevice.Files[Handle & FDIDX_MASK].File);

	memset(&s_FSDevice.Files[Handle & FDIDX_MASK], 0, sizeof(FSFile_t));

	return Handle;
}

// Read/Write
static int FsRead(void * const pDevObj, int Handle, uint8_t *pBuff, size_t Len)
{
    return lfs_file_read(&s_FSDevice.LtleFs, &s_FSDevice.Files[Handle & FDIDX_MASK].File, pBuff, Len);
}

static int FsWrite(void * const pDevObj, int Handle, uint8_t *pBuff, size_t Len)
{
    return lfs_file_write(&s_FSDevice.LtleFs, &s_FSDevice.Files[Handle & FDIDX_MASK].File, pBuff, Len);
}

// seek
static int FsSeek(void * const pDevObj, int Handle, int Offset)
{
	return lfs_file_seek(&s_FSDevice.LtleFs, &s_FSDevice.Files[Handle & FDIDX_MASK].File, Offset, LFS_SEEK_SET);
}

int LittleFsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	s_FSDevice.pDiskIO->Read(block, off, (uint8_t*)buffer, size);

	return LFS_ERR_OK;
}

int LittleFsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	s_FSDevice.pDiskIO->Write(block, off, (uint8_t*)buffer, size);
	return LFS_ERR_OK;
}

int LittleFsErase(const struct lfs_config *c, lfs_block_t block)
{
	s_FSDevice.pDiskIO->EraseSector(block, 1);
	return LFS_ERR_OK;
}

int LittleFsSync(const struct lfs_config *c)
{
	s_FSDevice.pDiskIO->Flush();

	return LFS_ERR_OK;
}

