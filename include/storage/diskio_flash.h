/**--------------------------------------------------------------------------
@file	diskio_flash.h

@brief	Generic flash disk I/O driver class

Most Flash devices work in MSB bit order. This implementation only support MSB.
Make sure that the Flash is configure for MSB mode

This implementation works with most Flash devices.  There is no need to implement
for each device, just fill the config data struct and pass it to init function

Example of defining Flash device info :

-----
MX25R1635F :

static const FlashCfg_t s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 16 * 1024 / 8,		// 16 Mbits in KBytes
	.SectSize = 4 * 1024,					// 4K
    .BlkSize = 64 * 1024,					// 64K
    .PageSize = 256,				// Write page size
    .AddrSize = 3,              	// 3 bytes addressing
	.pInitCB = NULL.				// no special init require.
    .pWaitCB = NULL,				// blocking, no wait callback
};

-----
S25FS :

static const FlashCfg_t s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 128 * 1024 / 8,	// 128 Mbits
	.SectSize = 4 * 1024,					// 4K
    .BlkSize = 64 * 1024,					// 64K
    .WriteSize = 512,				// Write page size
    .AddrSize = 3,
    .pInitCB = s25fs_init,			// Special initialization require
    .pWaitCB = NULL,
};

bool s25fs_init(int DevNo, DeviceIntrf *pInterf)
{
    if (pInterf == NULL)
        return false;
    int cnt = 0;

    uint32_t d;

    // Enable write
    d = NFLASH_S25FS_CMD_WREN;
    cnt = pInterf->Tx(DevNo, (uint8_t*)&d, 1);

    do {
        d = NFLASH_S25FS_CMD_RDSR1;
        pInterf->StartRx(DevNo);
        cnt = pInterf->TxData((uint8_t*)&d, 1);
        cnt = pInterf->RxData((uint8_t*)&d, 1);
        pInterf->StopRx();
    } while ((d & NFLASH_S25FS_REG_SR1V_WIP));

    // Configure uniform sector arch
    uint8_t p[8];
    p[0] = NFLASH_S25FS_CMD_WRAR;
    p[1] = (NFLASH_S25FS_REG_CR3V >> 16) & 0xFF;
    p[2] = (NFLASH_S25FS_REG_CR3V >> 8) & 0xFF;
    p[3] = NFLASH_S25FS_REG_CR3V & 0xFF;
    p[4] = NFLASH_S25FS_REG_CR3NV_20h_NV | NFLASH_S25FS_REG_CR3V_02h_NV;

    cnt = pInterf->Tx(DevNo, p, 5);

    return true;
}

-----
MX66U51235F :

static const FlashCfg_t s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 256 * 1024 / 8,	// 256 Mbits in KBytes
	.SectSize = 4 * 1024,					// 4K
    .BlkSize = 64 * 1024,					// 64K
    .WriteSize = 128,				// Write page size
    .AddrSize = 4,                  // 256+ Mbits needs 4 bytes addressing
    .pInitCB = NULL,
    .pWaitCB = NULL
};


// Quad SPI Flash Micron N25Q128A
static FlashCfg_t s_N25Q128A_QFlashCfg = {
    .DevNo = 0,
    .TotalSize = 128 * 1024 / 8,	// 128 Mbits in KBytes
	.SectSize = 4 * 1024,
    .BlkSize = 32 * 1024,
    .PageSize = 256,
    .AddrSize = 3,					// 3 bytes addressing
    .pInitCB = NULL,//MX25U1635E_init,
    .pWaitCB = NULL,//FlashWriteDelayCallback,
	.RdCmd = { FLASH_CMD_QREAD, 10},
	.WrCmd = { FLASH_CMD_QWRITE, 0 },
};

// Quad SPI Flash Macronix MX25R3235F
static FlashCfg_t s_MX25R3235F_QFlashCfg = {
    .DevNo = 0,
    .TotalSize = 32 * 1024 / 8,		// 16 Mbits
	.SectSize = 4 * 1024,
    .BlkSize = 64 * 1024,
    .PageSize = 256,
    .AddrSize = 3,					// 3 bytes addressing
    .pInitCB = NULL,//MX25U1635E_init,
    .pWaitCB = NULL,//FlashWriteDelayCallback,
	.RdCmd = { FLASH_CMD_4READ, 6},
	.WrCmd = { FLASH_CMD_4WRITE, 0 },
};

-----
Usage in C++ :

// SPI interface instance to be used.  Assuming it is already initialized
SPI g_Spi;

// Declare device instance
FlashDiskIO g_FlashDisk;

// Disk sector cache block in RAM
uint8_t g_FlashCacheMem[DISKIO_SECT_SIZE];
DISKIO_CACHE_DESC g_FlashCache = {
    -1, 0xFFFFFFFF, g_FlashCacheMem
};

// Initialize
g_FlashDisk.Init(s_FlashDiskCfg, &g_Spi, &g_FlashCache, 1);

// Read/Write
uint8_t buff[DISKIO_SECT_SIZE];

g_FlashDisk.SectRead(1, buff);	// Read sector 1
g_FlashDisk.SectWrite(2, buff);	// Write sector 2
g_FlashDisk.Erase();			// Mass erase flash


@author	Hoang Nguyen Hoan
@date	Aug. 30, 2016

@license

MIT License

Copyright (c) 2016 I-SYST inc. All rights reserved.

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
#ifndef __DISKIO_FLASH_H__
#define __DISKIO_FLASH_H__

#include <stdint.h>
#include <string.h>

#include "storage/diskio.h"
#include "storage/nvm_flash.h"
#include "device_intrf.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Flash disk block adapter.
///
/// Presents a NOR Flash as a rewritable block device for filesystem use
/// (FatFS, USB MSC). SectWrite performs erase then program of the whole
/// sector; combined with the DiskIO write back cache this is a read modify
/// write presentation and is not power loss safe. Log structured consumers
/// (littlefs, bt_pds) must use the NvmFlash device directly instead.
class FlashDiskIO : public DiskIO {
public:
	FlashDiskIO();
	virtual ~FlashDiskIO() {}

	/**
	 * @brief	Initialize the Flash and the sector cache.
	 *
	 * @param	Cfg			: Flash device configuration
	 * @param	pInterf		: Interface to access the device (SPI/QSPI)
	 * @param	pCacheBlk	: Sector cache block array, NULL : no cache
	 * @param	NbCacheBlk	: Number of cache blocks
	 *
	 * @return	true on success.
	 */
	bool Init(const FlashCfg_t &Cfg, DeviceIntrf * const pInterf,
			  DiskIOCache_t * const pCacheBlk = NULL, int NbCacheBlk = 0);

	/**
	 * @brief	Device size in KBytes.
	 */
	virtual uint32_t GetSize(void) { return (uint32_t)(vFlash.Size() / 1024ULL); }

	/**
	 * @brief	Sector size in bytes.
	 */
	virtual uint16_t GetSectSize(void) { return (uint16_t)vFlash.EraseSize(); }

	/**
	 * @brief	Program page size in bytes.
	 */
	virtual uint32_t GetPageSize(void) { return vFlash.PageSize(); }

	/**
	 * @brief	Erase the entire device.
	 */
	virtual void Erase(void) { vFlash.MassErase(); }

	/**
	 * @brief	Erase Flash block.
	 *
	 * @param	BlkNo	: Starting block number to erase
	 * @param	NbBlk	: Number of blocks to erase
	 */
	virtual void EraseBlock(uint32_t BlkNo, int NbBlk) {
		vFlash.Erase((uint64_t)BlkNo * BlockEraseSize(),
					 (uint32_t)NbBlk * BlockEraseSize());
	}

	/**
	 * @brief	Erase Flash sector.
	 *
	 * @param	SectNo	: Starting sector number to erase
	 * @param	NbSect	: Number of sectors to erase
	 */
	virtual void EraseSector(uint32_t SectNo, int NbSect) {
		vFlash.Erase((uint64_t)SectNo * vFlash.EraseSize(),
					 (uint32_t)NbSect * vFlash.EraseSize());
	}

	/**
	 * @brief	Read one sector.
	 *
	 * @param	SectNo	: Sector number to read
	 * @param	pBuff	: Buffer of at least one sector size
	 *
	 * @return	true on success.
	 */
	virtual bool SectRead(uint32_t SectNo, uint8_t *pBuff) {
		uint32_t ss = vFlash.EraseSize();
		return vFlash.Read((uint64_t)SectNo * ss, pBuff, ss) == (int)ss;
	}

	/**
	 * @brief	Write one sector: erase then program.
	 *
	 * @param	SectNo	: Sector number to write
	 * @param	pData	: Buffer of one sector size to write
	 *
	 * @return	true on success.
	 */
	virtual bool SectWrite(uint32_t SectNo, uint8_t *pData) {
		uint32_t ss = vFlash.EraseSize();
		if (vFlash.Erase((uint64_t)SectNo * ss, ss) != 0)
		{
			return false;
		}
		return vFlash.Write((uint64_t)SectNo * ss, pData, ss) == (int)ss;
	}

	/**
	 * @brief	Read Flash JEDEC id.
	 */
	uint32_t ReadId(int Len) { return vFlash.ReadId(Len); }

	/**
	 * @brief	Read Flash status register.
	 */
	uint8_t ReadStatus(void) { return vFlash.ReadStatus(); }

	/**
	 * @brief	Sector erase size in bytes.
	 */
	uint32_t SectEraseSize(void) { return vFlash.EraseSize(); }

	/**
	 * @brief	Block erase size in bytes.
	 */
	uint32_t BlockEraseSize(void) { return vBlkSize; }

	/**
	 * @brief	Access the underlying NvmFlash device.
	 */
	NvmFlash &Memory(void) { return vFlash; }

	/**
	 * @brief	Reset DiskIO to its default state.
	 */
	virtual void Reset(void);

private:
	NvmFlash vFlash;	//!< Underlying Flash device
	uint32_t vBlkSize;	//!< Block erase size in bytes
};

extern "C" {
#endif	// __cplusplus

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __DISKIO_FLASH_H__
