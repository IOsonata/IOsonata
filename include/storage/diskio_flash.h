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

static const FlashDiskIOCfg_t s_FlashDiskCfg = {
    .DevNo = 0,
    .TotalSize = 16 * 1024 / 8,		// 16 Mbits in KBytes
	.SectSize = 4 * 1024,					// 4K
    .BlkSize = 64 * 1024,					// 64K
    .WriteSize = 256,				// Write page size
    .AddrSize = 3,              	// 3 bytes addressing
	.pInitCB = NULL.				// no special init require.
    .pWaitCB = NULL,				// blocking, no wait callback
};

-----
S25FS :

static const FlashDiskIOCfg_t s_FlashDiskCfg = {
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

static const FlashDiskIOCfg_t s_FlashDiskCfg = {
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
static FlashDiskIOCfg_t s_N25Q128A_QFlashCfg = {
    .DevNo = 0,
    .TotalSize = 128 * 1024 / 8,	// 128 Mbits in KBytes
	.SectSize = 4 * 1024,
    .BlkSize = 32 * 1024,
    .WriteSize = 256,
    .AddrSize = 3,					// 3 bytes addressing
    .pInitCB = NULL,//MX25U1635E_init,
    .pWaitCB = NULL,//FlashWriteDelayCallback,
	.RdCmd = { FLASH_CMD_QREAD, 10},
	.WrCmd = { FLASH_CMD_QWRITE, 0 },
};

// Quad SPI Flash Macronix MX25R3235F
static FlashDiskIOCfg_t s_MX25R3235F_QFlashCfg = {
    .DevNo = 0,
    .TotalSize = 32 * 1024 / 8,		// 16 Mbits
	.SectSize = 4 * 1024,
    .BlkSize = 64 * 1024,
    .WriteSize = 256,
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

#include "diskio.h"
#include "device_intrf.h"
#include "flash.h"

/** @addtogroup Storage
  * @{
  */

#ifdef __cplusplus

/// @brief	Flash disk base class
///
/// Most Flash devices work in MSB bit order. This implementation
/// only supports MSB mode. Make sure that the Flash is configured
/// for MSB mode.
class FlashDiskIO : public DiskIO {
public:
	FlashDiskIO();
	virtual ~FlashDiskIO() {}

	operator FlashDev_t * const () { return &vDevData; }

	/**
	 * @brief	Initialize Flash Disk.
	 *
	 * @param	Cfg		: Flash disk configuration data
	 * @param	pInterf	: Pointer to device interface to access flash device
	 * @param	pCacheBlk	: Pointer to static cache block (optional)
	 * @param	NbCacheBlk	: Size of cache block (Number of cache sector)
	 *
	 * @return
	 * 			- true 	: Success
	 * 			- false	: Failed
	 */
	bool Init(const FlashCfg_t &Cfg, DeviceIntrf * const pInterf,
			  DiskIOCache_t * const pCacheBlk = NULL, int NbCacheBlk = 0);

	/**
	 * @brief	Get total disk size in bytes.
	 *
	 * @return	Total size in KBytes
	 */
	virtual uint32_t GetSize(void) { return vDevData.TotalSize; }

    /**
	 * @brief	Device specific minimum erasable block size in bytes.
	 *
	 * @return	Block size in bytes
	 */
	virtual uint16_t GetSectSize(void) { return vDevData.SectSize; }

	/**
	 * @brief	Device specific minimum write size in bytes
	 *
	 * @return	Size in bytes
	 */
	virtual uint32_t GetPageSize() { return vDevData.PageSize; }

	/**
	 * @brief	Perform mass erase (ERASE ALL).
	 *
	 * This function may take a long time to complete. If task switching is require, add delay
	 * callback function to the configuration at initialization.
	 */
	virtual void Erase() { FlashErase(&vDevData); }

	/**
	 * @brief	Erase Flash block.
	 *
	 * @param	BlkNo	: Starting block number to erase.
	 * @param	NbBlk	: Number of consecutive blocks to erase
	 */
	virtual void EraseBlock(uint32_t BlkNo, int NbBlk) { FlashEraseBlock(&vDevData, BlkNo, NbBlk); }

	/**
	 * @brief	Erase Flash block.
	 *
	 * @param	BlkNo	: Starting block number to erase.
	 * @param	NbBlk	: Number of consecutive blocks to erase
	 */
	virtual void EraseSector(uint32_t SectNo, int NbSect) { FlashEraseSector(&vDevData, SectNo, NbSect); }

	/**
	 * @brief	Read one sector from physical device.
	 *
	 * @param	SectNo	: Sector number to read
	 * @param	pBuff	: Pointer to buffer to receive sector data. Must be at least
	 * 					  1 sector size
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	virtual bool SectRead(uint32_t SectNo, uint8_t *pBuff) { return FlashSectRead(&vDevData, SectNo, pBuff); }

	/**
	 * @brief	Write one sector to physical device
	 *
	 * @param	SectNo	: Sector number to read
	 * @param	pData	: Pointer to sector data to write. Must be at least
	 * 					  1 sector size
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	virtual bool SectWrite(uint32_t SectNo, uint8_t *pData) {
		EraseSector(SectNo, 1);
		return FlashSectWrite(&vDevData, SectNo, pData); }

	/**
	 * @brief	Read Flash ID
	 *
	 * @param	Len : Length of id to read in bytes
	 *
	 * @return	Flash ID
	 */
	uint32_t ReadId(int Len) { return FlashReadId(&vDevData, Len); }

	/**
	 * @brief	Read Flash status.
	 *
	 * @return	Flash status
	 */
	uint8_t ReadStatus() { return FlashReadStatus(&vDevData); }

	/**
	 * @brief	Get the sector erase size
	 *
	 * The return value is normally set via configuration structure at init
	 *
	 * @return	Size in KBytes
	 */
	uint16_t SectEraseSize() { return vDevData.SectSize; }

	/**
	 * @brief	Get the block erase size
	 *
	 * The return value is normally set via configuration structure at init
	 *
	 * @return	Size in KBytes
	 */
	uint16_t BlockEraseSize() { return vDevData.BlkSize; }

	/**
	 * @brief	Reset DiskIO to its default state
	 */
	virtual void Reset();

protected:

	/**
	 * @brief	Disable Flash write
	 */
	void WriteDisable();

	/**
	 * @brief	Enable Flash write
	 *
	 * @param	Timeout : Timeout counter
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	bool WriteEnable(uint32_t Timeout = 100000);

	/**
	 * @brief	Wait for Flash ready flag
	 *
	 * @param	Timeout : Timeout counter
	 * @param	usRtyDelay	: Timeout in us before retry (optional)
	 *
	 * @return
	 * 			- true	: Success
	 * 			- false	: Failed
	 */
	bool WaitReady(uint32_t Timeout = 100000, uint32_t usRtyDelay = 0);

private:
	FlashDev_t vDevData;		//!< Flash device data
};

extern "C" {
#endif

#ifdef __cplusplus
}
#endif

/** @} End of group Storage */

#endif	// __DISKIO_FLASH_H__

