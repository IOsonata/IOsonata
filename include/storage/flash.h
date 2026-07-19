/**--------------------------------------------------------------------------
@file	flash.h

@brief	Generic flash device configuration.

This header holds the Flash command set, the configuration structure and the
predefined part configurations. The driver itself is the NvmFlash class in
storage/nvm_flash.h; the FlashDiskIO block adapter in storage/diskio_flash.h
sits on top of it for filesystem use.

Most Flash devices work in MSB bit order. Only MSB mode is supported. There is
no need to implement a driver per device, just fill the config data struct and
pass it to NvmFlash::Init

Example of defining Flash device info :

-----
MX25R1635F :

static const FlashCfg_t s_FlashCfg = {
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

static const FlashCfg_t s_FlashCfg = {
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

static const FlashCfg_t s_FlashCfg = {
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
    .WriteSize = 256,
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
NvmFlash g_Flash;

// Initialize covering the whole device
g_Flash.Init(s_FlashCfg, &g_Spi);

// Byte addressed access
uint8_t buff[256];

g_Flash.Read(0, buff, sizeof(buff));
g_Flash.Erase(0, g_Flash.EraseSize());
g_Flash.Write(0, buff, sizeof(buff));


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
#ifndef __FLASH_H__
#define __FLASH_H__

#include <stdint.h>
#include <string.h>

#include "device_intrf.h"

/** @addtogroup Storage
  * @{
  */

#define FLASH_CMD_READID			0x9F
#define FLASH_CMD_RDCR              0x15    //!< Read configuration register
#define FLASH_CMD_WRSR              0x01    //!< Write Register (Status 1, Configuration 1)
#define FLASH_CMD_WRITE             0x2
#define FLASH_CMD_WRITE_4B          0x12	//!< Program page in 4 bytes addr mode
#define FLASH_CMD_DWRITE            0xA2
#define FLASH_CMD_4WRITE            0x38
#define FLASH_CMD_4WRITE_4B         0x3E	//!< Quad write in 4 bytes addr mode
#define FLASH_CMD_QWRITE			0x32
#define FLASH_CMD_E4WRITE			0x12
#define FLASH_CMD_READ              0x3
#define FLASH_CMD_FAST_READ			0xB
#define FLASH_CMD_DREAD				0x3B	//!< Dual read - address single, data dual
#define FLASH_CMD_QREAD				0x6B	//!< Quad read - address single, data quad
#define FLASH_CMD_2READ				0xBB	//!< 2 x I/O read - address dual, data dual
#define FLASH_CMD_4READ				0xEB	//!< 4 x I/O read - address quad, data quad
#define FLASH_CMD_WRDISABLE         0x4
#define FLASH_CMD_READSTATUS        0x5
#define FLASH_CMD_WRENABLE          0x6
#define FLASH_CMD_EN4B              0xB7    //!< Enable 4 bytes address
#define FLASH_CMD_DIS4B             0xE9    //!< Disable 4 bytes address
#define FLASH_CMD_SECTOR_ERASE		0x20	//!< Sector erase
#define FLASH_CMD_SECTOR_ERASE_4B	0x21	//!< Sector erase in 4 bytes address mode
#define FLASH_CMD_BLOCK_ERASE32K    0x52	//!< 32KB Block erase
#define FLASH_CMD_BLOCK_ERASE32K_4B 0x5C	//!< 32KB Block erase in 4 bytes addr mode
#define FLASH_CMD_BLOCK_ERASE       0xD8	//!< Block erase
#define FLASH_CMD_BLOCK_ERASE_4B    0xDC	//!< Block erase in 4 bytes addr mode
#define FLASH_CMD_BULK_ERASE        0xC7	//!< Chip erase
#define FLASH_CMD_BULK_ERASE_ALT	0x60	//!< Alternate chip erase command

#define FLASH_CMD_RESET_ENABLE		0x66	//!< Enable reset
#define FLASH_CMD_RESET_DEVICE		0x99	//!< Reset
#define FLASH_CMD_DPD				0xB9	//!< Enter deep power down
#define FLASH_CMD_RDPD				0xAB	//!< Release deep power down

#define FLASH_STATUS_WIP            (1<<0)  // Write In Progress
#define FLASH_STATUS_WEL			(1<<1)	// Write enable
#define FLASH_STATUS_QE				(1<<6)	// Quad enable

#define FLASH_SECTOR_ERASE_SIZE		(4096)	//! Most flash has this sector size

#pragma pack(push, 1)
/// Quad SPI flash can have different command code and dummy cycle.
/// This structure is to define supported command for the Flash config.
/// Assign the appropriate command code and dymmy cycle count in the config structure
typedef struct __Quad_Flash_Cmd {
	uint8_t Cmd;				//!< Command code
	uint8_t DummyCycle;			//!< Dummy cycle
} CmdCycle_t;

typedef CmdCycle_t	CMDCYCLE;

/**
 * @brief FlashDiskIO callback function.
 *
 * This a general callback function hook for special purpose defined in the FLASHDISKIO_CFG
 *
 * @param   DevNo 	: Device number or address used by the interface
 * @param   pInterf : Interface used to access the flash (SPI, I2C or whatever
 *
 * @return  true - Success\n
 *          false - Failed.
 */
typedef bool (*FlaskCb_t)(int DevNo, DevIntrf_t * const pDevIntrf);

typedef struct __Flash_Config {
    int         DevNo;          //!< Device number or address for interface use
    uint32_t    TotalSize;      //!< Total Flash size in KBytes
    uint32_t	BlkSize;		//!< Block erase size in Bytes
    uint16_t    SectSize;		//!< Sector erase size in Bytes
    uint16_t    PageSize;      	//!< Writable page size in bytes
    int         AddrSize;       //!< Address size in bytes
    uint32_t	DevId;			//!< Device ID, read using FLASH_CMD_READID
    int			DevIdSize;		//!< Length of device id in bytes to read (max 4 bytes)
    FlaskCb_t 	pInitCB; 		//!< For custom initialization. Set to NULL if not used
    FlaskCb_t 	pWaitCB;		//!< If provided, this is called when there are
    							//!< long delays, such as mass erase, to allow application
    							//!< to perform other tasks while waiting
    CmdCycle_t	RdCmd;			//!< QSPI read cmd and dummy cycle
    CmdCycle_t	WrCmd;			//!< QSPI write cmd and dummy cycle
} FlashCfg_t;


#pragma pack(pop)


/// 512Mb MT25QL512 Flash definition
#define FLASH_MT25QL512_SECTSIZE		FLASH_SECTOR_ERASE_SIZE
#define FLASH_MT25QL512(InitCB, WaitCB)		{ \
	.DevNo = 0, \
	.TotalSize = 512 * 1024 / 8, \
	.BlkSize = 32 * 1024, \
	.SectSize = FLASH_MT25QL512_SECTSIZE, \
	.PageSize = 256, \
	.AddrSize = 4, \
	.DevId = 0x20ba20, \
	.DevIdSize = 3, \
	.pInitCB = InitCB,\
	.pWaitCB = WaitCB }

/// Micron N25Q128A 128Mb, QUAD
#define FLASH_N25Q128A_SECTSIZE			FLASH_SECTOR_ERASE_SIZE
#define FLASH_N25Q128A(InitCB, WaitCB)		{ \
    .DevNo = 0, \
    .TotalSize = 128 * 1024 / 8, \
    .BlkSize = 32 * 1024, \
	.SectSize = FLASH_N25Q128A_SECTSIZE, \
    .PageSize = 256, \
    .AddrSize = 3, \
    .pInitCB = InitCB, \
    .pWaitCB = WaitCB, \
	.RdCmd = { FLASH_CMD_QREAD, 10}, \
	.WrCmd = { FLASH_CMD_QWRITE, 0 }, }

/// MX25U1635E 16Mb
#define FLASH_MX25U1635E_SECTSIZE		FLASH_SECTOR_ERASE_SIZE
#define FLASH_MX25U1635E(InitCB, WaitCB)		{ \
	.DevNo = 0, \
	.TotalSize = 16 * 1024 / 8, \
	.BlkSize = 32 * 1024, \
	.SectSize = FLASH_MX25U1635E_SECTSIZE, \
	.PageSize = 256, \
	.AddrSize = 3, \
	.pInitCB = InitCB, \
	.pWaitCB = WaitCB, }

/// Macronix MX25R3235F 32Mb Quad
#define FLASH_MX25R3235F_SECTSIZE		FLASH_SECTOR_ERASE_SIZE
#define FLASH_MX25R3235F(InitCB, WaitCB)		{ \
    .DevNo = 0, \
    .TotalSize = 32 * 1024 / 8, \
    .BlkSize = 64 * 1024,\
	.SectSize = FLASH_MX25R3235F_SECTSIZE, \
    .PageSize = 256, \
    .AddrSize = 3, \
    .pInitCB = InitCB, \
    .pWaitCB = WaitCB, \
	.RdCmd = { FLASH_CMD_4READ, 6}, \
	.WrCmd = { FLASH_CMD_4WRITE, 0 }, }

/// Macronix MX25R6435F 64Mb Quad
#define FLASH_MX25R6435F_SECTSIZE		FLASH_SECTOR_ERASE_SIZE
#define FLASH_MX25R6435F(InitCB, WaitCB)		{ \
    .DevNo = 0, \
    .TotalSize = 64 * 1024 / 8, \
    .BlkSize = 64 * 1024, \
	.SectSize = FLASH_MX25R6435F_SECTSIZE, \
    .PageSize = 256, \
    .AddrSize = 3, \
	.DevId = 0x1728c2, \
	.DevIdSize = 3, \
    .pInitCB = InitCB, \
    .pWaitCB = WaitCB, \
	.RdCmd = { FLASH_CMD_4READ, 6}, \
	.WrCmd = { FLASH_CMD_4WRITE, 0 }, }

/// MX25L25645G 256Mb
#define FLASH_MX25L25645G_SECTSIZE		FLASH_SECTOR_ERASE_SIZE
#define FLASH_MX25L25645G(InitCB, WaitCB)		{ \
	.DevNo = 0, \
	.TotalSize = 256 * 1024 / 8, \
	.BlkSize = 32 * 1024, \
	.SectSize = FLASH_MX25L25645G_SECTSIZE, \
	.PageSize = 256, \
	.AddrSize = 4, \
	.DevId = 0x1920c2, \
	.DevIdSize = 3, \
	.pInitCB = InitCB, \
	.pWaitCB = WaitCB, \
	.RdCmd = { FLASH_CMD_QREAD, 6}, \
	.WrCmd = { FLASH_CMD_4WRITE, 0 }, }

/// IS25LP512M 512Mb Quad
#define FLASH_IS25LP512_SECTSIZE		FLASH_SECTOR_ERASE_SIZE
#define FLASH_IS25LP512(InitCB, WaitCB)			{ \
	.DevNo = 0, \
	.TotalSize = 512 * 1024 / 8, \
	.BlkSize = 32 * 1024, \
	.SectSize = FLASH_IS25LP512_SECTSIZE, \
	.PageSize = 256, \
	.AddrSize = 4, \
	.DevId = 0x1a609d, \
	.DevIdSize = 3, \
	.pInitCB = InitCB, \
	.pWaitCB = WaitCB, \
	.RdCmd = { FLASH_CMD_4READ, 6}, \
	.WrCmd = { FLASH_CMD_4WRITE, 0 }, }


/** @} End of group Storage */

#endif	// __FLASH_H__

