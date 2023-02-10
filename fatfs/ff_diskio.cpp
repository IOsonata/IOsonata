/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2019        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Obtains integer types */
#include "ff_diskio.h"		/* Declarations of device I/O functions */
#include "coredev/spi.h"
#include "diskio_flash.h"
#include "idelay.h"

#include "board.h"

/* Definitions of physical drive number for each drive */
#define DEV_RAM		0	/* Example: Map Ramdisk to physical drive 0 */
#define DEV_MMC		1	/* Example: Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Example: Map USB MSD to physical drive 2 */


static const IOPinCfg_t s_SpiPins[] = SPI_PINS_CFG;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = 8000000,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
    .ClkPol = SPICLKPOL_HIGH,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,	// DMA
	.bIntEn = true,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

bool FlashWriteDelayCallback(int DevNo, DevIntrf_t *pInterf);
bool MT25QL512_Init(int DevNo, DeviceIntrf* pInterface);

bool MT25R6435_Init(int DevNo, DeviceIntrf* pInterface);

static FlashCfg_t s_FlashCfg = FLASH_CFG(NULL, FlashWriteDelayCallback);

FlashDiskIO g_FlashDiskIO;

static uint8_t s_FlashCacheMem[FLASH_SECTOR_SIZE];
static DiskIOCache_t s_FlashCache[] = {
	{-1, 0xFFFFFFFF, s_FlashCacheMem},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 512},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 1024},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 1536},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 2048},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 2560},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 3072},
//	{-1, 0xFFFFFFFF, s_FlashCacheMem + 3584},
};

static const int s_NbFlashCache = sizeof(s_FlashCache) / sizeof(DiskIOCache_t);

static bool s_bInitialized = false;
static uint8_t s_TempBlock[4096];

bool FlashWriteDelayCallback(int DevNo, DevIntrf_t *pInterf)
{
	msDelay(3);
	return true;
}

bool MT25QL512_Init(int DevNo, DeviceIntrf* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

   // while (1) {
    d = FLASH_CMD_RESET_ENABLE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    d = FLASH_CMD_RESET_DEVICE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    msDelay(1);

    d = FLASH_CMD_READID;
    cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 3 );

    printf("R=%x\r\n", r);

    msDelay(1);
  //  }
    if (r != 0x20ba20)
    	return false;

    printf("Flash found!\r\n");
    // Enable write
    d = FLASH_CMD_EN4B;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    return true;
}

bool MT25R6435_Init(int DevNo, DeviceIntrf* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

   // while (1) {
    d = FLASH_CMD_RESET_ENABLE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    d = FLASH_CMD_RESET_DEVICE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    msDelay(1);

    d = FLASH_CMD_READID;
    cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 3 );

    printf("R=%x\r\n", r);

    msDelay(1);
  //  }
    if (r != 0x1728c2)
    	return false;

    printf("Flash found!\r\n");
    // Enable write
 //   d = FLASH_CMD_EN4B;
 //   cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    return true;
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
	if (pdrv > 0)
	{
		return RES_PARERR;
	}

	if (s_bInitialized == false)
	{
		g_Spi.Init(s_SpiCfg);

		if (g_FlashDiskIO.Init(s_FlashCfg, &g_Spi, s_FlashCache, s_NbFlashCache) == false)
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
	printf("read sect : %d\n", sector);

	while (count > 0)
	{
		int size = FF_MIN_SS;

		while (size > 0)
		{
			int l = g_FlashDiskIO.Read(offset, (uint8_t*)buff, size);
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

	uint8_t blkbuf[4096];
	uint8_t *p = (uint8_t *)buff;
	uint32_t blk = (sector >> 3);
	uint32_t boff = (sector % 8) * 512;
	uint64_t offs = blk << 12;//* 512;

	printf("write sect : %d %d %d %d\n", sector, count, blk, boff);

	g_FlashDiskIO.Read(offs, blkbuf, 4096);
	g_FlashDiskIO.EraseSector(blk, 1);

	while (count > 0)
	{
		memcpy(&blkbuf[boff], p, 512);
		boff += 512;
		count--;

		if (count > 0 && boff >= 4096)
		{
			g_FlashDiskIO.Write(offs, blkbuf, 4096);
			boff = 0;
			blk++;
			offs = blk << 12;//* 512;
			g_FlashDiskIO.Read(offs, blkbuf, 4096);
			g_FlashDiskIO.EraseSector(blk, 1);
		}
		p += 512;
	}
	g_FlashDiskIO.Write(offs, blkbuf, 4096);


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

			g_FlashDiskIO.Flush();

			return RES_OK;

		case GET_SECTOR_COUNT :

			// Process of the command for the MMC/SD card
			*(uint32_t *)buff = g_FlashDiskIO.GetNbSect();
			return RES_OK;

		case GET_SECTOR_SIZE :
			*(uint32_t *)buff = 512;//g_FlashDiskIO.GetSectSize();
			return RES_OK;
		case GET_BLOCK_SIZE:
			*(uint32_t *)buff = 32768;
			return RES_OK;
		case CTRL_TRIM:
			{
				uint32_t *p = (uint32_t*)buff;
				uint32_t st = p[0] * 512 / 4096;
				uint32_t end = p[1] * 512 / 4096;

				g_FlashDiskIO.EraseSector(st, end - st);
			}

			return RES_OK;
	}

	return RES_PARERR;
}

