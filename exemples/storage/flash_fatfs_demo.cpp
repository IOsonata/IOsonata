/**-------------------------------------------------------------------------
@example	flash_fatfs_demo.cpp

@brief	FatFs demo on Flash memory

@author	Hoang Nguyen Hoan
@date	Aug. 8, 2021

@license

MIT License

Copyright (c) 2021, I-SYST inc., all rights reserved

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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "ff.h"			/* Obtains integer types */
#include "ff_diskio.h"		/* Declarations of device I/O functions */

#include "stddev.h"
#include "idelay.h"
#include "coredev/uart.h"
#include "coredev/spi.h"
#include "storage/diskio_flash.h"

// This include contain i/o definition the board in use
#include "board.h"

#define TIMER_TRIG_ID0			0
#define TIMER_TRIG_ID0_PERIOD	1000UL		// 1 second

#define TEST_BUFSIZE		256

int nRFUartEvtHandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen);

#define FIFOSIZE			CFIFO_MEMSIZE(TEST_BUFSIZE * 4)

uint8_t g_TxBuff[FIFOSIZE];
uint8_t g_RxBuff[FIFOSIZE];

static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	//{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	//{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1,
	.EvtCallback = nRFUartEvtHandler,
	.bFifoBlocking = true,
	.RxMemSize = FIFOSIZE,
	.pRxMem = g_RxBuff,
	.TxMemSize = FIFOSIZE,
	.pTxMem = g_TxBuff,
	.bDMAMode = true,
};

UART g_Uart;

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

const MKFS_PARM s_MkFsParm = {
	.fmt = FM_ANY,
	.n_fat = 0,
	.align = 4096,
	.n_root = 0,
	.au_size = 0
};

bool FlashWriteDelayCallback(int DevNo, DevIntrf_t *pInterf)
{
	msDelay(3);
	return true;
}

int nRFUartEvtHandler(UARTDev_t *pDev, UART_EVT EvtId, uint8_t *pBuffer, int BufferLen)
{
	int cnt = 0;
	uint8_t buff[TEST_BUFSIZE];
	uint8_t *p;

	switch (EvtId)
	{
		case UART_EVT_RXTIMEOUT:
		case UART_EVT_RXDATA:
			UARTRx(pDev, buff, BufferLen);
			break;
		case UART_EVT_TXREADY:
			break;
		case UART_EVT_LINESTATE:
			break;
	}

	return cnt;
}

uint8_t g_Data[4096];
uint8_t g_Temp[4096];
uint8_t g_FFBuf[4096];
static FATFS FatFs;

// nrfutil pkg generate --hw-version 52 --sd-req 0x0100 --application-version 0x0 --application app.hex --key-file ../../dfu_ble/src/capsule_key.pem app_package.zip

int main()
{
	bool res;

	res = g_Uart.Init(s_UartCfg);

	printf("flash_fatfs_demo\n");

	g_Spi.Init(s_SpiCfg);

	if (g_FlashDiskIO.Init(s_FlashCfg, &g_Spi, s_FlashCache, s_NbFlashCache) == true)
	{
		disk_ioctl(0, SET_DEVICE, &g_FlashDiskIO);
	}

	uint16_t *p = (uint16_t*)g_Data;

	for (int i = 0; i < sizeof(g_Data) / sizeof(uint16_t); i++)
	{
		p[i] = i;
	}

	FRESULT fres = f_mount(&FatFs, "0:", 1);

	if (fres != FR_OK)
	{
		printf("formating wait...\n");
		fres = f_mkfs ("0:", &s_MkFsParm, g_FFBuf, 4096);
		printf("Fres = %d\n", fres);
	}

	FIL fp;

	fres = f_open(&fp, "test.txt", FA_READ | FA_WRITE | FA_CREATE_ALWAYS);

	uint32_t l = 0;

	if (fres == FR_OK)
	{
		printf("file open/create success\n");

		fres = f_write(&fp, g_Data, 4096, (unsigned*)&l);
		fres = f_lseek(&fp, 0);
		fres = f_read(&fp, g_Temp, 4096, (unsigned*)&l);
		f_close(&fp);

		//if (memcmp(g_Temp, g_Data, 4096) != 0)
		for (int i = 0; i < 4096; i++)
		{
			if (g_Data[i] != g_Temp[i])
			{
				printf("read failed %d\n", i);
			}
		}
	}

	memset(g_Temp, 0xa5, 4096);

	fres = f_open(&fp, "test.txt", FA_READ);// | FA_CREATE_ALWAYS);

	if (fres == FR_OK)
	{
		printf("file open success\n");
		fres = f_read(&fp, g_Temp, 4096, (unsigned*)&l);
		f_close(&fp);

		if (memcmp(g_Temp, g_Data, 4096) != 0)
		{
			printf("read failed\n");
		}
	}

	printf("test complete\n");

	while(1)
	{
		__WFE();
	}

	return 0;
}

#if 0
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

	uint32_t offs = sector * FF_MIN_SS;

	printf("write sect : %d %d\n", sector, offs);

	g_FlashDiskIO.Write(offs, (uint8_t*)buff, count * FF_MIN_SS);

#if 0
	uint32_t sz = g_FlashDiskIO.GetSectSize();
	uint8_t blkbuf[sz];
	uint8_t *p = (uint8_t *)buff;
	uint32_t blk = (sector >> 3);
	uint32_t boff = (sector % 8) * 512;
	uint64_t offs = blk << 12;//* 512;

	printf("write sect : %d %d %d %d\n", sector, count, blk, boff);

	g_FlashDiskIO.Read(offs, blkbuf, 4096);
	//g_FlashDiskIO.EraseSector(blk, 1);

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
			//g_FlashDiskIO.EraseSector(blk, 1);
		}
		p += 512;
	}
	g_FlashDiskIO.Write(offs, blkbuf, 4096);

#endif

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
			*(uint32_t *)buff = g_FlashDiskIO.GetSize() * 1024U / 512U;
			printf("Nb Sector %u\r\n", *(uint32_t *)buff);
			return RES_OK;

		case GET_SECTOR_SIZE :
			*(uint32_t *)buff = 512;//g_FlashDiskIO.GetSectSize();
			return RES_OK;
		case GET_BLOCK_SIZE:
			*(uint32_t *)buff = g_FlashDiskIO.GetSectSize();
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
#endif


