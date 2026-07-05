/**-------------------------------------------------------------------------
@example	flash_littlefs_demo.cpp

@brief	LittleFs demo on Flash memory

@author	Hoang Nguyen Hoan
@date	Feb. 17, 2023

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
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "lfs.h"

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
int LittleFsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size);
int LittleFsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size);
int LittleFsErase(const struct lfs_config *c, lfs_block_t block);
int LittleFsSync(const struct lfs_config *c);


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

// variables used by the filesystem
lfs_t lfs;
lfs_file_t file;

// configuration of the filesystem is provided by this struct
const struct lfs_config cfg = {
    // block device operations
    .read  = LittleFsRead,
    .prog  = LittleFsProg,
    .erase = LittleFsErase,
    .sync  = LittleFsSync,

    // block device configuration
    .read_size = 16,
    .prog_size = 16,
    .block_size = s_FlashCfg.SectSize,
    .block_count = s_FlashCfg.TotalSize * 1024UL / s_FlashCfg.SectSize,
    .block_cycles = 500,
    .cache_size = 16,
    .lookahead_size = 16,
};

uint8_t g_Data[4096];
uint8_t g_Temp[4096];
uint8_t g_FFBuf[4096];

int LittleFsRead(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, void *buffer, lfs_size_t size)
{
	g_FlashDiskIO.Read(block, off, (uint8_t*)buffer, size);

	return LFS_ERR_OK;
}

int LittleFsProg(const struct lfs_config *c, lfs_block_t block, lfs_off_t off, const void *buffer, lfs_size_t size)
{
	g_FlashDiskIO.Write(block, off, (uint8_t*)buffer, size);
	return LFS_ERR_OK;
}

int LittleFsErase(const struct lfs_config *c, lfs_block_t block)
{
	g_FlashDiskIO.EraseSector(block, 1);
	return LFS_ERR_OK;
}

int LittleFsSync(const struct lfs_config *c)
{
	g_FlashDiskIO.Flush();

	return LFS_ERR_OK;
}

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

// nrfutil pkg generate --hw-version 52 --sd-req 0x0100 --application-version 0x0 --application app.hex --key-file ../../dfu_ble/src/capsule_key.pem app_package.zip

int main()
{
	bool res;

	res = g_Uart.Init(s_UartCfg);

	printf("flash_littlefs_demo\n");

	g_Spi.Init(s_SpiCfg);

	if (g_FlashDiskIO.Init(s_FlashCfg, &g_Spi, s_FlashCache, s_NbFlashCache) == true)
	{
	}

	uint16_t *p = (uint16_t*)g_Data;

	for (int i = 0; i < sizeof(g_Data) / sizeof(uint16_t); i++)
	{
		p[i] = i;
	}

    // mount the filesystem
    int err = lfs_mount(&lfs, &cfg);

    // reformat if we can't mount the filesystem
    // this should only happen on the first boot
    if (err)
    {
		printf("formating wait...\n");

		lfs_format(&lfs, &cfg);
        lfs_mount(&lfs, &cfg);
    }

    err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDWR | LFS_O_CREAT);
    if (err == LFS_ERR_OK)
    {
		printf("file open/create success\n");
	    lfs_file_write(&lfs, &file, g_Data, 4096);
	    lfs_file_rewind(&lfs, &file);
	    lfs_file_read(&lfs, &file, g_Temp, 4096);
	    lfs_file_close(&lfs, &file);

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

    err = lfs_file_open(&lfs, &file, "test.txt", LFS_O_RDWR | LFS_O_CREAT);
    if (err == LFS_ERR_OK)
    {
		printf("file open success\n");

	    lfs_file_read(&lfs, &file, g_Temp, 4096);
	    lfs_file_close(&lfs, &file);

	    if (memcmp(g_Temp, g_Data, 4096) != 0)
		{
			printf("read failed\n");
		}
    }

    // release any resources we were using
    lfs_unmount(&lfs);

	printf("test complete\n");

	while(1)
	{
		__WFE();
	}

	return 0;
}


