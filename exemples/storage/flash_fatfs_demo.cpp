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

#include "ff.h"			// Obtains integer types
#include "ff_diskio.h"		// Declarations of device I/O functions

#include "stddev.h"
#include "idelay.h"
#include "coredev/uart.h"
#include "coredev/spi.h"
#include "storage/diskio_flash.h"

// This include contain i/o definition the board in use
#include "board.h"

// Console rate, board.h can give another one
#ifndef UART_BAUDRATE
#define UART_BAUDRATE		1000000
#endif

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
	.Rate = UART_BAUDRATE,
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

alignas(4) static uint8_t s_DiskSectorBuffer[FLASH_SECTOR_SIZE];

uint8_t g_Data[4096];
uint8_t g_Temp[4096];
uint8_t g_FFBuf[4096];
static FATFS s_FatFs;

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

	if (!g_Uart.Init(s_UartCfg))
		return 1;
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);

	printf("flash_fatfs_demo\n");

	if (!g_Spi.Init(s_SpiCfg))
		return 1;

	if (!g_FlashDiskIO.Init(s_FlashCfg, &g_Spi))
	{
		printf("Init Flash failed\n");
		return 1;
	}

	uint16_t *p = (uint16_t*)g_Data;

	for (int i = 0; i < sizeof(g_Data) / sizeof(uint16_t); i++)
	{
		p[i] = i;
	}

	const FatFsMountCfg_t mount = {
		.pDisk = &g_FlashDiskIO,
		.pFatFs = &s_FatFs,
		.pPath = "0:",
		.pSectorBuffer = s_DiskSectorBuffer,
		.SectorBufferSize = sizeof(s_DiskSectorBuffer),
		.bFormatIfBlank = true,
		.pFormatWork = g_FFBuf,
		.FormatWorkSize = sizeof(g_FFBuf),
	};
	FRESULT fres = FatFsMount(mount);

	if (fres != FR_OK)
	{
		printf("FatFs mount failed: %d\n", fres);
		return 1;
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

