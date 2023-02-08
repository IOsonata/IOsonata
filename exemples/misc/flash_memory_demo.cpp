/**-------------------------------------------------------------------------
@example	flash_memory_demo.cpp

@brief	Example code using SPI Flash memory

@author	Hoang Nguyen Hoan
@date	Mars 8, 2019

@license

Copyright (c) 2019, I-SYST, all rights reserved

Permission to use, copy, modify, and distribute this software for any purpose
with or without fee is hereby granted, provided that the above copyright
notice and this permission notice appear in all copies, and none of the
names : I-SYST or its contributors may be used to endorse or
promote products derived from this software without specific prior written
permission.

For info or contributing contact : hnhoan at i-syst dot com

THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------*/
#include <stdio.h>

#include "coredev/uart.h"
#include "coredev/spi.h"
#include "flash.h"
#include "diskio_flash.h"
#include "stddev.h"
#include "idelay.h"

#include "board.h"

#define FIFOSIZE			CFIFO_MEMSIZE(256)

uint8_t g_UarTxBuff[FIFOSIZE];

// Assign UART pins
static IOPinCfg_t s_UartPins[] = {
	{UART_RX_PORT, UART_RX_PIN, UART_RX_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// RX
	{UART_TX_PORT, UART_TX_PIN, UART_TX_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// TX
	{UART_CTS_PORT, UART_CTS_PIN, UART_CTS_PINOP, IOPINDIR_INPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},	// CTS
	{UART_RTS_PORT, UART_RTS_PIN, UART_RTS_PINOP, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL},// RTS
};

// UART configuration data
static const UARTCfg_t s_UartCfg = {
	.DevNo = 0,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = 115200,			// Rate
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,					// Stop bit
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 1, 					// use APP_IRQ_PRIORITY_LOW with Softdevice
	.EvtCallback = NULL,//nRFUartEvthandler,
	.bFifoBlocking = true,				// fifo blocking mode
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = FIFOSIZE,
	.pTxMem = g_UarTxBuff,
};

UART g_Uart;

static const IOPinCfg_t s_SpiPins[] = SPI_PINS_CFG;

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
    .Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
    .Rate = 4000000,   // Speed in Hz
    .DataSize = 8,      // Data Size
    .MaxRetry = 5,      // Max retries
    .BitOrder = SPIDATABIT_MSB,
    .DataPhase = SPIDATAPHASE_FIRST_CLK, // Data phase
    .ClkPol = SPICLKPOL_HIGH,         // clock polarity
    .ChipSel = SPICSEL_AUTO,
	.bDmaEn = true,	// DMA
	.bIntEn = false,
    .IntPrio = 6, //APP_IRQ_PRIORITY_LOW,      // Interrupt priority
    .EvtCB = NULL
};

SPI g_Spi;

//#endif

bool MT25QL512_Init(int DevNo, DeviceIntrf* pInterface);
bool MX25U1635E_init(int pDevNo, DevIntrf_t* ppInterface);
bool IS25LP512M_Init(int DevNo, DeviceIntrf* pInterface);
bool MX25U6435F_init(int DevNo, DeviceIntrf* pInterface);
bool FlashWriteDelayCallback(int DevNo, DeviceIntrf *pInterf);

//static const FlashDiskIOCfg_t s_FlashCfg = FLASH_CFG(NULL, NULL);//FLASH_CFG(MX25U1635E_init, NULL);
static const FlashCfg_t s_FlashCfg = FLASH_CFG(MX25U1635E_init, NULL);//FLASH_CFG(MX25U1635E_init, NULL);

FlashDiskIO g_Flash;

static uint8_t s_FlashCacheMem[DISKIO_SECT_SIZE];
DiskIOCache_t g_FlashCache = {
    -1, 0xFFFFFFFF, s_FlashCacheMem
};

bool FlashWriteDelayCallback(int DevNo, DeviceIntrf *pInterf)
{
	msDelay(3);
	return true;
}

bool IS25LP512M_Init(int DevNo, DeviceIntrf* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

    d = FLASH_CMD_RESET_ENABLE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    d = FLASH_CMD_RESET_DEVICE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    d = FLASH_CMD_READID;
    cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 3 );

    if (r != 0x1a609d && r != 0x1a709d)
    	return false;

    printf("Flash found!\r\n");

    // Enable write
    d = FLASH_CMD_EN4B;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    return true;
}

bool MT25QL512_Init(int DevNo, DeviceIntrf* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

    d = FLASH_CMD_READID;
    cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 3 );

    if (r != 0x20ba20)
    	return false;

    printf("Flash found!\r\n");
    // Enable write
    d = FLASH_CMD_EN4B;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    return true;
}

bool MX25U1635E_init(int DevNo, DevIntrf_t* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

    d = FLASH_CMD_READID;
    //cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 2 );
    cnt = DeviceIntrfRead(pInterface, DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 2 );
    if ( r != 0x25C2 )
    {
    	printf("Wrong FLASH_CMD_READID response\r\n");
    	return false;
    }

    printf("Flash found!\r\n");
    // Enable write
    d = FLASH_CMD_EN4B;
    //cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);
    cnt = DeviceIntrfTx(pInterface, DevNo, (uint8_t*)&d, 1);
    return true;
}

bool MX25U6435F_init(int DevNo, DeviceIntrf* pInterface)
{
    if (pInterface == NULL)
        return false;

    int cnt = 0;

    uint32_t d;
    uint32_t r = 0;

    d = FLASH_CMD_RESET_ENABLE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    d = FLASH_CMD_RESET_DEVICE;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    d = FLASH_CMD_READID;
    cnt = pInterface->Read(DevNo, (uint8_t*)&d, 1, (uint8_t*)&r, 3 );

    if (r != 0x1728C2)
    	return false;

    printf("Flash found!\r\n");

    // Enable write
    d = FLASH_CMD_EN4B;
    cnt = pInterface->Tx(DevNo, (uint8_t*)&d, 1);

    return true;
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//
int main()
{
	g_Uart.Init(s_UartCfg);

	// Retarget printf to UART
	UARTRetargetEnable(g_Uart, STDOUT_FILENO);
	UARTRetargetEnable(g_Uart, STDIN_FILENO);

	printf("Flash Memory Demo\r\n");
	//getchar();

	g_Spi.Init(s_SpiCfg);

	// IOPinConfig(FLASH_HOLD_PORT, FLASH_HOLD_PIN, FLASH_HOLD_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL);

	// Regular SPI FLash
	//g_FlashDiskIO.Init(s_N25Q128A_QFlashCfg, &g_Spi, &g_FlashCache, 1);

	// QSPI flash
	//g_FlashDiskIO.Init(s_N25Q128A_QFlashCfg, &g_Spi, &g_FlashCache, 1);

	//if (g_FlashDiskIO.Init(s_MX25R6435F_QFlashCfg, &g_Spi, &g_FlashCache, 1) == false)
//	if (g_Flash.Init(s_MX25L25645G_FlashCfg, &g_Spi)==false)//, &g_FlashCache, 1) == false)
	if (g_Flash.Init(s_FlashCfg, &g_Spi) == false)//, &g_FlashCache, 1) == false)
	{
		printf("Init Flash failed\r\n");
	}

	//g_QFlash.Init(s_QFlashCfg, &g_FlashCache, 1);

	uint8_t buff[s_FlashCfg.SectSize];
	uint8_t buff2[s_FlashCfg.SectSize];
	uint8_t tmp[s_FlashCfg.SectSize];
	uint16_t *p = (uint16_t*) buff;

	memset(tmp, 0xa5, 512);
	for (int i = 0; i < 256; i++)
	{
		p[i] = 255 - i;
	}

	// Erase the whole flash memory
	printf("Erasing... Please wait for a few minutes\r\n");
	g_Flash.Erase();
	//g_Flash.EraseBlock(0, 4);

	// Test Sector 0
	printf("Erase Sector 0...");
	g_Flash.EraseSector(0, 1);
	msDelay(1000);
	g_Flash.SectRead(0, tmp);

	memset(buff, 0xff, 512);
	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Failed\r\n");
	}
	else
	{
		printf("Success\r\n");
	}

	printf("Write new data to sector 0...");
	g_Flash.SectWrite(0, buff2);
	g_Flash.SectRead(0, tmp);

	if (memcmp(buff2, tmp, 512) != 0)
	{
		printf("Failed\r\n");
	}
	else
	{
		printf("Success\r\n");
	}

	// Test Sector 1
	printf("Writing %d data to Sector 1...", s_FlashCfg.SectSize);
	g_Flash.SectWrite(1, buff);
	g_Flash.SectRead(1, tmp);
//	for (int i = 0; i < 512; i++)
//	{
//		if (buff[i] != tmp[i])
//		{
//			printf("Failed %d\r\n", i);
//			break;
//		}
//	}

	if (memcmp(buff, tmp, 512) != 0)
	{
		printf("Failed\r\n");
	}
	else
	{
		printf("Success\r\n");
	}

	// Test Sector 2
	printf("Write to sector 2...");
	p = (uint16_t*) buff2;
	for (int i = 0; i < 256; i++)
	{
		p[i] = i;
	}
	g_Flash.SectWrite(2UL, buff2);

	printf("Validate readback Sector 2...");
	memset(tmp, 0, 512);
	g_Flash.SectRead(2, tmp);
//	for (int i = 0; i < 512; i++)
//	{
//		if (buff2[i] != tmp[i])
//		{
//			printf("Failed %d\r\n", i);
//			break;
//		}
//	}

	if (memcmp(buff2, tmp, 512) != 0)
	{
		printf("Failed\r\n");
	}
	else
	{
		printf("Success\r\n");
	}

	// Test Sector 30
	printf("Write and read back sector 30...");
	memset(tmp, 0, 512);
	g_Flash.SectWrite(30, buff2);
	g_Flash.SectRead(30, tmp);
	if (memcmp(buff2, tmp, 512) != 0)
	{
		printf("Failed\r\n");
	}
	else
	{
		printf("Success\r\n");
	}

	// Test Sector 40
	printf("Write and read back sector 40...");
	memset(tmp, 0, 512);
	g_Flash.SectWrite(40, buff2);
	g_Flash.SectRead(40, tmp);
	if (memcmp(buff2, tmp, 512) != 0)
	{
		printf("Failed\r\n");
	}
	else
	{
		printf("Success\r\n");
	}

	printf("FLash Test Completed\r\n");

	while (1)
	{
		__WFE();
	}

}
