/**-------------------------------------------------------------------------
@example	nvm_demo.cpp

@brief	Test erase, program, readback and persistence on the selected NVM.

@author	Hoang Nguyen Hoan
@date	July 23, 2026

@license

MIT License

Copyright (c) 2026, I-SYST inc., all rights reserved

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

#include "coredev/uart.h"
#include "storage/nvm.h"
#include "stddev.h"
#include "idelay.h"
#include "board.h"

#ifndef NVM_DEMO_MEDIUM
#define NVM_DEMO_MEDIUM 0
#endif
#if NVM_DEMO_MEDIUM == 0
#include "storage/nvm_mcu_intrf.h"
#elif NVM_DEMO_MEDIUM == 1
#include "coredev/spi.h"
#include "storage/flash.h"
#elif NVM_DEMO_MEDIUM == 2
#include "coredev/i2c.h"
#endif

#ifdef MCU_OSC
McuOsc_t g_McuOsc = MCU_OSC;
#endif
#ifndef UART_BAUDRATE
#define UART_BAUDRATE 1000000
#endif
#ifndef NVM_DEMO_UART_DEVNO
#define NVM_DEMO_UART_DEVNO 0
#endif
#ifndef NVM_DEMO_REGION_NO
#define NVM_DEMO_REGION_NO 1
#endif
#ifndef NVM_DEMO_TOP_RESERVE_PAGES
#define NVM_DEMO_TOP_RESERVE_PAGES 3
#endif

static const IOPinCfg_t s_UartPins[] = UART_PINS;

#define UART_TXFIFO_MEMSIZE			CFIFO_MEMSIZE(2048)

alignas(4) static uint8_t s_UartTxFifoMem[UART_TXFIFO_MEMSIZE];

static const UARTCfg_t s_UartCfg = {
	.DevNo = NVM_DEMO_UART_DEVNO,
	.pIOPinMap = s_UartPins,
	.NbIOPins = sizeof(s_UartPins) / sizeof(IOPinCfg_t),
	.Rate = UART_BAUDRATE,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.bFifoBlocking = true,
	.RxMemSize = 0,
	.pRxMem = NULL,
	.TxMemSize = UART_TXFIFO_MEMSIZE,
	.pTxMem = s_UartTxFifoMem,
	.bDMAMode = true,
};

static UART s_Uart;

#if NVM_DEMO_MEDIUM == 0

static NvmMcuIntrf s_MemIntrf;

#elif NVM_DEMO_MEDIUM == 1

static const IOPinCfg_t s_SpiPins[] = SPI_PINS_CFG;

// SPI clock, the board can set it
#ifndef NVM_DEMO_SPI_RATE
#define NVM_DEMO_SPI_RATE		8000000
#endif

static const SPICfg_t s_SpiCfg = {
	.DevNo = SPI_DEVNO,
	.Phy = SPI_PHY,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_SpiPins,
	.NbIOPins = sizeof(s_SpiPins) / sizeof(IOPinCfg_t),
	.Rate = NVM_DEMO_SPI_RATE,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH,
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = NULL
};

static SPI s_MemIntrf;

#ifdef NVM_DEMO_CHIP_CFG
// The part of the board, from board.h
static const NvmCfg_t s_ChipCfg = NVM_DEMO_CHIP_CFG;
#else
// MX25R3235F : 32 Mbits NOR flash, 4K sector, 256 byte page. From the
// device database in nvm.h; another part is another entry here.
static const NvmCfg_t s_ChipCfg = {
	.DevNo = 0,
	.TotalSize = 32 * 1024 * 1024 / 8,
	.EraseSize = 4 * 1024,
	.PageSize = 256,
	.AddrSize = 3,
	.WrProtMask = 0x3C,				// BP0..BP3
	.WrProtPin = { -1, -1, },		// no pin on this part
};
#endif

#elif NVM_DEMO_MEDIUM == 2

static const IOPinCfg_t s_I2cPins[] = I2C_PINS_CFG;

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = 100000,
	.MaxRetry = 5,
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,
	.SlaveAddr = {0,},
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.EvtCB = NULL
};

static I2C s_MemIntrf;

// CAT24C32 : 32 Kbits I2C EEPROM, 2 byte address, 32 byte page, 5 ms write.
// No erase: a command value of 0 means the medium does not have it.
static const NvmCfg_t s_ChipCfg = {
	.DevNo = 0x50,					// I2C device address
	.TotalSize = 32 * 1024 / 8,
	.EraseSize = 0,					// overwrites directly
	.PageSize = 32,
	.AddrSize = 2,
	.WrProtPin = { -1, -1, },		// no pin on this part
	.WriteDelayUs = 5000,			// Twr
};

#endif

static Nvm s_Nvm;

int main()
{
	NvmCfg_t cfg = {};
	uint32_t unit;
	uint64_t reserved;
	uint32_t offset;
	uint64_t start;
	alignas(4) uint8_t pattern[128];
	alignas(4) uint8_t readback[sizeof(pattern)];
	unsigned i;
	int count;

	if (!s_Uart.Init(s_UartCfg))
	{
		return 1;
	}
#ifdef NDEBUG
	UARTRetargetEnable(s_Uart, STDOUT_FILENO);
#endif
	setvbuf(stdout, NULL, _IONBF, 0);

#if NVM_DEMO_MEDIUM == 0
	if (!NvmMcuCfgInit(cfg, NVM_DEMO_REGION_NO))
	{
		printf("Missing reserved NVM region %d\r\n", NVM_DEMO_REGION_NO);
		return 1;
	}
	if (!s_MemIntrf.Init())
#elif NVM_DEMO_MEDIUM == 1
	cfg = s_ChipCfg;
	if (!s_MemIntrf.Init(s_SpiCfg))
#else
	cfg = s_ChipCfg;
	if (!s_MemIntrf.Init(s_I2cCfg))
#endif
	{
		printf("Memory interface init failed\r\n");
		return 1;
	}

	// Limit all offsets below to two scratch units.
	unit = cfg.EraseSize ? cfg.EraseSize : 256;
	reserved = 2ULL * unit;
#if NVM_DEMO_MEDIUM != 0
	reserved += (uint64_t)unit * NVM_DEMO_TOP_RESERVE_PAGES;
#endif
	if (unit < sizeof(pattern) || cfg.TotalSize < reserved)
	{
		printf("Test region too small\r\n");
		return 1;
	}
	start = cfg.TotalSize - reserved;
	if (!s_Nvm.Init(cfg, &s_MemIntrf, start, 2ULL * unit))
	{
		printf("NVM init failed\r\n");
		return 1;
	}
	printf("NVM demo: write 128 bytes and check that every byte reads back correctly\r\nTest region 0x%08lX, %lu bytes\r\n",
		   (unsigned long)(cfg.BaseAddr + start), (unsigned long)(2 * unit));
#if NVM_DEMO_MEDIUM == 1
	printf("Read 0x%02X, dummy %u, program 0x%02X\r\n",
		   cfg.RdCmd.Cmd ? cfg.RdCmd.Cmd : FLASH_CMD_READ, cfg.RdCmd.DummyCycle,
		   cfg.WrCmd.Cmd ? cfg.WrCmd.Cmd : FLASH_CMD_WRITE);
#endif

	// Read the same addresses before changing anything, so a power cycle
	// can be used to check whether the previous write was retained.
	offset = unit - sizeof(pattern) / 2;
	for (i = 0; i < sizeof(pattern); i++)
	{
		pattern[i] = (uint8_t)i;
	}
	count = s_Nvm.Read(offset, readback, sizeof(readback));
	if (count != sizeof(readback))
	{
		printf("Initial read failed (%d)\r\n", count);
		return 1;
	}
	if (memcmp(pattern, readback, sizeof(pattern)) == 0)
	{
		printf("Before writing: found all 128 bytes of the test pattern in memory.\r\n");
	}
	else
	{
		printf("Before writing: the test pattern is absent or incomplete.\r\n");
	}

	if (cfg.EraseSize)
	{
		printf("Erasing 2 sectors at 0x%08lX...\r\n", (unsigned long)(cfg.BaseAddr + start));
		if (s_Nvm.Erase(0, 2 * unit) != 0)
		{
			printf("Erase failed\r\n");
			return 1;
		}
	}

	printf("Writing %u bytes (0x00 through 0x7F) at 0x%08lX across the boundary at 0x%08lX\r\n",
		   (unsigned)sizeof(pattern), (unsigned long)(cfg.BaseAddr + start + offset),
		   (unsigned long)(cfg.BaseAddr + start + unit));
	count = s_Nvm.Write(offset, pattern, sizeof(pattern));
	if (count != sizeof(pattern))
	{
		printf("Pattern write failed (%d)\r\n", count);
		return 1;
	}
	count = s_Nvm.Read(offset, readback, sizeof(readback));
	if (count != sizeof(readback) || memcmp(pattern, readback, sizeof(pattern)) != 0)
	{
		printf("Pattern readback failed (%d)\r\n", count);
		return 1;
	}

	printf("PASS: all %u bytes read back exactly as written.\r\n", (unsigned)sizeof(pattern));
	printf("To check retention, power the board off and on. Look for the test pattern before writing.\r\n");
	while (true)
	{
		msDelay(1000);
	}
}
