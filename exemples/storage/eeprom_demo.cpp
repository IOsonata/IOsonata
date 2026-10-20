/**-------------------------------------------------------------------------
@example	eeprom_demo.cpp

@brief		Serial EEPROM example

Runs the same real EEPROM write/read test with either the legacy Seep driver or
the unified Nvm driver, selected by NVM_MODE below. The default is an I2C EEPROM. EEPROM_SPI selects
a SPI EEPROM through Nvm; the legacy Seep mode remains I2C-only. Both use the same test flow.

@author		Hoang Nguyen Hoan
@date		Sep. 19, 2020

@license

MIT License

Copyright (c) 2020 I-SYST inc. All rights reserved.

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
#include <stdbool.h>
#include <string.h>

#include "coredev/i2c.h"
#include "coredev/iopincfg.h"
#include "idelay.h"
#include "iopinctrl.h"
#include "board.h"

#ifndef EEPROM_SPI
#define EEPROM_SPI 0
#endif
#if EEPROM_SPI
#include "coredev/spi.h"
#endif
#ifndef EEPROM_DEMO_UART
#define EEPROM_DEMO_UART 0
#endif
#if EEPROM_DEMO_UART
#include "coredev/uart.h"
#include "stddev.h"
#endif

// Select the EEPROM implementation underneath the common test flow.
// 0 : legacy Seep driver
// 1 : unified Nvm driver
#ifndef NVM_MODE
#define NVM_MODE			1
#endif

#if NVM_MODE == 0
#include "storage/seep.h"
#elif NVM_MODE == 1
#include "storage/nvm.h"
#else
#error NVM_MODE must be 0 or 1
#endif

// Define for the legacy C API. Unified Nvm is a C++ class.
//#define C_CODE

#if NVM_MODE != 0 && defined(C_CODE)
#error C_CODE is supported only with the legacy Seep mode
#endif

#if EEPROM_SPI && NVM_MODE == 0
#error SPI EEPROM requires NVM_MODE=1; legacy Seep uses address-only I2C frames
#endif

#ifndef EEPROM_DEV_ADDR
#define EEPROM_DEV_ADDR 0x50
#endif
#ifndef EEPROM_ADDR_SIZE
#define EEPROM_ADDR_SIZE 2
#endif
#ifndef EEPROM_PAGE_SIZE
#define EEPROM_PAGE_SIZE 32
#endif
#ifndef EEPROM_SIZE
#define EEPROM_SIZE (128 * 32)
#endif
#ifndef EEPROM_WRITE_DELAY_MS
#define EEPROM_WRITE_DELAY_MS 100
#endif
#ifndef EEPROM_DEVICE_ID
#define EEPROM_DEVICE_ID 0
#endif
#ifndef EEPROM_DEVICE_ID_SIZE
#define EEPROM_DEVICE_ID_SIZE 0
#endif
#ifndef EEPROM_TEST_OFFSET
#define EEPROM_TEST_OFFSET 256
#endif
#ifndef EEPROM_TEST_SIZE
#define EEPROM_TEST_SIZE 512
#endif

#if EEPROM_SPI
static const IOPinCfg_t s_Pins[] = EEPROM_SPI_PINS;
static const SPICfg_t s_SpiCfg = {
	.DevNo = EEPROM_SPI_DEVNO,
	.Phy = SPIPHY_NORMAL,
	.Mode = SPIMODE_MASTER,
	.pIOPinMap = s_Pins,
	.NbIOPins = sizeof(s_Pins) / sizeof(s_Pins[0]),
	.Rate = 4000000,
	.DataSize = 8,
	.MaxRetry = 5,
	.BitOrder = SPIDATABIT_MSB,
	.DataPhase = SPIDATAPHASE_FIRST_CLK,
	.ClkPol = SPICLKPOL_HIGH, // Active high, CPOL=0, idle low
	.ChipSel = SPICSEL_AUTO,
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 6,
	.DummyByte = 0xFF,
	.EvtCB = nullptr
};
static SPI s_Spi;
#else
static const IOPinCfg_t s_Pins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// SDA
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// SCL
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_Pins,
	.NbIOPins = sizeof(s_Pins) / sizeof(IOPinCfg_t),
	.Rate = 100000,				// Rate
	.MaxRetry = 5,				// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},			// Slave addresses
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 7,				// Interrupt prio
	.EvtCB = NULL				// Event callback
};

#endif // EEPROM_SPI

#if NVM_MODE == 0

static const SeepCfg_t s_SeepCfg = {
	.DevAddr = EEPROM_DEV_ADDR,
	.AddrLen = EEPROM_ADDR_SIZE,
	.PageSize = EEPROM_PAGE_SIZE,
	.Size = EEPROM_SIZE,
	.WrDelay = EEPROM_WRITE_DELAY_MS,
	.WrProtPin = {-1, -1,},
};

#ifdef C_CODE
static I2CDev_t s_I2CDev;
static SeepDev_t s_SeepDev;
#else
static I2C s_I2c;
static Seep s_Seep;
#endif

#else

static const NvmCfg_t s_NvmCfg = {
	.DevNo = EEPROM_DEV_ADDR,
	.TotalSize = EEPROM_SIZE,
	.EraseSize = 0,				// EEPROM overwrites directly
	.PageSize = EEPROM_PAGE_SIZE,
	.AddrSize = EEPROM_ADDR_SIZE,
	.DevId = EEPROM_DEVICE_ID,
	.DevIdSize = EEPROM_DEVICE_ID_SIZE,
	.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	.WriteDelayUs = EEPROM_WRITE_DELAY_MS * 1000UL,
};

#if !EEPROM_SPI
static I2C s_I2c;
#endif
static Nvm g_Nvm;

#endif

static bool EepromInit(void)
{
#if EEPROM_SPI
	if (!s_Spi.Init(s_SpiCfg)) { return false; }
	// Deselect every device after GPIO clocks have been enabled. This also
	// holds the OLED high while EEPROM commands use the shared SPI3 bus.
	for (unsigned i = SPI_CS_IOPIN_IDX; i < sizeof(s_Pins) / sizeof(s_Pins[0]); i++)
	{
		IOPinSet(s_Pins[i].PortNo, s_Pins[i].PinNo);
	}
	return g_Nvm.Init(s_NvmCfg, &s_Spi);
#elif NVM_MODE == 0 && defined(C_CODE)
	if (I2CInit(&s_I2CDev, &s_I2cCfg) == false)
	{
		return false;
	}
	return SeepInit(&s_SeepDev, &s_SeepCfg, &s_I2CDev.DevIntrf);
#else
	if (s_I2c.Init(s_I2cCfg) == false)
	{
		return false;
	}

#if NVM_MODE == 0
	return s_Seep.Init(s_SeepCfg, &s_I2c);
#else
	return g_Nvm.Init(s_NvmCfg, &s_I2c);
#endif
#endif
}

static int EepromWrite(uint32_t Addr, uint8_t *pData, uint32_t Len)
{
#if NVM_MODE == 0 && defined(C_CODE)
	return SeepWrite(&s_SeepDev, Addr, pData, (int)Len);
#elif NVM_MODE == 0
	return s_Seep.Write(Addr, pData, (int)Len);
#else
	return g_Nvm.Write(Addr, pData, Len);
#endif
}

static int EepromRead(uint32_t Addr, uint8_t *pData, uint32_t Len)
{
#if NVM_MODE == 0 && defined(C_CODE)
	return SeepRead(&s_SeepDev, Addr, pData, (int)Len);
#elif NVM_MODE == 0
	return s_Seep.Read(Addr, pData, (int)Len);
#else
	return g_Nvm.Read(Addr, pData, Len);
#endif
}

static bool EepromTest(void)
{
	uint8_t x[EEPROM_TEST_SIZE];
	uint8_t y[EEPROM_TEST_SIZE];

	for (uint32_t i = 0; i < EEPROM_TEST_SIZE; i++)
	{
		x[i] = (uint8_t)(i ^ (i >> 8));
	}

	printf("Writing %lu bytes at 0x%lx...\r\n",
		   (unsigned long)EEPROM_TEST_SIZE,
		   (unsigned long)EEPROM_TEST_OFFSET);

	int cnt = EepromWrite(EEPROM_TEST_OFFSET, x, EEPROM_TEST_SIZE);
	if (cnt != (int)EEPROM_TEST_SIZE)
	{
		printf("EEPROM write failed: %d of %lu bytes\r\n", cnt,
			   (unsigned long)EEPROM_TEST_SIZE);
		return false;
	}

	memset(y, 0xA5, sizeof(y));

	printf("Validate readback...\r\n");
	cnt = EepromRead(EEPROM_TEST_OFFSET, y, EEPROM_TEST_SIZE);
	if (cnt != (int)EEPROM_TEST_SIZE)
	{
		printf("EEPROM read failed: %d of %lu bytes\r\n", cnt,
			   (unsigned long)EEPROM_TEST_SIZE);
		return false;
	}

	if (memcmp(x, y, sizeof(x)) != 0)
	{
		for (uint32_t i = 0; i < EEPROM_TEST_SIZE; i++)
		{
			if (x[i] != y[i])
			{
				printf("EEPROM verify failed at 0x%lx: wrote 0x%02x read 0x%02x\r\n",
					   (unsigned long)(EEPROM_TEST_OFFSET + i), x[i], y[i]);
				break;
			}
		}
		return false;
	}

	printf("EEPROM test passed.\r\n");
	return true;
}

#if EEPROM_DEMO_UART
static UART s_Console;
static const IOPinCfg_t s_ConsolePins[] = EEPROM_UART_PINS;
static const UARTCfg_t s_ConsoleCfg = {
	.DevNo = EEPROM_UART_DEVNO,
	.pIOPinMap = s_ConsolePins,
	.NbIOPins = sizeof(s_ConsolePins) / sizeof(s_ConsolePins[0]),
	.Rate = 1000000,
	.DataBits = 8,
	.Parity = UART_PARITY_NONE,
	.StopBits = 1,
	.FlowControl = UART_FLWCTRL_NONE,
	.bIntMode = true,
	.IntPrio = 6,
	.bFifoBlocking = true,
	.bDMAMode = false
};
#endif

int main()
{
#if EEPROM_DEMO_UART
	if (!s_Console.Init(s_ConsoleCfg)) { return 1; }
	UARTRetargetEnable(s_Console, STDOUT_FILENO);
	UARTRetargetEnable(s_Console, STDERR_FILENO);
	setvbuf(stdout, nullptr, _IONBF, 0);
#endif
#if NVM_MODE == 0
	printf("EEPROM Demo, legacy Seep driver\r\n");
#else
	printf("EEPROM Demo, unified Nvm driver\r\n");
#endif

	if (EepromInit() == false)
	{
		printf("EEPROM init failed\r\n");
	}
	else
	{
		EepromTest();
	}

	while (1);

	return 0;
}
