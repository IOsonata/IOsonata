/**-------------------------------------------------------------------------
@example	eeprom_demo.cpp

@brief		I2C EEPROM example

Runs the same real EEPROM write/read test with either the legacy Seep driver or
the unified Nvm driver, selected by NVM_MODE below. Both modes use the same I2C
configuration, EEPROM geometry, test address, data pattern and validation flow.

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

#include "board.h"

// Define for the legacy C API. Unified Nvm is a C++ class.
//#define C_CODE

#if NVM_MODE != 0 && defined(C_CODE)
#error C_CODE is supported only with the legacy Seep mode
#endif

static const uint8_t EEPROM_DEV_ADDR = 0x50;
static const uint8_t EEPROM_ADDR_SIZE = 2;
static const uint32_t EEPROM_PAGE_SIZE = 32;
static const uint32_t EEPROM_SIZE = 128 * 32;
static const uint32_t EEPROM_WRITE_DELAY_MS = 100;
static const uint32_t EEPROM_TEST_OFFSET = 256;
static const uint32_t EEPROM_TEST_SIZE = 512;

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
static I2CDev_t g_I2CDev;
static SeepDev_t g_SeepDev;
#else
static I2C g_I2c;
static Seep g_Seep;
#endif

#else

static const NvmCfg_t s_NvmCfg = {
	.DevNo = EEPROM_DEV_ADDR,
	.TotalSize = EEPROM_SIZE,
	.EraseSize = 0,				// EEPROM overwrites directly
	.PageSize = EEPROM_PAGE_SIZE,
	.AddrSize = EEPROM_ADDR_SIZE,
	.WrProtPin = { -1, -1, 0, IOPINDIR_OUTPUT, IOPINRES_NONE, IOPINTYPE_NORMAL },
	.WriteDelayUs = EEPROM_WRITE_DELAY_MS * 1000UL,
};

static I2C g_I2c;
static Nvm g_Nvm;

#endif

static bool EepromInit(void)
{
#if NVM_MODE == 0 && defined(C_CODE)
	if (I2CInit(&g_I2CDev, &s_I2cCfg) == false)
	{
		return false;
	}
	return SeepInit(&g_SeepDev, &s_SeepCfg, &g_I2CDev.DevIntrf);
#else
	if (g_I2c.Init(s_I2cCfg) == false)
	{
		return false;
	}

#if NVM_MODE == 0
	return g_Seep.Init(s_SeepCfg, &g_I2c);
#else
	return g_Nvm.Init(s_NvmCfg, &g_I2c);
#endif
#endif
}

static int EepromWrite(uint32_t Addr, uint8_t *pData, uint32_t Len)
{
#if NVM_MODE == 0 && defined(C_CODE)
	return SeepWrite(&g_SeepDev, Addr, pData, (int)Len);
#elif NVM_MODE == 0
	return g_Seep.Write(Addr, pData, (int)Len);
#else
	return g_Nvm.Write(Addr, pData, Len);
#endif
}

static int EepromRead(uint32_t Addr, uint8_t *pData, uint32_t Len)
{
#if NVM_MODE == 0 && defined(C_CODE)
	return SeepRead(&g_SeepDev, Addr, pData, (int)Len);
#elif NVM_MODE == 0
	return g_Seep.Read(Addr, pData, (int)Len);
#else
	return g_Nvm.Read(Addr, pData, Len);
#endif
}

static bool EepromTest(void)
{
	uint8_t x[EEPROM_TEST_SIZE];
	uint8_t y[EEPROM_TEST_SIZE];

	for (uint32_t i = 0; i < EEPROM_TEST_SIZE / 2; i++)
	{
		x[i] = (uint8_t)i;
		x[EEPROM_TEST_SIZE / 2 + i] = (uint8_t)(256 - i);
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

int main()
{
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
