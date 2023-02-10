/**-------------------------------------------------------------------------
@example	i2c_eeprom.cpp

@brief		I2C EEPROM example

This example shows how to use I2C driver to read/write EEPROM
Demo showing how to use Eprom with i2c interface in both C and C++

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

#include "coredev/i2c.h"
#include "coredev/iopincfg.h"
#include "iopinctrl.h"
#include "idelay.h"
#include "seep.h"

#include "board.h"

//#define C_CODE	// Define for C code, comment out for C++ object

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

#ifdef C_CODE
I2CDev_t g_I2CDev;
#else
I2C g_I2c;
#endif

static const SeepCfg_t s_SeepCfg = {
	.DevAddr = 0x50,
	.AddrLen = 2,
	.PageSize = 32,
	.Size = 128 * 32,
	.WrDelay = 100,
	.WrProtPin = {-1, -1,},
};

#ifdef C_CODE
SeepDev_t g_SeepDev;
#else
Seep g_Seep;
#endif

bool EepromTest()
{
	uint8_t x[512], y[512];
	bool retval = true;

	for (int i = 0; i < 256; i++)
	{
		x[i] = i;
		x[256+i] = 256-i;
	}

#ifdef C_CODE
	SeepWrite(&g_SeepDev, 256, x, 512);
#else
	g_Seep.Write(0, x, 512);
#endif

	memset(y, 0xa5, 512);

#ifdef C_CODE
	SeepRead(&g_SeepDev, 256, y, 512);
#else
	g_Seep.Read(0, y, 512);
#endif

	if (memcmp(x, y, 512) != 0)
	{
		printf("Failed\r\n");
		retval = false;
	}
	else
	{
		printf("Eeprom test passed.\n");
		retval = true;
	}

	return retval;
}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// --specs=rdimon.specs -Wl,--start-group -lgcc -lc -lm -lrdimon -Wl,--end-group
//
// Adjust it for other toolchains.
//

int main()
{
	// Configure

#ifdef C_CODE
	I2CInit(&g_I2CDev, &s_I2cCfg);
	SeepInit(&g_SeepDev, &s_SeepCfg, &g_I2CDev.DevIntrf);
#else
	g_I2c.Init(s_I2cCfg);
	g_Seep.Init(s_SeepCfg, &g_I2c);
#endif

	EepromTest();

	while (1);

	return 0;
}


