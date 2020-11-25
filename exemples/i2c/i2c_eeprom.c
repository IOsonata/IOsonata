/**-------------------------------------------------------------------------
@example	i2c_eeprom.c

@brief	I2C EEPROM example

This example shows how to use I2C driver to read/write EEPROM

@author	Hoang Nguyen Hoan
@date	Sep. 19, 2020

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

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Pins = {
		{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// SDA
		{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_NORMAL},	// SCL
	},
	.Rate = 200000,				// Rate
	.Mode = I2CMODE_MASTER,
	.MaxRetry = 5,				// Retry
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},			// Slave addresses
	.bDmaEn = false,
	.bIntEn = false,
	.IntPrio = 7,				// Interrupt prio
	.EvtCB = NULL				// Event callback
};

I2CDev_t g_I2CDev;

static const SEEP_CFG s_SeepCfg = {
	.DevAddr = 0x50,
	.AddrLen = 2,
	.PageSize = 32,
	.Size = 128 * 32,
	.WrDelay = 5,
	.WrProtPin = {-1, -1,},
};

SEEPDEV g_SeepDev;

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

	I2CInit(&g_I2CDev, &s_I2cCfg);

	SeepInit(&g_SeepDev, &s_SeepCfg, &g_I2CDev.DevIntrf);

	uint8_t x[512];
	uint8_t y[512];


	// Filling test pattern
	for (int i = 0; i < 256; i++)
	{
		x[i] = i;
		x[i + 256] = 256 - i;
	}

	SeepWrite(&g_SeepDev, 0, x, 512);

	memset(y, 0, 512);

	SeepRead(&g_SeepDev, 0, y, 512);

	for (int i = 0; i < 512; i++)
	{
		if (x[i] != y[i])
		{
			printf("Bad %d : %x %x\r\n", i, x, y);
		}
	}

	memset(y, 0, 512);
	SeepWrite(&g_SeepDev, 0, y, 512);
	return 0;
}


