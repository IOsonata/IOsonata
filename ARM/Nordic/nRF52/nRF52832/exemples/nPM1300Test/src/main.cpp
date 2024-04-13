//============================================================================
// Name        : main.cpp
// Author      : Nguyen Hoan Hoang
// Version     :
// Copyright   : Copyright 2023, I-SYST inc. All rights reserved.
// Description : Hello World in C++
//============================================================================

#include "coredev/i2c.h"
#include "coredev/uart.h"
#include "stddev.h"
#include "pwrmgnt/pm_npm1300.h"

#include "board.h"

#define I2C_SCL_RATE	100000 // Rate in Hz, supported 100k, 250k, and 400k

int I2CIntrfHandler(DevIntrf_t * const pDev, DEVINTRF_EVT EvtId, uint8_t *pBuffer, int BufferLen);

static const IOPinCfg_t s_I2cPins[] = {
	{I2C_SDA_PORT, I2C_SDA_PIN, I2C_SDA_PINOP, IOPINDIR_BI, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SDA
	{I2C_SCL_PORT, I2C_SCL_PIN, I2C_SCL_PINOP, IOPINDIR_OUTPUT, IOPINRES_PULLUP, IOPINTYPE_OPENDRAIN},	// SCL
};

static const I2CCfg_t s_I2cCfg = {
	.DevNo = I2C_DEVNO,			// I2C device number
	.Type = I2CTYPE_STANDARD,
	.Mode = I2CMODE_MASTER,
	.pIOPinMap = s_I2cPins,
	.NbIOPins = sizeof(s_I2cPins) / sizeof(IOPinCfg_t),
	.Rate = I2C_SCL_RATE,		// Rate in Hz
	.MaxRetry = 5,			// Retry
	.AddrType = I2CADDR_TYPE_NORMAL,
	.NbSlaveAddr = 0,			// Number of slave addresses
	.SlaveAddr = {0,},		// Slave addresses
	.bDmaEn = true,
	.bIntEn = false,
	.IntPrio = 7,			// Interrupt prio
	//.EvtCB = I2CIntrfHandler		// Event callback
};

I2C g_I2C;

const BatProfile_t s_BatProf = {
	.OpVolt = 3700,
	.ChrgVolt = 4100,
	.Capacity = 3000,
};

const PwrMgntVoutCfg_t s_PwrVoutCfg[] = {
	{ .mVout = 0, .mAlimit = 0 },
	{ .mVout = 3300, .mAlimit = 100 }
};

PwrMgntCfg_t s_PwrCfg = {
	.DevAddr = NPM1300_I2C_7BITS_ADDR,
	//.InCurLimit = 1500,
	.pVout = NULL,
	.NbVout = 0,
	.VEndChrg = 4100,
	.ChrgCurr = 800,
	.pBatProf = &s_BatProf,
};

PmnPM1300 g_nPM1300;

void HardwareInit()
{
	g_I2C.Init(s_I2cCfg);

	g_nPM1300.Init(s_PwrCfg, &g_I2C);

}

//
// Print a greeting message on standard output and exit.
//
// On embedded platforms this might require semi-hosting or similar.
//
// For example, for toolchains derived from GNU Tools for Embedded,
// to enable semi-hosting, the following was added to the linker:
//
// `--specs=rdimon.specs -Wl,--start-group -lgcc -lc -lc -lm -lrdimon -Wl,--end-group`
//
// Adjust it for other toolchains.
//
// If functionality is not required, to only pass the build, use
// `--specs=nosys.specs`.
//

int main()
{
	HardwareInit();

	//g_nPM1300.Enable();
	if (g_nPM1300.Battery())
	{
		printf("Battery present\n");
	}
	while (1)
	{
		if (g_nPM1300.Charging())
		{
			printf("Charging\n");
		}
		__WFE();
	}

	return 0;
}
