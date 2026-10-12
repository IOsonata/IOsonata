/**-------------------------------------------------------------------------
@file	i2c_lpc_master.h

@brief	Shared LPC state-machine I2C polling master state and family hooks.

@author	Hoang Nguyen Hoan
@date	Oct. 12, 2026

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
#ifndef IOSONATA_I2C_LPC_MASTER_H
#define IOSONATA_I2C_LPC_MASTER_H

#include "i2c_lpc_target.h"
#include "coredev/i2c.h"

#define LPC_I2C_AA		4U
#define LPC_I2C_SI		8U
#define LPC_I2C_STO		16U
#define LPC_I2C_STA		32U
#define LPC_I2C_EN		64U
#define LPC_I2C_CLEAR	(LPC_I2C_AA | LPC_I2C_SI | LPC_I2C_STA | LPC_I2C_EN)
#define LPC_I2C_WAIT_MAX	100000U

typedef struct {
	LpcI2CReg_t *pReg;
	I2CDev_t *pDev;
	uint32_t Clock;
	uint32_t MaxRate;
	uint32_t Low;
	uint32_t High;
	bool bEnabled;
	bool bOwned;
	bool bTransfer;
	bool bRead;
	bool bReadDone;
	bool bError;
} LpcI2CMaster_t;

bool LpcI2CMasterInit(LpcI2CMaster_t *pData, I2CDev_t *pDev, const I2CCfg_t *pCfg);
void LpcI2CMasterClock(LpcI2CMaster_t *pData, bool Enable);
void LpcI2CMasterReset(LpcI2CMaster_t *pData);

#endif
